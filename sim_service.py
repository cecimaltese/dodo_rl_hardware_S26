"""
sim_service.py — MuJoCo as a real-time VIRTUAL ROBOT on the dodo ZMQ topics.

This is the sim2sim-becomes-sim2real piece: it binds the exact same endpoints
as motor_service.py + imu_service.py and behaves like the hardware —

    PUB  motor_states  @ 200 Hz   joint pos/vel/torque from the physics
    PUB  imu           @ 100 Hz   gyro + accel + fused quat
                                  (+ ground-truth lin_vel, flagged SIM-ONLY)
    SUB  motor_commands           absolute position targets [rad], NaN = hold
    REP  ctl                      status / enable / disable / reset / gantry / ...

so `run_policy.py --backend zmq` and `tools/dodo_startup.py` cannot tell it from the
real robot. Unlike the old in-process SimEnv (lockstep: physics only advanced when
the policy called step()), this service runs physics against the WALL CLOCK on its
own — motors "keep moving" whether or not the client is on time, exactly like
hardware. Overruns are counted and reported, never silently absorbed.

TWO MODELS
  default   The old stand policy's training setup, as sim_env.py builds it from the
            URDF: 5 ms step, software PD (KP 32 / KD 3) toward the latest target,
            effort clamps 27/9 Nm, IMU site on the body frame. What the vendored
            stand policy (policies/) and its tests need.
  --scene   dodo_assets' mjcf/scene.xml (Shafeef's): MuJoCo position actuators, i.e.
            an implicit PD with kp 150/150/50/50, kd 1.5/1.5/0.8/0.8 (hip, upper_leg,
            knee, ankle; override with --kp/--kd), armature 0.005, keyframes `home`
            and `straight`. Physics at 1 ms, 5 steps per 5 ms tick (scene.xml says
            2 ms; finer is only more accurate, and keeps states at motor_service's
            200 Hz). The real drives also run their PD internally at kHz, so this is
            the closer model of the hardware. The IMU is the MJCF's own: sensors in
            the CHIP frame, rotated into the body frame by imu_protocol's AXIS_REMAP,
            like the real chip, so the deployment remap is exercised too.

GANTRY (--scene --gantry): the robot hangs from four ropes on its top face, feet
3 cm above the floor with straight legs, as on the real gantry. `dodo_ctl gantry
--arg lower=0.02` pays out rope; the ropes go slack once the feet carry the weight;
`--arg release=true` takes them away. With --require-enable this is the hardware
procedure: hang limp, enable, ramp to the stand pose in the air, lower, release.

Position targets are clamped like the robot will clamp them: STOP_MARGIN_RAD inside
each joint's measured stops (real2sim.command_band, joint_calib.py), in sim radians.
`enable` is bumpless like motor_service's: the targets start where the joints are.

Robustness rehearsal (optional): inject outbound latency and message drops —
if the stand policy survives `--extra-latency-ms 20 --drop 0.05`, hardware
timing noise is unlikely to be what kills it.

    python3 sim_service.py                          # stand-policy model, auto-enabled
    python3 sim_service.py --scene --render         # Shafeef's scene, standing in `home`
    python3 sim_service.py --scene --gantry --require-enable --render
                                                    # rehearse the gantry bring-up:
    python3 tools/dodo_startup.py --pose home       #   enable + ramp + hold (other terminal)
    python3 tools/dodo_ctl.py gantry --arg lower=0.03   # repeat until the feet land
    python3 tools/dodo_ctl.py gantry --arg release=true
    python3 sim_service.py --imu-no-quat            # rehearse accel-only IMUs

The robot comes from the dodo_assets repo next to this one (assets.py): its URDF by
default (override with --urdf), its scene.xml with --scene.
"""

from __future__ import annotations

import argparse
import logging
import math
import random
import time
from collections import deque
from typing import Dict, Optional, Sequence

import mujoco
import numpy as np

import assets
import dodo_ipc as ipc
import imu_protocol as proto
import real2sim
from fleet_config import STOP_MARGIN_RAD
from rl_env import DEFAULT_JOINT_POS, projected_gravity_from_quat
from sim_env import KD, KP, SIM_DT, SPAWN_HEIGHT, URDF_PATH, _build_model, _effort_for

log = logging.getLogger("sim_service")

GRAVITY = 9.81
IMU_DIV = 2                     # publish imu every 2nd tick -> 100 Hz

SCENE_DT = 0.001                # --scene physics step; SIM_DT / SCENE_DT steps per tick
GROUPS = ("hip", "upper_leg", "lower_leg", "foot")   # joint name = <group>_<side>
GANTRY_CLEARANCE_M = 0.03       # feet above the floor, hanging with straight legs
GANTRY_ROPE_M = 0.5
GANTRY_RELEASED_M = 10.0        # a rope this long never goes taut
# Rope attachment points on the body's top face (x, y), body frame: four corners of
# its collision box, pulled in a little. Parallel ropes keep the body level.
GANTRY_ROPES = ((0.12, 0.08), (0.12, -0.08), (-0.08, 0.08), (-0.08, -0.08))
_ACTUATION = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)


def build_scene_model(scene_path=assets.SCENE_PATH,
                      kp: Optional[Sequence[float]] = None,
                      kd: Optional[Sequence[float]] = None,
                      gantry: bool = False) -> mujoco.MjModel:
    """dodo_assets' scene.xml for sim_service: its PD actuators (kp/kd per joint group
    in GROUPS order replace scene.xml's), a 1 ms step, a ground-truth velocimeter
    `imu_vel` on the IMU site (SIM-ONLY, flagged as such on the wire), and with
    `gantry` four ropes (limited spatial tendons) from fixed anchors to the body."""
    spec = mujoco.MjSpec.from_file(str(assets.require(scene_path)))
    spec.option.timestep = SCENE_DT
    spec.add_sensor(name="imu_vel", type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    if gantry:
        box = spec.geom("body_collision")
        top = float(box.pos[2] + box.size[2])
        hang_z = float(spec.key("straight").qpos[2]) + GANTRY_CLEARANCE_M
        body = spec.body("body")
        for i, (x, y) in enumerate(GANTRY_ROPES):
            body.add_site(name=f"gantry_rope{i}_body", pos=[x, y, top])
            spec.worldbody.add_site(name=f"gantry_rope{i}_anchor",
                                    pos=[x, y, hang_z + top + GANTRY_ROPE_M])
            t = spec.add_tendon(name=f"gantry_rope{i}", range=[0.0, GANTRY_ROPE_M],
                                limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                                rgba=[0.9, 0.8, 0.1, 1.0])
            t.wrap_site(f"gantry_rope{i}_anchor")
            t.wrap_site(f"gantry_rope{i}_body")
    m = spec.compile()
    for gains, name in ((kp, "kp"), (kd, "kd")):
        if gains is not None and len(gains) != len(GROUPS):
            raise ValueError(f"{name}: one value per joint group {GROUPS}, got {list(gains)}")
    for a in range(m.nu):
        g = GROUPS.index(m.actuator(a).name.rsplit("_", 1)[0])
        if kp is not None:   # position actuator: force = kp*ctrl - kp*q - kd*qdot
            m.actuator_gainprm[a, 0], m.actuator_biasprm[a, 1] = kp[g], -kp[g]
        if kd is not None:
            m.actuator_biasprm[a, 2] = -kd[g]
    return m


class SimService:
    """Real-time MuJoCo service. Pass urdf_path, a prebuilt model (tests use a
    synthetic one), or use SimService.from_scene().

    A default-mode model must provide: hinge joints named as in dodo_ipc.JOINT_ORDER,
    torque actuators 'act_<joint>', sensors 'imu_gyro', 'imu_vel', 'imu_quat' on a
    body-frame site, and a free-joint base (first 7 qpos entries). A scene model
    (scene=True) has position actuators named like the joints, chip-frame sensors
    'imu_gyro', 'imu_acc', 'imu_quat', 'imu_vel', and keyframes."""

    def __init__(self,
                 urdf_path: Optional[str] = None,
                 model: Optional[mujoco.MjModel] = None,
                 auto_enable: bool = True,
                 render: bool = False,
                 imu_no_quat: bool = False,
                 extra_latency_s: float = 0.0,
                 drop_prob: float = 0.0,
                 spawn_height: float = SPAWN_HEIGHT,
                 scene: bool = False,
                 start: str = "home"):
        if model is None:
            if urdf_path is None:
                raise ValueError("need urdf_path or model")
            model = _build_model(urdf_path)
        self.model = model
        self.data = mujoco.MjData(model)
        self.spawn_height = spawn_height
        self.scene = scene
        self.start = start

        m = self.model
        steps = SIM_DT / m.opt.timestep
        if abs(steps - round(steps)) > 1e-9:
            raise ValueError(f"physics step {m.opt.timestep} s doesn't divide the "
                             f"{SIM_DT} s tick")
        self._substeps = int(round(steps))
        self._qpos_adr = np.array(
            [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
             for j in ipc.JOINT_ORDER])
        self._dof_adr = np.array(
            [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
             for j in ipc.JOINT_ORDER])
        self._act_id = np.array(
            [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, j if scene else f"act_{j}")
             for j in ipc.JOINT_ORDER])
        if (self._act_id < 0).any():
            raise ValueError("model lacks an actuator for every joint "
                             f"({'<joint>' if scene else 'act_<joint>'})")
        self._effort = np.array([_effort_for(j) for j in ipc.JOINT_ORDER],
                                dtype=np.float64)
        self._default = np.array([DEFAULT_JOINT_POS[j] for j in ipc.JOINT_ORDER],
                                 dtype=np.float64)
        self._clamp_lo, self._clamp_hi = self._command_band()
        self._sens: Dict[str, slice] = {}
        for name in ("imu_gyro", "imu_vel", "imu_quat") + (("imu_acc",) if scene else ()):
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid < 0:
                raise ValueError(f"model lacks the sensor '{name}'")
            adr, dim = m.sensor_adr[sid], m.sensor_dim[sid]
            self._sens[name] = slice(adr, adr + dim)
        self._body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "body")
        self._ropes = [t for t in range(m.ntendon)
                       if m.tendon(t).name.startswith("gantry_rope")]

        self._enabled = auto_enable
        self._targets = self._default.copy()
        self._tau = np.zeros(ipc.N_JOINTS, dtype=np.float64)
        self._imu_no_quat = imu_no_quat
        self._extra_latency_s = extra_latency_s
        self._drop_prob = drop_prob
        self._delayq: deque = deque()       # (due_mono_s, publisher, msg, t_ns)
        self._running = True
        self.track: Dict[str, float] = {}   # base height/tilt while enabled: _track()

        self.state_pub = ipc.Publisher(ipc.TOPIC_MOTOR_STATES)
        self.imu_pub = ipc.Publisher(ipc.TOPIC_IMU)
        self.cmd_sub = ipc.Subscriber(ipc.TOPIC_MOTOR_COMMANDS, conflate=True)
        self.ctl = ipc.CtlServer()
        self._ctl_handlers = {
            "status": self._ctl_status,
            "enable": self._ctl_enable,
            "disable": self._ctl_disable,
            "calibrate": lambda a: {"note": "sim is always calibrated"},
            "set_zero": lambda a: {"note": "sim is always calibrated"},
            "estop_reset": lambda a: {"note": "sim has no e-stop"},
            "reset": self._ctl_reset,
            "gantry": self._ctl_gantry,
        }

        self.viewer = None
        if render:
            try:
                import mujoco.viewer as mj_viewer
                self.viewer = mj_viewer.launch_passive(self.model, self.data)
            except RuntimeError as e:
                log.warning("viewer unavailable (%s) — running headless. "
                            "On macOS use `mjpython sim_service.py --render`.", e)

        self._reset_pose()
        self._set_actuation()
        log.info("sim_service up (%s): physics %.0f Hz, states %.0f Hz, imu %.0f Hz, "
                 "%s%s%s%s",
                 f"scene, start '{start}'" if scene else "stand-policy model",
                 1 / m.opt.timestep, 1 / SIM_DT, 1 / (SIM_DT * IMU_DIV),
                 "ENABLED" if self._enabled else "disabled (ctl enable first)",
                 ", on the gantry" if self._ropes else "",
                 f", +{extra_latency_s * 1e3:.0f}ms latency" if extra_latency_s else "",
                 f", {drop_prob:.0%} drop" if drop_prob else "")

    @classmethod
    def from_scene(cls, scene_path=assets.SCENE_PATH, kp=None, kd=None,
                   gantry: bool = False, start: str = "home", **kwargs) -> "SimService":
        """dodo_assets' scene.xml with its PD actuators (build_scene_model), starting
        in keyframe `start` (on the floor, or hanging if `gantry`)."""
        return cls(model=build_scene_model(scene_path, kp, kd, gantry),
                   scene=True, start=start, **kwargs)

    def _command_band(self):
        """Per joint: real2sim.command_band (measured stops - STOP_MARGIN_RAD), else the
        model's own joint range minus the margin, else unclamped."""
        lo, hi = [], []
        for j in ipc.JOINT_ORDER:
            band = real2sim.command_band(j)
            if band is None:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
                if self.model.jnt_limited[jid]:
                    r = self.model.jnt_range[jid]
                    band = (r[0] + STOP_MARGIN_RAD, r[1] - STOP_MARGIN_RAD)
                else:
                    band = (-math.inf, math.inf)
                log.warning("%s: no measured stops in joint_calib.py; commands clamped "
                            "to %s", j, band)
            lo.append(band[0])
            hi.append(band[1])
        return np.array(lo), np.array(hi)

    # ── ctl ───────────────────────────────────────────────────────
    def _base(self) -> Dict[str, float]:
        z_up = float(self.data.xmat[self._body_id][8])
        return {"base_z": float(self.data.xpos[self._body_id][2]),
                "tilt_deg": math.degrees(math.acos(max(-1.0, min(1.0, z_up))))}

    def _ctl_status(self, _a: dict) -> dict:
        sim = {"model": "scene" if self.scene else "stand-policy", **self._base(),
               "while_enabled": dict(self.track)}
        if self._ropes:
            sim["gantry"] = self._gantry_state()
        return {
            "backend": "sim",
            "enabled": self._enabled,
            "estopped": False,
            "all_calibrated": True,
            "sim_time_s": round(self.data.time, 3),
            "loop": self.loop.stats() if hasattr(self, "loop") else None,
            "sim": sim,
            "joints": {
                j: {"type": "sim", "pos_rad": round(float(self.data.qpos[qa]), 4),
                    "target_rad": round(float(t), 4) if self._enabled else None,
                    "has_feedback": True, "calibrated": True, "idled": False,
                    "fault": None}
                for j, qa, t in zip(ipc.JOINT_ORDER, self._qpos_adr, self._targets)},
        }

    def _ctl_enable(self, _a: dict) -> dict:
        # Like motor_service: a no-op while enabled (re-holding the measured angles
        # would drop the PD's preload under load), else bumpless: hold the joints
        # where they are, inside the band.
        if self._enabled:
            return {"enabled": True, "unparking": {}}
        q = self.data.qpos[self._qpos_adr]
        self._targets = np.clip(q, self._clamp_lo, self._clamp_hi)
        self._enabled = True
        self.track = {}
        self._set_actuation()
        log.info("sim ENABLED (holding the current pose)")
        return {"enabled": True}

    def _ctl_disable(self, _a: dict) -> dict:
        self._enabled = False
        self._set_actuation()
        log.info("sim DISABLED (joints go limp)")
        return {"enabled": False}

    def _ctl_reset(self, _a: dict) -> dict:
        self._reset_pose()
        log.info("sim reset to the start pose")
        return {"reset": True}

    def _gantry_state(self) -> dict:
        r = self.model.tendon_range[self._ropes, 1]
        length = self.data.ten_length[self._ropes]
        return {"rope_m": round(float(r.max()), 4),
                "released": bool(r.min() >= GANTRY_RELEASED_M),
                "taut": int((length > r - 1e-3).sum())}

    def _ctl_gantry(self, a: dict) -> dict:
        """lower=<m>: pay out that much rope; length=<m>: set it; release=true: take
        the ropes away (they never go taut again until `reset`)."""
        if not self._ropes:
            raise RuntimeError("no gantry: start with --scene --gantry")
        r = self.model.tendon_range
        if a.get("release"):
            r[self._ropes, 1] = GANTRY_RELEASED_M
        elif "length" in a:
            r[self._ropes, 1] = float(a["length"])
        elif "lower" in a:
            r[self._ropes, 1] = np.minimum(r[self._ropes, 1] + float(a["lower"]),
                                           GANTRY_RELEASED_M)
        st = self._gantry_state()
        log.info("gantry: rope %.3f m%s, %d of %d taut", st["rope_m"],
                 " (released)" if st["released"] else "", st["taut"], len(self._ropes))
        return st

    def _set_actuation(self):
        """Scene mode: a disabled robot is limp because its actuators are off (the
        position actuators would otherwise keep pulling to their last ctrl)."""
        if not self.scene:
            return
        if self._enabled:
            self.model.opt.disableflags &= ~_ACTUATION
        else:
            self.model.opt.disableflags |= _ACTUATION

    def _reset_pose(self):
        m, d = self.model, self.data
        if self.scene:
            mujoco.mj_resetDataKeyframe(m, d, m.key(self.start).id)
            if self._ropes:
                m.tendon_range[self._ropes, 1] = GANTRY_ROPE_M
                d.qpos[2] = m.key("straight").qpos[2] + GANTRY_CLEARANCE_M
            self._targets = d.qpos[self._qpos_adr].copy()
        else:
            mujoco.mj_resetData(m, d)
            d.qpos[0:3] = [0.0, 0.0, self.spawn_height]
            d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
            d.qpos[self._qpos_adr] = self._default
            self._targets = self._default.copy()
        d.qvel[:] = 0.0
        self.track = {}
        mujoco.mj_forward(m, d)

    def _track(self):
        """Lowest base height and largest tilt since the last enable, and the latest,
        while enabled: what the PD stand checks read."""
        b = self._base()
        t = self.track
        t["base_z"], t["tilt_deg"] = b["base_z"], b["tilt_deg"]
        t["min_base_z"] = min(t.get("min_base_z", math.inf), b["base_z"])
        t["max_tilt_deg"] = max(t.get("max_tilt_deg", 0.0), b["tilt_deg"])
        t["max_target_err_rad"] = float(np.abs(self.data.qpos[self._qpos_adr]
                                               - self._targets).max())

    # ── outbound path with optional latency/drop injection ────────
    def _publish(self, pub: ipc.Publisher, msg) -> None:
        if self._drop_prob and random.random() < self._drop_prob:
            return
        if self._extra_latency_s <= 0.0:
            pub.send(msg)
            return
        self._delayq.append((time.monotonic() + self._extra_latency_s,
                             pub, msg, ipc.now_ns()))

    def _flush_delayed(self) -> None:
        now = time.monotonic()
        while self._delayq and self._delayq[0][0] <= now:
            _, pub, msg, t_ns = self._delayq.popleft()
            pub.send(msg, t_mono_ns=t_ns)

    # ── message assembly ──────────────────────────────────────────
    def _states_msg(self) -> ipc.MotorStates:
        pos = self.data.qpos[self._qpos_adr]
        vel = self.data.qvel[self._dof_adr]
        f = ipc.JF_HAS_FEEDBACK | ipc.JF_CALIBRATED
        if self._enabled:
            f |= ipc.JF_ENABLED
        g = ipc.GF_ALL_CALIBRATED | (ipc.GF_ENABLED if self._enabled else 0)
        return ipc.MotorStates(
            pos=[float(x) for x in pos], vel=[float(x) for x in vel],
            torque=[float(x) for x in self._tau],
            joint_flags=[f] * ipc.N_JOINTS, global_flags=g)

    def _imu_msg(self) -> ipc.ImuSample:
        d = self.data
        s = {k: d.sensordata[v] for k, v in self._sens.items()}
        if self.scene:
            # The MJCF's sensors read in the chip frame, like the real BHI260AP: rotate
            # them into the body frame exactly as imu_service does (AXIS_REMAP).
            gyro = proto.board_to_base(s["imu_gyro"])
            lin_vel = proto.board_to_base(s["imu_vel"])
            quat = proto.board_quat_to_base(s["imu_quat"])
            accel = proto.board_to_base(s["imu_acc"])
        else:
            gyro = tuple(float(x) for x in s["imu_gyro"])
            lin_vel = tuple(float(x) for x in s["imu_vel"])
            quat = tuple(float(x) for x in s["imu_quat"])  # wxyz, body frame
            # Accelerometer model (quasi-static): specific force = -g in body frame.
            # Good enough to rehearse the accel-fallback gravity estimate; policies
            # normally use the fused quat.
            accel = tuple(float(-GRAVITY * x) for x in projected_gravity_from_quat(quat))
        flags = ipc.IMU_LIN_VEL_VALID
        if not self._imu_no_quat:
            flags |= ipc.IMU_QUAT_VALID
        return ipc.ImuSample(gyro=gyro, accel=accel,
                             quat=(0.0, 0.0, 0.0, 0.0) if self._imu_no_quat else quat,
                             lin_vel=lin_vel, flags=flags)

    # ── physics ───────────────────────────────────────────────────
    def _physics_tick(self):
        """One SIM_DT tick of physics toward the current targets."""
        if self.scene:
            # Position actuators: the PD runs inside MuJoCo (implicit in kd), like the
            # drives' own loops. Off while disabled (_set_actuation).
            self.data.ctrl[self._act_id] = self._targets
            for _ in range(self._substeps):
                mujoco.mj_step(self.model, self.data)
            self._tau = (self.data.actuator_force[self._act_id].copy() if self._enabled
                         else np.zeros(ipc.N_JOINTS))
        else:
            # software PD (mirrors IsaacLab/sim_env)
            q = self.data.qpos[self._qpos_adr]
            qd = self.data.qvel[self._dof_adr]
            if self._enabled:
                self._tau = np.clip(KP * (self._targets - q) - KD * qd,
                                    -self._effort, self._effort)
            else:
                self._tau = np.zeros(ipc.N_JOINTS)
            self.data.ctrl[self._act_id] = self._tau
            for _ in range(self._substeps):
                mujoco.mj_step(self.model, self.data)
        if self._enabled:
            self._track()

    # ── main loop ─────────────────────────────────────────────────
    def run(self, max_wall_s: Optional[float] = None):
        self.loop = ipc.RateLoop(1.0 / SIM_DT, name="sim_service")
        tick = 0
        last_report = time.monotonic()
        last_overruns = 0
        t_start = time.monotonic()
        try:
            while self._running:
                if max_wall_s is not None and time.monotonic() - t_start > max_wall_s:
                    break
                self.ctl.poll_once(self._ctl_handlers)

                cmd = self.cmd_sub.latest()
                if cmd is not None and self._enabled:
                    t = np.asarray(cmd.target_pos, dtype=np.float64)
                    keep = np.isnan(t)
                    t = np.clip(t, self._clamp_lo, self._clamp_hi)
                    self._targets = np.where(keep, self._targets, t)

                self._physics_tick()

                self._publish(self.state_pub, self._states_msg())
                if tick % IMU_DIV == 0:
                    self._publish(self.imu_pub, self._imu_msg())
                self._flush_delayed()

                if self.viewer is not None and tick % 4 == 0:
                    if not self.viewer.is_running():
                        break
                    self.viewer.sync()

                now = time.monotonic()
                if now - last_report > 5.0:
                    new_over = self.loop.overruns - last_overruns
                    lvl = logging.WARNING if new_over else logging.DEBUG
                    log.log(lvl, "%s (sim time %.1f s, wall %.1f s)",
                            self.loop.stats_str(), self.data.time, now - t_start)
                    last_overruns = self.loop.overruns
                    last_report = now

                tick += 1
                self.loop.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            log.info(self.loop.stats_str())
            self.close()

    def close(self):
        self._running = False
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None
        for s in (self.state_pub, self.imu_pub, self.cmd_sub, self.ctl):
            try:
                s.close()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=str(URDF_PATH), help="robot URDF (default model)")
    ap.add_argument("--scene", action="store_true",
                    help="dodo_assets' scene.xml with its PD actuators (PD stand)")
    ap.add_argument("--start", default="home",
                    help="--scene: keyframe to start in (home | straight)")
    ap.add_argument("--gantry", action="store_true",
                    help="--scene: hang from ropes, feet just off the floor "
                         "(ctl gantry lowers/releases)")
    ap.add_argument("--kp", type=float, nargs=4, metavar=GROUPS,
                    help="--scene: stiffness per joint group [Nm/rad] (default scene.xml)")
    ap.add_argument("--kd", type=float, nargs=4, metavar=GROUPS,
                    help="--scene: damping per joint group [Nm s/rad] (default scene.xml)")
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--require-enable", action="store_true",
                    help="start disabled, like the real motor_service "
                         "(bring-up parity: dodo_ctl enable first)")
    ap.add_argument("--imu-no-quat", action="store_true",
                    help="suppress the fused quat (rehearse accel-only IMUs)")
    ap.add_argument("--extra-latency-ms", type=float, default=0.0,
                    help="inject outbound latency on states+imu")
    ap.add_argument("--drop", type=float, default=0.0,
                    help="probability of dropping each outbound message (0..1)")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s %(name)s %(levelname)s: %(message)s")
    if not args.scene and (args.gantry or args.kp or args.kd):
        raise SystemExit("--gantry, --kp and --kd need --scene")

    common = dict(auto_enable=not args.require_enable, render=args.render,
                  imu_no_quat=args.imu_no_quat,
                  extra_latency_s=args.extra_latency_ms / 1e3, drop_prob=args.drop)
    try:
        if args.scene:
            svc = SimService.from_scene(kp=args.kp, kd=args.kd, gantry=args.gantry,
                                        start=args.start, **common)
        else:
            svc = SimService(urdf_path=str(assets.require(args.urdf)), **common)
    except FileNotFoundError as e:
        raise SystemExit(str(e))
    svc.run()


if __name__ == "__main__":
    main()
