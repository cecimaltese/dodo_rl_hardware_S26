"""
sim_service.py — MuJoCo as a real-time VIRTUAL ROBOT on the dodo ZMQ topics.

This is the sim2sim-becomes-sim2real piece: it binds the exact same endpoints
as motor_service.py + imu_service.py and behaves like the hardware —

    PUB  motor_states  @ 200 Hz   joint pos/vel/torque from the physics
    PUB  imu           @ 100 Hz   gyro + accel + fused quat
                                  (+ ground-truth lin_vel, flagged SIM-ONLY)
    SUB  motor_commands           absolute position targets [rad], NaN = hold
    REP  ctl                      status / enable / disable / reset / ...

so `run_policy.py --backend zmq` cannot tell it from the real robot. Unlike
the old in-process SimEnv (lockstep: physics only advanced when the policy
called step()), this service runs physics against the WALL CLOCK on its own —
motors "keep moving" whether or not the policy is on time, exactly like
hardware. Overruns are counted and reported, never silently absorbed.

Physics/actuation mirrors IsaacLab training exactly as sim_env.py does
(same _build_model): sim dt 5 ms, software PD (KP 32 / KD 3) toward the latest
target each step, effort clamps 27/9 Nm. The 50 Hz-ness of control comes from
the policy's publish rate — identical semantics to IsaacLab's decimation-4.
Position targets are clamped to the same safety limits as motor_service so the
sim rehearses the clamp behavior too.

Robustness rehearsal (optional): inject outbound latency and message drops —
if the stand policy survives `--extra-latency-ms 20 --drop 0.05`, hardware
timing noise is unlikely to be what kills it.

    python3 sim_service.py                          # headless, auto-enabled
    python3 sim_service.py --render                 # viewer (Linux; mac: mjpython)
    python3 sim_service.py --require-enable         # full bring-up parity:
                                                    #   dodo_ctl enable first
    python3 sim_service.py --imu-no-quat            # rehearse accel-only IMUs

Loads the robot from dodo_assets/urdf/dodo_daimao.urdf, the dodo_assets repo next to
this one (assets.py; override with --urdf).
"""

from __future__ import annotations

import argparse
import logging
import math
import random
import time
from collections import deque
from typing import Dict, List, Optional

import mujoco
import numpy as np

import dodo_ipc as ipc
from fleet_config import JOINT_LIMITS_RAD
from motor_service import SAFETY_FACTOR
from rl_env import DEFAULT_JOINT_POS, projected_gravity_from_quat
from sim_env import (KD, KP, SIM_DT, SPAWN_HEIGHT, URDF_PATH, _build_model,
                     _effort_for)

log = logging.getLogger("sim_service")

GRAVITY = 9.81
IMU_DIV = 2                     # publish imu every 2nd physics step -> 100 Hz


class SimService:
    """Real-time MuJoCo service. Pass either urdf_path or a prebuilt model
    (tests use a synthetic model). A model must provide: hinge joints named as
    in dodo_ipc.JOINT_ORDER, actuators 'act_<joint>', sensors 'imu_gyro',
    'imu_vel', 'imu_quat', and a free-joint base (first 7 qpos entries)."""

    def __init__(self,
                 urdf_path: Optional[str] = None,
                 model: Optional[mujoco.MjModel] = None,
                 auto_enable: bool = True,
                 render: bool = False,
                 imu_no_quat: bool = False,
                 extra_latency_s: float = 0.0,
                 drop_prob: float = 0.0,
                 spawn_height: float = SPAWN_HEIGHT):
        if model is None:
            if urdf_path is None:
                raise ValueError("need urdf_path or model")
            model = _build_model(urdf_path)
        self.model = model
        self.data = mujoco.MjData(model)
        self.spawn_height = spawn_height

        m = self.model
        self._qpos_adr = np.array(
            [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
             for j in ipc.JOINT_ORDER])
        self._dof_adr = np.array(
            [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
             for j in ipc.JOINT_ORDER])
        self._act_id = np.array(
            [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{j}")
             for j in ipc.JOINT_ORDER])
        self._effort = np.array([_effort_for(j) for j in ipc.JOINT_ORDER],
                                dtype=np.float64)
        self._default = np.array([DEFAULT_JOINT_POS[j] for j in ipc.JOINT_ORDER],
                                 dtype=np.float64)
        # Same clamp as motor_service: SAFETY_FACTOR × fleet_config limits (the
        # measured Damiao ranges), so the sim rehearses what the robot will clamp.
        self._clamp_lo = np.array(
            [JOINT_LIMITS_RAD[j][0] * SAFETY_FACTOR for j in ipc.JOINT_ORDER])
        self._clamp_hi = np.array(
            [JOINT_LIMITS_RAD[j][1] * SAFETY_FACTOR for j in ipc.JOINT_ORDER])
        self._sens: Dict[str, slice] = {}
        for name in ("imu_gyro", "imu_vel", "imu_quat"):
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
            adr, dim = m.sensor_adr[sid], m.sensor_dim[sid]
            self._sens[name] = slice(adr, adr + dim)

        self._enabled = auto_enable
        self._targets = self._default.copy()
        self._tau = np.zeros(ipc.N_JOINTS, dtype=np.float64)
        self._imu_no_quat = imu_no_quat
        self._extra_latency_s = extra_latency_s
        self._drop_prob = drop_prob
        self._delayq: deque = deque()       # (due_mono_s, publisher, msg, t_ns)
        self._running = True

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
        log.info("sim_service up: physics %.0f Hz, states %.0f Hz, imu %.0f Hz, "
                 "%s%s%s", 1 / SIM_DT, 1 / SIM_DT, 1 / (SIM_DT * IMU_DIV),
                 "ENABLED" if self._enabled else "disabled (ctl enable first)",
                 f", +{extra_latency_s * 1e3:.0f}ms latency" if extra_latency_s else "",
                 f", {drop_prob:.0%} drop" if drop_prob else "")

    # ── ctl ───────────────────────────────────────────────────────
    def _ctl_status(self, _a: dict) -> dict:
        return {
            "backend": "sim",
            "enabled": self._enabled,
            "estopped": False,
            "all_calibrated": True,
            "sim_time_s": round(self.data.time, 3),
            "loop": self.loop.stats() if hasattr(self, "loop") else None,
            "joints": {
                j: {"type": "sim", "pos_rad": round(float(self.data.qpos[qa]), 4),
                    "has_feedback": True, "calibrated": True, "idled": False,
                    "fault": None}
                for j, qa in zip(ipc.JOINT_ORDER, self._qpos_adr)},
        }

    def _ctl_enable(self, _a: dict) -> dict:
        self._enabled = True
        log.info("sim ENABLED")
        return {"enabled": True}

    def _ctl_disable(self, _a: dict) -> dict:
        self._enabled = False
        log.info("sim DISABLED (joints go limp)")
        return {"enabled": False}

    def _ctl_reset(self, _a: dict) -> dict:
        self._reset_pose()
        log.info("sim reset to default pose")
        return {"reset": True}

    def _reset_pose(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = [0.0, 0.0, self.spawn_height]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qpos[self._qpos_adr] = self._default
        self.data.qvel[:] = 0.0
        self._targets = self._default.copy()
        mujoco.mj_forward(self.model, self.data)

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
        gyro = tuple(float(x) for x in d.sensordata[self._sens["imu_gyro"]])
        lin_vel = tuple(float(x) for x in d.sensordata[self._sens["imu_vel"]])
        quat = tuple(float(x) for x in d.sensordata[self._sens["imu_quat"]])  # wxyz
        # Accelerometer model (quasi-static): specific force = -g in body frame.
        # Good enough to rehearse the accel-fallback gravity estimate; policies
        # normally use the fused quat.
        pg = projected_gravity_from_quat(quat)
        accel = tuple(float(-GRAVITY * x) for x in pg)
        flags = ipc.IMU_LIN_VEL_VALID
        if not self._imu_no_quat:
            flags |= ipc.IMU_QUAT_VALID
        return ipc.ImuSample(gyro=gyro, accel=accel,
                             quat=(0.0, 0.0, 0.0, 0.0) if self._imu_no_quat else quat,
                             lin_vel=lin_vel, flags=flags)

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

                # physics step with software PD (mirrors IsaacLab/sim_env)
                q = self.data.qpos[self._qpos_adr]
                qd = self.data.qvel[self._dof_adr]
                if self._enabled:
                    self._tau = np.clip(KP * (self._targets - q) - KD * qd,
                                        -self._effort, self._effort)
                else:
                    self._tau = np.zeros(ipc.N_JOINTS)
                self.data.ctrl[self._act_id] = self._tau
                mujoco.mj_step(self.model, self.data)

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
    ap.add_argument("--urdf", default=str(URDF_PATH), help="robot URDF")
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

    from pathlib import Path
    if not Path(args.urdf).exists():
        raise SystemExit(f"URDF not found: {args.urdf}")

    svc = SimService(urdf_path=args.urdf,
                     auto_enable=not args.require_enable,
                     render=args.render,
                     imu_no_quat=args.imu_no_quat,
                     extra_latency_s=args.extra_latency_ms / 1e3,
                     drop_prob=args.drop)
    svc.run()


if __name__ == "__main__":
    main()
