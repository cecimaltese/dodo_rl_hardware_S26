"""
motor_service.py — the dodo motor fleet as a pure-ZMQ service.

Owns the WHOLE motor fleet (Damiao + ODrive) behind one motor-type-agnostic
interface:

    PUB  motor_states    @ rate_hz (200 Hz)  pos/vel/torque [rad, rad/s, Nm]
                                             + per-joint & global flags
    SUB  motor_commands  latest-value        absolute position targets [rad],
                                             NaN = leave that joint alone
    REP  ctl             on demand           status / enable / disable /
                                             set_zero / estop_reset

This is a *pure* control service: it knows nothing about the policy (joint
order on the wire is dodo_ipc.JOINT_ORDER; default pose / action scaling live
in the Env layer). Units are converted to each motor's native units here
(Damiao = radians, ODrive = turns) and NEVER on the wire.

Design choices (all deliberate):
  * NO auto-enable at startup. Bring-up is explicit via tools/dodo_ctl.py:
        status → calibrate → enable            (normal startup, see below)
  * States keep publishing after an e-stop (with the ESTOPPED flag set) so
    consumers see WHY the robot stopped instead of a silent stream.
  * Per-joint `calibrated` flag on the wire: Damiao joints have a persistent
    hardware zero → calibrated from boot; ODrive positions are turns-since-boot
    → calibrated by ctl `calibrate` (boot pose, below) or `set_zero` (jig).
  * `enable` is BUMPLESS and fail-closed: it refuses if any joint is silent,
    presets every setpoint to the measured position (so nothing jumps when the
    loops close), then slowly "unparks" joints that sit outside the safety
    band (e.g. folded at the stops) to the band edge.

Boot-pose calibration (replaces a homing routine — there is none, on purpose:
driving the ODrive joints into their stops under power could damage the
3D-printed structure/belts):
  * POWER ON the robot with the ODrive-driven joints folded all the way back
    against their mechanical stops (knees + ankles at the BOOT_POSE_RAD below).
  * ctl `calibrate` then maps turns-since-boot onto those known angles.
    Damiao joints are untouched (their zero is hardware-persistent).
  * `calibrate`/`set_zero` are only accepted while DISABLED.

Safety layer:
  * position commands clamped to SAFETY_FACTOR × fleet_config joint limits
    (native units)
  * |torque| > 0.8 × rated  → that ONE motor to IDLE (professor's spec)
  * motor fault / over-speed / measured over-travel / silent feedback /
    stalled command stream → full e-stop (all motors disabled, latched until
    ctl estop_reset)
  * "feedback" always means FRESH feedback (younger than FEEDBACK_FRESH_S):
    both drivers cache their last frame forever, so a joint that stopped
    talking would otherwise look alive with frozen numbers. Silent means: an
    ODrive that has ever streamed (they stream whether enabled or not), or a
    Damiao while the fleet is enabled (they only answer the frames the loop
    sends every tick). A silent ODrive also loses its calibration — if it
    rebooted, its encoder re-zeroed.

Fleet, buses, limits and the boot pose all come from fleet_config.py — the
single source of truth; nothing about the robot is configured here.

CAN topology: two buses, one USB-CAN adapter each (Damiao on DM_CHANNEL, ODrive
on OD_CHANNEL; stable names from the udev rule, see fleet_config.py). Both types
can share one bus (pass the same channel twice) — then the Damiao socket gets
filtered, and the ODrive node ids must stay out of the Damiao id windows.
The buses come up at 1 Mbit when the adapters appear (setup/99-dodo-can.rules);
by hand:
    sudo ip link set can_dm up type can bitrate 1000000
    sudo ip link set can_od up type can bitrate 1000000

Run (robot):
    python3 motor_service.py                    # full 8-joint fleet
    python3 motor_service.py --only hip_left,lower_leg_left   # bench subset
"""

from __future__ import annotations

import argparse
import logging
import threading
import time
from typing import Dict, List, Optional, Union

import can
from damiao_motor import DaMiaoController

import dodo_ipc as ipc
import real2sim
from damiao_joint import DamiaoJoint
from odrive_can_joint import OdriveCanJoint
from fleet_config import (DM_JOINTS, OD_JOINTS, DM_CHANNEL, OD_CHANNEL, BUSTYPE,
                          JOINT_LIMITS_RAD, RATED_TORQUE_NM, JOINT_VEL_LIMIT_RAD,
                          BOOT_POSE_RAD, FEEDBACK_FRESH_S)

log = logging.getLogger("motor_service")

DM_REGISTER_ID = 0x7FF  # Damiao register read/write reply arbitration id

# --- Safety factors / thresholds -----------------------------------------------
SAFETY_FACTOR = 0.9          # margin applied to position-clamp + velocity limit
TORQUE_IDLE_FACTOR = 0.8     # |torque| > 0.8 * rated  ->  that motor goes to IDLE
CMD_WATCHDOG_S = 0.5         # once commands stream (while enabled), a gap -> e-stop
FB_WATCHDOG_S = 0.5          # a joint that should be reporting goes silent -> e-stop
POS_OVERTRAVEL_MARGIN = 0.10 # rad past the hard URDF limit (measured) -> e-stop
STARTUP_GRACE_S = 1.0        # ignore safety trips for the first second
UNPARK_DURATION_S = 1.5      # slow move from the stop to the safety-band edge at enable

DM_FAULT_STATUS = {
    "OVER_VOLTAGE", "UNDER_VOLTAGE", "OVER_CURRENT",
    "MOS_OVER_TEMP", "ROTOR_OVER_TEMP", "LOST_COMM", "OVERLOAD",
}

AnyJoint = Union[DamiaoJoint, OdriveCanJoint]


# ---------------------------------------------------------------------------
# Per-joint adapter: unifies units (rad <-> native) + safety limits + state.
# ---------------------------------------------------------------------------
class _JointAdapter:
    """Wraps any joint so the service treats Damiao (rad) and ODrive (turns)
    uniformly: positions/velocities go on the wire in radians, while clamping
    and feedback monitoring happen in the joint's native units."""

    def __init__(self, joint: AnyJoint):
        self.joint = joint
        self.name = joint.cfg.name
        self.is_odrive = isinstance(joint, OdriveCanJoint)
        self.idled = False              # set True once this joint is IDLEd by safety
        # The setpoint this joint is being held at / moving to, joint rad (None while
        # disabled). Reported by `status` so a client can continue from the targets
        # instead of the measured angles: under load those sag below the targets, and
        # re-targeting there drops the preload (a standing robot tips over).
        self.target_rad: Optional[float] = None
        # Damiao has a persistent hardware zero -> calibrated from boot.
        # ODrive reports turns-since-boot -> needs ctl `calibrate` (powered on
        # in the boot pose, legs at the stops) or `set_zero` (jig).
        self.calibrated = not self.is_odrive

        cfg = joint.cfg
        lo_rad, hi_rad = JOINT_LIMITS_RAD.get(self.name, (None, None))
        self.pos_hard_lo_rad, self.pos_hard_hi_rad = lo_rad, hi_rad
        self.pos_lo = self.to_native(lo_rad * SAFETY_FACTOR) if lo_rad is not None else None
        self.pos_hi = self.to_native(hi_rad * SAFETY_FACTOR) if hi_rad is not None else None
        cpm, cpx = getattr(cfg, "pos_min", None), getattr(cfg, "pos_max", None)
        if cpm is not None:
            self.pos_lo = cpm if self.pos_lo is None else max(self.pos_lo, cpm)
        if cpx is not None:
            self.pos_hi = cpx if self.pos_hi is None else min(self.pos_hi, cpx)
        if self.pos_lo is None or self.pos_hi is None:
            log.warning("%s: position UNCLAMPED (no URDF limit found).", self.name)

        self.vel_limit = self.to_native(SAFETY_FACTOR * JOINT_VEL_LIMIT_RAD)
        self.rated_torque = RATED_TORQUE_NM.get(
            self.name, getattr(cfg, "rated_torque", None) or 9.0)
        self.torque_idle_limit = TORQUE_IDLE_FACTOR * self.rated_torque

    # ── unit conversion (native is MOTOR turns for ODrive, rad for Damiao) ──
    # real2sim's scale: ODrive 2*pi / fleet_config.GEAR_RATIO (1 while unmeasured),
    # Damiao 1. Unit scale only: the measured sign/zero layer (joint_calib.py) is not
    # wired into the service yet (HANDOFF §2).
    def to_native(self, rad: float) -> float:
        return rad / real2sim.rad_per_native(self.name)

    def to_rad(self, native: float) -> float:
        return native * real2sim.rad_per_native(self.name)

    def clamp_native(self, q: float) -> float:
        if self.pos_lo is not None:
            q = max(q, self.pos_lo)
        if self.pos_hi is not None:
            q = min(q, self.pos_hi)
        return q

    # ── normalized state read ─────────────────────────────────────
    def read_state(self) -> dict:
        st = self.joint.get_state() or {}
        pos_native = float(st.get("pos", 0.0))
        vel_native = float(st.get("vel", 0.0))
        torque = float(st.get("torque", st.get("torq", 0.0)))

        # FRESH feedback only. "Has ever reported" (what this used to check) stays
        # true forever after the first frame, which turned a dead joint, bus or
        # polling thread into frozen numbers that looked live and kept the
        # feedback watchdog from ever firing.
        fb_age = self.joint.feedback_age()
        has_fb = fb_age is not None and fb_age < FEEDBACK_FRESH_S

        fault = None
        if self.is_odrive:
            if st.get("axis_error"):
                fault = f"ODrive axis_error=0x{int(st['axis_error']):X}"
        else:
            dm_status = (self.joint.motor.get_states() or {}).get("status")
            if dm_status in DM_FAULT_STATUS:
                fault = f"status '{dm_status}'"

        return {
            "pos_rad": self.to_rad(pos_native),
            "vel_rad": self.to_rad(vel_native),
            "pos_native": pos_native,
            "vel_native": vel_native,
            "torque": torque,
            "fault": fault,
            "has_feedback": has_fb,
            "fb_age": fb_age,
        }


class MotorService:
    """Pure ZMQ control service owning the whole motor fleet."""

    def __init__(self,
                 dm_channel: str = DM_CHANNEL,
                 od_channel: str = OD_CHANNEL,
                 bustype: str = BUSTYPE,
                 rate_hz: float = 200.0,
                 move_duration: float = 0.0,
                 only: Optional[List[str]] = None):
        self.rate_hz = rate_hz
        # 0.0 = track streamed targets directly (policy @ 50 Hz); nonzero adds a
        # smooth ramp per command (useful for hand-typed bench moves).
        self.move_duration = move_duration

        dm_cfgs = [c for c in DM_JOINTS if only is None or c.name in only]
        od_cfgs = [c for c in OD_JOINTS if only is None or c.name in only]
        if only:
            unknown = set(only) - {c.name for c in (*dm_cfgs, *od_cfgs)}
            if unknown:
                raise ValueError(f"--only names not in the fleet: {sorted(unknown)}")
        if not dm_cfgs and not od_cfgs:
            raise ValueError("empty fleet")

        self._estopped = False
        self._estop_reason = ""
        self._enabled = False
        self._enabled_at = 0.0          # time.monotonic() of the last enable
        self._stop = threading.Event()
        # All watchdog/grace timing is on the monotonic clock (immune to NTP steps).
        self._start_time = time.monotonic()
        self._got_cmd = False
        self._last_cmd_time = 0.0
        self._last_cmd_seq: Optional[int] = None

        # ── Damiao: one controller/socket ─────────────────────────
        self.dm_controller: Optional[DaMiaoController] = None
        self.dm_joints: List[DamiaoJoint] = []
        if dm_cfgs:
            self.dm_controller = DaMiaoController(channel=dm_channel, bustype=bustype)
            self.dm_joints = [DamiaoJoint(self.dm_controller, c) for c in dm_cfgs]

        # ── ODrive: a second (filtered) socket + reader thread ─────
        self.od_bus: Optional[can.BusABC] = None
        self.od_joints: List[OdriveCanJoint] = []
        self._od_by_node: Dict[int, OdriveCanJoint] = {}
        self._reader_thread: Optional[threading.Thread] = None
        if od_cfgs:
            od_filters = [{"can_id": c.node_id << 5, "can_mask": 0x7E0} for c in od_cfgs]
            self.od_bus = can.Bus(channel=od_channel, interface=bustype,
                                  can_filters=od_filters)
            for c in od_cfgs:
                j = OdriveCanJoint(self.od_bus, c)
                self.od_joints.append(j)
                self._od_by_node[c.node_id] = j
            # When DM and ODrive share a bus, filter the DM socket so it doesn't
            # mis-ingest ODrive frames (damiao_motor 1.0.6 can't take can_filters
            # in __init__, so apply to its bus afterwards).
            if self.dm_controller is not None and od_channel == dm_channel:
                dm_filters = [{"can_id": c.feedback_id, "can_mask": 0x7FF} for c in dm_cfgs]
                dm_filters.append({"can_id": DM_REGISTER_ID, "can_mask": 0x7FF})
                self.dm_controller.bus.set_filters(dm_filters)
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()

        # Unified joint list + adapters (wire slots come from dodo_ipc.JOINT_ORDER).
        self.joints: List[AnyJoint] = [*self.dm_joints, *self.od_joints]
        self.adapters: Dict[str, _JointAdapter] = {
            j.cfg.name: _JointAdapter(j) for j in self.joints}
        for name in self.adapters:
            if name not in ipc.JOINT_INDEX:
                raise ValueError(f"joint '{name}' not in dodo_ipc.JOINT_ORDER")

        # ── ZMQ endpoints (service side binds; see dodo_ipc docstring) ──
        self.state_pub = ipc.Publisher(ipc.TOPIC_MOTOR_STATES)
        self.cmd_sub = ipc.Subscriber(ipc.TOPIC_MOTOR_COMMANDS, conflate=True)
        self.ctl = ipc.CtlServer()
        self._ctl_handlers = {
            "status": self._ctl_status,
            "enable": self._ctl_enable,
            "disable": self._ctl_disable,
            "calibrate": self._ctl_calibrate,   # normal startup (boot pose)
            "set_zero": self._ctl_set_zero,     # jig/bench only (pose = URDF zero)
            "estop_reset": self._ctl_estop_reset,
        }

        log.info("motor_service up: %d Damiao (%s) + %d ODrive (%s), loop %.0f Hz. "
                 "Motors DISABLED — run tools/dodo_ctl.py enable.",
                 len(self.dm_joints), dm_channel, len(self.od_joints), od_channel, rate_hz)
        for name, a in self.adapters.items():
            lo = f"{a.pos_lo:+.2f}" if a.pos_lo is not None else "-inf"
            hi = f"{a.pos_hi:+.2f}" if a.pos_hi is not None else "+inf"
            unit = "turns" if a.is_odrive else "rad"
            log.info("  %s: pos[%s,%s] %s, |vel|<=%.1f, torque->IDLE @%.1f Nm (0.8 x %.0f)%s",
                     name, lo, hi, unit, a.vel_limit, a.torque_idle_limit,
                     a.rated_torque, "" if a.calibrated else "  [NEEDS calibrate]")

    # ── ODrive reader thread ──────────────────────────────────────
    def _reader_loop(self):
        while not self._stop.is_set():
            msg = self.od_bus.recv(timeout=0.1)
            if msg is None:
                continue
            j = self._od_by_node.get(msg.arbitration_id >> 5)
            if j is not None:
                j.process_frame(msg)

    # ── fleet management ──────────────────────────────────────────
    def _enable_all(self):
        if self.dm_controller is not None:
            self.dm_controller.enable_all()
        for j in self.od_joints:
            j.enable()
        time.sleep(0.1)

    def _disable_all(self):
        if self.dm_controller is not None:
            try:
                self.dm_controller.disable_all()
            except Exception as e:
                log.error("Damiao disable_all failed: %s", e)
        for j in self.od_joints:
            try:
                j.disable()
            except Exception:
                pass

    # ── ctl handlers (called from the main loop via CtlServer.poll_once) ──
    def _ctl_status(self, _args: dict) -> dict:
        joints = {}
        for name, a in self.adapters.items():
            st = a.read_state()
            joints[name] = {
                "type": "odrive" if a.is_odrive else "damiao",
                "pos_rad": round(st["pos_rad"], 4),
                "target_rad": (None if a.target_rad is None or not self._enabled
                               or a.idled else round(a.target_rad, 4)),
                "vel_rad": round(st["vel_rad"], 4),
                "torque_nm": round(st["torque"], 3),
                "has_feedback": st["has_feedback"],
                "feedback_age_s": (None if st["fb_age"] is None
                                   else round(st["fb_age"], 3)),
                "calibrated": a.calibrated,
                "idled": a.idled,
                "fault": st["fault"],
            }
        return {
            "enabled": self._enabled,
            "estopped": self._estopped,
            "estop_reason": self._estop_reason,
            "all_calibrated": all(a.calibrated for a in self.adapters.values()),
            "move_duration": self.move_duration,
            "loop": self.loop.stats() if hasattr(self, "loop") else None,
            "joints": joints,
        }

    def _ctl_enable(self, _args: dict) -> dict:
        """Bumpless, fail-closed enable.

        1. ODrives must already be chatting (cyclic encoder feedback) — refuse
           a blind enable. Preset each ODrive input_pos to the MEASURED shaft
           position while still idle, so closed-loop entry holds in place
           (otherwise the axis would jump to a stale/zero setpoint).
        2. Enable the fleet, then probe the Damiaos with a zero-gain limp frame
           (they only report state once enabled). Silent joint -> disable all,
           refuse. Then hold each Damiao at its measured position.
        3. Joints resting OUTSIDE the safety band (boot pose = folded at the
           stops) are slowly "unparked" to the band edge so the first streamed
           command never causes a step jump.
        """
        if self._estopped:
            raise RuntimeError(f"e-stopped ({self._estop_reason}) — estop_reset first")
        if self._enabled:
            return {"enabled": True, "unparking": {}}

        states = {n: a.read_state() for n, a in self.adapters.items()}
        silent_od = [n for n, a in self.adapters.items()
                     if a.is_odrive and not states[n]["has_feedback"]]
        if silent_od:
            raise RuntimeError(
                f"no encoder feedback from {silent_od} — check power/CAN wiring "
                f"and that odrive_can_setup.py turned on cyclic feedback. "
                f"Refusing to enable blind.")

        # ODrive: preset the setpoint at the measured position BEFORE closing the
        # loop. Joint-level send (unclamped) on purpose: at the boot pose the leg
        # sits outside the safety band, and holding it exactly where it is beats
        # yanking it to the band edge.
        for n, a in self.adapters.items():
            if a.is_odrive:
                a.joint.send_position(states[n]["pos_native"])

        self._enable_all()

        # Damiao: limp probe (zero torque) so they start reporting, then re-read.
        if self.dm_joints:
            for j in self.dm_joints:
                j.send_limp()
            time.sleep(0.05)
            silent_dm = []
            for j in self.dm_joints:
                n = j.cfg.name
                states[n] = self.adapters[n].read_state()
                if not states[n]["has_feedback"]:
                    silent_dm.append(n)
            if silent_dm:
                self._disable_all()
                raise RuntimeError(
                    f"no feedback from Damiao {silent_dm} after limp probe — "
                    f"check power/CAN ids. Fleet disabled again (fail closed).")

        # Hold in place / unpark toward the safety band.
        unparking = {}
        for n, a in self.adapters.items():
            pos = states[n]["pos_native"]
            tgt = a.clamp_native(pos)
            if abs(tgt - pos) > 1e-6:
                a.joint.start_move(tgt, duration=UNPARK_DURATION_S)
                unparking[n] = {"from_rad": round(a.to_rad(pos), 4),
                                "to_rad": round(a.to_rad(tgt), 4),
                                "duration_s": UNPARK_DURATION_S}
            else:
                a.joint.send_position(pos)
            a.target_rad = a.to_rad(tgt)
        self._enabled = True
        self._enabled_at = time.monotonic()   # Damiao silence is counted from here
        # Re-arm the command watchdog for the new session.
        self._got_cmd = False
        if unparking:
            log.info("fleet ENABLED, unparking %s to the safety band over %.1fs",
                     sorted(unparking), UNPARK_DURATION_S)
        else:
            log.info("fleet ENABLED (holding measured positions)")
        return {"enabled": True, "unparking": unparking}

    def _ctl_disable(self, _args: dict) -> dict:
        self._disable_all()
        self._enabled = False
        for a in self.adapters.values():
            a.target_rad = None
        self._got_cmd = False
        log.info("fleet DISABLED")
        return {"enabled": False}

    def _ctl_calibrate(self, args: dict) -> dict:
        """Boot-pose calibration — THE normal startup step (replaces homing).

        Precondition: the robot was POWERED ON with the ODrive joints folded
        against their mechanical stops (BOOT_POSE_RAD). Maps each ODrive's
        turns-since-boot onto that known angle. Damiao zeros are hardware-
        persistent and are not touched.
        """
        pose = args.get("pose", "boot")
        if pose != "boot":
            raise RuntimeError(f"unknown pose '{pose}' — use 'boot' "
                               f"(or set_zero for the jig pose)")
        if self._enabled:
            raise RuntimeError("refusing calibrate while enabled — disable first")
        assumed = {}
        for n, a in self.adapters.items():
            if not a.is_odrive:
                continue
            if n not in BOOT_POSE_RAD:
                raise RuntimeError(f"no BOOT_POSE_RAD entry for '{n}' — add it")
            if not a.read_state()["has_feedback"]:
                raise RuntimeError(f"{n}: no encoder feedback — check power/CAN "
                                   f"(cannot calibrate a silent joint)")
            a.joint.set_zero(current=a.to_native(BOOT_POSE_RAD[n]))
            a.calibrated = True
            assumed[n] = BOOT_POSE_RAD[n]
        log.info("calibrate(boot): mapped %s onto the boot pose %s — VERIFY with "
                 "status before enabling!", sorted(assumed), assumed)
        return {"pose": "boot", "assumed_rad": assumed,
                "note": "assumes the legs were folded at the stops at power-on; "
                        "Damiao zeros untouched (hardware). Verify with status."}

    def _ctl_set_zero(self, _args: dict) -> dict:
        """Jig calibration (bench only): the robot is physically posed at the
        URDF zero, and the CURRENT position becomes 0 for every joint that is
        reporting. Normal startup uses `calibrate` (boot pose) instead."""
        if self._enabled:
            raise RuntimeError("refusing set_zero while enabled — disable first, "
                               "pose the robot in the jig (URDF zero), then set_zero")
        zeroed, skipped = [], []
        for n, a in self.adapters.items():
            st = a.read_state()
            if not st["has_feedback"]:
                if a.is_odrive:
                    raise RuntimeError(f"{n}: no encoder feedback — check power/CAN "
                                       f"(cannot zero a silent joint)")
                # Damiao only reports while enabled; its zero is hardware-
                # persistent anyway — keep it.
                skipped.append(n)
                continue
            a.joint.set_zero()
            a.calibrated = True
            zeroed.append(n)
        log.info("set_zero: current pose is now 0 rad for %s%s", sorted(zeroed),
                 f" (kept hardware zero for silent {sorted(skipped)})" if skipped else "")
        return {"calibrated": sorted(zeroed), "hardware_zero_kept": sorted(skipped)}

    def _ctl_estop_reset(self, _args: dict) -> dict:
        if not self._estopped:
            return {"was_estopped": False}
        self._estopped = False
        self._estop_reason = ""
        self._got_cmd = False
        self._start_time = time.monotonic()     # fresh startup grace
        for a in self.adapters.values():
            a.idled = False
        # A joint that is still silent trips the watchdog again after the grace:
        # resetting doesn't make a dead bus alive.
        log.warning("e-stop RESET — motors remain disabled; 'enable' to resume")
        return {"was_estopped": True, "enabled": self._enabled}

    # ── command path ──────────────────────────────────────────────
    def _apply_command(self, cmd: ipc.MotorCommands):
        if self._estopped:
            return
        if not self._enabled:
            if self._last_cmd_seq is None or cmd.seq != self._last_cmd_seq:
                log.warning("motor_commands arriving while DISABLED — ignoring "
                            "(dodo_ctl enable first)")
            self._last_cmd_seq = cmd.seq
            return
        self._got_cmd = True
        self._last_cmd_time = time.monotonic()
        self._last_cmd_seq = cmd.seq

        for name, a in self.adapters.items():
            if a.idled:
                continue
            target_rad = cmd.target_pos[ipc.JOINT_INDEX[name]]
            if ipc.no_command(target_rad):
                continue
            native = a.clamp_native(a.to_native(float(target_rad)))
            if abs(a.to_rad(native) - float(target_rad)) > 1e-6:
                log.warning("%s: target %+.3f rad clamped to %+.3f rad (safety limit)",
                            a.name, float(target_rad), a.to_rad(native))
            a.joint.start_move(native, duration=self.move_duration)
            a.target_rad = a.to_rad(native)

    # ── state publishing ──────────────────────────────────────────
    def _publish_states(self, states: Dict[str, dict]):
        n = ipc.N_JOINTS
        pos, vel, tau = [0.0] * n, [0.0] * n, [0.0] * n
        jflags = [0] * n
        for name, st in states.items():
            i = ipc.JOINT_INDEX[name]
            a = self.adapters[name]
            pos[i], vel[i], tau[i] = st["pos_rad"], st["vel_rad"], st["torque"]
            f = 0
            if st["has_feedback"]:
                f |= ipc.JF_HAS_FEEDBACK
            if self._enabled and not a.idled and not self._estopped:
                f |= ipc.JF_ENABLED
            if a.idled:
                f |= ipc.JF_IDLED
            if a.calibrated:
                f |= ipc.JF_CALIBRATED
            if st["fault"]:
                f |= ipc.JF_FAULT
            jflags[i] = f
        g = 0
        if self._estopped:
            g |= ipc.GF_ESTOPPED
        if all(a.calibrated for a in self.adapters.values()):
            g |= ipc.GF_ALL_CALIBRATED
        if self._enabled:
            g |= ipc.GF_ENABLED
        self.state_pub.send(ipc.MotorStates(
            pos=pos, vel=vel, torque=tau, joint_flags=jflags, global_flags=g))

    # ── safety ────────────────────────────────────────────────────
    def _safety_check(self, states: Dict[str, dict]):
        if time.monotonic() - self._start_time < STARTUP_GRACE_S:
            return
        for name, st in states.items():
            if not st["has_feedback"]:
                continue        # stale numbers aren't judged; _watchdog_check owns silence
            a = self.adapters[name]
            if a.idled:
                continue
            if st["fault"]:
                self._estop(f"{name} reported fault: {st['fault']}")
                return
            if a.vel_limit is not None and abs(st["vel_native"]) > a.vel_limit:
                self._estop(f"{name} velocity {abs(st['vel_native']):.2f} > "
                            f"{a.vel_limit:.2f} ({'turns/s' if a.is_odrive else 'rad/s'})")
                return
            if a.pos_hard_lo_rad is not None:
                p = st["pos_rad"]
                if (p < a.pos_hard_lo_rad - POS_OVERTRAVEL_MARGIN or
                        p > a.pos_hard_hi_rad + POS_OVERTRAVEL_MARGIN):
                    self._estop(f"{name} position {p:+.3f} rad past mechanical limit "
                                f"[{a.pos_hard_lo_rad:+.3f},{a.pos_hard_hi_rad:+.3f}]")
                    return
            if abs(st["torque"]) > a.torque_idle_limit:
                self._idle_joint(name, f"torque {abs(st['torque']):.2f} > "
                                       f"{a.torque_idle_limit:.2f} Nm "
                                       f"(0.8 x {a.rated_torque:.0f} Nm rated)")

    def _silent_for(self, a: _JointAdapter, now: float) -> Optional[float]:
        """How long a joint has been silent while it SHOULD be talking, or None
        if it isn't expected to talk right now."""
        age = a.joint.feedback_age()
        if a.is_odrive:
            # ODrives stream cyclic feedback whether enabled or not, so once heard
            # they must keep talking. Never heard = not expected (and `enable` /
            # `calibrate` refuse a joint without fresh feedback).
            return age
        # A Damiao only answers frames, which the main loop sends every tick
        # while enabled. Disabled, silence is normal. Count from the enable so
        # a reply from before it doesn't make a fresh session look silent.
        if not self._enabled:
            return None
        since_enable = now - self._enabled_at
        return since_enable if age is None else min(age, since_enable)

    def _watchdog_check(self):
        now = time.monotonic()
        if now - self._start_time < STARTUP_GRACE_S or self._estopped:
            return
        # Command watchdog: only while enabled and once the stream has started.
        if self._enabled and self._got_cmd and (now - self._last_cmd_time) > CMD_WATCHDOG_S:
            self._estop(f"no motor_commands for {now - self._last_cmd_time:.2f}s "
                        f"(> {CMD_WATCHDOG_S}s watchdog)")
            return
        for name, a in self.adapters.items():
            if a.idled:
                continue
            silent = self._silent_for(a, now)
            if silent is None or silent <= FB_WATCHDOG_S:
                continue
            note = ""
            if a.is_odrive and a.calibrated:
                # We can't tell a bus glitch from a reboot, and a rebooted ODrive
                # re-zeroes its encoder: the boot-pose mapping is void either way.
                a.calibrated = False
                note = (" — its calibration is void (a rebooted ODrive re-zeroes its "
                        "encoder): fold it to its stop and ctl `calibrate` again")
            self._estop(f"{name} feedback silent for {silent:.2f}s "
                        f"(> {FB_WATCHDOG_S}s watchdog){note}")
            return

    def _idle_joint(self, name: str, reason: str):
        a = self.adapters[name]
        if a.idled:
            return
        a.idled = True
        log.error("IDLE %s: %s. Disabling this motor.", name, reason)
        try:
            a.joint.disable()
        except Exception as e:
            log.error("%s: disable failed: %s", name, e)

    def _estop(self, reason: str):
        if self._estopped:
            return
        self._estopped = True
        self._estop_reason = reason
        self._enabled = False
        log.error("E-STOP: %s. Disabling all motors.", reason)
        self._disable_all()

    # ── main loop ─────────────────────────────────────────────────
    def run(self):
        self.loop = ipc.RateLoop(self.rate_hz, name="motor_service")
        last_stats = time.monotonic()
        try:
            while not self._stop.is_set():
                # 1) rare ops (never blocks)
                self.ctl.poll_once(self._ctl_handlers)
                # 2) newest command only
                cmd = self.cmd_sub.latest()
                if cmd is not None:
                    self._apply_command(cmd)
                # 3) drive ramps / hold targets (only when actively controlled)
                if self._enabled and not self._estopped:
                    for joint in self.joints:
                        if self.adapters[joint.cfg.name].idled:
                            continue
                        try:
                            joint.update()
                        except Exception as e:
                            log.error("%s: update failed: %s", joint.cfg.name, e)
                # 4) read + publish + protect (also while disabled/e-stopped:
                #    consumers must SEE the state, not lose the stream)
                states = {name: a.read_state() for name, a in self.adapters.items()}
                self._publish_states(states)
                self._safety_check(states)
                self._watchdog_check()

                if time.monotonic() - last_stats > 10.0:
                    log.debug(self.loop.stats_str())
                    last_stats = time.monotonic()
                self.loop.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            log.info(self.loop.stats_str())
            self.shutdown()

    # ── shutdown ──────────────────────────────────────────────────
    def shutdown(self):
        log.info("shutting down: disabling motors.")
        self._stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
        self._disable_all()
        if self.dm_controller is not None:
            try:
                self.dm_controller.shutdown()
            except Exception as e:
                log.error("Damiao shutdown failed: %s", e)
        if self.od_bus is not None:
            try:
                self.od_bus.shutdown()
            except Exception:
                pass
        for s in (self.state_pub, self.cmd_sub, self.ctl):
            try:
                s.close()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dm-channel", default=DM_CHANNEL)
    ap.add_argument("--od-channel", default=OD_CHANNEL,
                    help="same as --dm-channel to run both motor types on one adapter")
    ap.add_argument("--bustype", default=BUSTYPE)
    ap.add_argument("--rate", type=float, default=200.0, help="control loop Hz")
    ap.add_argument("--move-duration", type=float, default=0.0,
                    help="ramp time per received command; 0 = track directly (policy)")
    ap.add_argument("--only", default=None,
                    help="comma-separated joint names for a bench subset, "
                         "e.g. hip_left,lower_leg_left")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s %(name)s %(levelname)s: %(message)s")
    only = [s.strip() for s in args.only.split(",")] if args.only else None
    svc = MotorService(dm_channel=args.dm_channel, od_channel=args.od_channel,
                       bustype=args.bustype, rate_hz=args.rate,
                       move_duration=args.move_duration, only=only)
    svc.run()


if __name__ == "__main__":
    main()
