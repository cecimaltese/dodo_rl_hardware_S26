"""
motor_service.py — the dodo motor fleet as a pure-ZMQ service.

Owns the WHOLE motor fleet (Damiao + ODrive) behind one motor-type-agnostic
interface:

    PUB  motor_states    @ rate_hz (200 Hz)  pos/vel/torque [rad, rad/s, Nm]
                                             + per-joint & global flags
    SUB  motor_commands  latest-value        absolute position targets [rad],
                                             NaN = leave that joint alone
    REP  ctl             on demand           status / calibrate / enable /
                                             disable / gains / estop_reset

This is a *pure* control service: it knows nothing about the policy (joint
order on the wire is dodo_ipc.JOINT_ORDER; default pose / action scaling live
in the Env layer). The wire is in JOINT units, the sim's: radians as in the
URDF, rad/s, joint Nm. Every joint goes through its MEASURED calibration
(real2sim.JointCal from joint_calib.py: sign, zero, gear ratio) into its motor's
native units (Damiao rad at the output, ODrive MOTOR turns), and nothing native
ever reaches the wire.

Design choices (all deliberate):
  * NO auto-enable at startup. Bring-up is explicit via tools/dodo_ctl.py:
        status → calibrate → enable            (normal startup, see below)
  * States keep publishing after an e-stop (with the ESTOPPED flag set) so
    consumers see WHY the robot stopped instead of a silent stream.
  * Per-joint `calibrated` flag on the wire: a Damiao keeps its measured zero
    (joint_calib.py) across power cycles → calibrated from boot; an ODrive's
    position is turns-since-boot → calibrated by ctl `calibrate` (below).
  * `enable` is BUMPLESS and fail-closed: it refuses unless every joint is
    calibrated, has measured stops and talks; sets the PD gains (Damiao: in
    every MIT frame; ODrive: over CAN); presets every setpoint to the measured
    position (so nothing jumps when the loops close), then slowly "unparks"
    joints outside the command band to its edge.
  * PD gains: fleet_config.JOINT_KP/JOINT_KD (the PD stand's, = dodo_assets'
    scene.xml), times --gain-scale / ctl `gains` (kp x s, kd x sqrt s).

ODrive drive calibration, once per power-up: the drives forget their motor/encoder
calibration when switched off and REFUSE closed loop until it's redone (heartbeat
procedure_result NOT_CALIBRATED). ctl `odrive_calibrate` runs it over CAN, legs
hanging free: each knee/ankle motor beeps, then turns a fraction of a turn and back
(measured 2026-11-05: ankles 8 deg; the right knee 16 deg, ending ~7 deg off, as its
encoder also searches its index). It voids the joint calibration below; redo that
after, legs straightened by hand. ~16 s.
`enable` checks every ODrive actually reaches closed loop, and an ODrive that leaves
it while enabled is an e-stop.

Joint calibration per session (no homing routine, on purpose: driving the ODrive
joints into their stops under power could damage the 3D-printed parts/belts):
  * put the knees and ankles in a reference pose (fleet_config.REFERENCE_POSES:
    `straight` = legs hanging straight on the gantry, as in the calibration's
    zero pose), then ctl `calibrate pose=straight` maps each ODrive's current
    reading onto that angle. The Damiaos keep their measured zeros.
  * `calibrate` is only accepted while DISABLED. Check the result with
    visualize_hardware.py BEFORE starting the service, or `status` after.

Safety layer:
  * position commands clamped STOP_MARGIN_RAD inside the measured stops, in
    joint radians (real2sim.command_band)
  * |joint torque| > 0.8 × rated  → that ONE motor to IDLE (professor's spec;
    ODrive torque converted to the joint, fleet_config.ODRIVE_TORQUE_IS_MOTOR_SIDE)
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

Fleet, buses, gains and reference poses come from fleet_config.py, the measured
calibration and stops from joint_calib.py (through real2sim); nothing about the
robot is configured here.

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
    python3 motor_service.py --gain-scale 0.3   # soft PD for first tests
    python3 motor_service.py --only hip_left,lower_leg_left   # bench subset
"""

from __future__ import annotations

import argparse
import dataclasses
import logging
import math
import threading
import time
from typing import Dict, List, Optional, Tuple, Union

import can
from damiao_motor import DaMiaoController

import dodo_ipc as ipc
import real2sim
from damiao_joint import DamiaoJoint
from odrive_can_joint import (AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE,
                              PROCEDURE_NOT_CALIBRATED,
                              PROCEDURE_PHASE_RESISTANCE_OUT_OF_RANGE, OdriveCanJoint,
                              axis_state_name, error_names, procedure_result_name)
from fleet_config import (DM_JOINTS, OD_JOINTS, DM_CHANNEL, OD_CHANNEL, BUSTYPE,
                          RATED_TORQUE_NM, JOINT_VEL_LIMIT_RAD, REFERENCE_POSES,
                          FEEDBACK_FRESH_S, JOINT_KP, JOINT_KD, joint_group,
                          ODRIVE_TORQUE_IS_MOTOR_SIDE)

log = logging.getLogger("motor_service")

DM_REGISTER_ID = 0x7FF  # Damiao register read/write reply arbitration id

# --- Safety factors / thresholds -----------------------------------------------
SAFETY_FACTOR = 0.9          # margin applied to the velocity limit
TORQUE_IDLE_FACTOR = 0.8     # |torque| > 0.8 * rated  ->  that motor goes to IDLE
CMD_WATCHDOG_S = 0.5         # once commands stream (while enabled), a gap -> e-stop
FB_WATCHDOG_S = 0.5          # a joint that should be reporting goes silent -> e-stop
POS_OVERTRAVEL_MARGIN = 0.10 # rad past a measured stop -> e-stop
STARTUP_GRACE_S = 1.0        # ignore safety trips for the first second
UNPARK_DURATION_S = 1.5      # slow move from outside the command band to its edge at enable
MAX_GAIN_SCALE = 1.0         # ctl `gains`: never stiffer than fleet_config's
CLOSED_LOOP_TIMEOUT_S = 1.0  # enable: every ODrive must REPORT closed loop by then
                             # (heartbeats are 10-100 Hz) or the fleet is disabled again

DM_FAULT_STATUS = {
    "OVER_VOLTAGE", "UNDER_VOLTAGE", "OVER_CURRENT",
    "MOS_OVER_TEMP", "ROTOR_OVER_TEMP", "LOST_COMM", "OVERLOAD",
}

AnyJoint = Union[DamiaoJoint, OdriveCanJoint]


# ---------------------------------------------------------------------------
# Per-joint adapter: unifies units (rad <-> native) + safety limits + state.
# ---------------------------------------------------------------------------
class _JointAdapter:
    """One joint: JOINT units outside (sim radians, rad/s, joint Nm), the motor's
    native units inside (Damiao rad at the output, ODrive MOTOR turns), through its
    measured calibration (real2sim.JointCal: q = sign * (raw - zero) * scale).

    The drivers' own software zero stays 0: all mapping happens here, so the
    native values are the motors' raw readings."""

    def __init__(self, joint: AnyJoint, cal: real2sim.JointCal,
                 stops: Optional[Tuple[float, float]],
                 band: Optional[Tuple[float, float]]):
        self.joint = joint
        self.name = joint.cfg.name
        self.is_odrive = isinstance(joint, OdriveCanJoint)
        self.idled = False              # set True once this joint is IDLEd by safety
        # The setpoint this joint is being held at / moving to, joint rad (None while
        # disabled). Reported by `status` so a client can continue from the targets
        # instead of the measured angles: under load those sag below the targets, and
        # re-targeting there drops the preload (a standing robot tips over).
        self.target_rad: Optional[float] = None
        self.cal = cal
        # Sign and mapping measured (joint_calib.py)? Without, nothing is commanded.
        self.measured = cal.measured
        # A Damiao's measured zero holds across power cycles; an ODrive's doesn't:
        # its zero comes from ctl `calibrate` every session.
        self.calibrated = cal.measured and not self.is_odrive
        self.stops = stops              # measured hard stops [rad], None = unmeasured
        self.band = band                # where commands may go [rad]
        self.vel_limit_rad = SAFETY_FACTOR * JOINT_VEL_LIMIT_RAD
        self.rated_torque = RATED_TORQUE_NM.get(
            self.name, getattr(joint.cfg, "rated_torque", None) or 9.0)
        self.torque_idle_limit = TORQUE_IDLE_FACTOR * self.rated_torque
        # Reported torque -> joint Nm: the sign, and an ODrive's motor-side Nm through
        # the gearing.
        self.torque_scale = cal.sign * (
            real2sim.gear_ratio(self.name)
            if self.is_odrive and ODRIVE_TORQUE_IS_MOTOR_SIDE else 1.0)
        self.kp = JOINT_KP[joint_group(self.name)]   # joint-side, before gain_scale
        self.kd = JOINT_KD[joint_group(self.name)]

    # ── units: joint rad <-> the motor's raw native reading ──
    def to_native(self, rad: float) -> float:
        return self.cal.to_raw(rad)

    def to_rad(self, native: float) -> float:
        return self.cal.to_sim(native)

    def clamp_rad(self, q: float) -> float:
        lo, hi = self.band
        return min(max(q, lo), hi)

    def calibrate_at(self, raw: float, q_ref: float) -> None:
        """The joint is physically at q_ref [rad] while its motor reads raw."""
        self.cal = self.cal.zeroed_at(raw, q_ref)
        self.calibrated = True

    def apply_gains(self, scale: float) -> Tuple[float, float]:
        """Set this joint's PD to (kp x scale, kd x sqrt(scale)); returns them. Damiao:
        the gains of its own config copy, sent in every MIT frame. ODrive: over CAN
        (applies at once, also in closed loop)."""
        kp, kd = self.kp * scale, self.kd * math.sqrt(scale)
        if self.is_odrive:
            self.joint.set_gains(*real2sim.odrive_gains(self.name, kp, kd))
        else:
            self.joint.cfg.kp, self.joint.cfg.kd = kp, kd
        return kp, kd

    # ── normalized state read ─────────────────────────────────────
    def read_state(self) -> dict:
        st = self.joint.get_state() or {}
        pos_native = float(st.get("pos", 0.0))
        vel_native = float(st.get("vel", 0.0))
        torque_native = float(st.get("torque", st.get("torq", 0.0)))

        # FRESH feedback only. "Has ever reported" (what this used to check) stays
        # true forever after the first frame, which turned a dead joint, bus or
        # polling thread into frozen numbers that looked live and kept the
        # feedback watchdog from ever firing.
        fb_age = self.joint.feedback_age()
        has_fb = fb_age is not None and fb_age < FEEDBACK_FRESH_S

        fault = None
        if self.is_odrive:
            if st.get("axis_error"):
                err = int(st["axis_error"])
                fault = f"ODrive {error_names(err)} (axis_error 0x{err:X})"
                if self.joint.procedure_result == PROCEDURE_PHASE_RESISTANCE_OUT_OF_RANGE:
                    fault += (": its calibration measured no usable phase resistance, "
                              "i.e. no current flows: check this motor's phase wires / "
                              "connector")
                elif self.joint.procedure_result:
                    fault += f" after {procedure_result_name(self.joint.procedure_result)}"
        else:
            dm_status = (self.joint.motor.get_states() or {}).get("status")
            if dm_status in DM_FAULT_STATUS:
                fault = f"status '{dm_status}'"

        return {
            "pos_rad": self.to_rad(pos_native),
            "vel_rad": self.cal.vel_to_sim(vel_native),
            "pos_native": pos_native,
            "vel_native": vel_native,
            "torque": torque_native * self.torque_scale,     # joint Nm
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
                 only: Optional[List[str]] = None,
                 calib_path=real2sim.CALIB_PATH,
                 gain_scale: float = 1.0):
        self.rate_hz = rate_hz
        if not 0.0 < gain_scale <= MAX_GAIN_SCALE:
            raise ValueError(f"gain_scale {gain_scale}: must be in (0, {MAX_GAIN_SCALE}]")
        self.gain_scale = gain_scale
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
        self._fault_logged: Dict[str, str] = {}   # disabled-time faults, logged once

        # ── Damiao: one controller/socket ─────────────────────────
        self.dm_controller: Optional[DaMiaoController] = None
        self.dm_joints: List[DamiaoJoint] = []
        if dm_cfgs:
            self.dm_controller = DaMiaoController(channel=dm_channel, bustype=bustype)
            # Own config copies: apply_gains() changes kp/kd, and DM_JOINTS are the
            # shared fleet_config objects.
            self.dm_joints = [DamiaoJoint(self.dm_controller, dataclasses.replace(c))
                              for c in dm_cfgs]

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

        # Unified joint list + adapters (wire slots come from dodo_ipc.JOINT_ORDER),
        # each through its measured calibration.
        self.joints: List[AnyJoint] = [*self.dm_joints, *self.od_joints]
        cals, warnings = real2sim.load(calib_path)
        for w in warnings:
            if "zeroes its encoder at power-up" not in w:    # ODrive zeros: not used
                log.warning("calibration: %s", w)
        self.adapters: Dict[str, _JointAdapter] = {}
        for j in self.joints:
            name = j.cfg.name
            if name not in ipc.JOINT_INDEX:
                raise ValueError(f"joint '{name}' not in dodo_ipc.JOINT_ORDER")
            cal = cals[name]
            if cal.motor != name:
                raise ValueError(f"joint_calib.py: {name} is driven by motor "
                                 f"'{cal.motor}', but fleet_config names that motor "
                                 f"'{name}'. Fix the names in fleet_config.py.")
            if real2sim.is_odrive(name):
                cal = dataclasses.replace(cal, zero=0.0)    # set by ctl `calibrate`
            self.adapters[name] = _JointAdapter(
                j, cal, real2sim.stops(calib_path).get(name),
                real2sim.command_band(name, calib_path))

        # ── ZMQ endpoints (service side binds; see dodo_ipc docstring) ──
        self.state_pub = ipc.Publisher(ipc.TOPIC_MOTOR_STATES)
        self.cmd_sub = ipc.Subscriber(ipc.TOPIC_MOTOR_COMMANDS, conflate=True)
        self.ctl = ipc.CtlServer()
        self._ctl_handlers = {
            "status": self._ctl_status,
            "enable": self._ctl_enable,
            "disable": self._ctl_disable,
            "calibrate": self._ctl_calibrate,   # every session: pose=<reference pose>
            "set_zero": self._ctl_set_zero,     # = calibrate pose=straight
            "gains": self._ctl_gains,
            "odrive_calibrate": self._ctl_odrive_calibrate,
            "estop_reset": self._ctl_estop_reset,
        }

        log.info("motor_service up: %d Damiao (%s) + %d ODrive (%s), loop %.0f Hz, "
                 "gain scale %.2f. Motors DISABLED — calibrate, then enable.",
                 len(self.dm_joints), dm_channel, len(self.od_joints), od_channel,
                 rate_hz, gain_scale)
        for name, a in self.adapters.items():
            band = (f"[{a.band[0]:+.2f},{a.band[1]:+.2f}] rad" if a.band
                    else "NO MEASURED STOPS (enable refused)")
            log.info("  %-15s sign %+d, commands %s, |vel|<=%.1f rad/s, kp %.0f kd %.2f, "
                     "torque->IDLE @%.1f Nm%s", name, a.cal.sign, band, a.vel_limit_rad,
                     a.kp * gain_scale, a.kd * math.sqrt(gain_scale), a.torque_idle_limit,
                     "" if a.measured else "  [NOT CALIBRATED: joint_calib.py]")
            if a.measured and not a.calibrated:
                log.info("  %-15s needs ctl `calibrate` (ODrive zero is per session)", "")

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
            target = (None if a.target_rad is None or not self._enabled or a.idled
                      else a.target_rad)
            joints[name] = {
                "type": "odrive" if a.is_odrive else "damiao",
                "pos_rad": round(st["pos_rad"], 4),
                "target_rad": None if target is None else round(target, 4),
                # target - pos: with torque_nm, the stiffness actually acting (~kp)
                "err_rad": None if target is None else round(target - st["pos_rad"], 4),
                "vel_rad": round(st["vel_rad"], 4),
                "torque_nm": round(st["torque"], 3),
                "kp": round(a.kp * self.gain_scale, 2),
                "kd": round(a.kd * math.sqrt(self.gain_scale), 3),
                "band_rad": a.band and [round(v, 3) for v in a.band],
                **({"drive": a.joint.drive_report()} if a.is_odrive else {}),
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
            "gain_scale": self.gain_scale,
            "reference_poses": sorted(REFERENCE_POSES),
            "move_duration": self.move_duration,
            "loop": self.loop.stats() if hasattr(self, "loop") else None,
            "joints": joints,
        }

    def _ctl_enable(self, _args: dict) -> dict:
        """Bumpless, fail-closed enable.

        0. Every joint must be calibrated (measured sign; ODrive: `calibrate` this
           session) and have measured stops: otherwise its angles, clamps and
           commands mean nothing.
        1. ODrives must already be chatting (cyclic encoder feedback) — refuse
           a blind enable. Set their PD gains over CAN, then preset each input_pos
           to the MEASURED shaft position while still idle, so closed-loop entry
           holds in place (otherwise the axis would jump to a stale/zero setpoint).
        2. Enable the fleet, then probe the Damiaos with a zero-gain limp frame
           (they only report state once enabled). Silent joint -> disable all,
           refuse. Then hold each Damiao at its measured position.
        3. Joints resting OUTSIDE the command band (e.g. at a stop) are slowly
           "unparked" to the band edge so the first streamed command never causes
           a step jump.
        """
        if self._estopped:
            raise RuntimeError(f"e-stopped ({self._estop_reason}) — estop_reset first")
        if self._enabled:
            return {"enabled": True, "unparking": {}}
        not_ready = {n: ("sign/zero not measured (joint_calib.py)" if not a.measured
                         else "no ctl `calibrate` this session" if not a.calibrated
                         else "no measured stops (calibrate_joints.py --limits)")
                     for n, a in self.adapters.items()
                     if not (a.measured and a.calibrated and a.band)}
        if not_ready:
            raise RuntimeError(f"refusing to enable: {not_ready}")
        busy = self._busy_drives()
        if busy:
            raise RuntimeError(f"refusing to enable: drives still busy {busy}")

        states = {n: a.read_state() for n, a in self.adapters.items()}
        faults = {n: st["fault"] for n, st in states.items() if st["fault"]}
        if faults:
            raise RuntimeError(f"refusing to enable, drive faults: {faults}. "
                               f"(`odrive_calibrate` clears and retries an ODrive; or "
                               f"leave the joint out: motor_service --only ...)")
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
            a.apply_gains(self.gain_scale)
            if a.is_odrive:
                a.joint.send_position(states[n]["pos_native"])

        self._enable_all()

        # ODrive: closed loop is a REQUEST the drive may refuse (NOT_CALIBRATED after
        # a power cycle) without any error bit. Wait for every heartbeat to say so.
        deadline = time.monotonic() + CLOSED_LOOP_TIMEOUT_S
        while True:
            refused = [a for a in self.adapters.values()
                       if a.is_odrive and a.joint.axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL]
            if not refused or time.monotonic() > deadline:
                break
            time.sleep(0.02)
        if refused:
            self._disable_all()
            why = {a.name: f"{axis_state_name(a.joint.axis_state)}, "
                           f"{procedure_result_name(a.joint.procedure_result)}"
                   for a in refused}
            hint = ""
            if any(a.joint.procedure_result == PROCEDURE_NOT_CALIBRATED for a in refused):
                hint = (" NOT_CALIBRATED = the drive's motor/encoder calibration is lost "
                        "at every power-up: legs hanging free, ctl `odrive_calibrate`, "
                        "then `calibrate pose=straight` again.")
            raise RuntimeError(f"ODrive(s) didn't enter closed loop: {why}. Fleet "
                               f"disabled again (fail closed).{hint}")

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

        # Hold in place / unpark toward the command band.
        unparking = {}
        for n, a in self.adapters.items():
            q = states[n]["pos_rad"]
            tgt = a.clamp_rad(q)
            if abs(tgt - q) > 1e-6:
                a.joint.start_move(a.to_native(tgt), duration=UNPARK_DURATION_S)
                unparking[n] = {"from_rad": round(q, 4), "to_rad": round(tgt, 4),
                                "duration_s": UNPARK_DURATION_S}
            else:
                a.joint.send_position(states[n]["pos_native"])
            a.target_rad = tgt
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
        """Per-session calibration of the ODrive joints (knees, ankles): they are
        physically in reference pose `pose` (fleet_config.REFERENCE_POSES) right now,
        so each one's current reading becomes that angle. The Damiaos keep their
        measured zeros (joint_calib.py). Only while DISABLED; no default pose, so
        nobody calibrates against a pose the robot isn't in."""
        pose = args.get("pose")
        if pose not in REFERENCE_POSES:
            raise RuntimeError(f"say which pose the knees and ankles are in: "
                               f"pose={'|'.join(sorted(REFERENCE_POSES))} (got {pose!r})")
        if self._enabled:
            raise RuntimeError("refusing calibrate while enabled — disable first")
        ref = REFERENCE_POSES[pose]
        todo = {n: a for n, a in self.adapters.items() if a.is_odrive}
        missing = sorted(set(todo) - set(ref))
        if missing:
            raise RuntimeError(f"reference pose '{pose}' has no angle for {missing}")
        busy = self._busy_drives()
        if busy:
            raise RuntimeError(f"drives still busy {busy}: wait for odrive_calibrate to "
                               f"finish (status), then straighten the legs")
        unmeasured = sorted(n for n, a in todo.items() if not a.measured)
        if unmeasured:
            raise RuntimeError(f"{unmeasured}: sign not measured (joint_calib.py); "
                               f"calibrate_joints.py first")
        states = {n: a.read_state() for n, a in todo.items()}
        silent = sorted(n for n, st in states.items() if not st["has_feedback"])
        if silent:
            raise RuntimeError(f"{silent}: no encoder feedback — check power/CAN "
                               f"(cannot calibrate a silent joint)")
        for n, a in todo.items():
            a.calibrate_at(states[n]["pos_native"], ref[n])
        log.info("calibrate(%s): %s now read %s rad — VERIFY (visualize_hardware.py "
                 "before, `status` after) before enabling!", pose, sorted(todo),
                 {n: ref[n] for n in sorted(todo)})
        return {"pose": pose, "assumed_rad": {n: ref[n] for n in sorted(todo)},
                "note": "the knees/ankles must BE in this pose now. Damiao zeros "
                        "untouched (measured, persistent)."}

    def _ctl_set_zero(self, _args: dict) -> dict:
        """Kept for old scripts: = calibrate pose=straight (legs hanging straight).
        It used to zero every joint, Damiaos included, at whatever pose the robot was
        in; their measured zeros are better than a by-eye pose."""
        return self._ctl_calibrate({"pose": "straight"})

    def _ctl_gains(self, args: dict) -> dict:
        """scale=<s>: the PD at s x fleet_config's kp (kd x sqrt s, same damping
        ratio), 0 < s <= MAX_GAIN_SCALE. Applies at once (also while enabled) and at
        every later enable."""
        if "scale" in args:
            s = float(args["scale"])
            if not 0.0 < s <= MAX_GAIN_SCALE:
                raise RuntimeError(f"scale {s}: must be in (0, {MAX_GAIN_SCALE}]")
            self.gain_scale = s
            if self._enabled:
                for a in self.adapters.values():
                    if not a.idled:
                        a.apply_gains(s)
            log.info("gain scale %.2f%s", s, " (applied)" if self._enabled
                     else " (applies at enable)")
        return {"gain_scale": self.gain_scale,
                "gains": {n: {"kp": round(a.kp * self.gain_scale, 2),
                              "kd": round(a.kd * math.sqrt(self.gain_scale), 3)}
                          for n, a in self.adapters.items()}}

    def _busy_drives(self) -> Dict[str, str]:
        """ODrives in a state other than IDLE while the fleet is disabled: calibrating."""
        return {n: axis_state_name(a.joint.axis_state) for n, a in self.adapters.items()
                if a.is_odrive and a.joint.axis_state not in (AXIS_STATE_IDLE,)}

    def _ctl_odrive_calibrate(self, args: dict) -> dict:
        """Run each ODrive's FULL_CALIBRATION_SEQUENCE (joints=[...], default all):
        once per power-up, legs hanging free, while DISABLED. THE MOTORS MOVE: a beep,
        then each knee/ankle turns 8-16 deg and back (the right knee ends ~7 deg off:
        its index search). It clears the drives' latched errors first. ~16 s,
        non-blocking: `status` shows
        each drive's state (back to IDLE when done) and procedure_result (SUCCESS).
        Voids those joints' calibration: `calibrate pose=...` again afterwards."""
        if self._enabled:
            raise RuntimeError("refusing odrive_calibrate while enabled — disable first")
        names = args.get("joints") or [n for n, a in self.adapters.items() if a.is_odrive]
        bad = [n for n in names if n not in self.adapters or not self.adapters[n].is_odrive]
        if bad:
            raise RuntimeError(f"not ODrive joints of this fleet: {bad}")
        silent = [n for n in names if not self.adapters[n].read_state()["has_feedback"]]
        if silent:
            raise RuntimeError(f"{silent}: no feedback — check power/CAN")
        for n in names:
            self.adapters[n].joint.clear_errors()    # a latched error would block it
        time.sleep(0.05)
        for n in names:
            a = self.adapters[n]
            a.joint.request_calibration()
            a.calibrated = False          # the rotor moves; the joint zero is redone after
        log.warning("odrive_calibrate: %s — the knee/ankle motors MOVE now (beep, then "
                    "8-16 deg and back, ~16 s); then straighten the legs and "
                    "calibrate pose=straight again", names)
        return {"started": names,
                "note": "~15 s. Watch status joints.<name>.drive: state back to IDLE "
                        "and procedure_result SUCCESS; then calibrate pose=straight"}

    def _ctl_estop_reset(self, _args: dict) -> dict:
        if not self._estopped:
            return {"was_estopped": False}
        self._estopped = False
        self._estop_reason = ""
        self._got_cmd = False
        self._start_time = time.monotonic()     # fresh startup grace
        for a in self.adapters.values():
            a.idled = False
            if a.is_odrive:
                a.joint.clear_errors()      # else a latched drive error blocks enable
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
            q = a.clamp_rad(float(target_rad))
            if abs(q - float(target_rad)) > 1e-6:
                log.warning("%s: target %+.3f rad clamped to %+.3f rad (measured stops "
                            "- margin)", a.name, float(target_rad), q)
            a.joint.start_move(a.to_native(q), duration=self.move_duration)
            a.target_rad = q

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
                # Enabled: stop everything. Disabled there's nothing to stop, and an
                # e-stop would send IDLE to every drive, cancelling the calibrations
                # of the others (2026-11-05: one bad knee motor did exactly that).
                # Reported instead; `enable` refuses while it lasts.
                if self._enabled:
                    self._estop(f"{name} reported fault: {st['fault']}")
                    return
                if self._fault_logged.get(name) != st["fault"]:
                    log.warning("%s: %s (motors disabled: reported, not an e-stop; "
                                "`enable` refuses while it lasts)", name, st["fault"])
                    self._fault_logged[name] = st["fault"]
                continue
            self._fault_logged.pop(name, None)
            if (self._enabled and a.is_odrive
                    and a.joint.axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL):
                r = a.joint.drive_report()
                self._estop(f"{name} left closed loop ({r['state']}, "
                            f"{r['procedure_result']}, disarm_reason {r['disarm_reason']})")
                return
            if abs(st["vel_rad"]) > a.vel_limit_rad:
                self._estop(f"{name} velocity {abs(st['vel_rad']):.2f} > "
                            f"{a.vel_limit_rad:.2f} rad/s")
                return
            # An uncalibrated ODrive's angle means nothing yet: not judged.
            if a.stops is not None and a.calibrated:
                p = st["pos_rad"]
                lo, hi = a.stops
                if p < lo - POS_OVERTRAVEL_MARGIN or p > hi + POS_OVERTRAVEL_MARGIN:
                    self._estop(f"{name} position {p:+.3f} rad past its measured stops "
                                f"[{lo:+.3f},{hi:+.3f}] (+{POS_OVERTRAVEL_MARGIN} rad)")
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
                        "encoder): reference pose, then ctl `calibrate` again")
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
    ap.add_argument("--gain-scale", type=float, default=1.0,
                    help="PD at this fraction of fleet_config's kp (kd x sqrt), "
                         f"(0, {MAX_GAIN_SCALE}]; ctl `gains` changes it live")
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
                       move_duration=args.move_duration, only=only,
                       gain_scale=args.gain_scale)
    svc.run()


if __name__ == "__main__":
    main()
