"""
odrive_can_joint.py — control a single ODrive axis over CAN using the "CAN Simple"
protocol (raw python-can), exposing the SAME joint interface as DamiaoJoint and
OdriveJoint so all motors share one control loop.

This is the RUNTIME path for the ODrive. The position/velocity gains ARE settable
here (Set_Pos_Gain / Set_Vel_Gains: RAM only, lost at power-off; motor_service sets
them at every enable). Everything else (limits, encoder, CAN ids, cyclic messages)
is configured once over USB with odrive_can_setup.py.

CAN Simple basics
-----------------
  arbitration id = (node_id << 5) | cmd_id
A frame "belongs" to a node if (arb_id >> 5) == node_id; the low 5 bits are the
command id. The MotorController routes received frames to the right joint by
node id, then each joint decodes by cmd id in process_frame().

Command ids used (see HANDOFF / ros_odrive cmd table):
  TX:  0x07 Set_Axis_State, 0x0B Set_Controller_Mode, 0x0C Set_Input_Pos,
       0x1A Set_Pos_Gain, 0x1B Set_Vel_Gains (firmware 0.5 and 0.6 alike)
  RX:  0x01 Heartbeat, 0x03 Get_Error, 0x09 Get_Encoder_Estimates, 0x1C Get_Torques

Every power-up the drives lose their motor/encoder calibration (incremental encoders:
positions start at 0): Set_Axis_State(CLOSED_LOOP) is then REFUSED, the axis stays
IDLE and the heartbeat's procedure_result says NOT_CALIBRATED (14), with axis_error 0.
request_calibration() runs the full calibration sequence (motor R/L: a beep; encoder
offset: the rotor turns a fraction of a turn and back), after which closed loop works
until the next power-off.

Units: ODrive positions are in *turns* (revolutions). 1 turn = 2*pi rad.
"""

import struct
import time
from dataclasses import dataclass
from typing import Optional

import can

# ── CAN Simple command ids ────────────────────────────────────────
CMD_HEARTBEAT = 0x01
CMD_GET_ERROR = 0x03
CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_CONTROLLER_MODE = 0x0B
CMD_SET_INPUT_POS = 0x0C
CMD_CLEAR_ERRORS = 0x18
CMD_SET_POS_GAIN = 0x1A
CMD_SET_VEL_GAINS = 0x1B
CMD_GET_TORQUES = 0x1C

# ── axis states / modes ───────────────────────────────────────────
AXIS_STATE_IDLE = 1
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATES = {0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE",
               3: "FULL_CALIBRATION_SEQUENCE", 4: "MOTOR_CALIBRATION",
               6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
               8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND",
               11: "HOMING", 12: "ENCODER_HALL_POLARITY_CALIBRATION",
               13: "ENCODER_HALL_PHASE_CALIBRATION"}
# Heartbeat byte 5: the result of the last requested state/procedure (firmware 0.6).
PROCEDURE_RESULTS = ("SUCCESS", "BUSY", "CANCELLED", "DISARMED", "NO_RESPONSE",
                     "POLARITY_MISMATCH", "PHASE_RESISTANCE_OUT_OF_RANGE",
                     "PHASE_INDUCTANCE_OUT_OF_RANGE", "UNBALANCED_PHASES",
                     "INVALID_MOTOR_TYPE", "ILLEGAL_HALL_STATE", "TIMEOUT",
                     "HOMING_WITHOUT_ENDSTOP", "INVALID_STATE", "NOT_CALIBRATED",
                     "NOT_CONVERGING")
PROCEDURE_NOT_CALIBRATED = 14
PROCEDURE_PHASE_RESISTANCE_OUT_OF_RANGE = 6
# Error bits (firmware 0.6 ODriveError): heartbeat axis_error, Get_Error's
# active_errors / disarm_reason. They stay latched until Clear_Errors.
ODRIVE_ERRORS = {
    0x1: "INITIALIZING", 0x2: "SYSTEM_LEVEL", 0x4: "TIMING_ERROR",
    0x8: "MISSING_ESTIMATE", 0x10: "BAD_CONFIG", 0x20: "DRV_FAULT",
    0x40: "MISSING_INPUT", 0x100: "DC_BUS_OVER_VOLTAGE", 0x200: "DC_BUS_UNDER_VOLTAGE",
    0x400: "DC_BUS_OVER_CURRENT", 0x800: "DC_BUS_OVER_REGEN_CURRENT",
    0x1000: "CURRENT_LIMIT_VIOLATION", 0x2000: "MOTOR_OVER_TEMP",
    0x4000: "INVERTER_OVER_TEMP", 0x8000: "VELOCITY_LIMIT_VIOLATION",
    0x10000: "POSITION_LIMIT_VIOLATION", 0x1000000: "WATCHDOG_TIMER_EXPIRED",
    0x2000000: "ESTOP_REQUESTED", 0x4000000: "SPINOUT_DETECTED",
    0x8000000: "BRAKE_RESISTOR_DISARMED", 0x10000000: "THERMISTOR_DISCONNECTED",
    0x40000000: "CALIBRATION_ERROR",
}


def error_names(bits: int) -> str:
    """'CALIBRATION_ERROR' / 'A|B' / 'none' for an error bit field."""
    names = [n for b, n in ODRIVE_ERRORS.items() if bits & b]
    rest = bits & ~sum(ODRIVE_ERRORS)
    if rest:
        names.append(f"0x{rest:X}")
    return "|".join(names) or "none"


def axis_state_name(state: int) -> str:
    return AXIS_STATES.get(state, f"state {state}")


def procedure_result_name(result: int) -> str:
    return (PROCEDURE_RESULTS[result] if 0 <= result < len(PROCEDURE_RESULTS)
            else f"result {result}")
CONTROL_MODE_POSITION = 3
INPUT_MODE_PASSTHROUGH = 1


@dataclass
class OdriveCanJointConfig:
    name: str
    node_id: int                        # CAN node id set in odrive_can_setup.py (e.g. 7)
    vel_limit: Optional[float] = None   # informational only (set over USB)
    pos_min: Optional[float] = None     # optional soft limit (turns)
    pos_max: Optional[float] = None     # optional soft limit (turns)
    rated_torque: Optional[float] = None  # Nm, for the later normalized safety layer


class OdriveCanJoint:
    """Wraps a single ODrive axis on a shared CAN bus, identical joint interface.

    The bus is created and owned by MotorController and passed in; many joints
    share it. A reader thread in MotorController calls process_frame() with every
    frame addressed to this node.
    """

    def __init__(self, bus: can.BusABC, config: OdriveCanJointConfig):
        self.cfg = config
        self.bus = bus
        self.node_id = config.node_id

        # Cached feedback, updated by process_frame() from the reader thread.
        self._raw_pos: float = 0.0      # turns
        self._raw_vel: float = 0.0      # turns/s
        self._torque_est: float = 0.0   # Nm
        self._axis_error: int = 0
        self._axis_state: int = AXIS_STATE_IDLE
        self._procedure_result: int = 0
        self._active_errors: int = 0     # Get_Error (streamed by some drives only)
        self._disarm_reason: int = 0
        # time.monotonic() of the last encoder frame; None until one arrives. The
        # cached pos/vel are only as current as this — see feedback_age().
        self._last_fb_time: Optional[float] = None

        # Software zero offset (turns).
        self._zero_offset: float = 0.0
        self.gains: Optional[tuple] = None   # last set_gains(), None = the drive's own
        self._last_target: Optional[float] = None

        # Non-blocking move state (mirrors DamiaoJoint).
        self._move_active: bool = False
        self._move_q0: float = 0.0
        self._move_q1: float = 0.0
        self._move_t0: float = 0.0
        self._move_duration: float = 0.0
        self._move_ramp: float = 0.0

    # ── low-level CAN send ────────────────────────────────────────
    def _arb_id(self, cmd_id: int) -> int:
        return (self.node_id << 5) | cmd_id

    def _send(self, cmd_id: int, data: bytes, rtr: bool = False) -> None:
        self.bus.send(can.Message(
            arbitration_id=self._arb_id(cmd_id),
            data=data,
            is_extended_id=False,
            is_remote_frame=rtr,
        ))

    # ── enable / disable ──────────────────────────────────────────
    def _set_axis_state(self, state: int) -> None:
        self._send(CMD_SET_AXIS_STATE, struct.pack("<I", state))

    def set_controller_mode(self, control_mode: int = CONTROL_MODE_POSITION,
                            input_mode: int = INPUT_MODE_PASSTHROUGH) -> None:
        self._send(CMD_SET_CONTROLLER_MODE, struct.pack("<II", control_mode, input_mode))

    def set_gains(self, pos_gain: float, vel_gain: float,
                  vel_integrator_gain: float = 0.0) -> None:
        """Set the controller gains (RAM only; a power cycle restores the saved ones).
        pos_gain [(turns/s)/turn], vel_gain [Nm/(turns/s)], vel_integrator_gain
        [Nm/turn]. Safe in closed loop. real2sim.odrive_gains converts a joint PD."""
        self._send(CMD_SET_POS_GAIN, struct.pack("<f", pos_gain))
        self._send(CMD_SET_VEL_GAINS, struct.pack("<ff", vel_gain, vel_integrator_gain))
        self.gains = (pos_gain, vel_gain, vel_integrator_gain)

    def clear_errors(self) -> None:
        """Clear the drive's latched errors (active_errors, disarm_reason, and the
        heartbeat's axis_error). Doesn't change the axis state."""
        self._send(CMD_CLEAR_ERRORS, bytes([0]))

    def request_calibration(self) -> None:
        """Run the drive's FULL_CALIBRATION_SEQUENCE (motor R/L, then the encoder
        offset: THE ROTOR TURNS, a fraction of a turn and back). Needed once after
        every power-up before closed loop is accepted. Watch axis_state go back to IDLE
        and procedure_result to SUCCESS."""
        self._set_axis_state(AXIS_STATE_FULL_CALIBRATION_SEQUENCE)

    @property
    def axis_state(self) -> int:
        return self._axis_state

    @property
    def procedure_result(self) -> int:
        return self._procedure_result

    def drive_report(self) -> dict:
        """What the drive last said about itself (heartbeat + Get_Error)."""
        return {"state": axis_state_name(self._axis_state),
                "procedure_result": procedure_result_name(self._procedure_result),
                "axis_error": f"0x{self._axis_error:X}",
                "errors": error_names(self._axis_error | self._active_errors
                                      | self._disarm_reason),
                "active_errors": f"0x{self._active_errors:X}",
                "disarm_reason": f"0x{self._disarm_reason:X}"}

    def enable(self) -> None:
        # Ensure position/passthrough then enter closed loop.
        self.set_controller_mode()
        self._set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)

    def disable(self) -> None:
        self._set_axis_state(AXIS_STATE_IDLE)

    def is_enabled(self) -> bool:
        return self._axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL

    # ── feedback (from cached frames) ─────────────────────────────
    def set_zero(self, current: float = 0.0) -> None:
        """Declare that the joint is physically at `current` (turns) right now.

        Sets the software offset so get_position() == current at this shaft
        position. current=0.0 keeps the old semantics ("this pose is zero");
        a nonzero value is the boot-pose calibration: power the robot on with
        the joint folded against its known mechanical stop and pass that
        angle — no homing move needed.
        """
        self._zero_offset = self._raw_pos - current

    def get_position(self) -> float:
        return self._raw_pos - self._zero_offset

    def get_velocity(self) -> float:
        return self._raw_vel

    def get_torque(self) -> float:
        return self._torque_est

    def feedback_age(self) -> Optional[float]:
        """Seconds since the last encoder frame, or None if none ever arrived.

        The cached position/velocity never expire on their own: a drive that
        stopped talking keeps "reporting" its last values. Anything that acts on
        them must check this first.
        """
        t = self._last_fb_time
        return None if t is None else time.monotonic() - t

    def request_feedback(self) -> None:
        """RTR-poll Get_Encoder_Estimates (harmless if cyclic feedback is on)."""
        self._send(CMD_GET_ENCODER_ESTIMATES, b"", rtr=True)

    def get_state(self) -> dict:
        return {
            "pos": self.get_position(),
            "vel": self._raw_vel,
            "torque": self._torque_est,
            "axis_state": self._axis_state,
            "axis_error": self._axis_error,
            "fb_age": self.feedback_age(),
        }

    # ── commands ──────────────────────────────────────────────────
    def _clamp(self, q: float) -> float:
        if self.cfg.pos_min is not None:
            q = max(q, self.cfg.pos_min)
        if self.cfg.pos_max is not None:
            q = min(q, self.cfg.pos_max)
        return q

    def send_position(self, q: float) -> None:
        """Command an absolute position (turns), relative to software zero.

        Set_Input_Pos payload: <f h h> = pos (turns),
        vel_ff (0.001 turns/s), torque_ff (0.001 Nm). We send 0 feedforwards.
        """
        q = self._clamp(q)
        data = struct.pack("<fhh", q + self._zero_offset, 0, 0)
        self._send(CMD_SET_INPUT_POS, data)
        self._last_target = q

    @staticmethod
    def _ramp_profile(q0: float, q1: float, t: float, ramp: float) -> float:
        if ramp > 0.0 and t < ramp:
            s = t / ramp
            alpha = 3.0 * s**2 - 2.0 * s**3  # smooth cubic ramp from 0 to 1
            return q0 + alpha * (q1 - q0)
        return q1

    def goto(self, target: float, duration: float = 2.0,
             rate_hz: float = 200.0, ramp: float = 0.0) -> None:
        """Blocking smooth move to target (turns)."""
        target = self._clamp(target)
        dt = 1.0 / rate_hz
        t0 = time.time()
        q0 = self.get_position()

        while time.time() - t0 < duration:
            t = time.time() - t0
            self.send_position(self._ramp_profile(q0, target, t, ramp))
            time.sleep(dt)

        self.send_position(target)

    def start_move(self, target: float, duration: float = 2.0,
                   ramp: Optional[float] = None) -> None:
        self._move_q0 = self.get_position()
        self._move_q1 = self._clamp(target)
        self._move_t0 = time.time()
        self._move_duration = max(1e-3, duration)
        self._move_ramp = duration if ramp is None else ramp
        self._move_active = True

    def is_moving(self) -> bool:
        return self._move_active

    def update(self) -> bool:
        if not self._move_active:
            if self._last_target is not None:
                self.send_position(self._last_target)
            return False

        t = time.time() - self._move_t0
        if t >= self._move_duration:
            self.send_position(self._move_q1)
            self._move_active = False
            return False

        self.send_position(self._ramp_profile(self._move_q0, self._move_q1, t,
                                              self._move_ramp))
        return True

    # ── inbound frame decode (called by MotorController reader) ────
    def process_frame(self, msg: can.Message) -> None:
        """Decode a CAN Simple feedback frame addressed to this node."""
        cmd_id = msg.arbitration_id & 0x1F

        if cmd_id == CMD_HEARTBEAT and len(msg.data) >= 7:
            # <I B B B B> : axis_error, axis_state, procedure_result,
            #               trajectory_done_flag, (reserved)
            err, state, proc, _traj, _ = struct.unpack("<IBBBB", msg.data[:8].ljust(8, b"\x00"))
            self._axis_error = err
            self._axis_state = state
            self._procedure_result = proc

        elif cmd_id == CMD_GET_ERROR and len(msg.data) >= 8:
            self._active_errors, self._disarm_reason = struct.unpack("<II", msg.data[:8])

        elif cmd_id == CMD_GET_ENCODER_ESTIMATES and len(msg.data) >= 8:
            pos, vel = struct.unpack("<ff", msg.data[:8])
            self._raw_pos = pos
            self._raw_vel = vel
            self._last_fb_time = time.monotonic()

        elif cmd_id == CMD_GET_TORQUES and len(msg.data) >= 8:
            _torque_target, torque_est = struct.unpack("<ff", msg.data[:8])
            self._torque_est = torque_est

    def shutdown(self) -> None:
        self.disable()
