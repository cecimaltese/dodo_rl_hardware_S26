"""
odrive_can_joint.py — control a single ODrive axis over CAN using the "CAN Simple"
protocol (raw python-can), exposing the SAME joint interface as DamiaoJoint and
OdriveJoint so all motors share one control loop.

This is the RUNTIME path for the ODrive. Gains / modes are NOT settable here —
configure those once over USB with odrive_can_setup.py.

CAN Simple basics
-----------------
  arbitration id = (node_id << 5) | cmd_id
A frame "belongs" to a node if (arb_id >> 5) == node_id; the low 5 bits are the
command id. The MotorController routes received frames to the right joint by
node id, then each joint decodes by cmd id in process_frame().

Command ids used (see HANDOFF / ros_odrive cmd table):
  TX:  0x07 Set_Axis_State, 0x0B Set_Controller_Mode, 0x0C Set_Input_Pos
  RX:  0x01 Heartbeat, 0x09 Get_Encoder_Estimates, 0x1C Get_Torques

Units: ODrive positions are in *turns* (revolutions). 1 turn = 2*pi rad.
"""

import struct
import time
from dataclasses import dataclass
from typing import Optional

import can

# ── CAN Simple command ids ────────────────────────────────────────
CMD_HEARTBEAT = 0x01
CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_CONTROLLER_MODE = 0x0B
CMD_SET_INPUT_POS = 0x0C
CMD_GET_TORQUES = 0x1C

# ── axis states / modes ───────────────────────────────────────────
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
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
        self._last_fb_time: float = 0.0

        # Software zero offset (turns).
        self._zero_offset: float = 0.0
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

    def enable(self) -> None:
        # Ensure position/passthrough then enter closed loop.
        self.set_controller_mode()
        self._set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)

    def disable(self) -> None:
        self._set_axis_state(AXIS_STATE_IDLE)

    def is_enabled(self) -> bool:
        return self._axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL

    # ── feedback (from cached frames) ─────────────────────────────
    def set_zero(self) -> None:
        """Define the current shaft position as 0 (software offset, turns)."""
        self._zero_offset = self._raw_pos

    def get_position(self) -> float:
        return self._raw_pos - self._zero_offset

    def get_velocity(self) -> float:
        return self._raw_vel

    def get_torque(self) -> float:
        return self._torque_est

    def get_state(self) -> dict:
        return {
            "pos": self.get_position(),
            "vel": self._raw_vel,
            "torque": self._torque_est,
            "axis_state": self._axis_state,
            "axis_error": self._axis_error,
            "fb_age": time.time() - self._last_fb_time if self._last_fb_time else None,
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
            err, state, _proc, _traj, _ = struct.unpack("<IBBBB", msg.data[:8].ljust(8, b"\x00"))
            self._axis_error = err
            self._axis_state = state

        elif cmd_id == CMD_GET_ENCODER_ESTIMATES and len(msg.data) >= 8:
            pos, vel = struct.unpack("<ff", msg.data[:8])
            self._raw_pos = pos
            self._raw_vel = vel
            self._last_fb_time = time.time()

        elif cmd_id == CMD_GET_TORQUES and len(msg.data) >= 8:
            _torque_target, torque_est = struct.unpack("<ff", msg.data[:8])
            self._torque_est = torque_est

    def shutdown(self) -> None:
        self.disable()
