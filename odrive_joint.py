"""
odrive_joint.py — a hardware abstraction wrapper around a single ODrive axis
driven over CAN ("CAN Simple"), mirroring damiao_joint.DamiaoJoint exactly so
both motor types share one control loop (see demo.py).

This is the CAN-runtime counterpart to odrive_position_control.py (which is now
the USB gain-tuning / config tool). The interface is identical
(enable/disable, set_zero, get_position/get_state, send_position, goto,
start_move/is_moving/update) — only the transport differs.

NOTE on units: ODrive positions are in *turns* (revolutions), whereas the
Damiao motor works in *radians*. 1 turn = 2*pi rad. Targets are written in each
motor's own native units.

Gains and control mode are NOT set here — they cannot be set over CAN Simple.
They are USB config persisted on the drive (see odrive_can_setup.py). This class
assumes the drive is already configured for POSITION_CONTROL / PASSTHROUGH.
"""

import struct
import time
from dataclasses import dataclass
from typing import Optional

import can

from odrive_can import NODE_ID_SHIFT, OdriveCanController

# ── CAN Simple command ids (arb_id = node_id<<5 | cmd_id) ──────────
CMD_HEARTBEAT = 0x01          # RX: active_errors, axis_state, ...
CMD_SET_AXIS_STATE = 0x07     # TX: request axis state
CMD_GET_ENCODER_ESTIMATES = 0x09  # RX: pos (turns), vel (turns/s)
CMD_SET_CONTROLLER_MODE = 0x0B    # TX: control_mode, input_mode
CMD_SET_INPUT_POS = 0x0C      # TX: input_pos (turns), vel_ff, torque_ff
CMD_GET_TORQUES = 0x1C        # RX: torque_target, torque_estimate

# Axis states / control modes (match odrive.enums on firmware 0.6.x)
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROL_MODE_POSITION = 3
INPUT_MODE_PASSTHROUGH = 1


@dataclass
class OdriveJointConfig:
    name: str
    node_id: int                       # CAN Simple node id (e.g. 7)
    pos_min: Optional[float] = None    # optional soft limit (turns)
    pos_max: Optional[float] = None    # optional soft limit (turns)
    set_mode_on_init: bool = True      # push POSITION/PASSTHROUGH on construction


class OdriveJoint:
    """Wraps a single ODrive axis over CAN, same interface as DamiaoJoint."""

    def __init__(self, controller: OdriveCanController, config: OdriveJointConfig):
        self.cfg = config
        self.controller = controller
        self.node_id = config.node_id

        # Register with the controller so our node's RX frames get routed here.
        controller.register(self)

        # Cached state, updated by process_frame() from the reader thread.
        self._raw_pos: float = 0.0        # turns (firmware frame)
        self._vel: float = 0.0            # turns/s
        self._torque_target: float = 0.0
        self._torque_estimate: float = 0.0
        self._axis_state: int = AXIS_STATE_IDLE
        self._active_errors: int = 0
        self._got_encoder: bool = False

        # Software zero offset (turns), so set_zero() needs no firmware write.
        self._zero_offset: float = 0.0

        self._last_target: Optional[float] = None

        self._move_active: bool = False
        self._move_q0: float = 0.0
        self._move_q1: float = 0.0
        self._move_t0: float = 0.0
        self._move_duration: float = 0.0
        self._move_ramp: float = 0.0

        if config.set_mode_on_init:
            self._set_controller_mode(CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH)

    # ── frame helpers ─────────────────────────────────────────────
    def _send(self, cmd_id: int, data: bytes) -> None:
        self.controller.bus.send(can.Message(
            arbitration_id=(self.node_id << NODE_ID_SHIFT) | cmd_id,
            data=data,
            is_extended_id=False,
        ))

    def _set_controller_mode(self, control_mode: int, input_mode: int) -> None:
        self._send(CMD_SET_CONTROLLER_MODE, struct.pack("<II", control_mode, input_mode))

    def _set_axis_state(self, state: int) -> None:
        self._send(CMD_SET_AXIS_STATE, struct.pack("<I", state))

    # ── enable / disable ──────────────────────────────────────────
    def enable(self) -> None:
        self._set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)

    def disable(self) -> None:
        self._set_axis_state(AXIS_STATE_IDLE)

    def is_enabled(self) -> bool:
        return self._axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL

    # ── feedback (RX, called from the controller's reader thread) ──
    def process_frame(self, msg: can.Message) -> None:
        cmd_id = msg.arbitration_id & 0x1F
        data = msg.data
        if cmd_id == CMD_GET_ENCODER_ESTIMATES and len(data) >= 8:
            self._raw_pos, self._vel = struct.unpack_from("<ff", data, 0)
            self._got_encoder = True
        elif cmd_id == CMD_HEARTBEAT and len(data) >= 6:
            self._active_errors, self._axis_state = struct.unpack_from("<IB", data, 0)
        elif cmd_id == CMD_GET_TORQUES and len(data) >= 8:
            self._torque_target, self._torque_estimate = struct.unpack_from("<ff", data, 0)

    def _read_raw_pos(self) -> float:
        return self._raw_pos

    def set_zero(self) -> None:
        """Define the current shaft position as 0 (software offset)."""
        self._zero_offset = self._read_raw_pos()

    def get_position(self) -> float:
        return self._read_raw_pos() - self._zero_offset

    def get_state(self) -> dict:
        return {
            "pos": self.get_position(),
            "vel": self._vel,
            "torque": self._torque_estimate,
            "state": self._axis_state,
            "errors": self._active_errors,
        }

    # ── commands ──────────────────────────────────────────────────
    def _clamp(self, q: float) -> float:
        if self.cfg.pos_min is not None:
            q = max(q, self.cfg.pos_min)
        if self.cfg.pos_max is not None:
            q = min(q, self.cfg.pos_max)
        return q

    def send_position(self, q: float) -> None:
        """Command an absolute position (turns), relative to the software zero."""
        q = self._clamp(q)
        # Set_Input_Pos: input_pos (float32 turns), vel_ff & torque_ff (int16, 0.001 scale)
        self._send(CMD_SET_INPUT_POS, struct.pack("<fhh", q + self._zero_offset, 0, 0))
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

    def shutdown(self) -> None:
        self.disable()
