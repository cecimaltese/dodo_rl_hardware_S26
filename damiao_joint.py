"""
damiao_joint.py — a small hardware abstraction wrapper around a single Damiao
motor (MIT mode over CAN), mirroring the OdriveCanJoint / OdriveJoint interface so
all motors on the robot expose an identical joint interface and can share one
control loop (see motor_controller.py / demo_combined.py).

NOTE on units: Damiao positions are in *radians*, whereas the ODrive works in
*turns* (revolutions). Keep that in mind when combining the two: 1 turn = 2*pi rad.
Targets are written in each motor's own native units; the rad<->turns convention
for the policy vector is handled in MotorController.

This wrapper sits on top of the `damiao_motor` pip library (DaMiaoController).
Several Damiao motors share ONE DaMiaoController (one CAN socket); each
DamiaoJoint is created with that shared controller plus its own config.
"""

import time
from dataclasses import dataclass
from typing import Optional

from damiao_motor import DaMiaoController


@dataclass
class DamiaoJointConfig:
    name: str
    motor_id: int                       # CAN id the motor listens on (e.g. 0x02)
    feedback_id: int                    # CAN id the motor reports on (e.g. 0x12)
    motor_type: str = "4310"            # damiao_motor model string
    kp: float = 30.0                    # MIT-mode stiffness
    kd: float = 0.5                     # MIT-mode damping
    pos_min: Optional[float] = None     # optional soft limit (rad)
    pos_max: Optional[float] = None     # optional soft limit (rad)
    rated_torque: Optional[float] = None  # Nm, for the later normalized safety layer


class DamiaoJoint:
    """Wraps a single Damiao motor and exposes the shared joint interface.

    Interface (identical to OdriveCanJoint):
        enable / disable / is_enabled
        set_zero
        get_position / get_state
        send_position
        goto                              (blocking smooth ramp)
        start_move / is_moving / update   (non-blocking, ticked by a shared loop)
    """

    def __init__(self, controller: DaMiaoController, config: DamiaoJointConfig):
        self.cfg = config
        self.controller = controller

        # Register this motor on the shared controller / CAN socket.
        self.motor = controller.add_motor(
            motor_id=config.motor_id,
            feedback_id=config.feedback_id,
            motor_type=config.motor_type,
        )
        self.motor.ensure_control_mode("MIT")

        # Software zero offset (rad), so set_zero() can be re-applied at runtime
        # without re-flashing the motor's persistent zero.
        self._zero_offset: float = 0.0
        self._last_target: Optional[float] = None

        # Non-blocking move state (mirrors OdriveCanJoint).
        self._move_active: bool = False
        self._move_q0: float = 0.0
        self._move_q1: float = 0.0
        self._move_t0: float = 0.0
        self._move_duration: float = 0.0
        self._move_ramp: float = 0.0

    # ── enable / disable ──────────────────────────────────────────
    def enable(self) -> None:
        self.motor.enable()

    def disable(self) -> None:
        self.motor.disable()

    def is_enabled(self) -> bool:
        # damiao_motor doesn't expose a clean enabled flag; track via states if available.
        states = self.motor.get_states() or {}
        return bool(states.get("enabled", True))

    # ── feedback ──────────────────────────────────────────────────
    def _read_raw_pos(self) -> float:
        states = self.motor.get_states() or {}
        return float(states.get("pos", 0.0))

    def set_zero_hardware(self) -> None:
        """Persist the current shaft position as the motor's hardware zero.

        Must be called while the motor is disabled (per damiao_motor).
        """
        self.motor.disable()
        time.sleep(0.1)
        self.motor.set_zero_position()
        time.sleep(0.2)
        self._zero_offset = 0.0

    def set_zero(self) -> None:
        """Define the current shaft position as 0 via a software offset."""
        self._zero_offset = self._read_raw_pos()

    def get_position(self) -> float:
        return self._read_raw_pos() - self._zero_offset

    def get_velocity(self) -> float:
        states = self.motor.get_states() or {}
        return float(states.get("vel", 0.0))

    def get_torque(self) -> float:
        states = self.motor.get_states() or {}
        # damiao_motor reports torque under "torq".
        return float(states.get("torq", 0.0))

    def get_state(self) -> dict:
        return {
            "pos": self.get_position(),
            "vel": self.get_velocity(),
            "torque": self.get_torque(),
        }

    # ── commands ──────────────────────────────────────────────────
    def _clamp(self, q: float) -> float:
        if self.cfg.pos_min is not None:
            q = max(q, self.cfg.pos_min)
        if self.cfg.pos_max is not None:
            q = min(q, self.cfg.pos_max)
        return q

    def send_position(self, q: float) -> None:
        """Command an absolute position (rad), relative to the software zero."""
        q = self._clamp(q)
        self.motor.send_cmd_mit(
            target_position=q + self._zero_offset,
            target_velocity=0.0,
            stiffness=self.cfg.kp,
            damping=self.cfg.kd,
            feedforward_torque=0.0,
        )
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
        """Blocking smooth move to target (rad)."""
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
        """Tick the non-blocking move. Returns True while still moving."""
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
