"""
odrive_position_control.py — a small hardware abstraction wrapper around a single
ODrive axis, mirroring the DamiaoJoint interface so the two motors can share one
control loop (see demo.py).

NOTE on units: ODrive positions are in *turns* (revolutions), whereas the Damiao
motor works in *radians*. Keep that in mind when combining the two: 1 turn = 2*pi rad.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional

import odrive
from odrive.enums import AxisState, ControlMode, InputMode
from odrive.utils import dump_errors, request_state


@dataclass
class OdriveJointConfig:
    name: str
    axis: int = 0                      # which axis on the ODrive (0 or 1)
    serial: Optional[str] = None       # connect to a specific ODrive by serial, or None for first found
    vel_limit: Optional[float] = None  # optional velocity limit (turns/s)
    pos_min: Optional[float] = None    # optional soft limit (turns)
    pos_max: Optional[float] = None    # optional soft limit (turns)

    # Controller gains (leave None to keep whatever is on the drive).
    # Lower vel_gain first if the motor buzzes/vibrates.
    pos_gain: Optional[float] = None            # turns/s per turn of error
    vel_gain: Optional[float] = None            # torque per (turn/s) of error
    vel_integrator_gain: Optional[float] = None # kills steady-state error
    save_config: bool = False                   # persist gains to the drive (survives reboot)


class OdriveJoint:
    """Wraps a single ODrive axis and exposes the same joint interface as DamiaoJoint."""

    def __init__(self, config: OdriveJointConfig, odrv=None):
        self.cfg = config

        # Find a connected ODrive (blocks until one is connected)
        if odrv is None:
            print(f"[{config.name}] waiting for ODrive...")
            odrv = odrive.find_sync(serial_number=config.serial) if config.serial \
                else odrive.find_sync()
            print(f"[{config.name}] found ODrive {odrv._dev.serial_number}")
        self.odrv = odrv
        self.axis = getattr(odrv, f"axis{config.axis}")

        # Configure position control
        self.axis.controller.config.input_mode = InputMode.PASSTHROUGH
        self.axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
        if config.vel_limit is not None:
            self.axis.controller.config.vel_limit = config.vel_limit

        # Controller gains — only override what the config specifies.
        if config.pos_gain is not None:
            self.axis.controller.config.pos_gain = config.pos_gain
        if config.vel_gain is not None:
            self.axis.controller.config.vel_gain = config.vel_gain
        if config.vel_integrator_gain is not None:
            self.axis.controller.config.vel_integrator_gain = config.vel_integrator_gain

        if config.save_config:
            self.odrv.save_configuration()

        # Software zero offset (turns), so set_zero() works without touching firmware state
        self._zero_offset: float = 0.0

        self._last_target: Optional[float] = None

        self._move_active: bool = False
        self._move_q0: float = 0.0
        self._move_q1: float = 0.0
        self._move_t0: float = 0.0
        self._move_duration: float = 0.0
        self._move_ramp: float = 0.0

    # ── enable / disable ──────────────────────────────────────────
    def enable(self) -> None:
        request_state(self.axis, AxisState.CLOSED_LOOP_CONTROL)

    def disable(self) -> None:
        request_state(self.axis, AxisState.IDLE)

    def is_enabled(self) -> bool:
        return self.axis.current_state == AxisState.CLOSED_LOOP_CONTROL

    # ── feedback ──────────────────────────────────────────────────
    def _read_raw_pos(self) -> float:
        """Position estimate in turns (firmware-version tolerant)."""
        try:
            return self.axis.pos_estimate
        except AttributeError:
            return self.axis.encoder.pos_estimate

    def set_zero(self) -> None:
        """Define the current shaft position as 0 (software offset)."""
        self._zero_offset = self._read_raw_pos()

    def get_position(self) -> float:
        return self._read_raw_pos() - self._zero_offset

    def get_state(self) -> dict:
        return {
            "pos": self.get_position(),
            "state": self.axis.current_state,
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
        self.axis.controller.input_pos = q + self._zero_offset
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
        dump_errors(self.odrv)
        self.disable()


# ---------------------------------------------------------------------------
# DEMO
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # If the motor buzzes/vibrates, lower vel_gain first, then re-tune.
    # Set save_config=True once you're happy with the values to persist them.
    def ask_float(prompt, default):
        s = input(f"{prompt} [{default}]: ").strip()
        return default if s == "" else float(s)
    
    print("\nEnter controller gains (leave blank to keep current values):")
    pos_gain = ask_float("Position gain", 15.0)
    vel_gain = ask_float("Velocity gain", 0.05)
    vel_integrator_gain = ask_float("Velocity integrator gain", 0.1)

    joint = OdriveJoint(OdriveJointConfig(
        name="test_joint",
        axis=0,
        pos_gain=pos_gain,
        vel_gain=vel_gain,
        vel_integrator_gain=vel_integrator_gain,
        save_config=False,
    ))

    if input("\nSet zero at current shaft position? (y/N): ").strip().lower() == "y":
        joint.set_zero()
        print("  Zero set.")

    joint.enable()
    time.sleep(0.1)

    try:
        joint.goto(+5.0, duration=2.0, ramp=1.0)
        joint.goto(-5.0, duration=2.0, ramp=1.0)
        joint.goto(0.0, duration=3.0, ramp=1.0)
    finally:
        joint.shutdown()
        print("\nDone, ODrive set to idle.")

