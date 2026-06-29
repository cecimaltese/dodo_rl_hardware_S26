"""
odrive_can_setup.py — one-time USB tool to inspect and configure an ODrive for
CAN ("CAN Simple") operation.

ODrive CAN config (node id, bus baud rate, cyclic feedback message rates) and
the controller gains/modes cannot be set over CAN Simple — they are USB config
persisted on the drive. Run this once over USB so the drive is ready for the
CAN-runtime path (odrive_can.py + odrive_joint.py).

What it does:
  1. Connect over USB and PRINT the current `config.can` tree + gains so you can
     see the actual attribute names/values on this firmware (0.6.11).
  2. Set node_id and bus baud_rate.
  3. Enable cyclic feedback: heartbeat / encoder / torques message rates (ms) —
     required so the CAN driver can read pos/vel/torque without polling.
  4. Ensure POSITION_CONTROL + PASSTHROUGH and apply controller gains.
  5. save_configuration() — reboot-tolerant (saving drops the USB link).

Attribute paths are guarded with hasattr() because the exact names depend on the
firmware build; the printed tree in step 1 is the ground truth.
"""

import time

import odrive
from odrive.enums import AxisState, ControlMode, InputMode, Protocol

# ── desired CAN config ────────────────────────────────────────────
NODE_ID = 7
BAUD_RATE = 1_000_000
# Cyclic feedback rates (ms)
HEARTBEAT_RATE_MS = 10
ENCODER_RATE_MS = 10
TORQUES_RATE_MS = 10

# Gains tuned earlier over USB (see HANDOFF.md); applied so the drive is ready.
POS_GAIN = 15.0
VEL_GAIN = 0.05
VEL_INTEGRATOR_GAIN = 0.1


def _trysetattr(obj, name, value) -> bool:
    """Set obj.name = value if the attribute exists; report what happened."""
    if obj is not None and hasattr(obj, name):
        try:
            setattr(obj, name, value)
            print(f"    set {name} = {value}")
            return True
        except Exception as exc:  # noqa: BLE001
            print(f"    FAILED to set {name}: {exc}")
            return False
    print(f"    (skip) attribute {name!r} not present on this firmware")
    return False


def _print_can_config(can_cfg) -> None:
    """Print every scalar attribute under a config.can object."""
    if can_cfg is None:
        print("  (no config.can object found)")
        return
    for name in sorted(dir(can_cfg)):
        if name.startswith("_"):
            continue
        try:
            val = getattr(can_cfg, name)
        except Exception:
            continue
        # Skip nested remote objects / callables; we want scalar settings.
        if callable(val):
            continue
        print(f"    can.{name} = {val}")


def main():
    print("Waiting for ODrive over USB ...")
    odrv = odrive.find_sync()
    print(f"Found ODrive {odrv._dev.serial_number}")

    axis = odrv.axis0
    can_cfg = getattr(axis.config, "can", None)

    # 1) Inspect current state
    print("\n[1] Current per-axis CAN config (axis0.config.can):")
    _print_can_config(can_cfg)
    if hasattr(odrv, "can") and hasattr(odrv.can, "config"):
        print(f"    odrv.can.config.baud_rate = {getattr(odrv.can.config, 'baud_rate', '?')}")

    # 2) Node id + baud rate + enable the CAN Simple protocol.
    # NOTE: protocol defaults to Protocol.NONE (0) on a fresh drive — with it the
    # ODrive ignores CAN entirely (no cyclic frames, no command response) even
    # though node_id/baud/rates are set. It MUST be Protocol.SIMPLE (1).
    print("\n[2] Setting node id + baud rate + protocol:")
    _trysetattr(can_cfg, "node_id", NODE_ID)
    if hasattr(odrv, "can") and hasattr(odrv.can, "config"):
        _trysetattr(odrv.can.config, "baud_rate", BAUD_RATE)
        _trysetattr(odrv.can.config, "protocol", Protocol.SIMPLE)

    # 3) Cyclic feedback message rates
    print("\n[3] Enabling cyclic feedback message rates:")
    _trysetattr(can_cfg, "heartbeat_msg_rate_ms", HEARTBEAT_RATE_MS)
    _trysetattr(can_cfg, "encoder_msg_rate_ms", ENCODER_RATE_MS)
    # firmware has used both 'torques_' and 'torque_' spellings — try both
    if not _trysetattr(can_cfg, "torques_msg_rate_ms", TORQUES_RATE_MS):
        _trysetattr(can_cfg, "torque_msg_rate_ms", TORQUES_RATE_MS)

    # 4) Controller mode + gains
    print("\n[4] Ensuring POSITION_CONTROL / PASSTHROUGH + gains:")
    axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
    axis.controller.config.input_mode = InputMode.PASSTHROUGH
    axis.controller.config.pos_gain = POS_GAIN
    axis.controller.config.vel_gain = VEL_GAIN
    axis.controller.config.vel_integrator_gain = VEL_INTEGRATOR_GAIN
    print(f"    control_mode=POSITION, input_mode=PASSTHROUGH, "
          f"pos_gain={POS_GAIN}, vel_gain={VEL_GAIN}, "
          f"vel_integrator_gain={VEL_INTEGRATOR_GAIN}")

    # 5) Save (reboots the drive and drops USB — tolerate the disconnect)
    print("\n[5] Saving configuration (drive will reboot, USB will drop) ...")
    try:
        odrv.save_configuration()
    except Exception as exc:  # noqa: BLE001 — expected: USB drops on reboot
        print(f"    save_configuration() raised (expected on reboot): {exc}")

    print("\nReconnecting to confirm saved config ...")
    time.sleep(2.0)
    odrv = odrive.find_sync()
    axis = odrv.axis0
    can_cfg = getattr(axis.config, "can", None)
    print("Config after reboot (axis0.config.can):")
    _print_can_config(can_cfg)
    if hasattr(odrv, "can") and hasattr(odrv.can, "config"):
        print(f"    odrv.can.config.baud_rate = {getattr(odrv.can.config, 'baud_rate', '?')}")

    print("\nDone. The ODrive is configured for CAN Simple. "
          "Next: bring up can0 @ 1M and run the CAN bench test.")


if __name__ == "__main__":
    main()
