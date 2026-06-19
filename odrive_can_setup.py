"""
odrive_can_setup.py — one-time USB configuration tool to prepare an ODrive for
runtime control over CAN ("CAN Simple" protocol).

WHY THIS EXISTS
---------------
ODrive gains and modes can NOT be set over CAN Simple — they live in the drive's
persistent config and must be written over USB and saved. So:
  * odrive_can_setup.py      (this file, USB)  -> configure + save, run once.
  * odrive_position_control.py (USB)           -> interactive gain tuning tool.
  * odrive_can_joint.py      (CAN, runtime)    -> the actual control path.

WHAT IT DOES
------------
  * prints the current CAN config so you can see what's there;
  * sets node_id (default 7), CAN baud rate (1 Mbit/s);
  * enables cyclic feedback messages (heartbeat / encoder estimates / torques)
    at ~10 ms so the runtime reader thread always has fresh state;
  * ensures POSITION_CONTROL + PASSTHROUGH input mode and (optional) gains;
  * save_configuration() — this REBOOTS the drive and drops USB, which is normal.

ATTRIBUTE-PATH NOTE
-------------------
Paths below target odrive 0.6.11 firmware. If an attribute is missing, your
firmware names it slightly differently — print `odrv0.axis0.config.can` and
`odrv0.can.config` to find the right one. The script guards each write so a
missing attribute warns instead of crashing.

Usage:
    python3 odrive_can_setup.py            # configure first ODrive, node_id 7
    python3 odrive_can_setup.py --node-id 8 --axis 0
"""

import argparse
import sys

import odrive
from odrive.enums import ControlMode, InputMode


def _try_set(obj, attr_path, value):
    """Set a (possibly nested) attribute; warn if the path doesn't exist."""
    parts = attr_path.split(".")
    target = obj
    try:
        for p in parts[:-1]:
            target = getattr(target, p)
        getattr(target, parts[-1])  # probe existence
        setattr(target, parts[-1], value)
        print(f"  set {attr_path} = {value}")
        return True
    except AttributeError:
        print(f"  [skip] {attr_path} not found on this firmware")
        return False


def _try_get(obj, attr_path):
    parts = attr_path.split(".")
    target = obj
    try:
        for p in parts:
            target = getattr(target, p)
        return target
    except AttributeError:
        return "<n/a>"


def main():
    ap = argparse.ArgumentParser(description="Configure an ODrive for CAN Simple.")
    ap.add_argument("--node-id", type=int, default=7,
                    help="CAN node id (keep 7/8 to avoid arb-id collisions, see HANDOFF).")
    ap.add_argument("--axis", type=int, default=0, help="Axis index (0 or 1).")
    ap.add_argument("--baud", type=int, default=1_000_000, help="CAN baud rate (Hz).")
    ap.add_argument("--fb-rate-ms", type=int, default=10,
                    help="Cyclic feedback period for heartbeat/encoder/torque (ms).")
    ap.add_argument("--serial", type=str, default=None, help="Connect to a specific ODrive serial.")
    ap.add_argument("--pos-gain", type=float, default=None)
    ap.add_argument("--vel-gain", type=float, default=None)
    ap.add_argument("--vel-integrator-gain", type=float, default=None)
    ap.add_argument("--no-save", action="store_true",
                    help="Configure but do NOT save (won't reboot / won't persist).")
    args = ap.parse_args()

    print("Waiting for ODrive over USB...")
    odrv = odrive.find_sync(serial_number=args.serial) if args.serial else odrive.find_sync()
    print(f"Found ODrive {odrv._dev.serial_number}")

    axis = getattr(odrv, f"axis{args.axis}")
    axis_can = f"axis{args.axis}.config.can"

    # ── current config ────────────────────────────────────────────
    print("\nCurrent CAN config:")
    print(f"  node_id          = {_try_get(odrv, axis_can + '.node_id')}")
    print(f"  baud_rate        = {_try_get(odrv, 'can.config.baud_rate')}")
    print(f"  heartbeat_rate   = {_try_get(odrv, axis_can + '.heartbeat_msg_rate_ms')}")
    print(f"  encoder_rate     = {_try_get(odrv, axis_can + '.encoder_msg_rate_ms')}")
    print(f"  torque_rate      = {_try_get(odrv, axis_can + '.torque_msg_rate_ms')}")

    # ── apply CAN config ──────────────────────────────────────────
    print("\nApplying CAN config:")
    _try_set(odrv, "can.config.baud_rate", args.baud)
    _try_set(odrv, axis_can + ".node_id", args.node_id)
    # Cyclic feedback so the runtime reader always has fresh pos/vel/torque.
    _try_set(odrv, axis_can + ".heartbeat_msg_rate_ms", args.fb_rate_ms)
    _try_set(odrv, axis_can + ".encoder_msg_rate_ms", args.fb_rate_ms)
    _try_set(odrv, axis_can + ".torque_msg_rate_ms", args.fb_rate_ms)

    # ── ensure position control + passthrough ─────────────────────
    print("\nEnsuring control mode:")
    _try_set(axis, "controller.config.control_mode", ControlMode.POSITION_CONTROL)
    _try_set(axis, "controller.config.input_mode", InputMode.PASSTHROUGH)
    if args.pos_gain is not None:
        _try_set(axis, "controller.config.pos_gain", args.pos_gain)
    if args.vel_gain is not None:
        _try_set(axis, "controller.config.vel_gain", args.vel_gain)
    if args.vel_integrator_gain is not None:
        _try_set(axis, "controller.config.vel_integrator_gain", args.vel_integrator_gain)

    # Make sure the drive comes up idle, not spinning, after reboot.
    _try_set(axis, "config.startup_closed_loop_control", False)

    if args.no_save:
        print("\n--no-save given: configuration NOT persisted (no reboot).")
        return

    # ── save (reboots + drops USB) ────────────────────────────────
    print("\nSaving configuration (drive will reboot and USB will drop — this is normal)...")
    try:
        odrv.save_configuration()
    except Exception as e:
        # save_configuration reboots, which tears down the USB link mid-call.
        print(f"  (expected disconnect on reboot: {type(e).__name__})")

    print("\nDone. Re-plug / reconnect if needed, then verify with `candump can0`:")
    nid = args.node_id
    print(f"  heartbeat frame id = 0x{(nid << 5) | 0x01:03X}")
    print(f"  encoder   frame id = 0x{(nid << 5) | 0x09:03X}")
    print(f"  torque    frame id = 0x{(nid << 5) | 0x1C:03X}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
