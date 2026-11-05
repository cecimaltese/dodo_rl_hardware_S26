"""
dodo_ctl.py — send ctl commands to the motor service (real or sim).

    python3 tools/dodo_ctl.py status
    python3 tools/dodo_ctl.py odrive_calibrate          # ONCE PER POWER-UP, legs free:
                                              # the drives' own calibration; the
                                              # knee/ankle motors MOVE 8-16 deg
    python3 tools/dodo_ctl.py calibrate --pose straight
                                              # EVERY SESSION: the knees/ankles are
                                              # in that reference pose (legs hanging
                                              # straight) — their ODrive zeros
    python3 tools/dodo_ctl.py enable          # bumpless; unparks joints to the
                                              # safety band over ~1.5 s
    python3 tools/dodo_ctl.py disable
    python3 tools/dodo_ctl.py gains --arg scale=0.5   # PD at half fleet_config's
                                              # kp (kd x sqrt): live, and at enable
    python3 tools/dodo_ctl.py set_zero        # old name for calibrate --pose straight
    python3 tools/dodo_ctl.py estop_reset     # clears the latch; motors stay
                                              # disabled until you 'enable'
    python3 tools/dodo_ctl.py reset           # sim_service only: re-pose robot
    python3 tools/dodo_ctl.py gantry --arg lower=0.03   # sim_service --gantry only:
    python3 tools/dodo_ctl.py gantry --arg release=true #   pay out rope / let go

Bring-up order on the real robot (README *Bring-up*):
    legs hanging free → odrive_calibrate → legs straight → calibrate --pose straight
    → status (VERIFY!)
    → tools/dodo_startup.py --pose home (enables, ramps, holds).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import dodo_ipc as ipc  # noqa: E402

KNOWN = ["status", "odrive_calibrate", "calibrate", "enable", "disable", "gains", "set_zero",
         "estop_reset", "reset", "gantry"]


def parse_arg(text: str):
    """key=value; the value as JSON if it parses (0.03, true), else a string."""
    key, sep, value = text.partition("=")
    if not sep or not key:
        raise argparse.ArgumentTypeError(f"expected key=value, got {text!r}")
    try:
        return key, json.loads(value)
    except json.JSONDecodeError:
        return key, value


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", help=f"one of {KNOWN} (or any cmd the service knows)")
    ap.add_argument("--pose", default=None,
                    help="calibrate: the reference pose the knees/ankles are in "
                         "(fleet_config.REFERENCE_POSES, e.g. straight)")
    ap.add_argument("--arg", type=parse_arg, action="append", default=[],
                    metavar="KEY=VALUE", help="extra argument for the command (repeatable)")
    ap.add_argument("--timeout", type=float, default=3.0)
    args = ap.parse_args()

    kwargs = dict(args.arg)
    if args.pose is not None:
        kwargs["pose"] = args.pose

    client = ipc.CtlClient()
    try:
        reply = client.request(args.cmd, timeout_s=args.timeout, **kwargs)
    except TimeoutError as e:
        print(f"ERROR: {e}")
        sys.exit(2)
    finally:
        client.close()

    print(json.dumps(reply, indent=2, sort_keys=True))
    sys.exit(0 if reply.get("ok") else 1)


if __name__ == "__main__":
    main()
