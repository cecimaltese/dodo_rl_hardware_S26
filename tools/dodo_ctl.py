"""
dodo_ctl.py — send ctl commands to the motor service (real or sim).

    python3 tools/dodo_ctl.py status
    python3 tools/dodo_ctl.py calibrate       # NORMAL STARTUP: robot was powered
                                              # on with the legs folded at the
                                              # stops (boot pose) — maps ODrive
                                              # turns onto those known angles
    python3 tools/dodo_ctl.py enable          # bumpless; unparks joints to the
                                              # safety band over ~1.5 s
    python3 tools/dodo_ctl.py disable
    python3 tools/dodo_ctl.py set_zero        # jig/bench only: robot posed at
                                              # URDF zero -> current pose = 0
    python3 tools/dodo_ctl.py estop_reset     # clears the latch; motors stay
                                              # disabled until you 'enable'
    python3 tools/dodo_ctl.py reset           # sim_service only: re-pose robot
    python3 tools/dodo_ctl.py gantry --arg lower=0.03   # sim_service --gantry only:
    python3 tools/dodo_ctl.py gantry --arg release=true #   pay out rope / let go

Bring-up order on the real robot:
    power on folded → status → calibrate → status (VERIFY the boot angles!)
    → enable → tools/dodo_startup.py (or run_policy --backend zmq).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import dodo_ipc as ipc  # noqa: E402

KNOWN = ["status", "calibrate", "enable", "disable", "set_zero",
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
                    help="calibrate only: assumed power-on pose (default 'boot')")
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
