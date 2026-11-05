"""
demo_combined.py — bench demo over ZMQ: drive any subset of joints with a
smooth sine through the full service stack (the same path the policy uses).

Replaces the old direct-CAN demo. Works IDENTICALLY against:
  * motor_service.py on the robot/bench (1 Damiao + 1 ODrive, or the full fleet)
  * sim_service.py in MuJoCo (expect the robot to flop around URDF-zero unless
    you pass --center default — this demo is a plumbing check, not a policy)

What it exercises end-to-end: ctl (status/enable/disable), motor_commands
streaming at 50 Hz with NaN for un-selected joints (they hold still), live
motor_states feedback, and the service's clamps + watchdogs.

    # bench: wiggle two joints ±0.3 rad at 0.25 Hz for 10 s
    python3 demo_combined.py --joints hip_left,lower_leg_left

    # everything, gentler, against sim_service around the standing pose
    python3 demo_combined.py --amp 0.1 --center default

    # robot, first motion: one joint, a few degrees around where it is held
    python3 demo_combined.py --joints hip_left --amp 0.05

Bring-up first (real robot): motor_service running, ODrives calibrated (knees and
ankles in a reference pose, e.g. legs hanging straight on the gantry):
    python3 tools/dodo_ctl.py status
    python3 tools/dodo_ctl.py calibrate --pose straight   # if not calibrated yet

--center current (the default) oscillates around each joint's held target after
the bumpless enable, so the first command never jumps; zero / default jump to
0 rad / the old stand policy's pose first (ramped in over 2 s by the sine's gain,
but the offset itself is a step).
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import dodo_ipc as ipc

# Default (home) pose for --center default, kept in sync with rl_env / dodo.py.
_DEFAULT_CENTER = {
    "hip_": 0.0, "upper_leg_": 0.20, "lower_leg_": -0.50, "foot_": 0.30,
}


def _center_for(joint: str, mode: str, held: dict = None) -> float:
    if mode == "current":
        return float(held[joint])
    if mode == "zero":
        return 0.0
    for prefix, val in _DEFAULT_CENTER.items():
        if joint.startswith(prefix):
            return val
    return 0.0


def _smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return 3.0 * x * x - 2.0 * x * x * x


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joints", default=None,
                    help="comma-separated subset (default: every joint the "
                         "service runs, per ctl status)")
    ap.add_argument("--amp", type=float, default=0.3, help="sine amplitude [rad]")
    ap.add_argument("--freq", type=float, default=0.25, help="sine frequency [Hz]")
    ap.add_argument("--secs", type=float, default=10.0, help="run time [s]")
    ap.add_argument("--rate", type=float, default=50.0, help="command rate [Hz]")
    ap.add_argument("--center", choices=["current", "zero", "default"], default="current",
                    help="sine center: where each joint is held after enable (default), "
                         "joint zero, or the old stand policy's pose (sim)")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    args = ap.parse_args()

    ctl = ipc.CtlClient()

    # ── 1) status: what is actually out there? ────────────────────
    try:
        status = ctl.request("status")
    except TimeoutError as e:
        print(f"ERROR: {e}")
        sys.exit(2)
    joints_info = status.get("joints", {})
    print(f"service: enabled={status.get('enabled')} estopped={status.get('estopped')} "
          f"all_calibrated={status.get('all_calibrated')}")
    for n, j in sorted(joints_info.items()):
        print(f"  {n:<16} {j.get('type', '?'):<7} fb={str(j.get('has_feedback')):<5} "
              f"cal={str(j.get('calibrated')):<5} pos={j.get('pos_rad', 0):+.3f} rad")

    if status.get("estopped"):
        print("service is E-STOPPED — tools/dodo_ctl.py estop_reset first.")
        sys.exit(1)

    # ── 2) pick joints ────────────────────────────────────────────
    if args.joints:
        selected = [s.strip() for s in args.joints.split(",")]
        unknown = [s for s in selected if s not in ipc.JOINT_INDEX]
        if unknown:
            print(f"ERROR: unknown joints {unknown}; known: {ipc.JOINT_ORDER}")
            sys.exit(1)
    else:
        # Every joint the service runs. Not "those with feedback": a disabled
        # Damiao is silent by design (it only answers frames), and `enable`
        # refuses any joint that doesn't answer, so that check happens there.
        selected = list(joints_info)
    if not selected:
        print("no joints to drive (the service reports none) — aborting.")
        sys.exit(1)
    print(f"\ndriving {selected} : ±{args.amp} rad @ {args.freq} Hz for {args.secs}s "
          f"(center={args.center})")
    if not args.yes and input("continue? (y/N): ").strip().lower() != "y":
        sys.exit(0)

    # ── 3) enable + streams ───────────────────────────────────────
    r = ctl.request("enable")
    if not r.get("ok"):
        print(f"enable failed: {r}")
        sys.exit(1)
    cmd_pub = ipc.Publisher(ipc.TOPIC_MOTOR_COMMANDS)
    state_sub = ipc.Subscriber(ipc.TOPIC_MOTOR_STATES, conflate=True)
    time.sleep(0.3)                       # slow-joiner settle before streaming

    idx = [ipc.JOINT_INDEX[n] for n in selected]
    held = {n: (j.get("target_rad") if j.get("target_rad") is not None
                else j.get("pos_rad", 0.0))
            for n, j in ctl.request("status").get("joints", {}).items()}
    center = [_center_for(n, args.center, held) for n in selected]
    print("centers: " + ", ".join(f"{n} {c:+.3f}" for n, c in zip(selected, center)))
    loop = ipc.RateLoop(args.rate, name="demo")
    t0 = time.monotonic()
    last_print = 0.0
    ramp_out_started = None

    try:
        while True:
            t = time.monotonic() - t0
            # smooth ramp-in over 2 s; ramp-out over the final second
            gain = _smoothstep(t / 2.0)
            if t >= args.secs and ramp_out_started is None:
                ramp_out_started = t
            if ramp_out_started is not None:
                gain *= 1.0 - _smoothstep((t - ramp_out_started) / 1.0)
                if t - ramp_out_started >= 1.0:
                    break
            wave = args.amp * gain * math.sin(2.0 * math.pi * args.freq * t)

            targets = [ipc.NO_COMMAND] * ipc.N_JOINTS
            for k, i in enumerate(idx):
                targets[i] = center[k] + wave
            cmd_pub.send(ipc.MotorCommands(target_pos=targets))

            if t - last_print >= 0.5:
                st = state_sub.latest()
                if st is not None:
                    got = " ".join(f"{n}={st.pos[i]:+.3f}" for n, i in zip(selected, idx))
                    print(f"  t={t:5.1f}s cmd={wave + center[0]:+.3f} | {got}")
                last_print = t
            loop.sleep()
    except KeyboardInterrupt:
        print("\ninterrupted — disabling.")
    finally:
        try:
            ctl.request("disable")
        except TimeoutError:
            print("WARNING: disable request timed out — check the service!")
        print(loop.stats_str())
        cmd_pub.close()
        state_sub.close()
        ctl.close()
        print("done.")


if __name__ == "__main__":
    main()
