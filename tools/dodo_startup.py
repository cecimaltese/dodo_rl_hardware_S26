"""
dodo_startup.py — rehearse the full power-on routine, WITHOUT a policy.

The startup convention (no homing — a homing drive could overload the
3D-printed structure/belts): the robot is POWERED ON with the ODrive joints
folded all the way back against their mechanical stops. This tool then:

    1. status      sanity: who is there, what do they read
    2. calibrate   map ODrive turns-since-boot onto the known boot pose
                   (skipped if already calibrated)
    3. enable      bumpless; the service slowly unparks joints into the
                   safety band
    4. ramp        smoothstep all joints to the stand pose (--pose: a dodo_assets
                   scene.xml keyframe, `home` by default, or `default` = the
                   vendored stand policy's rl_env.DEFAULT_JOINT_POS)
    5. hold        stand there, printing state, until Ctrl-C / --hold-secs
    6. park        ramp gently BACK toward the boot pose, then disable —
                   the legs fold instead of collapsing (skip: --no-park)

This is exactly what `run_policy.py --backend zmq` does in reset() (steps 4–5
become the policy); rehearse here first. Works against sim_service too.

    python3 tools/dodo_startup.py                 # full ritual, asks first
    python3 tools/dodo_startup.py --ramp 8 --yes  # slower, no questions
    python3 tools/dodo_startup.py --pose straight # any scene.xml keyframe
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import assets  # noqa: E402
import dodo_ipc as ipc  # noqa: E402
from rl_env import DEFAULT_JOINT_POS  # noqa: E402


def stand_pose(name: str) -> dict:
    """{joint: rad} for --pose: `default` = the vendored stand policy's default pose,
    anything else a keyframe of dodo_assets' scene.xml (Shafeef's `home`, `straight`)."""
    return dict(DEFAULT_JOINT_POS) if name == "default" else assets.scene_pose(name)


def smoothstep(s: float) -> float:
    s = min(1.0, max(0.0, s))
    return 3.0 * s * s - 2.0 * s * s * s


def fresh_states(sub: ipc.Subscriber, timeout_s: float = 2.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        m = sub.latest()
        if m is not None:
            return m
        time.sleep(0.02)
    raise SystemExit("no motor_states — is motor_service (or sim_service) running?")


def start_targets(ctl: ipc.CtlClient, states: ipc.MotorStates, present) -> list:
    """Where the first ramp starts: each joint's held TARGET (status `target_rad`), else
    its measured angle. Not the measured angles alone: under load a PD-held joint sags
    below its target, and commanding the sagged angle drops the preload that holds the
    robot up (in sim, 0.7 deg at the ankles tips the PD stand over). Right after a
    bumpless enable the two agree anyway."""
    joints = ctl.request("status").get("joints", {})
    out = []
    for i, name in enumerate(ipc.JOINT_ORDER):
        if name not in present:
            out.append(ipc.NO_COMMAND)
            continue
        t = joints.get(name, {}).get("target_rad")
        out.append(states.pos[i] if t is None else float(t))
    return out


def ramp_to(pub: ipc.Publisher, sub: ipc.Subscriber, loop: ipc.RateLoop,
            q0, q1, duration_s: float, label: str) -> None:
    """Publish smoothstepped wire targets q0 -> q1 (NaN slots stay NaN)."""
    n = max(1, int(round(duration_s / loop.period_s)))
    print(f"[{label}] {duration_s:.1f}s ...")
    for k in range(n + 1):
        a = smoothstep(k / n)
        wire = [q0[i] + a * (q1[i] - q0[i])
                if not (math.isnan(q0[i]) or math.isnan(q1[i])) else ipc.NO_COMMAND
                for i in range(ipc.N_JOINTS)]
        pub.send(ipc.MotorCommands(target_pos=wire))
        st = sub.latest()
        if st is not None and st.estopped:
            raise SystemExit(f"[{label}] service E-STOPPED mid-move — check its log")
        loop.sleep()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", default="home",
                    help="stand pose: a scene.xml keyframe (home, straight) or 'default' "
                         "(the vendored stand policy's). Default: home")
    ap.add_argument("--ramp", type=float, default=5.0,
                    help="seconds for the start->stand pose (and park) ramp")
    ap.add_argument("--hold-secs", type=float, default=0.0,
                    help="hold this long, then park (0 = hold until Ctrl-C)")
    ap.add_argument("--rate", type=float, default=50.0, help="command rate [Hz]")
    ap.add_argument("--no-park", action="store_true",
                    help="on exit just disable (legs go limp) instead of "
                         "folding back to the start pose first")
    ap.add_argument("--yes", action="store_true", help="don't ask before enabling")
    args = ap.parse_args()
    try:
        pose = stand_pose(args.pose)
    except (KeyError, FileNotFoundError) as e:
        raise SystemExit(str(e))
    print(f"stand pose '{args.pose}': " + ", ".join(
        f"{j} {pose[j]:+.2f}" for j in ipc.JOINT_ORDER if j in pose))

    ctl = ipc.CtlClient()

    # 1) status ---------------------------------------------------------------
    st = ctl.request("status")
    if not st.get("ok"):
        raise SystemExit(f"status failed: {st}")
    joints = st.get("joints", {})
    print(f"service: enabled={st.get('enabled')} estopped={st.get('estopped')} "
          f"all_calibrated={st.get('all_calibrated')}")
    for name, j in sorted(joints.items()):
        print(f"  {name:16s} {j.get('type', '?'):7s} pos {j.get('pos_rad', 0.0):+7.3f} rad  "
              f"fb={j.get('has_feedback')} cal={j.get('calibrated')}")
    if st.get("estopped"):
        raise SystemExit("service is E-STOPPED — fix the cause, then "
                         "tools/dodo_ctl.py estop_reset")

    # 2) calibrate (boot pose) --------------------------------------------------
    if not st.get("all_calibrated"):
        print("\ncalibrating: assuming the robot was POWERED ON with the legs "
              "folded at the stops.")
        rep = ctl.request("calibrate")
        if not rep.get("ok"):
            raise SystemExit(f"calibrate failed: {rep.get('error')}")
        print(f"  assumed angles: {rep.get('assumed_rad')}")
        st = ctl.request("status")
        for name, j in sorted(st.get("joints", {}).items()):
            print(f"  {name:16s} now reads {j.get('pos_rad', 0.0):+7.3f} rad")
        print("  ^ VERIFY these match reality before continuing!")

    # 3) enable -----------------------------------------------------------------
    if not args.yes:
        ans = input(f"\nEnable motors and ramp to '{args.pose}'? [y/N] ")
        if ans.strip().lower() not in ("y", "yes"):
            raise SystemExit("aborted (nothing enabled).")
    rep = ctl.request("enable")
    if not rep.get("ok"):
        raise SystemExit(f"enable failed: {rep.get('error')}")
    unparking = rep.get("unparking") or {}
    if unparking:
        wait = max(u["duration_s"] for u in unparking.values()) + 0.4
        print(f"unparking {sorted(unparking)} — waiting {wait:.1f}s")
        time.sleep(wait)

    # 4) ramp to default pose -----------------------------------------------------
    pub = ipc.Publisher(ipc.TOPIC_MOTOR_COMMANDS)   # we are THE commander now
    sub = ipc.Subscriber(ipc.TOPIC_MOTOR_STATES, conflate=True)
    loop = ipc.RateLoop(args.rate, name="dodo_startup")
    time.sleep(0.2)                                  # let SUB connect

    states = fresh_states(sub)
    present = set(joints)
    q_start = start_targets(ctl, states, present)
    q_home = [pose[ipc.JOINT_ORDER[i]] if ipc.JOINT_ORDER[i] in present else ipc.NO_COMMAND
              for i in range(ipc.N_JOINTS)]

    try:
        ramp_to(pub, sub, loop, q_start, q_home, args.ramp, f"ramp to '{args.pose}'")

        # 5) hold ---------------------------------------------------------------
        print(f"[hold] at '{args.pose}' — Ctrl-C to fold back down." if
              not args.hold_secs else f"[hold] {args.hold_secs:.0f}s ...")
        t0 = time.monotonic()
        last_print = 0.0
        while not args.hold_secs or time.monotonic() - t0 < args.hold_secs:
            pub.send(ipc.MotorCommands(target_pos=q_home))
            st_msg = sub.latest()
            if st_msg is not None:
                if st_msg.estopped:
                    raise SystemExit("[hold] service E-STOPPED — check its log")
                if time.monotonic() - last_print > 0.5:
                    err = [abs(st_msg.pos[i] - q_home[i])
                           for i in range(ipc.N_JOINTS)
                           if not math.isnan(q_home[i])]
                    print(f"\r[hold] max pose error {max(err):+.3f} rad ",
                          end="", flush=True)
                    last_print = time.monotonic()
            loop.sleep()
        print()
    except KeyboardInterrupt:
        print()
    finally:
        try:
            if not args.no_park:
                # From the targets being held now, not the measured (sagged) angles.
                q_now = start_targets(ctl, fresh_states(sub), present)
                ramp_to(pub, sub, loop, q_now, q_start, args.ramp,
                        "park (folding back)")
        except KeyboardInterrupt:
            print("\npark interrupted — disabling now.")
        except SystemExit as e:
            print(e)
        finally:
            try:
                ctl.request("disable", timeout_s=1.0)
                print("motors disabled.")
            except Exception as e:
                print(f"disable failed ({e}) — the service watchdog will e-stop.")
            for s in (pub, sub, ctl):
                try:
                    s.close()
                except Exception:
                    pass


if __name__ == "__main__":
    main()
