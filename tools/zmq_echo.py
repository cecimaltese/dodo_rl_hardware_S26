"""
zmq_echo.py — print messages from any dodo topic (our `ros2 topic echo`).

    python3 tools/zmq_echo.py motor_states
    python3 tools/zmq_echo.py motor_commands --every 10
    python3 tools/zmq_echo.py imu --latest
    DODO_ZMQ_HOST=<robot-ip> python3 tools/zmq_echo.py motor_states   # from a laptop

By default sees EVERY message (conflate off); --latest switches to
newest-only, i.e. exactly what a control-loop subscriber would see.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import dodo_ipc as ipc  # noqa: E402


def fmt_vec(v, nd=3):
    return "[" + " ".join(f"{x:+.{nd}f}" for x in v) + "]"


def fmt(msg) -> str:
    head = f"seq={msg.seq:<8d} age={ipc.age_s(msg) * 1e3:6.1f}ms"
    if isinstance(msg, ipc.MotorStates):
        flags = " ".join(
            f"{name}:{'E' if f & ipc.JF_ENABLED else '.'}"
            f"{'C' if f & ipc.JF_CALIBRATED else '.'}"
            f"{'I' if f & ipc.JF_IDLED else '.'}"
            f"{'F' if f & ipc.JF_FAULT else '.'}"
            for name, f in zip(ipc.JOINT_ORDER, msg.joint_flags))
        glob = (f"{'ESTOP ' if msg.estopped else ''}"
                f"{'enabled' if msg.enabled else 'disabled'}"
                f"{' all-cal' if msg.all_calibrated else ''}")
        return (f"{head} [{glob}]\n"
                f"  pos {fmt_vec(msg.pos)}\n"
                f"  vel {fmt_vec(msg.vel)}\n"
                f"  tau {fmt_vec(msg.torque, 2)}\n"
                f"  {flags}")
    if isinstance(msg, ipc.MotorCommands):
        return f"{head} targets {fmt_vec(msg.target_pos)}"
    if isinstance(msg, ipc.ImuSample):
        extra = ""
        if msg.quat_valid:
            extra += f" quat {fmt_vec(msg.quat)}"
        if msg.lin_vel_valid:
            extra += f" lin_vel {fmt_vec(msg.lin_vel)} (SIM)"
        return f"{head} gyro {fmt_vec(msg.gyro)} accel {fmt_vec(msg.accel, 2)}{extra}"
    return f"{head} {msg}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("topic", nargs="?", default=ipc.TOPIC_MOTOR_STATES,
                    choices=[ipc.TOPIC_MOTOR_STATES, ipc.TOPIC_MOTOR_COMMANDS,
                             ipc.TOPIC_IMU, ipc.TOPIC_TELEMETRY])
    ap.add_argument("--latest", action="store_true",
                    help="newest-only (what a control loop sees) instead of every message")
    ap.add_argument("--every", type=int, default=1, metavar="N",
                    help="print only every Nth message (default 1)")
    ap.add_argument("--count", type=int, default=0, metavar="N",
                    help="exit after printing N messages (0 = run until Ctrl-C)")
    args = ap.parse_args()

    sub = ipc.Subscriber(args.topic, conflate=args.latest)
    print(f"echo {args.topic} @ {ipc.connect_endpoint(args.topic)} "
          f"({'latest-only' if args.latest else 'every message'}) — Ctrl-C to stop")
    n = printed = 0
    try:
        while True:
            msg = sub.recv(timeout_s=1.0)
            if msg is None:
                print("  (no messages ... is the publisher running?)")
                continue
            n += 1
            if n % args.every == 0:
                print(fmt(msg))
                printed += 1
                if args.count and printed >= args.count:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()


if __name__ == "__main__":
    main()
