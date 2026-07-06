"""
zmq_hz.py — measure a dodo topic's rate, latency and jitter (our `ros2 topic hz`).

    python3 tools/zmq_hz.py motor_states
    python3 tools/zmq_hz.py imu --window 5

Per window prints:
    rate     messages/s actually received
    drops    seq gaps (messages published but not received)
    age      publish→receive latency via t_mono_ns — SAME-HOST ONLY (monotonic
             clocks aren't comparable across machines; cross-host, read jitter)
    jitter   std-dev of inter-arrival times (scheduling health of the publisher)
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import dodo_ipc as ipc  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("topic", nargs="?", default=ipc.TOPIC_MOTOR_STATES,
                    choices=[ipc.TOPIC_MOTOR_STATES, ipc.TOPIC_MOTOR_COMMANDS,
                             ipc.TOPIC_IMU, ipc.TOPIC_TELEMETRY])
    ap.add_argument("--window", type=float, default=2.0, help="stats window [s]")
    ap.add_argument("--windows", type=int, default=0, metavar="N",
                    help="exit after N windows (0 = run until Ctrl-C)")
    args = ap.parse_args()

    sub = ipc.Subscriber(args.topic, conflate=False)   # must see every message
    print(f"hz {args.topic} @ {ipc.connect_endpoint(args.topic)} "
          f"(window {args.window:.0f}s) — Ctrl-C to stop")

    done = 0
    try:
        while not args.windows or done < args.windows:
            done += 1
            t_end = time.monotonic() + args.window
            arrivals, ages = [], []
            drops = 0
            last_seq = None
            while time.monotonic() < t_end:
                msg = sub.recv(timeout_s=max(0.0, t_end - time.monotonic()))
                if msg is None:
                    continue
                now = time.monotonic()
                arrivals.append(now)
                ages.append(ipc.age_s(msg))
                if last_seq is not None:
                    gap = (msg.seq - last_seq) & 0xFFFFFFFF
                    if gap > 1:
                        drops += gap - 1
                last_seq = msg.seq
            if len(arrivals) < 2:
                print("  no traffic — is the publisher running?")
                continue
            dts = [b - a for a, b in zip(arrivals, arrivals[1:])]
            rate = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
            print(f"  rate {rate:7.1f} Hz | drops {drops:4d} | "
                  f"age avg {1e3 * statistics.fmean(ages):5.2f} ms "
                  f"max {1e3 * max(ages):5.2f} ms | "
                  f"jitter {1e3 * statistics.pstdev(dts):5.2f} ms "
                  f"(dt max {1e3 * max(dts):5.2f} ms)")
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()


if __name__ == "__main__":
    main()
