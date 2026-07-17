"""
odrive_can_scan.py — find live nodes on the CAN bus (diagnostic tool).

Two phases:
  1. PASSIVE listen — watch for cyclic feedback (heartbeat / encoder / torque).
     A configured ODrive on CAN Simple flooding at ~10 ms rates shows up here
     immediately, and we learn its node id for free.
  2. ACTIVE sweep — for every node id 0..63, send an RTR query for
     Get_Encoder_Estimates (cmd 0x09) and wait briefly for a reply. Any node
     that answers is alive even if its cyclic rates are disabled.

CAN Simple arbitration id = (node_id << 5) | cmd_id  (11-bit standard frame),
so node_id is 6 bits (0..63) and cmd_id is 5 bits (0..31).

Usage:
    python3 odrive_can_scan.py                 # scan can0
    python3 odrive_can_scan.py --channel can0 --listen 3.0

If NOTHING responds in either phase, the problem is physical (power, 120 Ohm
termination, CAN_H/CAN_L swap/break) or a bitrate mismatch — not software.
"""

import argparse
import struct
import time
from collections import defaultdict

import can

CMD_HEARTBEAT = 0x01
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_GET_TORQUES = 0x1C
CMD_NAMES = {0x01: "Heartbeat", 0x09: "Encoder", 0x1C: "Torques"}

NODE_ID_MAX = 63  # 6-bit node id in CAN Simple


def _node_of(arb_id: int) -> int:
    return (arb_id >> 5) & 0x3F


def _cmd_of(arb_id: int) -> int:
    return arb_id & 0x1F


def passive_listen(bus: can.BusABC, seconds: float) -> dict:
    """Listen without transmitting; tally frames per node id."""
    print(f"\n[1] Passive listen for {seconds:.1f}s (no TX) ...")
    seen = defaultdict(lambda: defaultdict(int))  # node -> cmd -> count
    t_end = time.time() + seconds
    total = 0
    while time.time() < t_end:
        msg = bus.recv(timeout=max(0.0, t_end - time.time()))
        if msg is None:
            continue
        # ignore our own echoed frames if the driver reflects them
        if getattr(msg, "is_rx", True) is False:
            continue
        total += 1
        seen[_node_of(msg.arbitration_id)][_cmd_of(msg.arbitration_id)] += 1

    if not seen:
        print(f"    NOTHING heard ({total} frames). Bus is silent.")
        return {}
    for node in sorted(seen):
        parts = ", ".join(
            f"{CMD_NAMES.get(c, hex(c))}={n}" for c, n in sorted(seen[node].items())
        )
        print(f"    node {node:2d}: {parts}")
    return seen


def active_sweep(bus: can.BusABC, per_node_wait: float) -> list:
    """RTR-query every node id; collect which ones answer."""
    print(f"\n[2] Active sweep of node ids 0..{NODE_ID_MAX} "
          f"(RTR Get_Encoder_Estimates, {per_node_wait*1000:.0f} ms each) ...")
    responders = []
    for node in range(NODE_ID_MAX + 1):
        arb = (node << 5) | CMD_GET_ENCODER_ESTIMATES
        # drain any stale frames
        while bus.recv(timeout=0.0) is not None:
            pass
        try:
            bus.send(can.Message(arbitration_id=arb, is_extended_id=False,
                                 is_remote_frame=True, data=[]))
        except can.CanError as exc:
            print(f"    node {node:2d}: TX error ({exc}) — bus not ACKing")
            continue

        t_end = time.time() + per_node_wait
        answered = None
        while time.time() < t_end:
            msg = bus.recv(timeout=max(0.0, t_end - time.time()))
            if msg is None:
                break
            if msg.is_remote_frame:            # ignore our own echoed RTR
                continue
            if _node_of(msg.arbitration_id) != node:
                continue
            if _cmd_of(msg.arbitration_id) == CMD_GET_ENCODER_ESTIMATES and len(msg.data) >= 8:
                pos, vel = struct.unpack("<ff", msg.data[:8])
                answered = (pos, vel)
                break
        if answered is not None:
            pos, vel = answered
            print(f"    node {node:2d}: ALIVE  pos={pos:+.4f} turns  vel={vel:+.4f} turns/s")
            responders.append(node)
    return responders


def main():
    ap = argparse.ArgumentParser(description="Scan a CAN bus for live ODrive nodes.")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--bustype", default="socketcan")
    ap.add_argument("--listen", type=float, default=2.0, help="passive listen seconds")
    ap.add_argument("--wait", type=float, default=0.03, help="per-node reply wait (s)")
    args = ap.parse_args()

    print(f"Opening {args.channel} ({args.bustype}) ...")
    bus = can.Bus(channel=args.channel, interface=args.bustype)
    try:
        heard = passive_listen(bus, args.listen)
        responders = active_sweep(bus, args.wait)
    finally:
        bus.shutdown()

    print("\n=== summary ===")
    nodes = sorted(set(heard) | set(responders))
    if nodes:
        for n in nodes:
            tags = []
            if n in heard:
                tags.append("cyclic-feedback")
            if n in responders:
                tags.append("answered-RTR")
            print(f"  node {n}: {', '.join(tags)}")
    else:
        print("  No nodes found. If everything is powered, the fault is physical:")
        print("    - 120 Ohm termination missing (measure ~60 Ohm across CAN_H/CAN_L, power off)")
        print("    - CAN_H/CAN_L swapped or a broken wire")
        print("    - ODrive baud != 1 Mbit, or protocol != Protocol.SIMPLE (run odrive_can_setup.py over USB)")


if __name__ == "__main__":
    main()
