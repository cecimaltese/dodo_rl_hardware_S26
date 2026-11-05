"""
odrive_check.py — does an ODrive and its motor work? Let the drive measure it.

Runs one drive's FULL_CALIBRATION_SEQUENCE (the same thing `dodo_ctl odrive_calibrate`
does) directly on the CAN bus and logs it at full rate. Its first step measures the
motor's phase resistance and inductance by pushing current through the windings, which
is exactly what a bad phase wire, crimp or solder joint fails. Then the encoder scan
turns the rotor. Reported per run:

  * each step's duration and the drive's DC BUS CURRENT during the motor step: a
    healthy MN4004 draws ~0.1 A from the 24 V supply for ~4 s (2026-11-05: both
    ankles 0.12 A, 4.2 s). The left knee's open phase drew 0.00 A and failed after
    1.6 s with PHASE_RESISTANCE_OUT_OF_RANGE.
  * how far the rotor swept in the encoder scan (ankles: 0.67 motor turns = 8 deg at
    the joint; the right knee also searches its encoder index: 1.17 turns = 16 deg,
    ending ~7 deg off) and the result + error bits.

THE MOTOR MOVES (a beep, then 8-16 deg at the joint and back): legs hanging free.
Stop motor_service first (this talks to the bus directly; it refuses if a service
answers). A drive calibrated here stays calibrated until power-off, like
odrive_calibrate; afterwards straighten the legs and `calibrate --pose straight`.

    python3 tools/odrive_check.py --joint lower_leg_left
    python3 tools/odrive_check.py --joint lower_leg_left --repeat 5   # wiggle the cable
    python3 tools/odrive_check.py --joint lower_leg_left --compare foot_left

--repeat runs it several times: wiggle the motor cable / connector between runs (or
during the ~4 s motor step) to catch a contact that only works sometimes.
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from pathlib import Path
from typing import Dict, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import can  # noqa: E402

import dodo_ipc as ipc  # noqa: E402
import fleet_config  # noqa: E402
import real2sim  # noqa: E402
from odrive_can_joint import (AXIS_STATE_FULL_CALIBRATION_SEQUENCE,  # noqa: E402
                              AXIS_STATE_IDLE, CMD_CLEAR_ERRORS, CMD_GET_ERROR,
                              CMD_GET_ENCODER_ESTIMATES, CMD_HEARTBEAT,
                              CMD_SET_AXIS_STATE, PROCEDURE_PHASE_RESISTANCE_OUT_OF_RANGE,
                              axis_state_name, error_names, procedure_result_name)

CMD_GET_IQ = 0x14
CMD_GET_BUS_VOLTAGE_CURRENT = 0x17
POLL_S = 0.02                 # RTR-poll bus V/I, Iq, errors (some drives don't stream them)
TIMEOUT_S = 40.0
MOTOR_STEP_MIN_A = 0.03       # a motor step drawing less than this pushed no real current
HEALTHY = "healthy (2026-11-05): motor step ~4.2 s at ~0.12 A; encoder scan ~9.2 s"


def node_of(joint: str) -> int:
    for c in fleet_config.OD_JOINTS:
        if c.name == joint:
            return c.node_id
    raise SystemExit(f"{joint} isn't an ODrive joint ({[c.name for c in fleet_config.OD_JOINTS]})")


def check_drive(bus: can.BusABC, node: int, joint: str, timeout_s: float = TIMEOUT_S,
                out=print) -> Dict:
    """Clear the drive's errors, run its full calibration, log it. Returns the facts."""
    def send(cmd, data=b"", rtr=False):
        bus.send(can.Message(arbitration_id=(node << 5) | cmd, data=data,
                             is_remote_frame=rtr, dlc=8 if rtr else len(data),
                             is_extended_id=False))

    st = {"state": None, "proc": None, "err": 0, "pos": None, "ibus": None, "vbus": None,
          "iq": None, "act": 0, "dis": 0}

    def pump(t):
        end = time.monotonic() + t
        next_poll = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_poll:
                for c in (CMD_GET_BUS_VOLTAGE_CURRENT, CMD_GET_IQ, CMD_GET_ERROR):
                    send(c, rtr=True)
                next_poll = time.monotonic() + POLL_S
            m = bus.recv(0.005)
            if m is None or m.is_remote_frame or (m.arbitration_id >> 5) != node:
                continue
            c, d = m.arbitration_id & 0x1F, bytes(m.data)
            if c == CMD_HEARTBEAT and len(d) >= 6:
                st["err"], st["state"], st["proc"] = struct.unpack("<IBB", d[:6])
                yield
            elif c == CMD_GET_ENCODER_ESTIMATES and len(d) >= 8:
                st["pos"] = struct.unpack("<f", d[:4])[0]
            elif c == CMD_GET_BUS_VOLTAGE_CURRENT and len(d) >= 8:
                st["vbus"], st["ibus"] = struct.unpack("<ff", d[:8])
            elif c == CMD_GET_IQ and len(d) >= 8:
                st["iq"] = struct.unpack("<ff", d[:8])[1]
            elif c == CMD_GET_ERROR and len(d) >= 8:
                st["act"], st["dis"] = struct.unpack("<II", d[:8])

    for _ in pump(0.4):
        pass
    if st["state"] is None:
        raise RuntimeError(f"node {node} ({joint}) is silent: powered? on {bus.channel_info}?")
    send(CMD_CLEAR_ERRORS, bytes([0]))
    for _ in pump(0.2):
        pass
    p0 = st["pos"]
    lo = hi = p0
    send(CMD_SET_AXIS_STATE, struct.pack("<I", AXIS_STATE_FULL_CALIBRATION_SEQUENCE))
    t0 = time.monotonic()
    steps = []                         # [state, t_start, t_end, ibus samples]
    left_idle = False
    for _ in pump(timeout_s):
        t = time.monotonic() - t0
        if st["pos"] is not None:
            lo, hi = min(lo, st["pos"]), max(hi, st["pos"])
        if not steps or steps[-1][0] != st["state"]:
            if steps:
                steps[-1][2] = t
            steps.append([st["state"], t, None, []])
        if st["ibus"] is not None:
            steps[-1][3].append(abs(st["ibus"]))
        if st["state"] != AXIS_STATE_IDLE:
            left_idle = True
        if left_idle and st["state"] == AXIS_STATE_IDLE and t > 0.5:
            break
    else:
        send(CMD_SET_AXIS_STATE, struct.pack("<I", AXIS_STATE_IDLE))
        raise RuntimeError(f"node {node} ({joint}): no end after {timeout_s:.0f} s")
    if steps:
        steps[-1][2] = time.monotonic() - t0
    for _ in pump(0.3):
        pass

    report = {"joint": joint, "node": node, "result": procedure_result_name(st["proc"]),
              "success": st["proc"] == 0 and left_idle,
              "errors": error_names(st["err"] | st["act"] | st["dis"]),
              "rotor_turns": None if p0 is None else hi - lo,
              "end_offset_turns": None if p0 is None else st["pos"] - p0,
              "steps": [], "vbus": st["vbus"]}
    for state, ta, tb, ib in steps:
        if state == AXIS_STATE_IDLE:
            continue
        report["steps"].append({"state": axis_state_name(state), "s": tb - ta,
                                "ibus_mean": sum(ib) / len(ib) if ib else None,
                                "ibus_peak": max(ib) if ib else None})
    motor = [s for s in report["steps"] if s["state"] == "MOTOR_CALIBRATION"]
    report["motor_step_ibus"] = motor[0]["ibus_peak"] if motor else None
    if report["success"]:
        report["verdict"] = "PASS"
    elif st["proc"] == PROCEDURE_PHASE_RESISTANCE_OUT_OF_RANGE:
        report["verdict"] = ("FAIL: no usable phase resistance"
                             + (" and no current drawn" if (report["motor_step_ibus"] or 0)
                                < MOTOR_STEP_MIN_A else "")
                             + ": a phase wire / crimp / solder joint / connector between "
                               "drive and motor is open or bad")
    else:
        report["verdict"] = f"FAIL: {report['result']} ({report['errors']})"

    deg = (lambda turns: math.degrees(turns * 2 * math.pi / real2sim.gear_ratio(joint)))
    out(f"  {joint} (node {node}), bus {st['vbus'] or float('nan'):.1f} V:")
    for s in report["steps"]:
        cur = ("" if s["ibus_peak"] is None else
               f"  bus current mean {s['ibus_mean']:.3f} A, peak {s['ibus_peak']:.3f} A")
        out(f"    {s['state']:28s} {s['s']:5.1f} s{cur}")
    if report["rotor_turns"] is not None:
        out(f"    rotor swept {report['rotor_turns']:.2f} turns = {deg(report['rotor_turns']):.1f}"
            f" deg at the joint, ended {deg(report['end_offset_turns']):+.1f} deg from start")
    out(f"    result {report['result']}, errors {report['errors']}  ->  {report['verdict']}")
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", required=True)
    ap.add_argument("--repeat", type=int, default=1, help="runs (wiggle the cable between)")
    ap.add_argument("--compare", default=None,
                    help="first run a known-good drive the same way, as the baseline")
    ap.add_argument("--channel", default=fleet_config.OD_CHANNEL)
    ap.add_argument("--bustype", default=fleet_config.BUSTYPE)
    ap.add_argument("--yes", action="store_true", help="don't ask before moving motors")
    args = ap.parse_args()
    joints = ([args.compare] if args.compare else []) + [args.joint]
    nodes = {j: node_of(j) for j in joints}

    ctl = ipc.CtlClient()
    try:
        ctl.request("status", timeout_s=0.3)
        sys.exit("a service answers on ctl (motor_service?): stop it first, this tool "
                 "talks to the drives directly")
    except TimeoutError:
        pass
    finally:
        ctl.close()

    print(f"Calibrating {', '.join(joints)}: each motor beeps, then turns 8-16 deg at "
          f"the joint and back. Legs hanging FREE.\n  {HEALTHY}")
    if not args.yes and input("go? [y/N] ").strip().lower() not in ("y", "yes"):
        sys.exit("aborted (nothing moved)")
    bus = can.Bus(channel=args.channel, interface=args.bustype,
                  can_filters=[{"can_id": n << 5, "can_mask": 0x7E0} for n in nodes.values()])
    results = []
    try:
        if args.compare:
            print("\nbaseline:")
            check_drive(bus, nodes[args.compare], args.compare)
        for k in range(args.repeat):
            print(f"\nrun {k + 1}/{args.repeat}:")
            results.append(check_drive(bus, nodes[args.joint], args.joint))
            if k + 1 < args.repeat:
                input("  wiggle the cable/connector if you like, then Enter for the "
                      "next run... ")
    except (RuntimeError, KeyboardInterrupt) as e:
        bus.send(can.Message(arbitration_id=(nodes[args.joint] << 5) | CMD_SET_AXIS_STATE,
                             data=struct.pack("<I", AXIS_STATE_IDLE), is_extended_id=False))
        print(f"\nstopped: {e}")
    finally:
        bus.shutdown()
    if results:
        ok = sum(r["success"] for r in results)
        print(f"\n{args.joint}: {ok}/{len(results)} runs passed"
              + ("" if ok == len(results) else
                 " — an intermittent or bad contact if some passed") + ".")
        if ok:
            print("Next: straighten the legs, then motor_service, calibrate --pose straight, "
                  "and the in-air tests (README Bring-up 5-6).")


if __name__ == "__main__":
    main()
