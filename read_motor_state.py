"""
read_motor_state.py — bench diagnostic: connect to the motors and read their
PARAMETERS + live STATES. Read-only; it does not command any motion.

This is the first hardware bring-up / sanity step: prove the CAN bus works and
every motor talks, and watch pos / vel / torque (and whatever else the driver
reports) stream in. It scans the WHOLE configured fleet and marks each motor
ONLINE or OFFLINE, so it works with just the bench motors connected now and
scales to the full fleet later — motors that aren't present simply show OFFLINE.

Motors are read LIMP by default:
  * Damiao — enabled but with kp=kd=torque=0, so it reports state while applying
    ~no torque (freely backdrivable). Great for checking joint direction/sign by
    hand: push the joint the way the sim's +angle goes and confirm pos increases.
  * ODrive — pure listen (+ optional RTR polling). Never commanded closed-loop.
Pass --hold to read Damiao with its configured kp/kd instead (it will resist).

Prereqs:
  sudo ip link set can0 up type can bitrate 1000000     # bus up (OS level)
  python3 odrive_can_setup.py                            # once: ODrive -> CAN node/baud/feedback
Then:
  python3 read_motor_state.py                            # scan the whole fleet
  python3 read_motor_state.py --damiao-only --hz 20      # just the Damiao motors, faster
  python3 read_motor_state.py --hold                     # Damiao with real gains (resists)

NOTE (verify against your damiao_motor version): DamiaoJoint.get_state() returns
pos/vel/torque built from motor.get_states(); this tool also dumps the raw
get_states() dict so any extra fields (temperature, error flags, ...) show up.
Deeper static reads (firmware, hard limits via the 0x7FF register) can be added
once we confirm the library's register API.
"""

from __future__ import annotations

import argparse
import threading
import time
from typing import Dict, Optional

import can
from damiao_motor import DaMiaoController

from damiao_joint import DamiaoJoint, DamiaoJointConfig
from odrive_can_joint import OdriveCanJoint, OdriveCanJointConfig

# --- Fleet config (kept in sync with motor_controller.py DM_JOINTS / OD_JOINTS) ---
# Edit here as motors are added to the bench. Names match the URDF / policy joints.
DAMIAO_MOTORS = [
    DamiaoJointConfig(name="hip_left",        motor_id=0x01, feedback_id=0x11, motor_type="4340P"),
    DamiaoJointConfig(name="upper_leg_left",  motor_id=0x02, feedback_id=0x12, motor_type="4340P"),
    DamiaoJointConfig(name="hip_right",       motor_id=0x05, feedback_id=0x15, motor_type="4340P"),
    DamiaoJointConfig(name="upper_leg_right", motor_id=0x06, feedback_id=0x16, motor_type="4340P"),
]
ODRIVE_MOTORS = [
    OdriveCanJointConfig(name="lower_leg_left",  node_id=7,  vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="foot_left",       node_id=8,  vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="lower_leg_right", node_id=9,  vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="foot_right",      node_id=10, vel_limit=10.0, rated_torque=9.0),
]

# ODrive CAN Simple command ids used for RTR polling (see HANDOFF §3).
OD_CMD_GET_ENCODER = 0x09
OD_CMD_GET_TORQUES = 0x1C
FRESH_S = 0.5  # a motor is "ONLINE" if we saw feedback within this many seconds


def _fmt(v: Optional[float], w: int = 8, p: int = 3) -> str:
    return f"{v:+{w}.{p}f}" if isinstance(v, (int, float)) else f"{'--':>{w}}"


def main():
    ap = argparse.ArgumentParser(description="Read parameters + live states from the dodo motors.")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--bustype", default="socketcan")
    ap.add_argument("--hz", type=float, default=5.0, help="print rate")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds; 0 = until Ctrl-C")
    ap.add_argument("--hold", action="store_true",
                    help="read Damiao with configured kp/kd (motor RESISTS); default is limp")
    ap.add_argument("--damiao-only", action="store_true")
    ap.add_argument("--odrive-only", action="store_true")
    ap.add_argument("--no-poll", action="store_true",
                    help="don't RTR-poll ODrive; rely only on cyclic feedback from odrive_can_setup.py")
    args = ap.parse_args()

    do_dm = not args.odrive_only
    do_od = not args.damiao_only

    # ── connect ───────────────────────────────────────────────────────────────
    dm_controller: Optional[DaMiaoController] = None
    dm_joints: Dict[str, DamiaoJoint] = {}
    od_bus: Optional[can.BusABC] = None
    od_joints: Dict[str, OdriveCanJoint] = {}
    od_by_node: Dict[int, OdriveCanJoint] = {}
    last_seen: Dict[str, float] = {}          # name -> monotonic time of last feedback
    stop = threading.Event()

    if do_dm and DAMIAO_MOTORS:
        # damiao_motor 1.0.6 shares one socket for all Damiao motors on this channel.
        dm_controller = DaMiaoController(channel=args.channel, bustype=args.bustype)
        for cfg in DAMIAO_MOTORS:
            j = DamiaoJoint(dm_controller, cfg)
            dm_joints[cfg.name] = j
            last_seen[cfg.name] = 0.0

    if do_od and ODRIVE_MOTORS:
        # Separate, node-filtered socket so we only ingest ODrive frames.
        filters = [{"can_id": c.node_id << 5, "can_mask": 0x7E0} for c in ODRIVE_MOTORS]
        od_bus = can.Bus(channel=args.channel, interface=args.bustype, can_filters=filters)
        for cfg in ODRIVE_MOTORS:
            j = OdriveCanJoint(od_bus, cfg)
            od_joints[cfg.name] = j
            od_by_node[cfg.node_id] = j
            last_seen[cfg.name] = 0.0

        name_by_node = {c.node_id: c.name for c in ODRIVE_MOTORS}

        def _reader():
            while not stop.is_set():
                msg = od_bus.recv(timeout=0.1)
                if msg is None:
                    continue
                node = msg.arbitration_id >> 5
                j = od_by_node.get(node)
                if j is not None:
                    j.process_frame(msg)
                    last_seen[name_by_node[node]] = time.monotonic()

        reader = threading.Thread(target=_reader, daemon=True)
        reader.start()

    # ── static parameters ───────────────────────────────────────────────────────
    print("\n=== MOTOR PARAMETERS (configured) ===")
    print(f"{'joint':16s} {'type':10s} {'ids / node':14s} {'gains'}")
    for cfg in (DAMIAO_MOTORS if do_dm else []):
        print(f"{cfg.name:16s} DM {cfg.motor_type:7s} "
              f"id=0x{cfg.motor_id:02X} fb=0x{cfg.feedback_id:02X}   kp={cfg.kp} kd={cfg.kd}")
    for cfg in (ODRIVE_MOTORS if do_od else []):
        print(f"{cfg.name:16s} ODrive     node={cfg.node_id:<9d} "
              f"vel_lim={cfg.vel_limit} rated_torque={cfg.rated_torque} Nm (gains via USB)")

    # ── enable Damiao for reading (limp unless --hold) ──────────────────────────
    for name, j in dm_joints.items():
        if not args.hold:
            j.cfg.kp = 0.0   # limp: report state, apply ~no torque
            j.cfg.kd = 0.0
        j.enable()
    if dm_joints:
        mode = "HOLD (configured kp/kd — will resist)" if args.hold else "LIMP (kp=kd=0 — backdrivable)"
        print(f"\nDamiao enabled in {mode} mode.")
    if od_joints and not args.no_poll:
        print("ODrive: RTR-polling encoder+torque each tick (also uses cyclic feedback if enabled).")

    # ── live loop ───────────────────────────────────────────────────────────────
    print("\n=== LIVE STATES (Ctrl-C to stop) ===")
    header = f"{'joint':16s} {'state':8s} {'pos':>9s} {'vel':>9s} {'torque':>9s}   raw/extra"
    dt = 1.0 / max(1e-3, args.hz)
    t0 = time.time()
    try:
        while not stop.is_set() and (args.duration == 0.0 or time.time() - t0 < args.duration):
            # Damiao: send a limp/hold query so the motor reports fresh feedback.
            for j in dm_joints.values():
                try:
                    j.send_position(j.get_position())  # uses cfg.kp/kd (0 in limp mode)
                except Exception:
                    pass
            # ODrive: request encoder + torque (harmless if cyclic feedback is already on).
            if od_bus is not None and not args.no_poll:
                for cfg in ODRIVE_MOTORS:
                    for cmd in (OD_CMD_GET_ENCODER, OD_CMD_GET_TORQUES):
                        try:
                            od_bus.send(can.Message(arbitration_id=(cfg.node_id << 5) | cmd,
                                                    data=[], is_remote_frame=True,
                                                    is_extended_id=False))
                        except Exception:
                            pass
            time.sleep(dt * 0.5)  # give feedback a moment to arrive

            now = time.monotonic()
            print(f"\n{header}")
            for name, j in dm_joints.items():
                raw = {}
                try:
                    raw = j.motor.get_states() or {}
                except Exception:
                    pass
                online = bool(raw)
                if online:
                    last_seen[name] = now
                st = j.get_state() if online else {}
                extra = {k: raw[k] for k in raw if k not in ("pos", "vel", "torq")}
                print(f"{name:16s} {'ONLINE' if online else 'OFFLINE':8s} "
                      f"{_fmt(st.get('pos'))} {_fmt(st.get('vel'))} {_fmt(st.get('torque'))}   {extra}")
            for name, j in od_joints.items():
                online = (now - last_seen[name]) < FRESH_S
                st = j.get_state() if online else {}
                print(f"{name:16s} {'ONLINE' if online else 'OFFLINE':8s} "
                      f"{_fmt(st.get('pos'))} {_fmt(st.get('vel'))} {_fmt(st.get('torque'))}   "
                      f"(turns; err={getattr(j, '_axis_error', '?')})")
            time.sleep(dt * 0.5)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nshutting down (disabling motors)...")
        stop.set()
        for j in dm_joints.values():
            try:
                j.disable()
            except Exception:
                pass
        if dm_controller is not None:
            try:
                dm_controller.shutdown()
            except Exception:
                pass
        if od_bus is not None:
            try:
                od_bus.shutdown()
            except Exception:
                pass
        print("done.")


if __name__ == "__main__":
    main()
