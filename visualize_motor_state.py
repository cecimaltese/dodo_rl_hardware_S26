"""
visualize_motor_state.py — live MuJoCo mirror of the REAL robot's joint states.

Same hardware setup as read_motor_state.py (Damiao read LIMP, ODrive listen+poll,
read-only — it never commands motion), but instead of printing a table it puppets
the dodo MuJoCo model: move a joint by hand and watch the model move. This is the
visual version of the sign/direction check — if you push a joint the way the sim's
+angle goes and the model swings the OTHER way, that joint's sign is mirrored.

No physics is stepped: hardware positions are written straight into qpos and the
model is re-posed with mj_forward. The base stays where the model puts it (this
tool shows joint states; it does not estimate the base): at the origin for the
default URDF, or at the "home" keyframe of an MJCF passed with --model.

Units / calibration:
  * joint_calib.py       if present (from calibrate_joints.py), its measured motor
                         mapping, signs and zeros are applied:
                             q_sim = to_rad(raw - zero) * sign
  * --zero-start         capture the pose ~1 s after startup as qpos = 0. Replaces
                         the calib ZEROS only; measured signs/mapping still apply.
  * raw fallback         no joint_calib.py: Damiao = rad as-is, ODrive = turns *
                         2*pi (direct-drive assumption, see fleet_config NOTE), no
                         sign layer — the L/R mirroring blocker in fleet_config
                         applies!

Prereqs (buses up, ODrives configured — see fleet_config.py / read_motor_state.py):
  sudo ip link set can_dm up type can bitrate 1000000
  sudo ip link set can_od up type can bitrate 1000000
Then:
  python3 visualize_motor_state.py                  # whole fleet
  python3 visualize_motor_state.py --damiao-only    # just the hips
  python3 visualize_motor_state.py --zero-start     # current pose becomes home
  python3 visualize_motor_state.py --hold           # Damiao resists (real gains)
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import threading
import time
from typing import Dict, Optional, Tuple

import can
import mujoco
import mujoco.viewer
from damiao_motor import DaMiaoController

from damiao_joint import DamiaoJoint
from odrive_can_joint import OdriveCanJoint
from fleet_config import (DM_JOINTS, OD_JOINTS, DM_CHANNEL, OD_CHANNEL, BUSTYPE)
from sim_env import URDF_PATH

TWO_PI = 2.0 * math.pi
FRESH_S = 0.5  # ODrive counts as ONLINE if feedback arrived within this many seconds
DEFAULT_MODEL = URDF_PATH  # collision boxes only; pass --model for a meshed MJCF

# ODrive CAN Simple command id for RTR encoder polling (as in read_motor_state.py).
OD_CMD_GET_ENCODER = 0x09

# Measured sign/zero layer, if calibrate_joints.py has been run. Optional.
try:
    from joint_calib import JOINT_CALIB  # {joint: {motor, sign, zero, raw_lo, raw_hi}}
except ImportError:
    JOINT_CALIB = None


def motor_to_sim(raw_pos: float, raw_vel: float, is_odrive: bool,
                 calib: Optional[dict], use_calib_zero: bool) -> Tuple[float, float]:
    """Convert a motor's native reading (Damiao rad / ODrive turns) to sim radians.

    Applies joint_calib.py's  q_sim = to_rad(raw - zero) * sign  when calib is given;
    otherwise just the unit conversion (sign +1, zero 0).
    """
    zero = calib.get("zero") if (calib and use_calib_zero) else None
    sign = calib["sign"] if calib else 1
    pos = raw_pos - (zero or 0.0)
    if is_odrive:
        pos, raw_vel = pos * TWO_PI, raw_vel * TWO_PI
    return pos * sign, raw_vel * sign


def main():
    ap = argparse.ArgumentParser(description="MuJoCo live view of the real robot's joint states.")
    ap.add_argument("--model", default=str(DEFAULT_MODEL), help=f"MuJoCo model, MJCF or URDF (default: {DEFAULT_MODEL})")
    ap.add_argument("--channel", "--dm-channel", dest="channel", default=DM_CHANNEL,
                    help=f"Damiao bus (default: {DM_CHANNEL})")
    ap.add_argument("--od-channel", dest="od_channel", default=OD_CHANNEL,
                    help=f"ODrive bus (default: {OD_CHANNEL})")
    ap.add_argument("--bustype", default=BUSTYPE)
    ap.add_argument("--hz", type=float, default=60.0, help="read/render rate")
    ap.add_argument("--hold", action="store_true",
                    help="read Damiao with configured kp/kd (motor RESISTS); default is limp")
    ap.add_argument("--zero-start", action="store_true",
                    help="capture the pose ~1 s after startup as qpos=0 (overrides calib zeros)")
    ap.add_argument("--damiao-only", action="store_true")
    ap.add_argument("--odrive-only", action="store_true")
    ap.add_argument("--no-poll", action="store_true",
                    help="don't RTR-poll ODrive encoders; rely on cyclic feedback only")
    args = ap.parse_args()

    do_dm = not args.odrive_only
    do_od = not args.damiao_only

    # ── MuJoCo model ────────────────────────────────────────────────────────────
    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home)  # base at standing height

    # name -> (qpos address, dof address) for every hinge joint in the model.
    jmap: Dict[str, Tuple[int, int]] = {}
    for jid in range(model.njnt):
        if model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            jmap[name] = (model.jnt_qposadr[jid], model.jnt_dofadr[jid])

    # ── connect to hardware (identical pattern to read_motor_state.py) ─────────
    dm_controller: Optional[DaMiaoController] = None
    dm_joints: Dict[str, DamiaoJoint] = {}
    od_bus: Optional[can.BusABC] = None
    od_joints: Dict[str, OdriveCanJoint] = {}
    last_seen: Dict[str, float] = {}
    stop = threading.Event()

    if do_dm and DM_JOINTS:
        dm_controller = DaMiaoController(channel=args.channel, bustype=args.bustype)
        for cfg in DM_JOINTS:
            # Copy before zeroing gains — DM_JOINTS are the shared fleet_config objects.
            if not args.hold:
                cfg = dataclasses.replace(cfg, kp=0.0, kd=0.0)
            dm_joints[cfg.name] = DamiaoJoint(dm_controller, cfg)

    if do_od and OD_JOINTS:
        filters = [{"can_id": c.node_id << 5, "can_mask": 0x7E0} for c in OD_JOINTS]
        od_bus = can.Bus(channel=args.od_channel, interface=args.bustype, can_filters=filters)
        od_by_node: Dict[int, OdriveCanJoint] = {}
        name_by_node = {c.node_id: c.name for c in OD_JOINTS}
        for cfg in OD_JOINTS:
            j = OdriveCanJoint(od_bus, cfg)
            od_joints[cfg.name] = j
            od_by_node[cfg.node_id] = j
            last_seen[cfg.name] = 0.0

        def _reader():
            while not stop.is_set():
                msg = od_bus.recv(timeout=0.1)
                if msg is None:
                    continue
                j = od_by_node.get(msg.arbitration_id >> 5)
                if j is not None:
                    j.process_frame(msg)
                    last_seen[name_by_node[msg.arbitration_id >> 5]] = time.monotonic()

        threading.Thread(target=_reader, daemon=True).start()

    for j in dm_joints.values():
        j.enable()
    if dm_joints:
        print("Damiao enabled " + ("in HOLD mode (will resist)." if args.hold
                                   else "LIMP (kp=kd=0 — move joints by hand)."))

    # Which sim joint reads which motor: joint_calib's measured mapping if present,
    # else name == name. Also report joints the viz can't drive.
    if JOINT_CALIB:
        print("joint_calib.py found: applying measured motor mapping + signs"
              + (" (zeros replaced by --zero-start capture)." if args.zero_start else " + zeros."))
    else:
        print("No joint_calib.py — RAW readings (no sign layer; L/R mirroring not corrected).")
    sources: Dict[str, str] = {}  # sim joint -> motor name
    for name in jmap:
        motor = (JOINT_CALIB.get(name, {}).get("motor", name) if JOINT_CALIB else name)
        if motor in dm_joints or motor in od_joints:
            sources[name] = motor
    missing = [n for n in jmap if n not in sources]
    if missing:
        print(f"not driven (no motor connected/selected): {', '.join(missing)}")

    zero_at = time.time() + 1.0 if args.zero_start else None
    order = list(sources)  # stable print order
    print("\n" + "  ".join(f"{n:>15s}" for n in order))

    dt = 1.0 / max(1e-3, args.hz)
    last_print = 0.0
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and not stop.is_set():
                t_next = time.time() + dt

                # Trigger fresh feedback: limp/hold query to Damiao, RTR poll to ODrive.
                for j in dm_joints.values():
                    try:
                        j.send_position(j.get_position())
                    except Exception:
                        pass
                if od_bus is not None and not args.no_poll:
                    for cfg in OD_JOINTS:
                        try:
                            od_bus.send(can.Message(
                                arbitration_id=(cfg.node_id << 5) | OD_CMD_GET_ENCODER,
                                data=[], is_remote_frame=True, is_extended_id=False))
                        except Exception:
                            pass

                now = time.monotonic()
                online: Dict[str, bool] = {}
                for name, j in dm_joints.items():
                    try:
                        online[name] = bool(j.motor.get_states())
                    except Exception:
                        online[name] = False
                for name in od_joints:
                    online[name] = (now - last_seen[name]) < FRESH_S

                if zero_at is not None and time.time() >= zero_at:
                    zeroed = [m for m in set(sources.values()) if online.get(m)]
                    for m in zeroed:
                        (dm_joints.get(m) or od_joints[m]).set_zero()
                    print(f"\nzeroed at current pose: {', '.join(zeroed) or 'NOTHING (all offline)'}")
                    zero_at = None

                # Hardware -> qpos. Offline joints keep their last written value.
                row = []
                for name in order:
                    motor = sources[name]
                    if online.get(motor):
                        j = dm_joints.get(motor) or od_joints[motor]
                        pos, vel = motor_to_sim(
                            j.get_position(), j.get_velocity(), motor in od_joints,
                            JOINT_CALIB.get(name) if JOINT_CALIB else None,
                            use_calib_zero=not args.zero_start)
                        qadr, dadr = jmap[name]
                        data.qpos[qadr] = pos
                        data.qvel[dadr] = vel
                        row.append(f"{pos:>+15.3f}")
                    else:
                        row.append(f"{'OFFLINE':>15s}")

                mujoco.mj_forward(model, data)
                viewer.sync()

                if time.time() - last_print > 0.5:
                    print("\r" + "  ".join(row), end="", flush=True)
                    last_print = time.time()
                time.sleep(max(0.0, t_next - time.time()))
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
