"""
visualize_motor_state.py — live MuJoCo mirror of the REAL robot's joint states.

The real2sim check: move a joint by hand and the meshed model moves with it. If you
push a joint the way the sim's +angle goes and the model swings the OTHER way, that
joint's sign is mirrored; if it moves the right way but too little or too much, the
gear ratio or the zero is off. Read-only: Damiao read LIMP (enabled at kp=kd=0, no
torque), ODrive listen + RTR poll. It never commands motion.

Every reading goes through real2sim.JointCal — the same map calibrate_joints.py
writes and motor_service will use:

    q = sign * (raw - zero) * scale        scale: Damiao 1, ODrive 2*pi / gear ratio

  * joint_calib.py present: its measured motor mapping, signs and zeros. Entries made
    with a gear ratio that no longer matches fleet_config.GEAR_RATIO are refused.
  * otherwise (or for a refused joint): identity — motor named like the joint, sign
    +1, zero 0. Marked * in the header. Unmeasured gear ratio: marked r.
  * --zero-start: ~1 s after start, the current pose reads 0 for every joint (zeros
    only; measured signs/mapping/ratios still apply). Hang the legs straight first.

While it runs it records the range each joint reached. Push joints to their stops
and the exit summary lists them next to the URDF and fleet_config limits: the first
look at the real joint limits (calibrate_joints.py --limits records them properly).

No physics is stepped: readings go straight into qpos and the model is re-posed with
mj_forward. The base is fixed at the origin (this shows joints, not the base).

Prereqs: buses up (setup/99-dodo-can.rules does it), ODrives configured; stop
motor_service and the other direct-CAN tools first. Then:
  python3 visualize_motor_state.py                  # whole fleet
  python3 visualize_motor_state.py --damiao-only    # just the hips
  python3 visualize_motor_state.py --zero-start     # current pose becomes q = 0
  python3 visualize_motor_state.py --hold           # Damiao resists (real gains)
  python3 visualize_motor_state.py --boxes          # show the collision boxes too
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import os
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import mujoco

import real2sim
from fleet_config import (DM_JOINTS, OD_JOINTS, DM_CHANNEL, OD_CHANNEL, BUSTYPE,
                          FEEDBACK_FRESH_S, JOINT_LIMITS_RAD)
import assets
from sim_env import URDF_PATH

HIDDEN_GROUP = 3   # the viewer shows geom groups 0-2 by default

# Short column labels, in the policy's joint order.
LABELS = {"hip_left": "hip_L", "hip_right": "hip_R",
          "upper_leg_left": "thigh_L", "upper_leg_right": "thigh_R",
          "lower_leg_left": "knee_L", "lower_leg_right": "knee_R",
          "foot_left": "ankle_L", "foot_right": "ankle_R"}


def build_view_model(urdf_path=URDF_PATH, boxes: bool = False) -> mujoco.MjModel:
    """The URDF with its STL meshes, for looking at. MuJoCo drops a URDF's visual
    meshes unless told otherwise at parse time, so the URDF is parsed from memory with
    discardvisual="false" and the meshes passed as assets; the file itself (and the
    physics models sim_env/sim_service build from it) are untouched. The collision
    boxes move to a hidden geom group unless `boxes` (toggle in the viewer: 3)."""
    urdf_path = assets.require(urdf_path)
    text = urdf_path.read_text()
    tag = '<compiler fusestatic="false" />'
    if tag not in text:
        raise ValueError(f"{urdf_path}: expected the {tag} line of the <mujoco> block")
    text = text.replace(tag, '<compiler fusestatic="false" discardvisual="false" />', 1)
    mesh_dir = urdf_path.parent.parent / "meshes"
    mesh_files = {f"../meshes/{p.name}": p.read_bytes() for p in mesh_dir.glob("*.STL")}
    model = mujoco.MjSpec.from_string(text, assets=mesh_files).compile()
    if not boxes:
        for g in range(model.ngeom):
            if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                model.geom_group[g] = HIDDEN_GROUP
    return model


def hinge_map(model) -> Dict[str, Tuple[int, int]]:
    """name -> (qpos address, dof address) for every hinge joint."""
    out = {}
    for jid in range(model.njnt):
        if model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            out[name] = (int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid]))
    return out


def urdf_range(model, joint: str) -> Optional[Tuple[float, float]]:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    if jid < 0 or not model.jnt_limited[jid]:
        return None
    lo, hi = model.jnt_range[jid]
    return float(lo), float(hi)


def range_summary(seen: Dict[str, list], model) -> str:
    """The exit table: reached range per joint vs the URDF and fleet_config limits."""
    lines = [f"  {'joint':17s} {'reached [deg]':>17s}   {'URDF [deg]':>17s}   "
             f"{'fleet_config [deg]':>18s}"]
    deg = math.degrees
    for j, (lo, hi) in seen.items():
        u = urdf_range(model, j)
        f = JOINT_LIMITS_RAD.get(j)
        lines.append(
            f"  {j:17s} {deg(lo):+7.1f} .. {deg(hi):+6.1f}   "
            + (f"{deg(u[0]):+7.1f} .. {deg(u[1]):+6.1f}" if u else f"{'--':>17s}")
            + "   " + (f"{deg(f[0]):+7.1f} .. {deg(f[1]):+6.1f}" if f else f"{'--':>17s}"))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=None,
                    help="another MuJoCo model (MJCF/URDF) instead of the meshed URDF")
    ap.add_argument("--boxes", action="store_true", help="also show the collision boxes")
    ap.add_argument("--channel", "--dm-channel", dest="channel", default=DM_CHANNEL,
                    help=f"Damiao bus (default: {DM_CHANNEL})")
    ap.add_argument("--od-channel", dest="od_channel", default=OD_CHANNEL,
                    help=f"ODrive bus (default: {OD_CHANNEL})")
    ap.add_argument("--bustype", default=BUSTYPE)
    ap.add_argument("--hz", type=float, default=60.0, help="read/render rate")
    ap.add_argument("--hold", action="store_true",
                    help="read Damiao with configured kp/kd (motor RESISTS); default is limp")
    ap.add_argument("--zero-start", action="store_true",
                    help="~1 s after start, the current pose reads 0 (zeros only)")
    ap.add_argument("--damiao-only", action="store_true")
    ap.add_argument("--odrive-only", action="store_true")
    ap.add_argument("--no-poll", action="store_true",
                    help="don't RTR-poll ODrive encoders; rely on cyclic feedback only")
    args = ap.parse_args()

    # Hardware imports here, so build_view_model() is importable without them.
    import can
    import mujoco.viewer
    from damiao_motor import DaMiaoController
    from damiao_joint import DamiaoJoint
    from odrive_can_joint import OdriveCanJoint

    do_dm = not args.odrive_only
    do_od = not args.damiao_only

    # ── model ────────────────────────────────────────────────────────────────────
    if args.model:
        model = mujoco.MjModel.from_xml_path(args.model)
    else:
        model = build_view_model(boxes=args.boxes)
    data = mujoco.MjData(model)
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home)
    jmap = hinge_map(model)

    # ── the reading -> angle maps ────────────────────────────────────────────────
    cals, warnings = real2sim.load()
    for w in warnings:
        print(f"  ! {w}")

    # ── connect (same pattern as read_motor_state.py) ────────────────────────────
    dm_controller = None
    motors: Dict[str, object] = {}         # motor name -> DamiaoJoint / OdriveCanJoint
    od_bus = None
    reader = None
    stop = threading.Event()
    if do_dm and DM_JOINTS:
        dm_controller = DaMiaoController(channel=args.channel, bustype=args.bustype)
        for cfg in DM_JOINTS:
            # Copy before zeroing gains: DM_JOINTS are the shared fleet_config objects.
            if not args.hold:
                cfg = dataclasses.replace(cfg, kp=0.0, kd=0.0)
            motors[cfg.name] = DamiaoJoint(dm_controller, cfg)
    if do_od and OD_JOINTS:
        filters = [{"can_id": c.node_id << 5, "can_mask": 0x7E0} for c in OD_JOINTS]
        od_bus = can.Bus(channel=args.od_channel, interface=args.bustype,
                         can_filters=filters)
        od_by_node = {}
        for cfg in OD_JOINTS:
            motors[cfg.name] = od_by_node[cfg.node_id] = OdriveCanJoint(od_bus, cfg)

        def _reader():
            while not stop.is_set():
                msg = od_bus.recv(timeout=0.1)
                if msg is not None and (j := od_by_node.get(msg.arbitration_id >> 5)):
                    j.process_frame(msg)
        reader = threading.Thread(target=_reader, daemon=True)
        reader.start()

    dm_motors = [m for n, m in motors.items() if not real2sim.is_odrive(n)]
    od_motors = [m for n, m in motors.items() if real2sim.is_odrive(n)]
    for j in dm_motors:
        j.enable()
    if dm_motors:
        print("Damiao enabled " + ("in HOLD mode (will resist)." if args.hold
                                   else "LIMP (kp=kd=0 — move joints by hand)."))

    order = [j for j in LABELS if j in jmap] + [j for j in jmap if j not in LABELS]
    shown = [j for j in order if j in cals and cals[j].motor in motors]
    missing = [j for j in order if j not in shown]
    if missing:
        print(f"not driven (no motor connected/selected): {', '.join(missing)}")
    marks = {j: ("" if cals[j].measured else "*") + ("" if real2sim.ratio_known(j) else "r")
             for j in shown}
    print("  * = not calibrated (identity map)   r = gear ratio not measured   "
          "! = outside the URDF range")
    print("\n" + " ".join(f"{LABELS.get(j, j[:10]) + marks[j]:>10s}" for j in shown)
          + "   [rad]")

    zero_at = time.monotonic() + 1.0 if args.zero_start else None
    seen: Dict[str, list] = {}
    dt = 1.0 / max(1e-3, args.hz)
    last_print = 0.0
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and not stop.is_set():
                t_next = time.monotonic() + dt
                # Ask for fresh feedback: a limp (or hold) frame to each Damiao, which
                # only answers frames, and an RTR to each ODrive.
                for j in dm_motors:
                    try:
                        j.send_position(j.get_position())
                    except Exception:
                        pass
                if not args.no_poll:
                    for j in od_motors:
                        try:
                            j.request_feedback()
                        except Exception:
                            pass

                def fresh(m) -> bool:
                    age = motors[m].feedback_age()
                    return age is not None and age < FEEDBACK_FRESH_S

                if zero_at is not None and time.monotonic() >= zero_at:
                    zeroed = []
                    for j in shown:
                        if fresh(cals[j].motor):
                            cals[j] = cals[j].zeroed_at(motors[cals[j].motor].get_position())
                            zeroed.append(j)
                    print(f"\nzeroed at the current pose: {', '.join(zeroed) or 'NOTHING'}")
                    zero_at = None

                row = []
                for j in shown:
                    cal = cals[j]
                    if not fresh(cal.motor):
                        row.append(f"{'OFFLINE':>10s}")      # model keeps the last value
                        continue
                    m = motors[cal.motor]
                    q = cal.to_sim(m.get_position())
                    qadr, dadr = jmap[j]
                    data.qpos[qadr] = q
                    data.qvel[dadr] = cal.vel_to_sim(m.get_velocity())
                    s = seen.setdefault(j, [q, q])
                    s[0], s[1] = min(s[0], q), max(s[1], q)
                    u = urdf_range(model, j)
                    out = u is not None and not (u[0] <= q <= u[1])
                    row.append(f"{q:>+9.3f}{'!' if out else ' '}")

                mujoco.mj_forward(model, data)
                viewer.sync()
                if time.monotonic() - last_print > 0.5:
                    print("\r" + " ".join(row), end="", flush=True)
                    last_print = time.monotonic()
                time.sleep(max(0.0, t_next - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        print("\nshutting down (disabling Damiao read mode)...")
        stop.set()
        for j in dm_motors:
            try:
                j.disable()
            except Exception:
                pass
        if dm_controller is not None:
            try:
                dm_controller.shutdown()
            except Exception:
                pass
        if reader is not None:
            reader.join(timeout=1.0)          # it reads od_bus: stop it before closing
        if od_bus is not None:
            try:
                od_bus.shutdown()
            except Exception:
                pass
        if seen:
            print("\nRange each joint reached this session (sim angles, through real2sim):")
            print(range_summary(seen, model))
            if any(not cals[j].measured or not real2sim.ratio_known(j) for j in seen):
                print("  (* / r joints: uncalibrated or unknown ratio, so their numbers "
                      "are raw-ish, not sim angles yet)")
        print("done.", flush=True)
        # The MuJoCo GLFW viewer can crash in interpreter teardown (seen: a core dump
        # after "done." on X11; run_policy.py has the same workaround). Everything is
        # flushed and shut down, so leave without the crashy global cleanup.
        os._exit(0)


if __name__ == "__main__":
    main()
