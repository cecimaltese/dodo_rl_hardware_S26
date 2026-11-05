"""
visualize_hardware.py — live MuJoCo mirror of the REAL robot: its joints and its IMU.
(Was visualize_motor_state.py until 2026-11-05, when the IMU was added.)

The real2sim check: move a joint by hand and the meshed model moves with it. If you
push a joint the way the sim's +angle goes and the model swings the OTHER way, that
joint's sign is mirrored; if it moves the right way but too little or too much, the
gear ratio or the zero is off. Tilt the robot and the model's body tilts with it: if
it tilts the other way, or about the wrong axis, AXIS_REMAP is wrong. Read-only:
Damiao read LIMP (enabled at kp=kd=0, no torque), ODrive listen + RTR poll, the IMU
polled like imu_service. It never commands motion.

JOINTS. Every reading goes through real2sim.JointCal — the same map calibrate_joints.py
writes and motor_service will use:

    q = sign * (raw - zero) * scale        scale: Damiao 1, ODrive 2*pi / gear ratio

  * joint_calib.py present: its measured motor mapping, signs and zeros. Entries made
    with a gear ratio that no longer matches fleet_config.GEAR_RATIO are refused.
  * otherwise (or for a refused joint): identity — motor named like the joint, sign
    +1, zero 0. Marked * in the header. Unmeasured gear ratio: marked r.
  * --zero-start: ~1 s after start, the current pose reads 0 for every joint (zeros
    only; measured signs/mapping/ratios still apply). Hang the legs straight first.

IMU. The Nicla through the deployment path (imu_protocol: firmware scales, AXIS_REMAP
into the body frame), so the model shows what the policy will see. The body is posed
by the TILT only (roll and pitch from the gravity direction): the fused quaternion
has no magnetometer, so its heading is arbitrary and drifts (--imu-yaw keeps it). It
uses the fused quaternion when the firmware sends one (v2), else the accelerometer.
Also shown: the tilt as nose-down / right-down degrees (the words imu_locate.py uses)
and the gyro in body axes (deg/s): a nose-down turn is +y, a right-side-down turn +x.

While it runs it records the range each joint reached and the largest tilt. Push
joints to their stops and the exit summary lists them next to the URDF and
fleet_config limits (calibrate_joints.py --limits records stops properly).

No physics is stepped: readings go straight into qpos and the model is re-posed with
mj_forward. The base stays at the origin; only its orientation follows the IMU.

Prereqs: buses up (setup/99-dodo-can.rules does it), ODrives configured; stop
motor_service, imu_service and the other direct-CAN / serial tools first. Then:
  python3 visualize_hardware.py                  # joints + IMU
  python3 visualize_hardware.py --no-imu         # joints only (the old tool)
  python3 visualize_hardware.py --imu-only       # IMU only, no CAN (also on a Mac:
                                                 #   --imu-port /dev/cu.usbmodem*)
  python3 visualize_hardware.py --damiao-only    # just the hips (+ IMU)
  python3 visualize_hardware.py --zero-start     # current pose becomes q = 0
  python3 visualize_hardware.py --hold           # Damiao resists (real gains)
  python3 visualize_hardware.py --boxes          # show the collision boxes too
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import os
import sys
import threading
import time
from typing import Dict, Optional, Tuple

import mujoco
import numpy as np

import assets
import real2sim
from fleet_config import (DM_JOINTS, OD_JOINTS, DM_CHANNEL, OD_CHANNEL, BUSTYPE,
                          FEEDBACK_FRESH_S, JOINT_LIMITS_RAD)
from rl_env import projected_gravity_from_quat
from sim_env import URDF_PATH

HIDDEN_GROUP = 3   # the viewer shows geom groups 0-2 by default
IMU_FRESH_S = 0.2  # an IMU reading older than this is not shown (imu_service polls ~60 Hz)

# Short column labels, in the policy's joint order.
LABELS = {"hip_left": "hip_L", "hip_right": "hip_R",
          "upper_leg_left": "thigh_L", "upper_leg_right": "thigh_R",
          "lower_leg_left": "knee_L", "lower_leg_right": "knee_R",
          "foot_left": "ankle_L", "foot_right": "ankle_R"}


def build_view_model(urdf_path=URDF_PATH, boxes: bool = False,
                     free_base: bool = True) -> mujoco.MjModel:
    """The URDF with its STL meshes, for looking at. MuJoCo drops a URDF's visual
    meshes unless told otherwise at parse time, so the URDF is parsed from memory with
    discardvisual="false" and the meshes passed as assets; the file itself (and the
    physics models sim_env/sim_service build from it) are untouched. The collision
    boxes move to a hidden geom group unless `boxes` (toggle in the viewer: 3).
    `free_base` gives `body` a free joint, so the IMU can tilt it (MuJoCo's URDF
    import welds the root to the world)."""
    urdf_path = assets.require(urdf_path)
    text = urdf_path.read_text()
    tag = '<compiler fusestatic="false" />'
    if tag not in text:
        raise ValueError(f"{urdf_path}: expected the {tag} line of the <mujoco> block")
    text = text.replace(tag, '<compiler fusestatic="false" discardvisual="false" />', 1)
    mesh_dir = urdf_path.parent.parent / "meshes"
    mesh_files = {f"../meshes/{p.name}": p.read_bytes() for p in mesh_dir.glob("*.STL")}
    spec = mujoco.MjSpec.from_string(text, assets=mesh_files)
    if free_base:
        spec.body("body").add_freejoint()
    model = spec.compile()
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


def base_qpos_adr(model) -> Optional[int]:
    """qpos address of the free joint on `body` (its quat is the next 4 entries after
    the position's 3), or None if the model's base can't move."""
    for jid in range(model.njnt):
        if (model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE
                and mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                      model.jnt_bodyid[jid]) == "body"):
            return int(model.jnt_qposadr[jid])
    return None


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


# ── IMU ────────────────────────────────────────────────────────────────────────────

def tilt_from_gravity(g) -> Tuple[float, float]:
    """(pitch, roll) [rad] of the zero-heading orientation R = Ry(pitch) Rx(roll) whose
    body-frame gravity is `g` = R^T (0, 0, -1) = (sin p, -sin r cos p, -cos r cos p).
    pitch > 0 is nose-down, roll > 0 right-side-down."""
    g = np.asarray(g, dtype=float)
    g = g / np.linalg.norm(g)
    return (math.asin(max(-1.0, min(1.0, g[0]))), math.atan2(-g[1], -g[2]))


def quat_from_tilt(pitch: float, roll: float) -> np.ndarray:
    """(w, x, y, z) of Ry(pitch) Rx(roll)."""
    qy = np.array([math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0])
    qx = np.array([math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0])
    q = np.zeros(4)
    mujoco.mju_mulQuat(q, qy, qx)
    return q


def base_quat(reading: dict, keep_yaw: bool = False) -> Tuple[np.ndarray, str]:
    """The base orientation to show for one imu_protocol reading (body frame), and
    where it came from. The tilt only, unless keep_yaw: the fused quaternion's heading
    is arbitrary (no magnetometer). No quaternion (firmware v1): the accelerometer's
    gravity direction, which is only right while the robot isn't accelerating."""
    if "quat" in reading:
        q = np.asarray(reading["quat"], dtype=float)
        q = q / np.linalg.norm(q)
        if keep_yaw:
            return q, "fused"
        return quat_from_tilt(*tilt_from_gravity(projected_gravity_from_quat(q))), "fused"
    a = np.asarray(reading["accel"], dtype=float)
    return quat_from_tilt(*tilt_from_gravity(-a)), "accel"


class ImuFeed:
    """Polls the Nicla in a thread, like imu_service (one 'L', one reply), and keeps
    the latest reading in the body frame (imu_protocol.parse_imu_line). `ser` is an
    open port, or anything with reset_input_buffer/write/readline (tests: a fake)."""

    def __init__(self, ser, ver: Optional[dict]):
        from imu_service import sensor_scales
        self.ser = ser
        self.gyro_scale, self.accel_scale, self.scale_src = sensor_scales(ver)
        self.ver = ver
        self.n = 0
        self._latest: Optional[Tuple[float, dict]] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.t_start = time.monotonic()

    @classmethod
    def open(cls, port: str, baud: int = 0) -> "ImuFeed":
        from imu_protocol import open_imu
        ser, _, ver = open_imu(port, baud or None)
        ser.timeout = 0.05
        return cls(ser, ver)

    def poll_once(self) -> Optional[dict]:
        from imu_protocol import parse_imu_line
        self.ser.reset_input_buffer()
        self.ser.write(b"L")
        raw = self.ser.readline()
        line = raw.decode("ascii", errors="ignore") if isinstance(raw, bytes) else raw
        r = parse_imu_line(line, self.gyro_scale, self.accel_scale) if line else None
        if r is not None:
            self._latest = (time.monotonic(), r)
            self.n += 1
        return r

    def start(self) -> "ImuFeed":
        def loop():
            while not self._stop.is_set():
                try:
                    self.poll_once()
                except Exception:
                    time.sleep(0.05)
        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()
        return self

    def latest(self) -> Optional[Tuple[float, dict]]:
        """(age [s], reading) of the newest reading, or None before the first."""
        got = self._latest
        return None if got is None else (time.monotonic() - got[0], got[1])

    def rate_hz(self) -> float:
        return self.n / max(1e-6, time.monotonic() - self.t_start)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass


def imu_text(reading: dict, quat: np.ndarray, source: str) -> str:
    pitch, roll = tilt_from_gravity(projected_gravity_from_quat(quat))
    w = np.degrees(reading["gyro"])
    return (f"IMU ({source}): nose-down {math.degrees(pitch):+5.1f}  right-down "
            f"{math.degrees(roll):+5.1f} deg   gyro x {w[0]:+6.1f} y {w[1]:+6.1f} "
            f"z {w[2]:+6.1f} deg/s")


# ── main ───────────────────────────────────────────────────────────────────────────

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
    ap.add_argument("--imu-port", default="/dev/ttyACM0",
                    help="Nicla serial port (Mac: /dev/cu.usbmodem*)")
    ap.add_argument("--imu-baud", type=int, default=0,
                    help="0 = auto-detect (v2: 921600, v1: 115200)")
    ap.add_argument("--no-imu", action="store_true", help="joints only")
    ap.add_argument("--imu-only", action="store_true",
                    help="IMU only: open no CAN bus (works without the motors)")
    ap.add_argument("--imu-yaw", action="store_true",
                    help="also show the fused heading (arbitrary and drifting: no "
                         "magnetometer)")
    args = ap.parse_args()
    if args.imu_only and args.no_imu:
        sys.exit("--imu-only and --no-imu: nothing to show")

    import mujoco.viewer

    do_dm = not (args.odrive_only or args.imu_only)
    do_od = not (args.damiao_only or args.imu_only)

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
    base_adr = base_qpos_adr(model)

    # ── IMU ──────────────────────────────────────────────────────────────────────
    imu: Optional[ImuFeed] = None
    if not args.no_imu:
        try:
            imu = ImuFeed.open(args.imu_port, args.imu_baud).start()
            print(f"IMU on {args.imu_port}: sensor ranges from {imu.scale_src}"
                  + ("" if base_adr is not None else " (this model's base can't tilt: "
                     "numbers only)"))
        except (RuntimeError, AttributeError, OSError) as e:
            msg = (f"IMU on {args.imu_port} unavailable: {e}\n  Is imu_service.py (or a "
                   f"serial monitor) holding the port? `ls /dev/ttyACM*`")
            if args.imu_only:
                sys.exit(msg)
            print(f"  ! {msg}\n  continuing with the joints only.")

    # ── the reading -> angle maps ────────────────────────────────────────────────
    cals, warnings = real2sim.load()
    if do_dm or do_od:
        for w in warnings:
            print(f"  ! {w}")

    # ── connect (same pattern as read_motor_state.py) ────────────────────────────
    dm_controller = None
    motors: Dict[str, object] = {}         # motor name -> DamiaoJoint / OdriveCanJoint
    od_bus = None
    reader = None
    stop = threading.Event()
    if do_dm and DM_JOINTS:
        from damiao_motor import DaMiaoController
        from damiao_joint import DamiaoJoint
        dm_controller = DaMiaoController(channel=args.channel, bustype=args.bustype)
        for cfg in DM_JOINTS:
            # Copy before zeroing gains: DM_JOINTS are the shared fleet_config objects.
            if not args.hold:
                cfg = dataclasses.replace(cfg, kp=0.0, kd=0.0)
            motors[cfg.name] = DamiaoJoint(dm_controller, cfg)
    if do_od and OD_JOINTS:
        import can
        from odrive_can_joint import OdriveCanJoint
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
    if missing and (do_dm or do_od):
        print(f"not driven (no motor connected/selected): {', '.join(missing)}")
    marks = {j: ("" if cals[j].measured else "*") + ("" if real2sim.ratio_known(j) else "r")
             for j in shown}
    if shown:
        print("  * = not calibrated (identity map)   r = gear ratio not measured   "
              "! = outside the URDF range")
        print("\n" + " ".join(f"{LABELS.get(j, j[:10]) + marks[j]:>10s}" for j in shown)
              + "   [rad]")

    zero_at = time.monotonic() + 1.0 if args.zero_start and shown else None
    seen: Dict[str, list] = {}
    max_tilt = {"deg": 0.0, "nose_down": 0.0, "right_down": 0.0}
    dt = 1.0 / max(1e-3, args.hz)
    last_print = 0.0
    n_lines = 0
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            overlay = hasattr(viewer, "set_texts")
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

                row, ov_labels, ov_values = [], [], []
                for j in shown:
                    cal = cals[j]
                    ov_labels.append(LABELS.get(j, j))
                    if not fresh(cal.motor):
                        row.append(f"{'OFFLINE':>10s}")      # model keeps the last value
                        ov_values.append("OFFLINE")
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
                    ov_values.append(f"{q:+.3f} rad  {math.degrees(q):+6.1f} deg"
                                     + ("  !" if out else ""))

                lines = [" ".join(row)] if row else []
                if imu is not None:
                    got = imu.latest()
                    if got is not None and got[0] < IMU_FRESH_S:
                        reading = got[1]
                        quat, src = base_quat(reading, keep_yaw=args.imu_yaw)
                        if base_adr is not None:
                            data.qpos[base_adr + 3:base_adr + 7] = quat
                        text = imu_text(reading, quat, src)
                        p, r = tilt_from_gravity(projected_gravity_from_quat(quat))
                        tilt = math.degrees(math.acos(max(-1.0, min(1.0,
                                            math.cos(p) * math.cos(r)))))
                        if tilt > max_tilt["deg"]:
                            max_tilt.update(deg=tilt, nose_down=math.degrees(p),
                                            right_down=math.degrees(r))
                    else:
                        text = ("IMU: no reading yet" if got is None
                                else f"IMU: STALE ({got[0]:.1f} s)")
                    lines.append(text)
                    ov_labels.append("IMU")
                    ov_values.append(text.split(": ", 1)[1] if ": " in text else text)

                mujoco.mj_forward(model, data)
                if overlay:
                    viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                                      "\n".join(ov_labels), "\n".join(ov_values)))
                viewer.sync()
                if time.monotonic() - last_print > 0.5 and lines:
                    # One terminal line per source, rewritten in place.
                    up = f"\033[{n_lines - 1}A" if n_lines > 1 else ""
                    print(up + "\r" + "\n".join("\033[K" + ln for ln in lines),
                          end="", flush=True)
                    n_lines = len(lines)
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
        if imu is not None:
            imu.close()
            print(f"\nIMU: {imu.n} readings, {imu.rate_hz():.0f} Hz; largest tilt "
                  f"{max_tilt['deg']:.1f} deg ({max_tilt['nose_down']:+.1f} nose-down, "
                  f"{max_tilt['right_down']:+.1f} right-down)")
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
