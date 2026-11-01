"""
imu_locate.py — measure where the IMU sits on the robot and write it everywhere it's needed.

The Nicla Sense ME is bolted to `body`. Training needs its pose in the URDF (`imu_link` +
fixed `imu_joint`); deployment needs the same rotation as `AXIS_REMAP` in
imu_protocol.py. Both are generated from ONE record, `description/imu_mount.json`, so
they can't drift apart.

  ORIENTATION  measured by the IMU itself. At rest the accelerometer reads +1 g along
               whichever board axis points up, so three poses of the robot (level,
               nose-down, left-side-down) pin the board axes in the body frame. The
               level pose fixes "up", the nose-down pose fixes the heading, and the
               left-side-down pose is an independent cross-check that catches a pose
               done the wrong way round.
  POSITION     measured by hand (ruler/caliper, ±5 mm is plenty) relative to the
               hip-pitch axis. An IMU can't find its own lever arm without a rotation
               rig, and the policy doesn't need it: angular velocity and the gravity
               direction are the same everywhere on a rigid body. Only raw
               accelerometer readings depend on where the chip sits.

Usage — robot PC, IMU on USB, motors unpowered. Step by step: docs/imu_locate.md.
    python3 imu_locate.py measure                    # poses + tape measure -> write -> check
    python3 imu_locate.py measure --skip-position    # redo only the orientation
    python3 imu_locate.py measure --skip-orientation # redo only the position
    python3 imu_locate.py measure --xyz-mm 12 0 110  # position already known (body frame)
    python3 imu_locate.py apply                      # record -> URDF, AXIS_REMAP, README
    python3 imu_locate.py check                      # do all of them agree, MuJoCo too?
    python3 imu_locate.py check --live               # + tilt the robot, watch the policy's view
"""

from __future__ import annotations

import argparse
import ast
import datetime
import importlib
import itertools
import json
import math
import operator
import re
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import imu_protocol as proto
from sim_env import URDF_PATH, _build_model

REPO = Path(__file__).resolve().parent
RECORD_PATH = REPO / "description" / "imu_mount.json"
README_PATH = REPO / "description" / "README.md"
PROTOCOL_PATH = REPO / "imu_protocol.py"

# Where world-up points in the BODY frame (x forward, y left, z up) in each pose.
# Nose-down puts +x toward the floor, so up is -x; left-side-down puts +y toward the
# floor, so up is -y.
POSES = (
    ("level", (0.0, 0.0, 1.0),
     "LEVEL: body upright and level, as if standing on flat ground. Rest it on a flat\n"
     "    table or on equal blocks; a phone spirit level on a flat body face helps."),
    ("nose_down", (-1.0, 0.0, 0.0),
     "NOSE-DOWN ~90 deg: pitch it forward until the FRONT face points at the floor."),
    ("left_down", (0.0, -1.0, 0.0),
     "LEFT-SIDE-DOWN ~90 deg: roll it until the robot's LEFT side points at the floor\n"
     "    (the robot's own left: stand behind it, looking where it looks)."),
)
POSE_UP = {name: np.array(up) for name, up, _ in POSES}

STILL_DPS = 8.0            # mean rotation rate above which a pose reading is redone
SNAP_LIMIT_DEG = 10.0      # max snapped-vs-exact gap still treated as a square mount
CROSSCHECK_LIMIT_DEG = 20.0

# Nicla Sense ME: a 22.86 x 22.86 mm board. The mass (board + mount) is a guess; at a
# few grams on a 4.7 kg robot it only exists so importers that keep fixed links don't
# invent one. Weigh it and edit the record if you care. The thin side is along the
# chip's z, which is the board normal for a chip soldered flat.
IMU_MASS_KG = 0.005
IMU_BOX_M = (0.023, 0.023, 0.005)

URDF_BEGIN = "  <!-- BEGIN imu_locate.py"
URDF_END = "  <!-- END imu_locate.py -->\n"
_URDF_BLOCK_RE = re.compile(re.escape(URDF_BEGIN) + r".*?" + re.escape(URDF_END), re.S)
_REMAP_RE = re.compile(r"^AXIS_REMAP = (\(\(.*?\)\))\n", re.S | re.M)
_README_POS_RE = re.compile(r"^\| Position in `body` frame \|.*$", re.M)
_README_ROT_RE = re.compile(r"^\| Orientation \(`imu_joint` rpy\) \|.*$", re.M)


# --- rotation math --------------------------------------------------------------
# Convention everywhere: R's rows are the body axes in board terms, so
# v_body = R @ v_board. That is AXIS_REMAP's convention, and exactly what a URDF joint
# rotation means (child imu_link -> parent body).

def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("zero vector")
    return v / n


def angle_deg(a, b) -> float:
    return math.degrees(math.acos(float(np.clip(_unit(a) @ _unit(b), -1.0, 1.0))))


def rotation_deg(ra, rb) -> float:
    """Angle of the rotation that takes ra to rb."""
    c = (np.trace(np.asarray(ra).T @ np.asarray(rb)) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


def _signed_permutations():
    """The 24 rotations that map axes onto axes (signed permutations with det +1)."""
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            m = np.zeros((3, 3))
            for row, (col, s) in enumerate(zip(perm, signs)):
                m[row, col] = s
            if np.linalg.det(m) > 0:
                out.append(m)
    return out


SIGNED_PERMUTATIONS = _signed_permutations()


def is_proper_rotation(r, tol=1e-6) -> bool:
    r = np.asarray(r, dtype=float)
    return (r.shape == (3, 3) and np.allclose(r @ r.T, np.eye(3), atol=tol)
            and abs(np.linalg.det(r) - 1.0) < tol)


def matrix_from_rpy(roll, pitch, yaw) -> np.ndarray:
    """URDF rpy: extrinsic x, then y, then z, i.e. R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def rpy_from_matrix(r) -> tuple:
    """Inverse of matrix_from_rpy. Square mountings often hit gimbal lock (pitch
    ±90 deg, e.g. the chip's x along body z); there roll and yaw are one degree of
    freedom, so yaw is set to 0 and roll takes all of it."""
    r = np.asarray(r, dtype=float)
    if abs(r[2, 0]) < 1.0 - 1e-9:
        rpy = (math.atan2(r[2, 1], r[2, 2]), math.asin(-r[2, 0]),
               math.atan2(r[1, 0], r[0, 0]))
    else:
        rpy = (math.atan2(-r[2, 0] * r[0, 1], r[1, 1]),
               -math.copysign(math.pi / 2, r[2, 0]), 0.0)
    if not np.allclose(matrix_from_rpy(*rpy), r, atol=1e-6):
        raise ValueError(f"rpy round trip failed for\n{r}")
    return rpy


def _flat(v, z) -> np.ndarray:
    """v with its component along z removed, normalised."""
    return _unit(v - (v @ z) * z)


def solve_orientation(ups: dict, exact: bool = False):
    """Board -> body rotation from the board-frame UP vector of each pose.

    The level pose fixes body z. Each tilted pose then gives the heading about z on
    its own (nose-down: body x; left-side-down: body y, hence x = y × z). Their
    disagreement is the cross-check, their average the heading. Neither tilted pose
    needs to be exactly 90 deg: only a sideways lean in it moves the heading.

    Returns (R, info, problems). By default R is snapped to the nearest square
    mounting (one of 24 axis permutations): a board screwed to a plate is square to
    within a degree or two, which hand-held poses are not. `exact` keeps the fitted
    rotation instead, for a board that really is mounted at an angle.
    """
    problems = []
    for a, b in (("level", "nose_down"), ("level", "left_down"), ("nose_down", "left_down")):
        sep = angle_deg(ups[a], ups[b])
        if not 45.0 < sep < 135.0:
            problems.append(f"the {a} and {b} readings are {sep:.0f} deg apart; they "
                            f"should be ~90. Was one pose skipped or held wrong?")
    if problems:
        return None, {}, problems

    z = _unit(ups["level"])
    x_nose = _flat(-np.asarray(ups["nose_down"]), z)          # nose-down: up = -x
    x_left = np.cross(_flat(-np.asarray(ups["left_down"]), z), z)  # left-down: up = -y
    cross = angle_deg(x_nose, x_left)
    if cross > CROSSCHECK_LIMIT_DEG:
        return None, {}, [
            f"the nose-down and left-side-down poses disagree by {cross:.0f} deg about "
            f"which way is forward. ~180 deg means one went the wrong way: pitched nose "
            f"UP, or rolled RIGHT side down."]
    x = _flat(x_nose + x_left, z)
    fit = np.vstack([x, np.cross(z, x), z])
    snapped = max(SIGNED_PERMUTATIONS, key=lambda p: np.trace(p.T @ fit))
    skew = rotation_deg(snapped, fit)
    r = fit if exact else snapped

    if not exact and skew > SNAP_LIMIT_DEG:
        problems.append(
            f"the nearest square mounting is {skew:.0f} deg from the measured one. Either "
            f"a pose was far from its target, or the board really is mounted at an angle. "
            f"Redo the poses; if it stays, re-run with --exact.")

    info = {
        "mode": "exact" if exact else "snapped",
        "snap_gap_deg": round(skew, 2),
        "crosscheck_deg": round(cross, 2),
        "level_tilt_deg": round(angle_deg(r.T @ POSE_UP["level"], ups["level"]), 2),
        "up_board": {k: [round(float(c), 4) for c in v] for k, v in ups.items()},
    }
    return r, info, problems


_BODY_DIRS = {(1, 0, 0): "forward", (-1, 0, 0): "back", (0, 1, 0): "left",
              (0, -1, 0): "right", (0, 0, 1): "up", (0, 0, -1): "down"}


def describe_axes(r) -> str:
    """'chip +x -> left, chip +y -> back, chip +z -> up' (columns of R)."""
    parts = []
    for i, name in enumerate("xyz"):
        col = np.asarray(r, dtype=float)[:, i]
        key = max(_BODY_DIRS, key=lambda d: np.dot(d, col))
        off = angle_deg(key, col)
        parts.append(f"chip +{name} -> {_BODY_DIRS[key]}"
                     + (f" ({off:.1f} deg off)" if off > 0.05 else ""))
    return ", ".join(parts)


def format_axis_remap(r) -> str:
    r = np.asarray(r, dtype=float)
    if np.allclose(r, np.round(r)):
        cells = [[f"{int(round(v)):2d}" for v in row] for row in r]
    else:
        cells = [[f"{v:+.9f}" for v in row] for row in r]
    rows = ["(" + ", ".join(c) + ")" for c in cells]
    return "AXIS_REMAP = (" + (",\n" + " " * len("AXIS_REMAP = (")).join(rows) + ")\n"


# --- the IMU at rest ------------------------------------------------------------------

def _sample(ser, n) -> np.ndarray:
    """n raw readings: board frame, unscaled counts, as parse_raw_line returns them.
    Raw on purpose: this measures what AXIS_REMAP must be, so it can't go through it."""
    out, deadline = [], time.time() + 5.0 + n * 0.05
    while len(out) < n and time.time() < deadline:
        ser.reset_input_buffer()
        ser.write(b"L")
        v = proto.parse_raw_line(ser.readline().decode("ascii", errors="ignore"))
        if v:
            out.append(v[:6])
    if len(out) < max(1, n // 2):
        raise RuntimeError(f"the IMU answered {len(out)}/{n} polls. Still plugged in?")
    return np.array(out)


def read_up(ser, ver, n=60):
    """Board-frame unit vector pointing UP, and the mean rotation rate in deg/s."""
    s = _sample(ser, n)
    dps_per_count = (ver["gyro_range_dps"] if ver else proto.GYRO_RANGE_DPS) / 32768.0
    rate = float(np.linalg.norm(s[:, 3:6], axis=1).mean() * dps_per_count)
    return _unit(s[:, 0:3].mean(axis=0)), rate


def _board_axis(up) -> str:
    i = int(np.argmax(np.abs(up)))
    return f"chip {'+' if up[i] > 0 else '-'}{'xyz'[i]} up, {abs(up[i]) * 100:.0f}%"


def report_orientation(r, info):
    print(f"\n  RESULT ({'exact fit' if info['mode'] == 'exact' else 'nearest square mounting'}):")
    print(f"    {describe_axes(r)}")
    print("    " + format_axis_remap(r).replace("\n", "\n    ").rstrip())
    print(f"    imu_joint rpy = {' '.join(_num(a, 6) for a in rpy_from_matrix(r))} rad")
    print("  Quality:")
    print(f"    level reading vs the result's z axis  {info['level_tilt_deg']:5.1f} deg"
          f"   board tilt on its mount + body not quite level")
    print(f"    snapped vs exact fit                  {info['snap_gap_deg']:5.1f} deg"
          f"   < {SNAP_LIMIT_DEG:.0f}: the board is square to the body")
    print(f"    nose-down vs left-side-down heading   {info['crosscheck_deg']:5.1f} deg"
          f"   < {CROSSCHECK_LIMIT_DEG:.0f}: the tilted poses agree")


def measure_orientation(ser, ver, exact=False):
    """Walk through the three poses. Returns (R, info), or None if abandoned."""
    print("\n=== ORIENTATION: three poses ===")
    print("Hold each pose still for ~2 s; resting the robot on the table is best. Poses 2")
    print("and 3 tolerate ~10 deg of slop (lying on its front / left side is plenty);")
    print("only the level pose needs care.")
    while True:
        ups = {}
        for k, (name, _, text) in enumerate(POSES, 1):
            print(f"\nPOSE {k}/3  {text}")
            while True:
                input("    hold it still, then press Enter... ")
                up, rate = read_up(ser, ver)
                print(f"    up in chip axes = ({up[0]:+.3f}, {up[1]:+.3f}, {up[2]:+.3f})"
                      f"   [{_board_axis(up)}]   moving {rate:.1f} deg/s")
                if rate <= STILL_DPS:
                    break
                print(f"    -> it was moving (> {STILL_DPS:.0f} deg/s). Again.")
            ups[name] = up
        r, info, problems = solve_orientation(ups, exact)
        if r is not None:
            report_orientation(r, info)
        if not problems:
            info["measured"] = datetime.date.today().isoformat()
            info["firmware"] = ver["proto"] if ver else 1
            return r, info
        print("\n  PROBLEMS:")
        for p in problems:
            print(f"   * {p}")
        if input("\n  Redo the three poses? [Y/n] ").strip().lower().startswith("n"):
            return None


# --- position ---------------------------------------------------------------------

_OPS = {ast.Add: operator.add, ast.Sub: operator.sub,
        ast.Mult: operator.mul, ast.Div: operator.truediv}


def arith(text: str) -> float:
    """A number or plain arithmetic ('212 - 95', '(30 + 31) / 2'), nothing else, so a
    reading taken from two marks can be typed as-is."""
    def ev(n):
        if isinstance(n, ast.Constant) and type(n.value) in (int, float):
            return float(n.value)
        if isinstance(n, ast.BinOp) and type(n.op) in _OPS:
            return _OPS[type(n.op)](ev(n.left), ev(n.right))
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, (ast.USub, ast.UAdd)):
            return -ev(n.operand) if isinstance(n.op, ast.USub) else ev(n.operand)
        raise ValueError(f"not a number: {text!r}")
    return ev(ast.parse(text.strip(), mode="eval").body)


def _ask_mm(prompt: str) -> float:
    while True:
        try:
            return arith(input(f"  {prompt} [mm]: "))
        except (ValueError, SyntaxError, ZeroDivisionError) as e:
            print(f"    {e}. Try again.")


def _joint_origin(root, name):
    o = root.find(f"joint[@name='{name}']/origin")
    if o is None:
        raise ValueError(f"URDF has no joint {name!r}")
    if any(float(v) for v in o.get("rpy", "0 0 0").split()):
        raise ValueError(f"joint {name!r} is rotated; hip_pitch_axis assumes it isn't")
    return np.array([float(v) for v in o.get("xyz", "0 0 0").split()])


def hip_pitch_axis(urdf_path) -> tuple:
    """(x, z) of the hip-pitch axis in the body frame, with hip roll at 0. Taken from
    the URDF chain body -> hip_left -> upper_leg_left, so it can't go stale."""
    root = ET.parse(urdf_path).getroot()
    p = _joint_origin(root, "hip_left") + _joint_origin(root, "upper_leg_left")
    return float(p[0]), float(p[2])


def body_bounds(urdf_path):
    """(min, max) corner of the body's visual mesh in the body frame, or None."""
    try:
        mesh = ET.parse(urdf_path).getroot().find(
            "link[@name='body']/visual/geometry/mesh").get("filename")
        data = (Path(urdf_path).parent / mesh).read_bytes()
        n = int.from_bytes(data[80:84], "little")
        if len(data) != 84 + 50 * n:            # ASCII STL: skip the sanity check
            return None
        tri = np.frombuffer(data, offset=84, count=n, dtype=np.dtype(
            [("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")]))
        v = tri["v"].reshape(-1, 3)
        return v.min(axis=0), v.max(axis=0)
    except (AttributeError, OSError):
        return None


def position_from_offsets(urdf_path, forward_mm, up_mm, left_mm, right_mm) -> np.ndarray:
    """Body-frame position [m] from offsets to the hip-pitch axis and a left/right pair.

    y = (right - left) / 2: distances to a feature and its mirror image put the
    centreline halfway between them, so it never has to be found."""
    ax, az = hip_pitch_axis(urdf_path)
    return np.array([ax + forward_mm / 1000.0,
                     (right_mm - left_mm) / 2000.0,
                     az + up_mm / 1000.0])


def position_warnings(urdf_path, xyz) -> list:
    out = []
    bounds = body_bounds(urdf_path)
    if bounds is not None:
        lo, hi = bounds
        outside = np.maximum(lo - xyz, 0) + np.maximum(xyz - hi, 0)
        if np.any(outside > 0.02):
            out.append(
                f"the chip is {outside.max() * 1000:.0f} mm outside the body's CAD outline "
                f"(x {lo[0] * 1000:.0f}..{hi[0] * 1000:.0f}, y {lo[1] * 1000:.0f}.."
                f"{hi[1] * 1000:.0f}, z {lo[2] * 1000:.0f}..{hi[2] * 1000:.0f} mm). "
                f"A sign mixed up (in front / above / left)?")
    return out


def measure_position(urdf_path):
    """Ask for the four hand measurements. Returns (xyz_m, info)."""
    ax, az = hip_pitch_axis(urdf_path)
    print("\n=== POSITION: four hand measurements, in mm ===")
    print("Measure to the centre of the IMU chip (the BHI260AP; the board centre if you")
    print("can't see the chip). Each distance runs ALONG one body axis, not straight")
    print("across. Legs straight down, so the hip-pitch axis sits at its URDF place:")
    print(f"x = {ax * 1000:+.2f} mm, z = {az * 1000:+.2f} mm in the body frame. You can")
    print("type arithmetic, e.g. 212-95 for two readings off the same ruler.\n")
    fwd = _ask_mm("1. chip IN FRONT of the hip-pitch axis (behind: negative)")
    up = _ask_mm("2. chip ABOVE the hip-pitch axis (below: negative)")
    left = _ask_mm("3. chip to a feature on the robot's LEFT side")
    right = _ask_mm("4. chip to the mirror-image feature on the RIGHT side")
    xyz = position_from_offsets(urdf_path, fwd, up, left, right)
    info = {"method": "hand", "measured": datetime.date.today().isoformat(),
            "inputs_mm": {"forward": fwd, "up": up, "left": left, "right": right}}
    return xyz, info


# --- the record and the files generated from it -------------------------------------

def _num(v: float, nd: int) -> str:
    s = f"{v:.{nd}f}".rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


def load_record(path=RECORD_PATH) -> dict:
    with open(path) as f:
        rec = json.load(f)
    if not is_proper_rotation(rec["axis_remap"]):
        raise ValueError(f"{path}: axis_remap is not a proper rotation")
    return rec


def save_record(rec: dict, path=RECORD_PATH):
    text = json.dumps(rec, indent=2)
    # One line per list of numbers, so the matrix reads as a matrix.
    text = re.sub(r"\[\s+([-+\d.e]+(?:,\s+[-+\d.e]+)*)\s+\]",
                  lambda m: "[" + re.sub(r",\s+", ", ", m.group(1)) + "]", text)
    Path(path).write_text(text + "\n")


def make_record(r, orientation: dict, xyz, position: dict, note: str = "",
                mass_kg: float = IMU_MASS_KG, box_m=IMU_BOX_M) -> dict:
    r = np.asarray(r, dtype=float)
    remap = ([[int(round(v)) for v in row] for row in r] if np.allclose(r, np.round(r))
             else [[round(float(v), 9) for v in row] for row in r])
    return {
        "_comment": "IMU mount on `body`. Written by imu_locate.py measure; "
                    "imu_locate.py apply turns it into the URDF, AXIS_REMAP and README.",
        "axis_remap": remap,
        "xyz_m": [round(float(v), 4) for v in xyz],
        "mass_kg": mass_kg,
        "box_m": list(box_m),
        "note": note,
        "orientation": orientation,
        "position": position,
    }


def urdf_block(rec: dict) -> str:
    r = np.asarray(rec["axis_remap"], dtype=float)
    rpy = " ".join(_num(a, 9) for a in rpy_from_matrix(r))
    xyz = " ".join(_num(v, 4) for v in rec["xyz_m"])
    m = rec["mass_kg"]
    a, b, c = rec["box_m"]
    ixx, iyy, izz = (m * (b * b + c * c) / 12, m * (a * a + c * c) / 12,
                     m * (a * a + b * b) / 12)
    return (
        f"{URDF_BEGIN}: generated from ../imu_mount.json. Edit the record and run\n"
        f"       `python3 imu_locate.py apply`, not this block. -->\n"
        f"  <!-- IMU: Arduino Nicla Sense ME (Bosch BHI260AP). imu_link's axes are the\n"
        f"       chip's own axes; the joint rotation equals imu_protocol.AXIS_REMAP\n"
        f"       (v_body = R v_imu). Its inertial is board + mount, a few grams. -->\n"
        f"  <link name=\"imu_link\">\n"
        f"    <inertial>\n"
        f"      <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />\n"
        f"      <mass value=\"{m:g}\" />\n"
        f"      <inertia ixx=\"{ixx:.4g}\" ixy=\"0\" ixz=\"0\" iyy=\"{iyy:.4g}\" iyz=\"0\" "
        f"izz=\"{izz:.4g}\" />\n"
        f"    </inertial>\n"
        f"  </link>\n"
        f"\n"
        f"  <joint name=\"imu_joint\" type=\"fixed\">\n"
        f"    <origin xyz=\"{xyz}\" rpy=\"{rpy}\" />\n"
        f"    <parent link=\"body\" />\n"
        f"    <child link=\"imu_link\" />\n"
        f"  </joint>\n"
        f"{URDF_END}"
    )


def write_urdf(rec: dict, urdf_path=URDF_PATH):
    text = Path(urdf_path).read_text()
    block = urdf_block(rec)
    if re.search(r"<(link|joint) name=\"imu_(link|joint)\"", _URDF_BLOCK_RE.sub("", text)):
        raise ValueError(f"{urdf_path} has a hand-written imu_link/imu_joint. Delete it, "
                         f"then run apply again.")
    if _URDF_BLOCK_RE.search(text):
        text = _URDF_BLOCK_RE.sub(lambda _: block, text, count=1)
    else:
        text = text.replace("</robot>", block + "\n</robot>", 1)
    Path(urdf_path).write_text(text)


def read_axis_remap(protocol_path=PROTOCOL_PATH) -> np.ndarray:
    m = _REMAP_RE.search(Path(protocol_path).read_text())
    if m is None:
        raise ValueError(f"no AXIS_REMAP assignment in {protocol_path}")
    return np.array(ast.literal_eval(m.group(1)), dtype=float)


def write_axis_remap(rec: dict, protocol_path=PROTOCOL_PATH):
    text = Path(protocol_path).read_text()
    if len(_REMAP_RE.findall(text)) != 1:
        raise ValueError(f"expected exactly one AXIS_REMAP assignment in {protocol_path}")
    Path(protocol_path).write_text(
        _REMAP_RE.sub(lambda _: format_axis_remap(rec["axis_remap"]), text, count=1))


def write_readme(rec: dict, readme_path=README_PATH) -> bool:
    """Fill the two pose rows of the IMU table. False if the rows aren't there."""
    text = Path(readme_path).read_text()
    if not (_README_POS_RE.search(text) and _README_ROT_RE.search(text)):
        return False
    r = np.asarray(rec["axis_remap"], dtype=float)
    x, y, z = (v * 1000 for v in rec["xyz_m"])
    when = rec.get("position", {}).get("measured", "")
    pos = (f"| Position in `body` frame | x {x:+.1f}, y {y:+.1f}, z {z:+.1f} mm"
           f"{f' (measured {when})' if when else ''} |")
    rot = (f"| Orientation (`imu_joint` rpy) | "
           f"{' '.join(_num(a, 4) for a in rpy_from_matrix(r))} rad: {describe_axes(r)} |")
    text = _README_POS_RE.sub(lambda _: pos, text, count=1)
    text = _README_ROT_RE.sub(lambda _: rot, text, count=1)
    Path(readme_path).write_text(text)
    return True


def apply_record(rec: dict, urdf_path=URDF_PATH, protocol_path=PROTOCOL_PATH,
                 readme_path=README_PATH):
    write_urdf(rec, urdf_path)
    print(f"  wrote imu_link + imu_joint   -> {urdf_path}")
    write_axis_remap(rec, protocol_path)
    print(f"  wrote AXIS_REMAP             -> {protocol_path}")
    if write_readme(rec, readme_path):
        print(f"  filled the IMU table         -> {readme_path}")
    else:
        print(f"  (no IMU pose rows found in {readme_path}; README left alone)")


# --- checks -------------------------------------------------------------------------

def run_checks(record_path=RECORD_PATH, urdf_path=URDF_PATH,
               protocol_path=PROTOCOL_PATH) -> list:
    """Record, URDF, AXIS_REMAP and MuJoCo's reading of the URDF all agree.
    Prints one line per check; returns the names of the failed ones."""
    import mujoco

    fails = []

    def check(name, ok, detail):
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:10s} {detail}")
        if not ok:
            fails.append(name)

    rec = load_record(record_path)
    r = np.asarray(rec["axis_remap"], dtype=float)
    xyz = np.asarray(rec["xyz_m"], dtype=float)
    check("RECORD", True, f"{describe_axes(r)}; xyz = "
                          f"({', '.join(f'{v * 1000:+.1f}' for v in xyz)}) mm")

    j = ET.parse(urdf_path).getroot().find("joint[@name='imu_joint']")
    if j is None:
        check("URDF", False, "no imu_joint (run `imu_locate.py apply`)")
    else:
        o = j.find("origin")
        jxyz = np.array([float(v) for v in o.get("xyz").split()])
        jr = matrix_from_rpy(*(float(v) for v in o.get("rpy").split()))
        ok = (j.get("type") == "fixed" and j.find("parent").get("link") == "body"
              and j.find("child").get("link") == "imu_link"
              and np.allclose(jxyz, xyz, atol=1e-6) and np.allclose(jr, r, atol=1e-6))
        check("URDF", ok, "imu_joint: fixed, body -> imu_link, pose = record" if ok
              else f"imu_joint differs from the record: xyz {jxyz}, R\n{jr}")

    remap = read_axis_remap(protocol_path)
    ok = remap.shape == (3, 3) and np.allclose(remap, r, atol=1e-6)
    check("AXIS_REMAP", ok, "imu_protocol.py = record" if ok
          else f"imu_protocol.py has\n{remap}\n  record has\n{r}")

    try:
        model = mujoco.MjSpec.from_file(str(urdf_path)).compile()
    except Exception as e:  # noqa: BLE001 — any import/compile error is the finding
        check("MUJOCO", False, f"{type(e).__name__}: {e}")
        return fails
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "imu_link")
    if bid < 0:
        check("MUJOCO", False, "imu_link missing after import (fusestatic?)")
    else:
        parent = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                   model.body_parentid[bid])
        q = np.array(proto._matrix_to_quat(r.tolist()))
        ok = (parent == "body" and np.allclose(model.body_pos[bid], xyz, atol=1e-6)
              and min(np.abs(model.body_quat[bid] - q).max(),
                      np.abs(model.body_quat[bid] + q).max()) < 1e-6)
        check("MUJOCO", ok, "imu_link pos/quat = record" if ok else
              f"parent {parent}, pos {model.body_pos[bid]}, quat {model.body_quat[bid]}"
              f" (want {xyz}, ±{q})")
    try:
        _build_model(str(urdf_path))
        check("SIM", True, "sim_env builds its floating-base model from this URDF")
    except Exception as e:  # noqa: BLE001
        check("SIM", False, f"sim_env._build_model: {type(e).__name__}: {e}")
    return fails


def live_check(ser, ver, timeout_s: float = 20.0) -> bool:
    """Tilt the robot and watch what the policy would see, through the deployment path
    (imu_protocol's AXIS_REMAP, imu_service's scales, ZmqEnv's projected gravity)."""
    from imu_service import sensor_scales
    from rl_env import projected_gravity_from_quat

    gs, as_, _ = sensor_scales(ver)

    def read():
        ser.reset_input_buffer()
        ser.write(b"L")
        r = proto.parse_imu_line(ser.readline().decode("ascii", errors="ignore"), gs, as_)
        if r is None:
            return None
        if "quat" in r:              # ZmqEnv prefers the fused quaternion
            g = np.asarray(projected_gravity_from_quat(r["quat"]), dtype=float)
        else:
            g = -_unit(r["accel"])
        return g, np.asarray(r["gyro"], dtype=float)

    print("\n=== LIVE CHECK: what the policy will see ===")
    print("projected_gravity is gravity in the body frame (level = (0, 0, -1)).")
    input("\n  1. Robot LEVEL and still. Press Enter... ")
    gravity = [s[0] for s in (read() for _ in range(30)) if s is not None]
    g = np.mean(gravity, axis=0) if gravity else np.zeros(3)
    ok_level = g[2] < -0.97
    print(f"  [{'PASS' if ok_level else 'FAIL'}] level      projected_gravity = "
          f"({g[0]:+.2f}, {g[1]:+.2f}, {g[2]:+.2f})   want (0, 0, -1)")

    def motion(step, what, g_axis, w_axis, w_sign):
        input(f"\n  {step}. From level, press Enter, then slowly tilt {what} ~45 deg and hold... ")
        turned, held, t_prev = np.zeros(3), 0, time.monotonic()
        t_end = t_prev + timeout_s
        while time.monotonic() < t_end and held < 10:
            s = read()
            now = time.monotonic()
            dt, t_prev = now - t_prev, now
            if s is None:
                continue
            g, w = s
            turned += w * dt
            held = held + 1 if g[g_axis] > 0.5 else 0
            print(f"\r     projected_gravity = ({g[0]:+.2f}, {g[1]:+.2f}, {g[2]:+.2f})   "
                  f"turned = ({', '.join(f'{math.degrees(v):+4.0f}' for v in turned)}) deg ",
                  end="", flush=True)
        print()
        dom = int(np.argmax(np.abs(turned)))
        ok = held >= 10 and dom == w_axis and np.sign(turned[dom]) == w_sign
        print(f"  [{'PASS' if ok else 'FAIL'}] {what:10s} want gravity {'xyz'[g_axis]} > +0.5 "
              f"and the turn mostly about {'+' if w_sign > 0 else '-'}{'xyz'[w_axis]}")
        return ok

    ok_x = motion(2, "nose-down", 0, 1, +1)    # +y rotation pitches the nose down
    ok_y = motion(3, "left-down", 1, 0, -1)    # -x rotation drops the left side
    ok = ok_level and ok_x and ok_y
    print(f"\n  LIVE: {'PASS' if ok else 'FAIL'}" + ("" if ok else
          " — AXIS_REMAP or a pose is wrong. Re-run `imu_locate.py measure --skip-position`."))
    return ok


# --- CLI ----------------------------------------------------------------------------

def _open(args):
    try:
        ser, _, ver = proto.open_imu(args.port, args.baud)
    except (RuntimeError, AttributeError) as e:   # AttributeError: pyserial missing
        sys.exit(f"{e}\n  Is imu_service.py (or a serial monitor) holding the port? "
                 f"`ls /dev/ttyACM*` to find it.")
    ser.timeout = 0.2
    return ser, ver


def cmd_measure(args) -> int:
    if args.skip_orientation and args.skip_position and args.xyz_mm is None:
        sys.exit("nothing to measure. To rewrite the files from the record: apply")
    old = load_record(args.record) if Path(args.record).exists() else None
    if (args.skip_orientation or (args.skip_position and args.xyz_mm is None)) and old is None:
        sys.exit(f"no record at {args.record} to keep the skipped half from")

    ser = ver = None
    if args.skip_orientation:
        r, orientation = np.asarray(old["axis_remap"], dtype=float), old["orientation"]
    else:
        ser, ver = _open(args)
        got = measure_orientation(ser, ver, exact=args.exact)
        if got is None:
            return 1
        r, orientation = got

    if args.xyz_mm is not None:
        xyz = np.asarray(args.xyz_mm, dtype=float) / 1000.0
        position = {"method": "given", "measured": datetime.date.today().isoformat()}
    elif args.skip_position:
        xyz, position = np.asarray(old["xyz_m"], dtype=float), old["position"]
    else:
        xyz, position = measure_position(args.urdf)

    print(f"\n  IMU position in the body frame: x {xyz[0] * 1000:+.1f}, "
          f"y {xyz[1] * 1000:+.1f}, z {xyz[2] * 1000:+.1f} mm")
    inputs = position.get("inputs_mm")
    if inputs:
        print(f"  (left + right = {inputs['left'] + inputs['right']:.0f} mm between the two "
              f"side features: does that match the robot?)")
    for w in position_warnings(args.urdf, xyz):
        print(f"  WARNING: {w}")

    rec = make_record(r, orientation, xyz, position, note=args.note or "",
                      mass_kg=old["mass_kg"] if old else IMU_MASS_KG,
                      box_m=old["box_m"] if old else IMU_BOX_M)
    if input(f"\n  Write {args.record.name}, the URDF, AXIS_REMAP and the README? [Y/n] "
             ).strip().lower().startswith("n"):
        return 1
    save_record(rec, args.record)
    print(f"  wrote the record             -> {args.record}")
    apply_record(rec, args.urdf)

    print("\n=== STATIC CHECK ===")
    fails = run_checks(args.record, args.urdf)
    if fails or args.no_live:
        return 1 if fails else 0
    if ser is None:
        if input("\n  Live check needs the IMU. Run it? [Y/n] ").strip().lower().startswith("n"):
            return 0
        ser, ver = _open(args)
    importlib.reload(proto)          # the AXIS_REMAP this process just wrote
    return 0 if live_check(ser, ver) else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--record", type=Path, default=RECORD_PATH, help="default: %(default)s")
    ap.add_argument("--urdf", type=Path, default=URDF_PATH, help="default: %(default)s")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def serial_args(p):
        p.add_argument("--port", default="/dev/ttyACM0", help="default: %(default)s")
        p.add_argument("--baud", type=int, default=None, help="skip baud auto-detect")

    m = sub.add_parser("measure", help="bench session: poses + hand measurements")
    serial_args(m)
    m.add_argument("--skip-orientation", action="store_true",
                   help="keep the recorded orientation")
    m.add_argument("--skip-position", action="store_true", help="keep the recorded position")
    m.add_argument("--xyz-mm", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="IMU position in the body frame, skips the hand measurements")
    m.add_argument("--exact", action="store_true",
                   help="keep the fitted rotation instead of snapping to a square mount")
    m.add_argument("--note", help="free text stored in the record (e.g. how it was measured)")
    m.add_argument("--no-live", action="store_true", help="skip the live tilt check")

    sub.add_parser("apply", help="regenerate URDF, AXIS_REMAP and README from the record")

    c = sub.add_parser("check", help="verify record, URDF, AXIS_REMAP and MuJoCo agree")
    serial_args(c)
    c.add_argument("--live", action="store_true", help="also tilt the robot and watch")

    args = ap.parse_args()
    if args.cmd != "measure" and not args.record.exists():
        sys.exit(f"no record at {args.record}. Measure first: python3 imu_locate.py measure")
    try:
        if args.cmd == "measure":
            return cmd_measure(args)
        if args.cmd == "apply":
            apply_record(load_record(args.record), args.urdf)
            return 1 if run_checks(args.record, args.urdf) else 0
        fails = run_checks(args.record, args.urdf)
        if args.live and not fails:
            ser, ver = _open(args)
            return 0 if live_check(ser, ver) else 1
        return 1 if fails else 0
    except KeyboardInterrupt:
        print("\n  interrupted.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
