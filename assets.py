"""
assets.py — where the robot model lives: the `dodo_assets` repo (Atarilab), checked out
NEXT TO this one. It is the single source of the robot description (URDF, MJCF, meshes,
IMU mount record); this repo keeps no copy (`description/` was removed 2026-11-05).

    git clone https://github.com/Atarilab/dodo_assets.git ../dodo_assets
    DODO_ASSETS=/some/other/checkout python3 ...      # or point at it explicitly

Every path to a model file is spelled here once. Import-only, and nothing is checked
at import: motor_service and the bench tools don't need the model, so they keep working
without the checkout. Whatever loads a file calls require() for a clear message.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parent
ASSETS_DIR = Path(os.environ.get("DODO_ASSETS") or REPO.parent / "dodo_assets").resolve()

URDF_PATH = ASSETS_DIR / "urdf" / "dodo_daimao.urdf"   # what sim_env and the visualizer load
MJCF_PATH = ASSETS_DIR / "mjcf" / "dodo_daimao.xml"     # imu_locate keeps its IMU block
SCENE_PATH = ASSETS_DIR / "mjcf" / "scene.xml"          # floor + PD actuators + keyframes
IMU_RECORD_PATH = ASSETS_DIR / "imu_mount.json"         # imu_locate's one record
README_PATH = ASSETS_DIR / "README.md"                  # imu_locate fills its IMU rows


def require(path=URDF_PATH) -> Path:
    """`path` if it exists, else a FileNotFoundError that says how to get the model."""
    path = Path(path)
    if not path.exists():
        hint = (f"The robot model lives in the dodo_assets repo: clone "
                f"https://github.com/Atarilab/dodo_assets.git to {REPO.parent / 'dodo_assets'}, "
                f"or set DODO_ASSETS to a checkout of it."
                if path.is_relative_to(ASSETS_DIR) else "")
        raise FileNotFoundError(f"{path} not found. {hint}".rstrip())
    return path


def scene_pose(name: str, scene_path=SCENE_PATH) -> dict:
    """{joint: rad} of a keyframe in scene.xml ("home", "straight"), read from its
    `ctrl` (the joint targets, in actuator order) without MuJoCo, so ZMQ clients such
    as tools/dodo_startup.py can use the poses Shafeef's scene defines."""
    root = ET.parse(require(scene_path)).getroot()
    joints = [a.get("joint") for a in root.find("actuator")]
    for key in root.iter("key"):
        if key.get("name") == name:
            ctrl = [float(v) for v in key.get("ctrl").split()]
            if len(ctrl) != len(joints):
                raise ValueError(f"{scene_path}: key '{name}' has {len(ctrl)} ctrl values "
                                 f"for {len(joints)} actuators")
            return dict(zip(joints, ctrl))
    names = [k.get("name") for k in root.iter("key")]
    raise KeyError(f"no keyframe '{name}' in {scene_path} (has {names})")
