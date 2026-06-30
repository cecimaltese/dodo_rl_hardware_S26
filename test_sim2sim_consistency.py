"""
test_sim2sim_consistency.py — guard the IsaacLab(dodo_rl) → MuJoCo(this repo) contract.

The trained policy only transfers if the MuJoCo side reproduces *exactly* what
IsaacLab trained against. The ground truth is the config IsaacLab dumps next to every
checkpoint: ``<run>/params/env.yaml`` (the fully-resolved ManagerBasedRLEnvCfg). These
tests parse that YAML and assert the MuJoCo-side constants in ``rl_env.py`` /
``sim_env.py`` match it, that the URDF MuJoCo loads describes the same robot as the
USD IsaacLab trained on, that the exported ONNX I/O dims line up, and — the real
acceptance test — that the stand policy actually *stands* in MuJoCo (which is also the
joint-order regression guard: the bug that motivated this file was POLICY_JOINT_ORDER
having left/right swapped, that the robot tips over in <1 s with the wrong order).

Run:   pytest -v dodo_rl_hardware_S26/test_sim2sim_consistency.py
or:    python3 dodo_rl_hardware_S26/test_sim2sim_consistency.py

Source of truth = the **dodo_stand** run (the deployable policy). The dodo_flat run is
sim-only and may be stale (older pose); see test_flat_pose_status.

Known accepted residuals (do NOT affect standing; documented, not asserted):
  * env.yaml actuators set velocity_limit_sim=6.0 (a PhysX joint max-velocity). sim_env
    models only the effort clamp, not a joint-velocity clamp; standing joint speeds are
    well under 6 rad/s so it is irrelevant for transfer.
  * Friction: IsaacLab randomizes the robot material to static 0.8 / dynamic 0.6 at
    startup; sim_env uses MuJoCo's default (~1.0). Within the training range; the feet
    do not slip while standing.
"""
from __future__ import annotations

import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from glob import glob
from pathlib import Path

import numpy as np
import yaml

# --- make `import rl_env` / `import sim_env` work however pytest is invoked ---------
HW_DIR = Path(__file__).resolve().parent
REPO_ROOT = HW_DIR.parent
sys.path.insert(0, str(HW_DIR))

import rl_env  # noqa: E402
from rl_env import (ACTION_SCALE, DEFAULT_JOINT_POS, OBS_FLAT, OBS_STAND,  # noqa: E402
                    POLICY_JOINT_ORDER)
import sim_env  # noqa: E402
from sim_env import DECIMATION, EFFORT_LIMIT, KD, KP, SIM_DT, _build_model  # noqa: E402

DODO_RL = REPO_ROOT / "dodo_rl"
URDF = REPO_ROOT / "dodo_files" / "urdf" / "dodo_daimao.urdf"
USD = DODO_RL / "source" / "dodo_rl" / "dodo_rl" / "assets" / "usd" / "dodo_daimao.usd"

# pytest's skip exception, with a no-pytest fallback so the file also runs under bare
# `python3` (the __main__ runner below treats it as SKIP).
try:
    from _pytest.outcomes import Skipped
except Exception:  # pragma: no cover
    class Skipped(Exception):
        pass


def skip(msg: str):
    raise Skipped(msg)


TOL = 1e-9

# env.yaml observation term name  ->  rl_env ObsCfg term name
ENV_TO_RL_OBS = {
    "base_lin_vel": "base_lin_vel",
    "base_ang_vel": "base_ang_vel",
    "projected_gravity": "projected_gravity",
    "velocity_commands": "velocity_commands",
    "joint_pos": "joint_pos_rel",
    "joint_vel": "joint_vel",
    "actions": "last_action",
}


# ----------------------------------------------------------------------------------
# ground-truth discovery / loading
# ----------------------------------------------------------------------------------
def _latest_run(task: str):
    """Newest <run> dir under dodo_rl that has BOTH params/env.yaml and the ONNX."""
    hits = []
    for y in glob(str(DODO_RL / "**" / f"dodo_{task}" / "*" / "params" / "env.yaml"),
                  recursive=True):
        run = Path(y).parent.parent
        onnx = run / "exported" / "policy.onnx"
        if onnx.exists():
            hits.append(run)
    if not hits:
        return None
    # run dir names are timestamps (YYYY-MM-DD_HH-MM-SS) -> lexicographic == chronological
    return sorted(hits, key=lambda p: p.name)[-1]


def _load_env_yaml(run: Path) -> dict:
    with open(run / "params" / "env.yaml") as f:
        return yaml.load(f, Loader=yaml.UnsafeLoader)


STAND_RUN = _latest_run("stand")
FLAT_RUN = _latest_run("flat")


def _require_stand() -> dict:
    if STAND_RUN is None:
        skip("no dodo_stand run with params/env.yaml + exported/policy.onnx found")
    return _load_env_yaml(STAND_RUN)


def _expand_pose(regex_pose: dict, joints) -> dict:
    """Resolve {'hip_.*': 0.0, ...} against concrete joint names."""
    out = {}
    for j in joints:
        for pat, val in regex_pose.items():
            if re.fullmatch(pat, j):
                out[j] = float(val)
                break
        else:
            raise KeyError(f"no pose pattern matched joint {j!r}")
    return out


def _ordered_obs_terms(policy_obs: dict):
    """Non-null obs terms in env.yaml declaration order, mapped to rl_env names."""
    return [ENV_TO_RL_OBS[k] for k, v in policy_obs.items()
            if k in ENV_TO_RL_OBS and v is not None]


# ----------------------------------------------------------------------------------
# 1. policy contract: action mapping + default pose + obs layout + timing
# ----------------------------------------------------------------------------------
def test_action_contract_matches_env_yaml():
    cfg = _require_stand()
    act = cfg["actions"]["joint_pos"]
    assert act["class_type"].endswith("JointPositionAction")
    assert abs(act["scale"] - ACTION_SCALE) < TOL, "action scale != rl_env.ACTION_SCALE"
    assert float(act["offset"]) == 0.0
    assert act["use_default_offset"] is True, "rl_env assumes target=default+scale*action"
    assert act["preserve_order"] is False, "rl_env assumes articulation (not regex) order"
    assert act["joint_names"] == [".*"]


def test_stand_pose_matches_contract():
    cfg = _require_stand()
    want = _expand_pose(cfg["scene"]["robot"]["init_state"]["joint_pos"], POLICY_JOINT_ORDER)
    for j in POLICY_JOINT_ORDER:
        assert abs(DEFAULT_JOINT_POS[j] - want[j]) < TOL, (
            f"{j}: rl_env default {DEFAULT_JOINT_POS[j]} != env.yaml {want[j]}")


def test_obs_layout_stand_matches_env_yaml():
    cfg = _require_stand()
    pol = cfg["observations"]["policy"]
    assert _ordered_obs_terms(pol) == OBS_STAND.terms, "stand obs order/membership mismatch"
    assert OBS_STAND.dim == 30
    # rl_env applies no per-term scale (implicit 1.0); env.yaml must agree.
    for k, v in pol.items():
        if k in ENV_TO_RL_OBS and v is not None:
            assert v.get("scale") in (None, 1.0), f"obs term {k} has non-unit scale {v.get('scale')}"
    # stand explicitly drops the two terms the real robot can't observe.
    assert pol["base_lin_vel"] is None and pol["velocity_commands"] is None


def test_obs_layout_flat_matches_env_yaml():
    if FLAT_RUN is None:
        skip("no dodo_flat run found")
    pol = _load_env_yaml(FLAT_RUN)["observations"]["policy"]
    assert _ordered_obs_terms(pol) == OBS_FLAT.terms, "flat obs order/membership mismatch"
    assert OBS_FLAT.dim == 36


def test_control_timing_matches_env_yaml():
    cfg = _require_stand()
    assert abs(cfg["sim"]["dt"] - SIM_DT) < TOL
    assert int(cfg["decimation"]) == DECIMATION
    # 50 Hz control = sim dt * decimation
    assert abs(SIM_DT * DECIMATION - 0.02) < TOL


# ----------------------------------------------------------------------------------
# 2. actuator gains / limits (env.yaml ImplicitActuator <-> sim_env software PD)
# ----------------------------------------------------------------------------------
def test_actuator_gains_match_env_yaml():
    cfg = _require_stand()
    acts = cfg["scene"]["robot"]["actuators"]
    hu, lf = acts["hip_upper"], acts["lower_foot"]
    # sim_env uses single KP/KD scalars -> both groups must share them.
    for g in (hu, lf):
        assert abs(g["stiffness"] - KP) < TOL, "stiffness != sim_env.KP"
        assert abs(g["damping"] - KD) < TOL, "damping != sim_env.KD"
        assert abs(g["armature"] - 0.01) < TOL
    # effort limits, by joint-name prefix
    assert abs(hu["effort_limit_sim"] - EFFORT_LIMIT["hip_"]) < TOL
    assert abs(hu["effort_limit_sim"] - EFFORT_LIMIT["upper_leg_"]) < TOL
    assert abs(lf["effort_limit_sim"] - EFFORT_LIMIT["lower_leg_"]) < TOL
    assert abs(lf["effort_limit_sim"] - EFFORT_LIMIT["foot_"]) < TOL
    assert (hu["effort_limit_sim"], lf["effort_limit_sim"]) == (27.0, 9.0)


# ----------------------------------------------------------------------------------
# 3. structural invariants of the joint order
# ----------------------------------------------------------------------------------
def test_joint_order_is_type_grouped_bfs():
    """PhysX orders DOFs breadth-first -> type-grouped. (L/R correctness is the
    behavioral test's job; here we lock the structure.)"""
    assert set(POLICY_JOINT_ORDER) == {
        "hip_left", "hip_right", "upper_leg_left", "upper_leg_right",
        "lower_leg_left", "lower_leg_right", "foot_left", "foot_right"}
    types = [j.rsplit("_", 1)[0] for j in POLICY_JOINT_ORDER]  # strip left/right
    assert types == ["hip", "hip", "upper_leg", "upper_leg",
                     "lower_leg", "lower_leg", "foot", "foot"], (
        "expected type-grouped (BFS) order: all hips, then upper_legs, lower_legs, feet")


# ----------------------------------------------------------------------------------
# 4. model consistency: URDF (MuJoCo) vs the contract / USD presence
# ----------------------------------------------------------------------------------
def _urdf_root():
    assert URDF.exists(), f"URDF missing: {URDF}"
    return ET.parse(URDF).getroot()


def test_urdf_joint_set_matches_policy_order():
    root = _urdf_root()
    rev = {j.get("name") for j in root.findall("joint") if j.get("type") == "revolute"}
    fixed = {j.get("name") for j in root.findall("joint") if j.get("type") == "fixed"}
    assert rev == set(POLICY_JOINT_ORDER), "URDF revolute joints != POLICY_JOINT_ORDER set"
    assert {"foot_sole_left", "foot_sole_right"} <= fixed


def test_urdf_axes_limits_and_symmetry():
    root = _urdf_root()
    joints = {j.get("name"): j for j in root.findall("joint") if j.get("type") == "revolute"}
    expect_axis = {"hip": (1, 0, 0), "upper_leg": (0, 1, 0),
                   "lower_leg": (0, 1, 0), "foot": (0, 1, 0)}
    expect_effort = {"hip": 27, "upper_leg": 27, "lower_leg": 9, "foot": 9}
    for name, j in joints.items():
        typ = name.rsplit("_", 1)[0]
        axis = tuple(int(round(float(x))) for x in j.find("axis").get("xyz").split())
        assert axis == expect_axis[typ], f"{name} axis {axis} != {expect_axis[typ]}"
        lim = j.find("limit")
        assert abs(float(lim.get("effort")) - expect_effort[typ]) < TOL, f"{name} effort"
        assert abs(float(lim.get("velocity")) - 6.0) < TOL, f"{name} velocity limit"
    # left/right limits identical (symmetry)
    for base in ("hip", "upper_leg", "lower_leg", "foot"):
        l, r = joints[f"{base}_left"].find("limit"), joints[f"{base}_right"].find("limit")
        for attr in ("lower", "upper"):
            assert abs(float(l.get(attr)) - float(r.get(attr))) < TOL, f"{base} {attr} L!=R"


def test_urdf_effort_matches_env_yaml_actuators():
    """URDF effort limits and env.yaml effort_limit_sim must agree (they both come
    from the same robot spec; this ties URDF <-> USD <-> sim_env together)."""
    cfg = _require_stand()
    acts = cfg["scene"]["robot"]["actuators"]
    eff = {"hip": acts["hip_upper"]["effort_limit_sim"],
           "upper_leg": acts["hip_upper"]["effort_limit_sim"],
           "lower_leg": acts["lower_foot"]["effort_limit_sim"],
           "foot": acts["lower_foot"]["effort_limit_sim"]}
    root = _urdf_root()
    for j in root.findall("joint"):
        if j.get("type") != "revolute":
            continue
        typ = j.get("name").rsplit("_", 1)[0]
        assert abs(float(j.find("limit").get("effort")) - eff[typ]) < TOL


def test_mujoco_model_structure():
    import mujoco
    m = _build_model(str(URDF))
    assert m.nu == 8, "expected 8 torque actuators"
    assert m.nq == 15 and m.nv == 14, "free base (7q/6v) + 8 hinges"
    hinges = [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
    frees = [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE]
    assert len(hinges) == 8 and len(frees) == 1
    # every policy joint exists in the model
    for j in POLICY_JOINT_ORDER:
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) >= 0, f"missing joint {j}"


def test_mujoco_actuator_and_passive_terms_match_isaaclab():
    """sim_env must reproduce the implicit actuator: armature 0.01, and NO passive
    joint damping/friction (the only velocity term is the software PD's KD)."""
    import mujoco
    cfg = _require_stand()
    arm = cfg["scene"]["robot"]["actuators"]["hip_upper"]["armature"]
    m = _build_model(str(URDF))
    for j in POLICY_JOINT_ORDER:
        dof = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
        assert abs(m.dof_armature[dof] - arm) < TOL, f"{j} armature != env.yaml {arm}"
        assert abs(m.dof_damping[dof]) < TOL, (
            f"{j} has passive damping {m.dof_damping[dof]} (IsaacLab adds none; "
            f"damping must come only from sim_env KD)")
        assert abs(m.dof_frictionloss[dof]) < TOL, f"{j} has passive frictionloss"
    # effort clamp in sim_env matches env.yaml effort_limit_sim
    eff_env = {"hip_": cfg["scene"]["robot"]["actuators"]["hip_upper"]["effort_limit_sim"],
               "upper_leg_": cfg["scene"]["robot"]["actuators"]["hip_upper"]["effort_limit_sim"],
               "lower_leg_": cfg["scene"]["robot"]["actuators"]["lower_foot"]["effort_limit_sim"],
               "foot_": cfg["scene"]["robot"]["actuators"]["lower_foot"]["effort_limit_sim"]}
    for pref, e in eff_env.items():
        assert abs(EFFORT_LIMIT[pref] - e) < TOL


def test_total_mass_reasonable():
    import mujoco
    m = _build_model(str(URDF))
    total = float(sum(m.body_mass))
    assert abs(total - 4.7) < 0.1, f"total mass {total:.3f} kg not ~4.7"


def test_usd_asset_present_and_is_dodo_daimao():
    """IsaacLab trained on the dodo_daimao USD; assert that asset (and its layered
    sublayers) is present locally and is the one the env.yaml references."""
    cfg = _require_stand()
    yaml_usd = cfg["scene"]["robot"]["spawn"]["usd_path"]
    assert os.path.basename(yaml_usd) == "dodo_daimao.usd", (
        f"env.yaml trained on {yaml_usd!r}, not the dodo_daimao asset")
    assert USD.exists(), f"local USD missing: {USD}"
    cfgdir = USD.parent / "configuration"
    for layer in ("base", "physics", "robot", "sensor"):
        f = cfgdir / f"dodo_daimao_{layer}.usd"
        assert f.exists(), f"USD sublayer missing: {f}"


# ----------------------------------------------------------------------------------
# 5. exported ONNX I/O dims
# ----------------------------------------------------------------------------------
def _onnx_io(run: Path):
    try:
        import onnxruntime as ort
    except ImportError:
        skip("onnxruntime not installed")
    s = ort.InferenceSession(str(run / "exported" / "policy.onnx"),
                             providers=["CPUExecutionProvider"])
    return int(s.get_inputs()[0].shape[-1]), int(s.get_outputs()[0].shape[-1])


def test_onnx_io_stand():
    if STAND_RUN is None:
        skip("no dodo_stand run")
    obs_dim, act_dim = _onnx_io(STAND_RUN)
    assert obs_dim == OBS_STAND.dim == 30
    assert act_dim == len(POLICY_JOINT_ORDER) == 8


def test_onnx_io_flat():
    if FLAT_RUN is None:
        skip("no dodo_flat run")
    obs_dim, act_dim = _onnx_io(FLAT_RUN)
    assert obs_dim == OBS_FLAT.dim == 36
    assert act_dim == len(POLICY_JOINT_ORDER) == 8


# ----------------------------------------------------------------------------------
# 6. behavioral acceptance + joint-order regression guard
# ----------------------------------------------------------------------------------
_RUNNER = r"""
import sys, os
sys.path.insert(0, %(hw)r)
import numpy as np
from rl_env import OnnxPolicy, OBS_STAND, projected_gravity_from_quat
from sim_env import SimEnv
pol = OnnxPolicy(%(onnx)r)
env = SimEnv(urdf_path=%(urdf)r, obs_cfg=OBS_STAND, render=False)
obs = env.reset()
for _ in range(%(steps)d):
    obs = env.step(pol(obs))
z = float(env.data.qpos[2])
pgz = float(projected_gravity_from_quat(env.data.qpos[3:7])[2])
print("RESULT", z, pgz)
"""


def _run_stand(steps=300, order_csv=None):
    """Run the stand policy in a clean subprocess (so DODO_JOINT_ORDER is honored at
    import). Returns (base_z, proj_gravity_z) after `steps`."""
    if STAND_RUN is None:
        skip("no dodo_stand run")
    onnx = str(STAND_RUN / "exported" / "policy.onnx")
    code = _RUNNER % {"hw": str(HW_DIR), "onnx": onnx, "urdf": str(URDF), "steps": steps}
    env = dict(os.environ)
    if order_csv is None:
        env.pop("DODO_JOINT_ORDER", None)
    else:
        env["DODO_JOINT_ORDER"] = order_csv
    p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env)
    line = next((l for l in p.stdout.splitlines() if l.startswith("RESULT")), None)
    assert line, f"runner produced no RESULT.\nstdout:\n{p.stdout}\nstderr:\n{p.stderr}"
    _, z, pgz = line.split()
    return float(z), float(pgz)


def test_behavior_stand_policy_stands():
    """Acceptance: with the SHIPPED POLICY_JOINT_ORDER the stand policy holds an
    upright stance (this is the real sim2sim transfer check)."""
    z, pgz = _run_stand(steps=300)
    assert z > 0.33, f"base collapsed to {z:.3f} m — policy fell"
    assert pgz < -0.95, f"proj_gravity_z {pgz:+.3f} — robot not upright"


def test_behavior_wrong_order_falls():
    """Regression guard: the left/right-swapped order (the original bug) must FAIL,
    proving the standing test actually discriminates joint order."""
    wrong = ("hip_right,hip_left,upper_leg_right,upper_leg_left,"
             "lower_leg_right,lower_leg_left,foot_right,foot_left")
    z, pgz = _run_stand(steps=300, order_csv=wrong)
    assert not (z > 0.33 and pgz < -0.95), (
        f"right-first order unexpectedly stayed up (z={z:.3f}, pgz={pgz:+.3f}); "
        f"the standing test cannot detect a joint-order regression")


def test_flat_pose_status():
    """Informational: flat is sim-only and may be stale. If its pose no longer matches
    the current contract, skip with a loud note instead of silently passing."""
    if FLAT_RUN is None:
        skip("no dodo_flat run")
    pose = _load_env_yaml(FLAT_RUN)["scene"]["robot"]["init_state"]["joint_pos"]
    want = _expand_pose(pose, POLICY_JOINT_ORDER)
    mismatch = {j: (DEFAULT_JOINT_POS[j], want[j])
                for j in POLICY_JOINT_ORDER if abs(DEFAULT_JOINT_POS[j] - want[j]) > TOL}
    if mismatch:
        skip(f"flat policy ({FLAT_RUN.name}) trained with a DIFFERENT default pose "
             f"{mismatch} — it is stale vs the current contract; sim-only, do NOT deploy.")


# ----------------------------------------------------------------------------------
# bare-`python3` runner (no pytest needed)
# ----------------------------------------------------------------------------------
if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v)]
    n_pass = n_fail = n_skip = 0
    print(f"discovered: stand={STAND_RUN.name if STAND_RUN else None} "
          f"flat={FLAT_RUN.name if FLAT_RUN else None}\n")
    for t in tests:
        try:
            t()
            print(f"  PASS  {t.__name__}")
            n_pass += 1
        except Skipped as e:
            print(f"  SKIP  {t.__name__}: {e}")
            n_skip += 1
        except AssertionError as e:
            print(f"  FAIL  {t.__name__}: {e}")
            n_fail += 1
        except Exception as e:  # unexpected error
            print(f"  ERROR {t.__name__}: {type(e).__name__}: {e}")
            n_fail += 1
    print(f"\n{n_pass} passed, {n_fail} failed, {n_skip} skipped")
    sys.exit(1 if n_fail else 0)
