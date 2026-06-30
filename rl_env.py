"""
rl_env.py — the sim-to-real Env interface + the IsaacLab policy contract.

This is the single substitution point for deploying the trained policy: a runner
loops `action = policy(obs); obs = env.step(action)` against an `Env`, and the SAME
loop runs in MuJoCo (`SimEnv`) for verification and on the real robot (`RealEnv`)
for deployment. Everything the policy "knows" — joint order, default pose, action
scaling, and the observation layout — lives here so both backends are identical.

Contract extracted from the IsaacLab training config
(`dodo_rl/.../tasks/locomotion/`, dumped `params/env.yaml`):

  * Action = per-joint position target, `JointPositionAction`:
        target = default_joint_pos + ACTION_SCALE * action          (radians)
    with ACTION_SCALE = 0.5, use_default_offset = True. action_dim = 8.

  * Observation (concatenated, declaration order, all scales = 1.0):
        dodo_flat  (walking, obs_dim 36): base_lin_vel(3), base_ang_vel(3),
            projected_gravity(3), velocity_commands(3), joint_pos_rel(8),
            joint_vel(8), last_action(8)
        dodo_stand (balance, obs_dim 30): base_ang_vel(3), projected_gravity(3),
            joint_pos_rel(8), joint_vel(8), last_action(8)
      (stand drops base_lin_vel + velocity_commands — base_lin_vel is unobservable
       on the real robot, so dodo_stand is the deployable target; dodo_flat is
       sim-verification-only.)

  * Joint order is the IsaacLab/PhysX articulation order (regex ".*",
    preserve_order=False → BFS of the kinematic tree), which is NOT the URDF/MuJoCo
    order. SimEnv/RealEnv therefore map to this order BY NAME.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

TWO_PI = 2.0 * math.pi

# --- Joint order the POLICY expects (IsaacLab/PhysX articulation order). ----------
# PhysX orders DOFs breadth-first by tree depth → TYPE-GROUPED (all hips, then all
# upper_legs, then lower_legs, then feet), and within each level LEFT precedes RIGHT.
# This is NOT the URDF/MuJoCo order (which is per-leg, right-first); SimEnv/RealEnv
# remap to this order BY NAME.
# Empirically pinned in MuJoCo with the dodo_stand policy (test_sim2sim_consistency):
# this order stands stably 30 s+; right-first or per-leg orders flip the robot <1 s.
POLICY_JOINT_ORDER: List[str] = [
    "hip_left", "hip_right",
    "upper_leg_left", "upper_leg_right",
    "lower_leg_left", "lower_leg_right",
    "foot_left", "foot_right",
]

# Test/override hook: set DODO_JOINT_ORDER="a,b,..." to override (used to verify
# the order in sim). Remove or ignore in production once the order is confirmed.
import os as _os
if _os.environ.get("DODO_JOINT_ORDER"):
    POLICY_JOINT_ORDER = _os.environ["DODO_JOINT_ORDER"].split(",")

# Default (home) joint positions in radians. MUST match dodo.py init_state.joint_pos
# (the pose the policy is trained around) — change both together and retrain.
# Bird-like crouch: knee (lower_leg) folds BACKWARD (negative). Validated upright in
# MuJoCo at ~0.44 m base height.
_DEFAULT_POS_PATTERNS = {
    "hip_": 0.0,         # hip roll neutral
    "upper_leg_": 0.20,  # hip pitch (thigh forward)
    "lower_leg_": -0.50, # knee folds backward (bird-like)
    "foot_": 0.30,       # ankle keeps the sole flat
}


def _default_pos_for(joint: str) -> float:
    for prefix, val in _DEFAULT_POS_PATTERNS.items():
        if joint.startswith(prefix):
            return val
    raise KeyError(f"no default pose pattern for joint '{joint}'")


DEFAULT_JOINT_POS: Dict[str, float] = {j: _default_pos_for(j) for j in POLICY_JOINT_ORDER}
DEFAULT_POS_VEC: np.ndarray = np.array([DEFAULT_JOINT_POS[j] for j in POLICY_JOINT_ORDER],
                                       dtype=np.float32)

ACTION_SCALE = 0.5  # JointPositionActionCfg.scale

# Gravity direction in world frame (IsaacLab GRAVITY_VEC_W), used for projected_gravity.
GRAVITY_VEC_W = np.array([0.0, 0.0, -1.0], dtype=np.float32)


# --- Observation layout -----------------------------------------------------------
# Each term -> its size. Order here is the concatenation order.
_TERM_SIZES = {
    "base_lin_vel": 3,
    "base_ang_vel": 3,
    "projected_gravity": 3,
    "velocity_commands": 3,
    "joint_pos_rel": len(POLICY_JOINT_ORDER),
    "joint_vel": len(POLICY_JOINT_ORDER),
    "last_action": len(POLICY_JOINT_ORDER),
}


@dataclass
class ObsCfg:
    """Ordered list of observation terms the policy was trained on."""
    terms: List[str]

    @property
    def dim(self) -> int:
        return sum(_TERM_SIZES[t] for t in self.terms)

    def validate(self):
        for t in self.terms:
            if t not in _TERM_SIZES:
                raise KeyError(f"unknown obs term '{t}'")


# Preset layouts (match the two registered IsaacLab tasks).
OBS_FLAT = ObsCfg(terms=[
    "base_lin_vel", "base_ang_vel", "projected_gravity", "velocity_commands",
    "joint_pos_rel", "joint_vel", "last_action",
])  # dim 36 — dodo_flat (walking), SIM-ONLY (needs base_lin_vel)
OBS_STAND = ObsCfg(terms=[
    "base_ang_vel", "projected_gravity", "joint_pos_rel", "joint_vel", "last_action",
])  # dim 30 — dodo_stand (balance), DEPLOYABLE


class Env(ABC):
    """Sim-to-real environment interface.

    Subclasses provide the raw signals (`_read_signals`) and how a position target
    is applied + time advanced (`_apply_and_advance`). This base owns the policy
    contract: action->target mapping, observation assembly, and last_action state.
    All joint quantities here are in the POLICY joint order, radians / rad·s.
    """

    def __init__(self, obs_cfg: ObsCfg, command: Sequence[float] = (0.0, 0.0, 0.0),
                 control_dt: float = 0.02):
        obs_cfg.validate()
        self.obs_cfg = obs_cfg
        self.command = np.asarray(command, dtype=np.float32)  # (vx, vy, yaw_rate)
        self.control_dt = control_dt                          # 50 Hz (sim dt 0.005 * decimation 4)
        self.n_joints = len(POLICY_JOINT_ORDER)
        self._last_action = np.zeros(self.n_joints, dtype=np.float32)

    # ---- dimensions ----
    @property
    def obs_dim(self) -> int:
        return self.obs_cfg.dim

    @property
    def action_dim(self) -> int:
        return self.n_joints

    # ---- policy contract ----
    def action_to_targets(self, action: np.ndarray) -> np.ndarray:
        """Map a raw policy action to absolute joint position targets [rad],
        in policy order: target = default_pos + ACTION_SCALE * action."""
        return DEFAULT_POS_VEC + ACTION_SCALE * np.asarray(action, dtype=np.float32)

    def _assemble_obs(self, signals: Dict[str, np.ndarray]) -> np.ndarray:
        """Concatenate the configured obs terms from a signal dict."""
        signals = {
            **signals,
            "velocity_commands": self.command,
            "last_action": self._last_action,
        }
        parts = []
        for term in self.obs_cfg.terms:
            v = np.asarray(signals[term], dtype=np.float32).reshape(-1)
            if v.size != _TERM_SIZES[term]:
                raise ValueError(f"obs term '{term}' has size {v.size}, "
                                 f"expected {_TERM_SIZES[term]}")
            parts.append(v)
        return np.concatenate(parts).astype(np.float32)

    def get_observation(self) -> np.ndarray:
        return self._assemble_obs(self._read_signals())

    def step(self, action: Sequence[float]) -> np.ndarray:
        """Apply one policy action and return the next observation."""
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.size != self.n_joints:
            raise ValueError(f"action dim {action.size} != {self.n_joints}")
        self._last_action = action
        targets = self.action_to_targets(action)
        self._apply_and_advance(targets)
        return self.get_observation()

    def reset(self) -> np.ndarray:
        self._last_action = np.zeros(self.n_joints, dtype=np.float32)
        self._reset_backend()
        return self.get_observation()

    # ---- backend hooks ----
    @abstractmethod
    def _read_signals(self) -> Dict[str, np.ndarray]:
        """Return raw signals in POLICY joint order: base_lin_vel(3),
        base_ang_vel(3), projected_gravity(3), joint_pos_rel(8), joint_vel(8).
        (velocity_commands and last_action are added by the base.)"""

    @abstractmethod
    def _apply_and_advance(self, targets: np.ndarray) -> None:
        """Apply joint position targets [rad, policy order] and advance one
        control step (sim: step physics; real: publish + wait control_dt)."""

    def _reset_backend(self) -> None:
        """Optional per-backend reset (sim: re-pose robot)."""

    def close(self) -> None:
        """Optional cleanup."""


# --- Helpers shared by backends -------------------------------------------------
def projected_gravity_from_quat(quat_wxyz: Sequence[float]) -> np.ndarray:
    """Gravity unit vector expressed in the body frame = R_wb^T @ (0,0,-1).
    quat is (w, x, y, z). Matches IsaacLab mdp.projected_gravity."""
    w, x, y, z = quat_wxyz
    # Rotation matrix world<-body; we need body<-world = transpose, applied to g_w.
    # Equivalent: rotate g_w by the inverse (conjugate) quaternion.
    gx, gy, gz = GRAVITY_VEC_W
    # v' = q^-1 * v * q  ; for unit quat q^-1 = (w,-x,-y,-z).
    # Use rotation-matrix transpose form for clarity.
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)
    return (R.T @ np.array([gx, gy, gz], dtype=np.float32)).astype(np.float32)


class OnnxPolicy:
    """Loads an exported rsl_rl policy.onnx (input 'obs', output 'actions')."""

    def __init__(self, path: str):
        import onnxruntime as ort
        self.sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        self.in_name = self.sess.get_inputs()[0].name
        self.out_name = self.sess.get_outputs()[0].name
        self.obs_dim = self.sess.get_inputs()[0].shape[-1]
        self.action_dim = self.sess.get_outputs()[0].shape[-1]

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        out = self.sess.run([self.out_name], {self.in_name: x})[0]
        return out.reshape(-1).astype(np.float32)
