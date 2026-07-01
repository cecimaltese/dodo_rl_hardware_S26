"""
sim_env.py — SimEnv(Env): MuJoCo backend for verifying the policy before hardware.

Loads the robot URDF, makes it a floating base on a ground plane, adds torque
actuators + an IMU site (gyro / velocimeter / orientation), and runs software PD
control that mirrors IsaacLab's ImplicitActuator (stiffness 32, damping 3.0,
effort limits 27 Nm hips/upper, 9 Nm knee/ankle), at sim dt 0.005 with
decimation 4 → 50 Hz control. Observations are read from the sim and remapped
from MuJoCo joint order into the policy joint order.

This is the sim-to-sim verification gate: run the SAME policy + runner here first;
if it behaves (e.g. dodo_flat walks forward on command), the obs/action plumbing
and joint mapping are correct and RealEnv is a drop-in swap.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Dict, List, Optional

import mujoco
import numpy as np

from rl_env import (Env, ObsCfg, OBS_FLAT, POLICY_JOINT_ORDER,
                    DEFAULT_JOINT_POS, projected_gravity_from_quat)

# PD gains + effort limits. MUST match the trained policy's actuator cfg so the same
# policy behaves the same across IsaacLab, MuJoCo, and hardware. The dodo_stand run
# (params/env.yaml) uses an ImplicitActuator with stiffness=32, damping=3.0 and
# effort_limit_sim 27 Nm (hips/upper) / 9 Nm (knee/ankle). This MuJoCo torque-PD loop
# mirrors that: tau = KP*(target-q) - KD*qd, clipped to the effort limits.
KP = 32.0
KD = 3.0
EFFORT_LIMIT = {  # Nm, by joint-name prefix
    "hip_": 27.0, "upper_leg_": 27.0, "lower_leg_": 9.0, "foot_": 9.0,
}
SIM_DT = 0.005
DECIMATION = 4
SPAWN_HEIGHT = 0.50  # m, from init_state.pos


def _effort_for(joint: str) -> float:
    for p, v in EFFORT_LIMIT.items():
        if joint.startswith(p):
            return v
    raise KeyError(joint)


def _build_model(urdf_path: str) -> mujoco.MjModel:
    """Load the URDF and augment it into a floating-base, actuated, sensed model."""
    spec = mujoco.MjSpec.from_file(str(urdf_path))
    spec.option.timestep = SIM_DT
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

    # Disable robot self-collisions: only robot<->floor should contact (the URDF's
    # coarse box collisions overlap at the joints and blow up otherwise). Collision
    # rule: collide iff (contype_a & conaff_b) or (contype_b & conaff_a).
    #   robot geoms: contype=2, conaff=1   floor: contype=1, conaff=1
    #   robot-robot: (2&1)|(2&1)=0  (no);  robot-floor: (2&1)|(1&1)=1 (yes).
    for g in spec.geoms:
        g.contype = 2
        g.conaffinity = 1
    # Match IsaacLab's implicit actuator on each revolute joint:
    #  - armature 0.01 (reflected rotor inertia), from the actuator cfg;
    #  - zero passive joint damping/friction so the ONLY velocity-dependent term is
    #    the software PD's KD below. (MuJoCo imports <dynamics damping="0.01"> from the
    #    URDF as passive dof_damping; IsaacLab's ImplicitActuator drives damping=KD and
    #    adds no separate passive damping, so the 0.01 would be a sim2sim mismatch.)
    for j in spec.joints:
        if j.type == mujoco.mjtJoint.mjJNT_HINGE:
            j.armature = 0.01
            j.damping = [0.0, 0.0, 0.0]   # MjSpec stores joint damping as a 3-vector
            j.frictionloss = 0.0

    # Ground plane in the world (a light is only needed for rendering).
    spec.worldbody.add_geom(
        name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[0, 0, 0.05], contype=1, conaffinity=1, rgba=[0.4, 0.5, 0.6, 1.0])

    # Floating base: give the root body ("body") a free joint.
    base = spec.body("body")
    base.add_freejoint()

    # IMU site on the base (aligned with body frame).
    imu = base.add_site(name="imu", pos=[0, 0, 0])

    # Torque (motor) actuators on each revolute joint; PD done in software.
    for j in POLICY_JOINT_ORDER:
        a = spec.add_actuator(name=f"act_{j}")
        a.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a.target = j
        a.gainprm[0] = 1.0  # ctrl is torque directly

    # IMU sensors: angular velocity (gyro) + linear velocity (velocimeter), both in
    # the site/body frame, and orientation quaternion for projected_gravity.
    spec.add_sensor(name="imu_gyro", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_vel", type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_quat", type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")

    return spec.compile()


class SimEnv(Env):
    def __init__(self, urdf_path: str = "dodo_files/urdf/dodo_daimao.urdf",
                 obs_cfg: ObsCfg = OBS_FLAT, command=(0.0, 0.0, 0.0),
                 render: bool = False):
        super().__init__(obs_cfg, command=command, control_dt=SIM_DT * DECIMATION)

        self.model = _build_model(urdf_path)
        self.data = mujoco.MjData(self.model)

        m = self.model
        # Policy-order index maps into MuJoCo qpos/qvel/ctrl.
        self._qpos_adr = np.array(
            [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
             for j in POLICY_JOINT_ORDER])
        self._dof_adr = np.array(
            [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
             for j in POLICY_JOINT_ORDER])
        self._act_id = np.array(
            [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{j}")
             for j in POLICY_JOINT_ORDER])
        self._effort = np.array([_effort_for(j) for j in POLICY_JOINT_ORDER], dtype=np.float32)
        self._default = np.array([DEFAULT_JOINT_POS[j] for j in POLICY_JOINT_ORDER],
                                 dtype=np.float32)

        # Sensor address lookup.
        self._sens = {name: self._sensor_adr(name)
                      for name in ("imu_gyro", "imu_vel", "imu_quat")}
        self._targets = self._default.copy()

        self.render = render
        self.viewer = None
        self._wall_next = None
        if render:
            import mujoco.viewer as mj_viewer
            self.viewer = mj_viewer.launch_passive(self.model, self.data)

        self._reset_backend()

    def is_running(self) -> bool:
        """Whether the loop should continue (False once the viewer window closes)."""
        return self.viewer.is_running() if self.viewer is not None else True

    def _sensor_adr(self, name: str) -> slice:
        m = self.model
        sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
        adr, dim = m.sensor_adr[sid], m.sensor_dim[sid]
        return slice(adr, adr + dim)

    # ---- backend hooks ----
    def _reset_backend(self):
        mujoco.mj_resetData(self.model, self.data)
        # Free-joint base: [x,y,z, qw,qx,qy,qz]; first free joint qpos starts at 0.
        self.data.qpos[0:3] = [0.0, 0.0, SPAWN_HEIGHT]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]  # upright
        self.data.qpos[self._qpos_adr] = self._default
        self.data.qvel[:] = 0.0
        self._targets = self._default.copy()
        mujoco.mj_forward(self.model, self.data)

    def _apply_and_advance(self, targets: np.ndarray):
        self._targets = np.asarray(targets, dtype=np.float32)
        for _ in range(DECIMATION):
            q = self.data.qpos[self._qpos_adr]
            qd = self.data.qvel[self._dof_adr]
            tau = np.clip(KP * (self._targets - q) - KD * qd, -self._effort, self._effort)
            self.data.ctrl[self._act_id] = tau
            mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()
            # Pace to wall-clock so the motion is watchable in real time.
            now = time.perf_counter()
            if self._wall_next is None:
                self._wall_next = now
            self._wall_next += self.control_dt
            sleep = self._wall_next - now
            if sleep > 0:
                time.sleep(sleep)
            else:
                self._wall_next = now  # fell behind; don't accumulate lag

    def _read_signals(self) -> Dict[str, np.ndarray]:
        d = self.data
        base_ang_vel = np.array(d.sensordata[self._sens["imu_gyro"]], dtype=np.float32)
        base_lin_vel = np.array(d.sensordata[self._sens["imu_vel"]], dtype=np.float32)
        quat = np.array(d.sensordata[self._sens["imu_quat"]], dtype=np.float32)  # wxyz
        proj_g = projected_gravity_from_quat(quat)
        joint_pos_rel = (d.qpos[self._qpos_adr] - self._default).astype(np.float32)
        joint_vel = d.qvel[self._dof_adr].astype(np.float32)
        return {
            "base_lin_vel": base_lin_vel,
            "base_ang_vel": base_ang_vel,
            "projected_gravity": proj_g,
            "joint_pos_rel": joint_pos_rel,
            "joint_vel": joint_vel,
        }

    def close(self):
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None
