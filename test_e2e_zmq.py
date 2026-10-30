"""
test_e2e_zmq.py — end-to-end tests of the ZMQ pipeline:

    SimService (real-time MuJoCo, own thread)  <-- tcp -->  ZmqEnv (policy side)

Two tiers:

1. `TestSyntheticRobot` — runs ANYWHERE (no URDF needed): a synthetic 8-joint
   test rig (heavy pancake base resting on the floor + 8 PD-held pendulums,
   joints named exactly like the dodo) exercises the ENTIRE stack: sockets,
   schemas, conflate/latest, calibration/enable gating, real-time pacing,
   staleness detection. This is the Phase-2 plumbing gate on the dev Macs.

2. `test_stand_policy_over_zmq` — the REAL acceptance test (robot URDF + a
   trained dodo_stand ONNX): the stand policy must keep the dodo upright for
   10 s through the full network stack. Skips if no dodo_stand ONNX is found.
   Run:

       .venv/bin/python -m pytest test_e2e_zmq.py -v
"""
from __future__ import annotations

import socket
import threading
import time
from glob import glob
from pathlib import Path

import mujoco
import numpy as np
import pytest

import dodo_ipc as ipc
from rl_env import DEFAULT_JOINT_POS, DEFAULT_POS_VEC, OBS_STAND
from sim_env import SIM_DT, URDF_PATH

HW_DIR = Path(__file__).resolve().parent
URDF = URDF_PATH
RUN_ROOTS = (HW_DIR / "policies", HW_DIR.parent / "dodo_rl")  # dodo_rl is optional


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(autouse=True)
def isolated_ports(monkeypatch):
    for topic in ipc.DEFAULT_PORTS:
        monkeypatch.setenv(f"DODO_ZMQ_PORT_{topic.upper()}", str(_free_port()))
    yield


# ---------------------------------------------------------------------------
# synthetic 8-joint test rig (no URDF required)
# ---------------------------------------------------------------------------
def build_test_rig() -> mujoco.MjModel:
    """Heavy flat base resting on the floor with 8 small PD-held pendulums,
    joints/actuators/sensors named exactly as sim_service requires. The base
    cannot tip over, so zero-action must hold the default pose indefinitely —
    any drift/instability the test sees comes from the pipeline, not physics."""
    spec = mujoco.MjSpec()
    spec.option.timestep = SIM_DT

    spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE,
                            size=[0, 0, 0.05], contype=1, conaffinity=1)
    base = spec.worldbody.add_body(name="body", pos=[0, 0, 0.3])
    base.add_freejoint()
    base.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.5, 0.5, 0.05],
                  mass=20.0, contype=2, conaffinity=1)
    base.add_site(name="imu", pos=[0, 0, 0])

    for i, j in enumerate(ipc.JOINT_ORDER):
        b = base.add_body(name=f"link_{j}", pos=[0.1 * i - 0.35, 0.3, 0.06])
        jt = b.add_joint(name=j, type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
        jt.armature = 0.01
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                   fromto=[0, 0, 0, 0, 0, 0.12], size=[0.015, 0, 0],
                   mass=0.1, contype=0, conaffinity=0)
        a = spec.add_actuator(name=f"act_{j}")
        a.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a.target = j
        a.gainprm[0] = 1.0

    for name, stype in (("imu_gyro", mujoco.mjtSensor.mjSENS_GYRO),
                        ("imu_vel", mujoco.mjtSensor.mjSENS_VELOCIMETER),
                        ("imu_quat", mujoco.mjtSensor.mjSENS_FRAMEQUAT)):
        spec.add_sensor(name=name, type=stype,
                        objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    return spec.compile()


@pytest.fixture
def sim_rig():
    """SimService on the synthetic rig, running in a background thread."""
    from sim_service import SimService
    svc = SimService(model=build_test_rig(), auto_enable=True, spawn_height=0.08)
    th = threading.Thread(target=svc.run, kwargs={"max_wall_s": 60.0}, daemon=True)
    th.start()
    time.sleep(0.3)                      # let physics + publishers spin up
    yield svc
    svc._running = False
    th.join(timeout=3.0)


class TestSyntheticRobot:
    def test_full_pipeline_realtime(self, sim_rig):
        """The core Phase-2 gate: obs flow, values sane, pacing is wall-clock."""
        from zmq_env import ZmqEnv
        env = ZmqEnv(obs_cfg=OBS_STAND, control_dt=0.02)
        try:
            obs = env.reset()
            assert obs.shape == (30,)
            assert np.all(np.isfinite(obs))

            n_steps = 100                            # 2.0 s at 50 Hz
            t0 = time.monotonic()
            for _ in range(n_steps):
                obs = env.step(np.zeros(env.action_dim, dtype=np.float32))
            elapsed = time.monotonic() - t0

            # REAL-TIME pacing: 100 steps @ 20 ms must take ~2 s of wall time
            # (the old lockstep SimEnv would blast through this in ~0.05 s).
            assert 1.8 <= elapsed <= 2.6, f"not wall-clock paced: {elapsed:.2f}s"

            # zero action -> default pose PD-held on a base that can't tip:
            # joint_pos_rel ≈ 0, gravity straight down, base still.
            joint_pos_rel = obs[6:14]
            assert np.max(np.abs(joint_pos_rel)) < 0.15
            proj_g = obs[3:6]
            assert proj_g[2] == pytest.approx(-1.0, abs=0.05)
            base_ang_vel = obs[0:3]
            assert np.max(np.abs(base_ang_vel)) < 0.5
        finally:
            env.close()

    def test_pipeline_survives_latency_and_drops(self):
        """Injected 20 ms latency + 5% drops must not break the obs pipeline
        (staleness threshold is 250 ms; conflate keeps newest)."""
        from sim_service import SimService
        from zmq_env import ZmqEnv
        svc = SimService(model=build_test_rig(), auto_enable=True,
                         spawn_height=0.08, extra_latency_s=0.020, drop_prob=0.05)
        th = threading.Thread(target=svc.run, kwargs={"max_wall_s": 30.0}, daemon=True)
        th.start()
        time.sleep(0.3)
        env = ZmqEnv(obs_cfg=OBS_STAND, control_dt=0.02)
        try:
            obs = env.reset()
            for _ in range(50):
                obs = env.step(np.zeros(env.action_dim, dtype=np.float32))
            assert np.all(np.isfinite(obs))
        finally:
            env.close()
            svc._running = False
            th.join(timeout=3.0)

    def test_reset_refuses_disabled_service(self, sim_rig):
        """Bring-up gating: a disabled fleet must be rejected loudly."""
        from zmq_env import ZmqEnv
        ctl = ipc.CtlClient()
        env = ZmqEnv(obs_cfg=OBS_STAND, control_dt=0.02)
        try:
            assert ctl.request("disable")["ok"]
            time.sleep(0.15)          # let a post-disable states msg circulate
            with pytest.raises(RuntimeError, match="not enabled"):
                env.reset()
            assert ctl.request("enable")["ok"]
            time.sleep(0.15)
            env.reset()                              # now fine
        finally:
            env.close()
            ctl.close()

    def test_step_detects_dead_service(self):
        """Staleness watchdog: killing the service mid-run must raise, not
        silently feed the policy frozen observations."""
        from sim_service import SimService
        from zmq_env import ZmqEnv
        svc = SimService(model=build_test_rig(), auto_enable=True, spawn_height=0.08)
        th = threading.Thread(target=svc.run, kwargs={"max_wall_s": 30.0}, daemon=True)
        th.start()
        time.sleep(0.3)
        env = ZmqEnv(obs_cfg=OBS_STAND, control_dt=0.02, max_state_age_s=0.25)
        try:
            env.reset()
            env.step(np.zeros(env.action_dim, dtype=np.float32))
            svc._running = False                     # "unplug" the robot
            th.join(timeout=3.0)
            time.sleep(0.4)                          # > max_state_age_s
            # One message may still be parked in the conflated queue (see the
            # _check_health note) — detection must fire within a few steps.
            with pytest.raises(RuntimeError, match="stale"):
                for _ in range(3):
                    env.step(np.zeros(env.action_dim, dtype=np.float32))
        finally:
            env.close()

    def test_startup_ramp_reaches_default_pose(self, sim_rig):
        """The pre-start routine (boot pose -> default pose, no homing): reset
        with startup_ramp_s must take the ramp+settle time, keep the command
        watchdog fed the whole way, and END with the joints at the default
        pose so the policy takes over with zero discontinuity."""
        from zmq_env import ZmqEnv
        env = ZmqEnv(obs_cfg=OBS_STAND, control_dt=0.02, startup_ramp_s=1.0)
        try:
            t0 = time.monotonic()
            obs = env.reset()                        # includes the 1.0s ramp
            elapsed = time.monotonic() - t0
            assert elapsed >= 1.2, f"ramp skipped? reset took {elapsed:.2f}s"
            assert elapsed <= 3.5, f"ramp too slow: {elapsed:.2f}s"
            joint_pos_rel = obs[6:14]
            assert np.max(np.abs(joint_pos_rel)) < 0.15
        finally:
            env.close()

    def test_flat_obs_allowed_only_with_sim_lin_vel(self, sim_rig):
        """OBS_FLAT needs base_lin_vel: sim_service provides it (flag set),
        so this passes here — and ZmqEnv must reject it if the flag is absent
        (covered implicitly: imu_service never sets IMU_LIN_VEL_VALID)."""
        from rl_env import OBS_FLAT
        from zmq_env import ZmqEnv
        env = ZmqEnv(obs_cfg=OBS_FLAT, control_dt=0.02)
        try:
            obs = env.reset()
            assert obs.shape == (36,)
        finally:
            env.close()


# ---------------------------------------------------------------------------
# the real acceptance test (needs URDF + a trained dodo_stand ONNX)
# ---------------------------------------------------------------------------
def _latest_stand_onnx():
    hits = [h for root in RUN_ROOTS for h in glob(str(
        root / "**" / "dodo_stand" / "*" / "exported" / "policy.onnx"), recursive=True)]
    # run dirs are timestamps -> newest run last
    return max(hits, key=lambda h: Path(h).parent.parent.name) if hits else None


@pytest.mark.skipif(not URDF.exists(), reason=f"robot URDF not found at {URDF}")
@pytest.mark.skipif(_latest_stand_onnx() is None, reason="no dodo_stand policy.onnx")
def test_stand_policy_over_zmq():
    """PHASE-2 ACCEPTANCE: the stand policy keeps the real dodo model upright
    for 10 s through the full ZMQ stack (sim_service + ZmqEnv + ONNX policy)."""
    from rl_env import OnnxPolicy
    from sim_service import SimService
    from zmq_env import ZmqEnv

    svc = SimService(urdf_path=str(URDF), auto_enable=True)
    th = threading.Thread(target=svc.run, kwargs={"max_wall_s": 60.0}, daemon=True)
    th.start()
    time.sleep(0.3)

    policy = OnnxPolicy(_latest_stand_onnx())
    env = ZmqEnv(obs_cfg=OBS_STAND, control_dt=0.02)
    try:
        assert policy.obs_dim == env.obs_dim
        obs = env.reset()
        for k in range(500):                         # 10 s @ 50 Hz
            obs = env.step(policy(obs))
        # Upright = base still high + gravity still pointing down in body frame.
        base_z = float(svc.data.qpos[2])
        assert base_z > 0.30, f"robot fell: base z {base_z:.2f} m"
        proj_g_z = obs[3:6][2]
        assert proj_g_z < -0.85, f"robot tipped: proj_g z {proj_g_z:.2f}"
    finally:
        env.close()
        svc._running = False
        th.join(timeout=3.0)
