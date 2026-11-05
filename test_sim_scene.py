"""
test_sim_scene.py — `sim_service --scene`: dodo_assets' scene.xml (Shafeef's PD actuators)
as the virtual robot, and the PD stand through the same ZMQ path the real robot uses:
tools/dodo_startup.py, run as a subprocess, against the service in a thread.

The two rehearsals are the hardware plan (HANDOFF log 2026-11-05 (c)):
  * on the floor: straight legs -> ramp to `home` -> stand;
  * on the gantry: hang limp -> enable -> ramp to `home` in the air -> lower until the
    feet carry the weight -> release -> stand.
"""
from __future__ import annotations

import math
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import mujoco
import numpy as np
import pytest

import assets
import dodo_ipc as ipc
import real2sim
from rl_env import projected_gravity_from_quat
from sim_service import (GANTRY_CLEARANCE_M, GANTRY_ROPE_M, GROUPS, SCENE_DT,
                         SimService, build_scene_model)

STARTUP = Path(__file__).resolve().parent / "tools" / "dodo_startup.py"
HOME_Z = 0.412          # pd_stand.py's settled base height in `home` (0.4121 m)
MAX_TILT_DEG = 8.0      # pd_stand.py: about 6 deg in `home`


@contextmanager
def service(**kwargs):
    """SimService.from_scene running in a thread, stopped (sockets closed) after."""
    svc = SimService.from_scene(**kwargs)
    th = threading.Thread(target=svc.run, kwargs={"max_wall_s": 60.0}, daemon=True)
    th.start()
    time.sleep(0.3)                      # let physics + publishers spin up
    try:
        yield svc
    finally:
        svc._running = False
        th.join(timeout=3.0)


def startup(*args) -> subprocess.Popen:
    """tools/dodo_startup.py, non-interactive, holding until --hold-secs runs out and
    then disabling without the park ramp (the tests read the stand before that)."""
    return subprocess.Popen([sys.executable, str(STARTUP), "--yes", "--no-park", *args],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def finish(p: subprocess.Popen, timeout_s: float = 30.0) -> str:
    out, _ = p.communicate(timeout=timeout_s)
    assert p.returncode == 0, out
    return out


class TestSceneModel:
    def test_gains_and_step_are_shafeefs_unless_overridden(self):
        m = build_scene_model()
        assert m.opt.timestep == SCENE_DT
        kp = {m.actuator(a).name: m.actuator_gainprm[a, 0] for a in range(m.nu)}
        kd = {m.actuator(a).name: -m.actuator_biasprm[a, 2] for a in range(m.nu)}
        assert kp["upper_leg_left"] == 150 and kp["foot_right"] == 50
        assert kd["hip_right"] == pytest.approx(1.5) and kd["lower_leg_left"] == 0.8

        m = build_scene_model(kp=(10, 20, 30, 40), kd=(1, 2, 3, 4))
        for a in range(m.nu):
            g = GROUPS.index(m.actuator(a).name.rsplit("_", 1)[0])
            assert m.actuator_gainprm[a, 0] == (10, 20, 30, 40)[g]
            assert m.actuator_biasprm[a, 1] == -(10, 20, 30, 40)[g]
            assert m.actuator_biasprm[a, 2] == -(1, 2, 3, 4)[g]

    def test_gain_scale_like_motor_service(self):
        svc = SimService.from_scene()
        try:
            kp0, kd0 = svc._kp0.copy(), svc._kd0.copy()
            assert svc._ctl_gains({"scale": 0.25}) == {"gain_scale": 0.25}
            m = svc.model
            assert m.actuator_gainprm[:, 0] == pytest.approx(kp0 * 0.25)
            assert -m.actuator_biasprm[:, 1] == pytest.approx(kp0 * 0.25)
            assert -m.actuator_biasprm[:, 2] == pytest.approx(kd0 * 0.5)
            with pytest.raises(RuntimeError):
                svc._ctl_gains({"scale": 1.5})
        finally:
            svc.close()

    def test_gantry_adds_four_ropes(self):
        m = build_scene_model(gantry=True)
        ropes = [m.tendon(t).name for t in range(m.ntendon)]
        assert len(ropes) == 4 and all(r.startswith("gantry_rope") for r in ropes)
        assert (m.tendon_range[:, 1] == GANTRY_ROPE_M).all()

    def test_commands_are_clamped_inside_the_measured_stops(self):
        svc = SimService.from_scene()
        try:
            for i, j in enumerate(ipc.JOINT_ORDER):
                lo, hi = real2sim.command_band(j)
                assert (svc._clamp_lo[i], svc._clamp_hi[i]) == (lo, hi)
                s_lo, s_hi = real2sim.stops()[j]
                assert s_lo < lo < hi < s_hi
            home = assets.scene_pose("home")
            for i, j in enumerate(ipc.JOINT_ORDER):
                assert svc._clamp_lo[i] < home[j] < svc._clamp_hi[i], j
        finally:
            svc.close()


class TestSceneImu:
    """The scene's IMU sensors read in the CHIP frame, like the real chip; what the
    service publishes must be the BODY frame, via imu_protocol's AXIS_REMAP."""

    def test_published_imu_is_the_body_frame(self):
        svc = SimService.from_scene()
        try:
            m, d = svc.model, svc.data
            rng = np.random.default_rng(3)
            for _ in range(5):
                q = rng.normal(size=4)
                d.qpos[3:7] = q / np.linalg.norm(q)
                d.qvel[0:3] = rng.normal(size=3)
                d.qvel[3:6] = rng.normal(size=3)        # free joint: body-frame omega
                mujoco.mj_forward(m, d)
                msg = svc._imu_msg()
                R = d.xmat[m.body("body").id].reshape(3, 3)
                # 1e-6, not float noise: the record keeps the rotation to 9 decimals.
                # A frame mistake shows up at order 1.
                assert np.array(msg.gyro) == pytest.approx(d.qvel[3:6], abs=1e-6)
                assert projected_gravity_from_quat(msg.quat) == pytest.approx(
                    R.T @ [0, 0, -1], abs=1e-6)
                v = np.zeros(6)
                mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_SITE,
                                         m.site("imu").id, v, 0)
                assert np.array(msg.lin_vel) == pytest.approx(R.T @ v[3:], abs=1e-6)
        finally:
            svc.close()

    def test_accelerometer_reads_up_when_standing(self):
        svc = SimService.from_scene()
        try:
            for _ in range(2000):                       # 10 s, PD holding `home`: it
                svc._physics_tick()                     # settles slowly (still by ~8 s)
            assert np.abs(svc.data.qvel).max() < 1e-3   # still, so accel = -gravity
            msg = svc._imu_msg()
            a = np.array(msg.accel)
            assert np.linalg.norm(a) == pytest.approx(9.81, rel=0.02)
            g = projected_gravity_from_quat(msg.quat)
            assert math.degrees(math.acos(np.clip(-g @ a / np.linalg.norm(a), -1, 1))) < 0.5
        finally:
            svc.close()


class TestPdStand:
    def test_on_the_floor_through_dodo_startup(self):
        """Straight legs on the floor -> dodo_startup ramps to `home` and holds."""
        with service(start="straight") as svc:
            out = finish(startup("--pose", "home", "--ramp", "1.5", "--hold-secs", "2"))
            t = svc.track
        assert "ramp to 'home'" in out
        assert t["base_z"] == pytest.approx(HOME_Z, abs=0.01), t
        assert t["max_tilt_deg"] < MAX_TILT_DEG, t
        assert t["max_target_err_rad"] < 0.1, t

    def test_gantry_bring_up(self):
        """The hardware procedure: hang limp, enable + ramp in the air, lower until the
        feet carry the robot, release, stand."""
        with service(start="straight", gantry=True, auto_enable=False) as svc:
            ctl = ipc.CtlClient()
            try:
                st = ctl.request("status")["sim"]
                hang = st["base_z"]
                assert hang == pytest.approx(0.4468 + GANTRY_CLEARANCE_M, abs=0.005)
                assert st["gantry"]["taut"] == 4 and not st["gantry"]["released"]

                p = startup("--pose", "home", "--ramp", "1.5", "--hold-secs", "7")
                time.sleep(2.5)                         # ramped to `home`, in the air
                st = ctl.request("status")["sim"]
                assert st["base_z"] == pytest.approx(hang, abs=0.005)   # still hanging
                for _ in range(6):                      # pay out 12 cm of rope
                    ctl.request("gantry", lower=0.02)
                    time.sleep(0.25)
                assert ctl.request("gantry")["taut"] == 0       # feet carry it now
                assert ctl.request("gantry", release=True)["released"]
                time.sleep(1.5)
                finish(p)
            finally:
                ctl.close()
            t = svc.track
        assert t["base_z"] == pytest.approx(HOME_Z, abs=0.01), t
        assert t["max_tilt_deg"] < MAX_TILT_DEG, t


def test_pd_push_test_against_the_sim():
    """tools/pd_push_test.py through the real stack: the knee held in the air, pushed
    by a slowly varying external torque. The sim's PD has no velocity cap, so the
    stiffness must match kp and there is no plateau."""
    import threading as th
    push_tool = Path(__file__).resolve().parent / "tools" / "pd_push_test.py"
    with service(start="straight", gantry=True) as svc:
        dof = svc.model.jnt_dofadr[svc.model.joint("lower_leg_left").id]
        stop = th.Event()

        def push():
            t0 = time.monotonic()
            while not stop.is_set():
                t = time.monotonic() - t0
                svc.data.qfrc_applied[dof] = 1.5 * math.sin(0.6 * t) ** 2   # slow, <= 1.5 Nm
                time.sleep(0.005)
        pusher = th.Thread(target=push, daemon=True)
        pusher.start()
        try:
            out = subprocess.run([sys.executable, str(push_tool), "--joint",
                                  "lower_leg_left", "--secs", "5"],
                                 capture_output=True, text=True, timeout=30).stdout
        finally:
            stop.set()
            pusher.join(timeout=1.0)
    assert "stiffness matches kp" in out, out
    assert "no plateau" in out, out
