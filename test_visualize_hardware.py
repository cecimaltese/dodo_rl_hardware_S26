"""
test_visualize_hardware.py — the IMU half of visualize_hardware.py, without hardware.

The model's body must tilt the way the robot tilts: through the deployment path
(imu_protocol's firmware scales and AXIS_REMAP), showing the tilt only, since the fused
heading is arbitrary without a magnetometer. A fake Nicla (test_imu_locate's) mounted
with the real AXIS_REMAP stands in for the board.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np
import pytest

import imu_protocol as proto
import visualize_hardware as viz
from rl_env import projected_gravity_from_quat
from test_imu_locate import V2, FakeNicla, rot_x, rot_y, rot_z

RNG = np.random.default_rng(7)


# rl_env.projected_gravity_from_quat works in float32: compare to 1e-6, not 1e-9.
def same_rotation(q1, q2, tol=1e-6) -> bool:
    return min(np.abs(np.subtract(q1, q2)).max(), np.abs(np.add(q1, q2)).max()) < tol


def mat_to_quat(r) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(r, dtype=float).ravel())
    return q


class TestTiltMath:
    @pytest.mark.parametrize("k", range(20))
    def test_round_trip(self, k):
        pitch, roll = np.radians(RNG.uniform(-80, 80, size=2))
        q = viz.quat_from_tilt(pitch, roll)
        assert viz.tilt_from_gravity(projected_gravity_from_quat(q)) == pytest.approx(
            (pitch, roll), abs=1e-6)

    def test_signs_are_imu_locates(self):
        """nose-down = +pitch (about +y), right-side-down = +roll (about +x)."""
        p, r = viz.tilt_from_gravity(projected_gravity_from_quat(mat_to_quat(rot_y(30))))
        assert (math.degrees(p), r) == pytest.approx((30, 0), abs=1e-5)
        p, r = viz.tilt_from_gravity(projected_gravity_from_quat(mat_to_quat(rot_x(20))))
        assert (p, math.degrees(r)) == pytest.approx((0, 20), abs=1e-5)

    def test_heading_dropped_unless_asked(self):
        full = mat_to_quat(rot_z(70) @ rot_y(12) @ rot_x(-7))
        q, src = viz.base_quat({"quat": tuple(full), "accel": (0, 0, 9.81)})
        assert src == "fused"
        assert same_rotation(q, mat_to_quat(rot_y(12) @ rot_x(-7)))
        q, _ = viz.base_quat({"quat": tuple(full), "accel": (0, 0, 9.81)}, keep_yaw=True)
        assert same_rotation(q, full)

    def test_accel_fallback_agrees_with_the_quaternion(self):
        r = rot_y(-9) @ rot_x(14)
        g = projected_gravity_from_quat(mat_to_quat(r))
        q, src = viz.base_quat({"accel": tuple(-9.81 * g)})     # v1: no quaternion
        assert src == "accel"
        assert same_rotation(q, mat_to_quat(r))


class TestImuFeed:
    def fake(self):
        return FakeNicla(proto.AXIS_REMAP)       # the board as actually mounted

    def test_tilt_and_gyro_through_the_deployment_path(self):
        imu = self.fake()
        imu.world = rot_z(40) @ rot_y(10) @ rot_x(-5)    # heading 40: must not show
        imu.omega = np.array([0.0, 0.5, 0.0])            # turning nose-down
        feed = viz.ImuFeed(imu, V2)
        reading = feed.poll_once()
        assert reading is not None and feed.n == 1
        q, src = viz.base_quat(reading)
        assert src == "fused"
        p, r = viz.tilt_from_gravity(projected_gravity_from_quat(q))
        assert (math.degrees(p), math.degrees(r)) == pytest.approx((10, -5), abs=1e-3)
        assert np.array(reading["gyro"]) == pytest.approx([0, 0.5, 0], abs=1e-3)
        age, latest = feed.latest()
        assert age < 1.0 and latest is reading
        assert "nose-down +10.0" in viz.imu_text(reading, q, src)

    def test_the_model_body_tilts_like_the_robot(self):
        """Posed with base_quat, the meshed model's body has the robot's gravity."""
        m = viz.build_view_model()
        d = mujoco.MjData(m)
        adr = viz.base_qpos_adr(m)
        assert adr is not None
        imu = self.fake()
        imu.world = rot_z(-25) @ rot_y(-6) @ rot_x(18)
        reading = viz.ImuFeed(imu, V2).poll_once()
        q, _ = viz.base_quat(reading)
        d.qpos[adr + 3:adr + 7] = q
        mujoco.mj_forward(m, d)
        body_g = d.xmat[m.body("body").id].reshape(3, 3).T @ [0, 0, -1]
        robot_g = rot_x(18).T @ rot_y(-6).T @ [0, 0, -1]          # heading drops out
        assert body_g == pytest.approx(robot_g, abs=1e-3)

    def test_a_silent_board_leaves_no_reading(self):
        class Silent:
            def reset_input_buffer(self): pass
            def write(self, _): pass
            def readline(self): return b""
        feed = viz.ImuFeed(Silent(), V2)
        assert feed.poll_once() is None and feed.latest() is None and feed.n == 0
