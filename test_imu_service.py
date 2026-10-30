"""
test_imu_service.py — parse_imu_line against REAL firmware line formats.

Poll firmware = imu.cpp (Nicla Sense ME): "ALL ax ay az gx gy gz pitch roll",
accel/gyro as raw int16 LSB (BHI260AP defaults: accel ±8 g -> 4096 LSB/g,
gyro ±2000 °/s -> 16.384 LSB/°/s), pitch/roll accel-derived degrees.

Stream firmware = firmware/imu_stream_nicla: "IMU gx gy gz ax ay az qw qx qy qz"
already in SI with a fused unit quaternion.
"""
from __future__ import annotations

import math

import pytest

import dodo_ipc as ipc
from imu_service import (ACC_LSB_PER_G, GRAVITY, GYRO_LSB_PER_DPS,
                         parse_imu_line, reading_to_msg)


class TestPollFirmware:  # imu.cpp, 'L' -> ALL line
    def test_flat_and_still(self):
        # Robot flat and still: accel = +1 g on z (raw 4096), gyro = 0.
        r = parse_imu_line("ALL 0.000 0.000 4096.000 0.000 0.000 0.000 0.00 0.00\n")
        assert r is not None
        assert r["accel"][0] == pytest.approx(0.0)
        assert r["accel"][2] == pytest.approx(GRAVITY, abs=1e-6)
        assert r["gyro"] == pytest.approx((0.0, 0.0, 0.0))
        assert "quat" not in r  # pitch/roll deliberately discarded

    def test_field_order_accel_first(self):
        # The ALL line is accel-first; our wire/dict convention is gyro-first.
        # 16.384 raw gyro = 1 °/s; 4096 raw accel = 1 g.
        r = parse_imu_line("ALL 4096.0 0.0 0.0 16.384 0.0 0.0 12.34 -5.67")
        assert r["accel"][0] == pytest.approx(GRAVITY)
        assert r["gyro"][0] == pytest.approx(math.radians(1.0), rel=1e-6)

    def test_scale_constants(self):
        assert ACC_LSB_PER_G == pytest.approx(32768 / 8)       # ±8 g
        assert GYRO_LSB_PER_DPS == pytest.approx(32768 / 2000)  # ±2000 °/s

    def test_truncated_and_garbage(self):
        assert parse_imu_line("ALL 1.0 2.0 3.0\n") is None
        assert parse_imu_line("READY\n") is None
        assert parse_imu_line("ACCEL 1.0 2.0 3.0\n") is None
        assert parse_imu_line("ALL a b c d e f g h\n") is None
        assert parse_imu_line("\n") is None
        assert parse_imu_line("") is None


class TestStreamFirmware:  # firmware/imu_stream_nicla, SI units + fused quat
    LINE = "IMU 0.0100 -0.0200 0.0300 0.100 -0.200 9.810 1.0000 0.0000 0.0000 0.0000"

    def test_si_passthrough_and_quat(self):
        r = parse_imu_line(self.LINE)
        assert r["gyro"] == pytest.approx((0.01, -0.02, 0.03))
        assert r["accel"] == pytest.approx((0.1, -0.2, 9.81))
        assert r["quat"] == pytest.approx((1.0, 0.0, 0.0, 0.0))

    def test_unconverged_zero_quat_dropped(self):
        r = parse_imu_line("IMU 0 0 0 0 0 9.81 0.0 0.0 0.0 0.0")
        assert r is not None and "quat" not in r

    def test_gyro_accel_only_line(self):
        r = parse_imu_line("IMU 0.1 0.2 0.3 0.4 0.5 0.6")
        assert r["gyro"] == pytest.approx((0.1, 0.2, 0.3))
        assert "quat" not in r


class TestToMessage:
    def test_quat_flag_set_only_when_fused(self):
        with_q = reading_to_msg(parse_imu_line(TestStreamFirmware.LINE))
        assert with_q.flags & ipc.IMU_QUAT_VALID
        no_q = reading_to_msg(
            parse_imu_line("ALL 0 0 4096 0 0 0 0.0 0.0"))
        assert not (no_q.flags & ipc.IMU_QUAT_VALID)

    def test_lin_vel_never_claimed(self):
        # base_lin_vel is unobservable on hardware — the flag must NEVER be set
        # by imu_service (only sim_service may set it).
        msg = reading_to_msg(parse_imu_line(TestStreamFirmware.LINE))
        assert not (msg.flags & ipc.IMU_LIN_VEL_VALID)

    def test_wire_roundtrip(self):
        msg = reading_to_msg(parse_imu_line(TestStreamFirmware.LINE))
        back = ipc.decode(msg.pack(seq=1))
        assert back.gyro == pytest.approx(msg.gyro, rel=1e-6)
        assert back.accel == pytest.approx(msg.accel, rel=1e-6)
        assert back.quat_valid and not back.lin_vel_valid
