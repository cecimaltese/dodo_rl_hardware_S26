"""
test_imu_service.py — the IMU path from firmware line to ZMQ message, no hardware.

Lines use the real formats of firmware/imu_firmware_v1 and _v2 as parsed by
imu_protocol.py: accel/gyro are raw int16 counts scaled by the sensor range, and
everything is rotated into the body frame by imu_protocol.AXIS_REMAP. Expectations
go through that same remap, so they stay valid once AXIS_REMAP is measured.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

import dodo_ipc as ipc
import imu_protocol as proto
from imu_service import reading_to_msg, sensor_scales
from rl_env import projected_gravity_from_quat

G = 9.80665
V2 = {"proto": 2, "accel_range_g": 4.0, "gyro_range_dps": 1000.0}  # as flashed
GYRO_V2, ACCEL_V2, _ = sensor_scales(V2)
GYRO_V1, ACCEL_V1, _ = sensor_scales(None)  # v1: the guessed ranges


def body(x, y, z):
    """A board-frame vector as the parser reports it (through AXIS_REMAP)."""
    return proto._remap(x, y, z)


class TestScales:
    def test_v2_self_report_wins_over_guess(self):
        gyro, accel, src = sensor_scales(V2, gyro_range_dps=2000.0, accel_range_g=8.0)
        assert gyro == pytest.approx(math.radians(1000.0) / 32768.0)
        assert accel == pytest.approx(4.0 * G / 32768.0)
        assert "self-reported" in src

    def test_v1_falls_back_to_guess(self):
        gyro, accel, src = sensor_scales(None)
        assert gyro == pytest.approx(math.radians(proto.GYRO_RANGE_DPS) / 32768.0)
        assert accel == pytest.approx(proto.ACCEL_RANGE_G * G / 32768.0)
        assert "GUESSED" in src


class TestFirmwareV2:  # 'L' -> "ALL" + 8 fields + fused quaternion
    FLAT = ("ALL 0.000 0.000 8192.000 0.000 0.000 0.000 0.00 0.00 "
            "1.000000 0.000000 0.000000 0.000000\r\n")

    def test_flat_and_still(self):
        r = proto.parse_imu_line(self.FLAT, GYRO_V2, ACCEL_V2)
        assert r["accel"] == pytest.approx(body(0.0, 0.0, G))  # 8192 counts = 1 g
        assert r["gyro"] == pytest.approx(body(0.0, 0.0, 0.0))
        assert "quat" in r

    def test_field_order_accel_first(self):
        # ALL is accel-first; 32.768 counts = 1 °/s at ±1000 °/s.
        r = proto.parse_imu_line("ALL 8192 0 0 32.768 0 0 1.0 2.0 1 0 0 0",
                                 GYRO_V2, ACCEL_V2)
        assert r["accel"] == pytest.approx(body(G, 0.0, 0.0))
        assert r["gyro"] == pytest.approx(body(math.radians(1.0), 0.0, 0.0))

    def test_quat_goes_through_the_remap(self):
        q = (0.9238795, 0.3826834, 0.0, 0.0)  # 45° about board x
        line = "ALL 0 0 8192 0 0 0 0 0 " + " ".join(f"{c:.7f}" for c in q)
        r = proto.parse_imu_line(line, GYRO_V2, ACCEL_V2)
        assert r["quat"] == pytest.approx(proto._quat_mul(q, proto._Q_REMAP_CONJ))

    def test_unconverged_zero_quat_dropped(self):
        r = proto.parse_imu_line("ALL 0 0 8192 0 0 0 0 0 0 0 0 0", GYRO_V2, ACCEL_V2)
        assert r is not None and "quat" not in r


class TestFirmwareV1:  # 'L' -> "ALL" + 8 fields, no quaternion
    def test_flat_and_still(self):
        r = proto.parse_imu_line("ALL 0.000 0.000 4096.000 0.000 0.000 0.000 0.00 0.00\r\n",
                                 GYRO_V1, ACCEL_V1)
        assert r["accel"] == pytest.approx(body(0.0, 0.0, G))  # 4096 counts = 1 g
        assert "quat" not in r

    def test_truncated_and_garbage(self):
        for line in ("ALL 1.0 2.0 3.0\n", "READY\n", "ACCEL 1.0 2.0 3.0\n",
                     "ALL a b c d e f g h\n", "ALL 1 2 3 4 5 6 7 8 9 10\n", "\n", ""):
            assert proto.parse_imu_line(line, GYRO_V1, ACCEL_V1) is None, line


class TestQuatRemapMath:
    """imu_protocol rotates the fused quaternion with q_base = q ⊗ conj(q_R). That
    must move projected_gravity exactly like the vectors move: g_base = R · g_board.
    Checked for mountings other than the current identity AXIS_REMAP."""

    @pytest.mark.parametrize("remap", [
        ((0, -1, 0), (1, 0, 0), (0, 0, 1)),    # board yawed 90°
        ((-1, 0, 0), (0, 1, 0), (0, 0, -1)),   # board upside down
        ((0, 0, 1), (1, 0, 0), (0, 1, 0)),     # axes cycled
    ])
    def test_gravity_follows_the_vector_remap(self, remap):
        r = np.array(remap, dtype=float)
        w, x, y, z = proto._matrix_to_quat(remap)
        q_r_conj = (w, -x, -y, -z)
        rng = np.random.default_rng(0)
        for _ in range(20):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            g_board = projected_gravity_from_quat(q)
            g_base = projected_gravity_from_quat(proto._quat_mul(tuple(q), q_r_conj))
            assert g_base == pytest.approx(r @ g_board, abs=1e-5)


class TestToMessage:
    def test_quat_flag_only_when_fused(self):
        assert reading_to_msg(proto.parse_imu_line(
            TestFirmwareV2.FLAT, GYRO_V2, ACCEL_V2)).quat_valid
        assert not reading_to_msg(proto.parse_imu_line(
            "ALL 0 0 4096 0 0 0 0 0", GYRO_V1, ACCEL_V1)).quat_valid

    def test_lin_vel_never_claimed(self):
        # base_lin_vel is unobservable on hardware — only sim_service may set it.
        msg = reading_to_msg(proto.parse_imu_line(TestFirmwareV2.FLAT, GYRO_V2, ACCEL_V2))
        assert not msg.lin_vel_valid

    def test_wire_roundtrip(self):
        msg = reading_to_msg(proto.parse_imu_line(TestFirmwareV2.FLAT, GYRO_V2, ACCEL_V2))
        back = ipc.decode(msg.pack(seq=1))
        assert back.gyro == pytest.approx(msg.gyro, rel=1e-6)
        assert back.accel == pytest.approx(msg.accel, rel=1e-6)
        assert back.quat == pytest.approx(msg.quat, rel=1e-6)
        assert back.quat_valid and not back.lin_vel_valid


class FakeVPort:
    """Answers each 'V' with the next scripted line (b"" = a dropped reply)."""

    def __init__(self, *lines):
        self.lines, self.asked = list(lines), 0

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.asked += 1

    def readline(self):
        return self.lines.pop(0) if self.lines else b""


class TestQueryVersion:
    """'V' -> ranges. 2026-11-05 on the robot: 'VER 2 0 1000' (accel readback 0) long
    after boot, then 'VER 2 4 1000' a moment later. A 0 used to end the query and every
    tool fell back to the v1 guesses: 2x too large for v2, on the gyro too."""

    def test_clean_reply(self):
        v = proto.query_version(FakeVPort(b"VER 2 4 1000\r\n"))
        assert v == {"proto": 2, "accel_range_g": 4.0, "gyro_range_dps": 1000.0,
                     "readback": True}

    def test_first_command_dropped_then_answers(self):
        v = proto.query_version(FakeVPort(b"", b"VER 2 4 1000\r\n"))
        assert v["readback"] and v["accel_range_g"] == 4.0

    def test_an_intermittent_zero_is_retried(self):
        v = proto.query_version(FakeVPort(b"VER 2 0 1000\r\n", b"VER 2 4 1000\r\n"))
        assert v["readback"] and v["accel_range_g"] == 4.0

    def test_a_lasting_zero_gets_v2s_configured_ranges_not_v1_guesses(self):
        v = proto.query_version(FakeVPort(*[b"VER 2 0 1000\r\n"] * 20))
        assert v == {"proto": 2, "accel_range_g": proto.V2_ACCEL_RANGE_G,
                     "gyro_range_dps": 1000.0, "readback": False}
        assert proto.V2_ACCEL_RANGE_G != proto.ACCEL_RANGE_G          # not the v1 guess
        gyro, _, src = sensor_scales(v)
        assert gyro == pytest.approx(GYRO_V2) and "not verified" in src

    def test_v1_never_answers(self):
        port = FakeVPort()
        assert proto.query_version(port) is None
        assert port.asked == 4                                       # gives up in time


class TestOpenImuPermission:
    def test_permission_denied_says_so(self, monkeypatch, tmp_path):
        """A shell without the port's group used to get "no IMU response ... is
        something else holding the port?" (2026-11-05: the user's terminals predated
        `usermod -aG dialout`)."""
        def denied(*a, **kw):
            raise proto.serial.SerialException(13, "could not open port: Permission denied")
        monkeypatch.setattr(proto.serial, "Serial", denied)
        with pytest.raises(RuntimeError, match=r"permission denied.*log out and back in"):
            proto.open_imu(str(tmp_path / "ttyACM0"), settle=0.0)
