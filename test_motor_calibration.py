"""
test_motor_calibration.py — motor_service's per-joint calibration layer, no hardware.

motor_service talks JOINT units on the wire (sim radians, rad/s, joint Nm) and each
motor's raw native units on the bus, through real2sim.JointCal (sign, zero, gear
ratio). These drive a real OdriveCanJoint against a fake CAN bus (frames decoded with
the actual struct formats) through motor_service._JointAdapter, and check the robot's
own configuration (fleet_config + joint_calib.py) is complete enough to enable.

(Was test_boot_calibration.py, for the boot-pose placeholder BOOT_POSE_RAD, which
went with the calibration wiring on 2026-11-05.)
"""
from __future__ import annotations

import math
import struct

import can
import pytest

import assets
import fleet_config
import real2sim
from motor_service import _JointAdapter
from odrive_can_joint import (CMD_GET_ENCODER_ESTIMATES, CMD_GET_TORQUES,
                              CMD_SET_INPUT_POS, CMD_SET_POS_GAIN, CMD_SET_VEL_GAINS,
                              OdriveCanJoint, OdriveCanJointConfig)

TWO_PI = 2.0 * math.pi
KNEE = "lower_leg_left"


class FakeBus:
    """Collects sent frames; feed feedback by calling joint.process_frame."""
    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)


def feed(j: OdriveCanJoint, turns: float, torque: float = 0.0) -> None:
    for cmd, data in ((CMD_GET_ENCODER_ESTIMATES, struct.pack("<ff", turns, 0.0)),
                      (CMD_GET_TORQUES, struct.pack("<ff", 0.0, torque))):
        j.process_frame(can.Message(arbitration_id=(j.node_id << 5) | cmd, data=data,
                                    is_extended_id=False))


def make_knee(raw_turns=2.53, sign=-1):
    """The left knee (sign -1 like the robot's) on a fake bus, at raw_turns."""
    bus = FakeBus()
    j = OdriveCanJoint(bus, OdriveCanJointConfig(name=KNEE, node_id=3))
    feed(j, raw_turns)
    cal = real2sim.JointCal(KNEE, KNEE, sign, 0.0, real2sim.gear_ratio(KNEE), True)
    return bus, j, _JointAdapter(j, cal, (-2.7, 1.6), (-2.6, 1.5))


def last(bus: FakeBus, cmd: int, fmt: str):
    for msg in reversed(bus.sent):
        if msg.arbitration_id & 0x1F == cmd:
            return struct.unpack(fmt, bytes(msg.data))
    raise AssertionError(f"no frame 0x{cmd:02X} sent")


class TestAdapter:
    def test_uncalibrated_until_the_reference_pose(self):
        _, _, a = make_knee()
        assert a.measured and not a.calibrated               # ODrive: per session

    def test_calibrate_at_the_reference_pose_reads_it(self):
        _, j, a = make_knee(raw_turns=2.53)
        a.calibrate_at(a.read_state()["pos_native"], 0.0)    # `straight`
        assert a.calibrated
        assert a.read_state()["pos_rad"] == pytest.approx(0.0, abs=1e-9)
        feed(j, 2.53 + 1.0)                                  # one motor turn
        assert a.read_state()["pos_rad"] == pytest.approx(
            -TWO_PI / real2sim.gear_ratio(KNEE), abs=1e-9)   # sign -1, geared

    def test_commands_map_back_to_raw_turns(self):
        bus, j, a = make_knee(raw_turns=2.53)
        a.calibrate_at(2.53, 0.0)
        j.send_position(a.to_native(a.read_state()["pos_rad"]))     # bumpless
        assert last(bus, CMD_SET_INPUT_POS, "<fhh")[0] == pytest.approx(2.53, abs=1e-6)
        j.send_position(a.to_native(-0.5))                   # the knee bent 0.5 rad
        want = 2.53 + 0.5 / (TWO_PI / real2sim.gear_ratio(KNEE))
        assert last(bus, CMD_SET_INPUT_POS, "<fhh")[0] == pytest.approx(want, abs=1e-5)

    def test_clamp_is_the_band_in_joint_radians(self):
        _, _, a = make_knee()
        assert a.clamp_rad(-3.0) == -2.6 and a.clamp_rad(2.0) == 1.5
        assert a.clamp_rad(0.3) == 0.3

    def test_odrive_torque_is_joint_side_and_signed(self):
        _, j, a = make_knee(sign=-1)
        feed(j, 0.0, torque=0.1)                             # motor Nm
        assert fleet_config.ODRIVE_TORQUE_IS_MOTOR_SIDE
        assert a.read_state()["torque"] == pytest.approx(
            -0.1 * real2sim.gear_ratio(KNEE))

    def test_gains_go_to_the_drive_as_its_cascade(self):
        bus, _, a = make_knee()
        kp, kd = a.apply_gains(0.25)
        assert (kp, kd) == pytest.approx((50 * 0.25, 0.8 * 0.5))
        pos_gain, = last(bus, CMD_SET_POS_GAIN, "<f")
        vel_gain, vel_int = last(bus, CMD_SET_VEL_GAINS, "<ff")
        n = real2sim.gear_ratio(KNEE)
        assert vel_int == 0.0                                # a PD, no integrator
        assert n * n * pos_gain * vel_gain / TWO_PI == pytest.approx(kp, rel=1e-6)
        assert n * n * vel_gain / TWO_PI == pytest.approx(kd, rel=1e-6)


class TestOdriveGains:
    @pytest.mark.parametrize("joint", ["lower_leg_left", "foot_right"])
    def test_cascade_reproduces_the_joint_pd(self, joint):
        g = fleet_config.joint_group(joint)
        kp, kd = fleet_config.JOINT_KP[g], fleet_config.JOINT_KD[g]
        pos_gain, vel_gain, vel_int = real2sim.odrive_gains(joint, kp, kd)
        n = real2sim.gear_ratio(joint)
        assert vel_int == 0.0
        assert n * n * pos_gain * vel_gain / TWO_PI == pytest.approx(kp)
        assert n * n * vel_gain / TWO_PI == pytest.approx(kd)

    def test_zero_damping_is_refused(self):
        with pytest.raises(ValueError):
            real2sim.odrive_gains("foot_left", 50.0, 0.0)


class TestTheRobotsConfiguration:
    """What motor_service needs before it will enable, from the robot's own files."""

    def test_reference_poses_cover_exactly_the_odrives(self):
        odrives = {c.name for c in fleet_config.OD_JOINTS}
        for pose, angles in fleet_config.REFERENCE_POSES.items():
            assert set(angles) == odrives, pose

    def test_reference_poses_are_inside_the_measured_stops(self):
        stops = real2sim.stops()
        for pose, angles in fleet_config.REFERENCE_POSES.items():
            for j, q in angles.items():
                lo, hi = stops[j]
                assert lo < q < hi, (pose, j)

    def test_every_joint_measured_with_stops(self):
        cals, _ = real2sim.load()
        for j in real2sim.JOINTS:
            assert cals[j].measured and cals[j].motor == j, j
            assert real2sim.command_band(j) is not None, j

    @pytest.mark.parametrize("pose", ["home", "straight"])
    def test_the_stand_poses_are_inside_the_command_band(self, pose):
        for j, q in assets.scene_pose(pose).items():
            lo, hi = real2sim.command_band(j)
            assert lo < q < hi, (pose, j)
