"""
test_boot_calibration.py — the boot-pose calibration math, no hardware needed.

Startup convention on the real robot: NO homing routine (it could overload the
3D-printed parts/belts). Instead the robot is powered on with the ODrive
joints folded against their mechanical stops, and ctl `calibrate` maps
turns-since-boot onto the known BOOT_POSE_RAD angles.

These tests drive a real OdriveCanJoint against a fake CAN bus (frames decoded
with the actual struct formats) and check the whole unit chain:
raw turns -> software zero -> radians on the wire -> commanded turns back out.
"""
from __future__ import annotations

import math
import struct

import can
import pytest

import motor_service as ms
from fleet_config import BOOT_POSE_RAD, JOINT_LIMITS_RAD, OD_JOINTS
from motor_service import _JointAdapter
from odrive_can_joint import (CMD_GET_ENCODER_ESTIMATES, CMD_SET_INPUT_POS,
                              OdriveCanJoint, OdriveCanJointConfig)

TWO_PI = 2.0 * math.pi


class FakeBus:
    """Collects sent frames; feed feedback by calling joint.process_frame."""
    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)


def make_joint(node_id=7, raw_pos_turns=0.0217):
    """Joint on a fake bus that has already 'reported' raw_pos_turns."""
    bus = FakeBus()
    j = OdriveCanJoint(bus, OdriveCanJointConfig(name="lower_leg_left",
                                                 node_id=node_id))
    fb = can.Message(arbitration_id=(node_id << 5) | CMD_GET_ENCODER_ESTIMATES,
                     data=struct.pack("<ff", raw_pos_turns, 0.0),
                     is_extended_id=False)
    j.process_frame(fb)
    return bus, j


def last_input_pos_turns(bus: FakeBus) -> float:
    """Decode the most recent Set_Input_Pos frame (raw turns on the wire)."""
    for msg in reversed(bus.sent):
        if msg.arbitration_id & 0x1F == CMD_SET_INPUT_POS:
            return struct.unpack("<fhh", bytes(msg.data))[0]
    raise AssertionError("no Set_Input_Pos frame sent")


class TestSetZeroSemantics:
    def test_plain_set_zero_unchanged(self):
        _, j = make_joint(raw_pos_turns=12.34)
        j.set_zero()                                  # old semantics: here = 0
        assert j.get_position() == pytest.approx(0.0)

    def test_boot_pose_set_zero(self):
        # Powered on at the knee stop (-pi rad = -0.5 turns), encoder reads
        # ~0 turns-since-boot: after calibrate the joint must READ -0.5 turns.
        boot_turns = -math.pi / TWO_PI
        _, j = make_joint(raw_pos_turns=0.0217)
        j.set_zero(current=boot_turns)
        assert j.get_position() == pytest.approx(boot_turns, abs=1e-9)

    def test_commands_map_back_to_raw_shaft_turns(self):
        # After boot calibration, commanding the position we currently READ
        # must reproduce the RAW shaft position on the wire (bumpless).
        raw = 0.0217
        boot_turns = -0.5
        bus, j = make_joint(raw_pos_turns=raw)
        j.set_zero(current=boot_turns)
        j.send_position(j.get_position())
        assert last_input_pos_turns(bus) == pytest.approx(raw, abs=1e-6)

    def test_moves_are_relative_to_calibrated_zero(self):
        # Command the default knee pose (-0.50 rad): wire turns must equal
        # raw_at_boot + (target - boot_angle)/2pi.
        raw, boot_rad, target_rad = 0.0, -math.pi, -0.50
        bus, j = make_joint(raw_pos_turns=raw)
        j.set_zero(current=boot_rad / TWO_PI)
        j.send_position(target_rad / TWO_PI)
        expect = raw + (target_rad - boot_rad) / TWO_PI
        assert last_input_pos_turns(bus) == pytest.approx(expect, abs=1e-6)


class TestAdapterChain:
    def test_calibrate_via_adapter_reads_boot_pose_in_rad(self):
        # The exact operation _ctl_calibrate performs, checked in radians.
        _, j = make_joint(raw_pos_turns=0.003)
        a = _JointAdapter(j)
        assert not a.calibrated                      # ODrive: uncalibrated at boot
        boot = BOOT_POSE_RAD["lower_leg_left"]
        j.set_zero(current=a.to_native(boot))
        assert a.read_state()["pos_rad"] == pytest.approx(boot, abs=1e-6)

    def test_boot_pose_within_overtravel_margin(self):
        # Every boot pose must sit inside URDF limits ± the e-stop margin,
        # or the service would e-stop right after the startup grace period.
        for name, pose in BOOT_POSE_RAD.items():
            lo, hi = JOINT_LIMITS_RAD[name]
            assert lo - ms.POS_OVERTRAVEL_MARGIN <= pose <= hi + ms.POS_OVERTRAVEL_MARGIN, name

    def test_boot_pose_covers_exactly_the_odrive_fleet(self):
        # Damiao joints have hardware zeros; every ODrive joint needs an entry.
        assert set(BOOT_POSE_RAD) == {c.name for c in OD_JOINTS}

    def test_unpark_target_is_the_safety_band_edge(self):
        # At the boot stop the joint is OUTSIDE the 0.9x band; enable unparks
        # it to the clamp edge — verify the clamp math in native turns.
        _, j = make_joint()
        a = _JointAdapter(j)
        boot_native = a.to_native(BOOT_POSE_RAD["lower_leg_left"])
        clamped = a.clamp_native(boot_native)
        lo, _ = JOINT_LIMITS_RAD["lower_leg_left"]
        assert clamped == pytest.approx(a.to_native(lo * ms.SAFETY_FACTOR))
        assert clamped != pytest.approx(boot_native)
