"""
test_fleet_config.py — the fleet is configured in ONE place.

Twice a hand-copied fleet config drifted from the real robot (read_motor_state.py
hunting ODrive nodes that no longer existed; motor_service.py defaulting to node ids
7-10 on a single shared bus). These tests make the "import it, don't copy it" rule a
mechanism instead of a comment.
"""
from __future__ import annotations

import inspect

import fleet_config
import motor_service


def test_motor_service_uses_fleet_config_objects():
    for name in ("DM_JOINTS", "OD_JOINTS", "RATED_TORQUE_NM", "REFERENCE_POSES",
                 "JOINT_KP", "JOINT_KD"):
        assert getattr(motor_service, name) is getattr(fleet_config, name), (
            f"motor_service.{name} is a copy — import it from fleet_config")


def test_motor_service_bus_defaults_come_from_fleet_config():
    params = inspect.signature(motor_service.MotorService.__init__).parameters
    assert params["dm_channel"].default == fleet_config.DM_CHANNEL
    assert params["od_channel"].default == fleet_config.OD_CHANNEL
    assert params["bustype"].default == fleet_config.BUSTYPE


def test_every_joint_configured_once():
    names = [c.name for c in (*fleet_config.DM_JOINTS, *fleet_config.OD_JOINTS)]
    assert len(names) == len(set(names)) == 8
    assert set(fleet_config.RATED_TORQUE_NM) == set(names)
    groups = {fleet_config.joint_group(n) for n in names}
    assert set(fleet_config.JOINT_KP) == set(fleet_config.JOINT_KD) == groups


def test_damiao_gains_are_the_joint_pd():
    for c in fleet_config.DM_JOINTS:
        g = fleet_config.joint_group(c.name)
        assert (c.kp, c.kd) == (fleet_config.JOINT_KP[g], fleet_config.JOINT_KD[g])
