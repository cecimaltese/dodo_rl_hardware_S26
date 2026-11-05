"""
test_odrive_check.py — tools/odrive_check.py against test_motor_watchdog's fake drives
(python-can virtual bus): a healthy drive PASSes, an open motor phase FAILs the way
node 3 (the left knee) did on 2026-11-05, and a latched error from before is cleared.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import can
import pytest

from test_motor_watchdog import FakeFleet

_spec = importlib.util.spec_from_file_location(
    "odrive_check", Path(__file__).resolve().parent / "tools" / "odrive_check.py")
oc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(oc)

pytestmark = pytest.mark.filterwarnings("ignore::DeprecationWarning")


@pytest.fixture
def fleet():
    f = FakeFleet()
    yield f
    f.close()


def run(fleet, joint):
    node = fleet.od_id[joint]
    bus = can.Bus(interface="virtual", channel=fleet.od_channel)
    try:
        lines = []
        return oc.check_drive(bus, node, joint, timeout_s=5.0, out=lines.append), lines
    finally:
        bus.shutdown()


def test_healthy_drive_passes(fleet):
    fleet.od_drive_cal[fleet.od_id["foot_left"]] = False
    r, lines = run(fleet, "foot_left")
    assert r["success"] and r["verdict"] == "PASS", r
    assert [s["state"] for s in r["steps"]] == ["MOTOR_CALIBRATION"]
    assert fleet.od_drive_cal[fleet.od_id["foot_left"]]
    assert any("PASS" in line for line in lines)


def test_open_phase_fails_like_the_left_knee(fleet):
    node = fleet.od_id["lower_leg_left"]
    fleet.od_phase_open[node] = True
    r, _ = run(fleet, "lower_leg_left")
    assert not r["success"]
    assert r["result"] == "PHASE_RESISTANCE_OUT_OF_RANGE"
    assert "CALIBRATION_ERROR" in r["errors"]
    assert "phase wire" in r["verdict"]


def test_a_latched_error_is_cleared_first(fleet):
    node = fleet.od_id["lower_leg_left"]
    fleet.od_err[node], fleet.od_proc[node] = 0x40000000, 6      # from before the repair
    r, _ = run(fleet, "lower_leg_left")
    assert r["success"] and r["errors"] == "none", r


def test_node_lookup():
    assert oc.node_of("lower_leg_left") == 3
    with pytest.raises(SystemExit):
        oc.node_of("hip_left")
