"""
test_odrive_config.py — tools/odrive_config.py's logic, without a bus: which settings a clone
writes, which it keeps, the duplicate-node refusal, and that the key names match the real
drives (setup/odrive_config/: the ankles, backed up from the robot 2026-11-13, fw 0.6.11).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent / "tools"))
import odrive_config as oc          # noqa: E402

import fleet_config                 # noqa: E402

KNEE = {
    oc.NODE_KEY: 7,
    "axis0.config.motor.current_soft_max": 10.0,
    "axis0.commutation_mapper.config.use_index_gpio": False,
    "axis0.config.motor.phase_resistance": 0.166,
    "axis0.config.motor.direction": 1.0,
    "axis0.controller.config.vel_limit": 100.0,
    "axis0.config.commutation_encoder_bandwidth": math.nan,
    "spi_encoder0.config.biss_c_bits": 18,           # 0.6.11 only
}
NEW = {
    oc.NODE_KEY: 7,
    "axis0.config.motor.current_soft_max": 3.0,
    "axis0.commutation_mapper.config.use_index_gpio": True,
    "axis0.config.motor.phase_resistance": 0.196,
    "axis0.config.motor.direction": -1.0,
    "axis0.controller.config.vel_limit": 100.0000001,  # equal as float32
    "axis0.config.commutation_encoder_bandwidth": math.nan,
    "config.led_brightness": 255,                    # 0.6.12 only
}


def test_clone_writes_the_differing_settings_and_the_node_id_last():
    plan = oc.plan_clone(KNEE, NEW, node_id=3)
    assert plan["writes"] == {"axis0.config.motor.current_soft_max": 10.0,
                              "axis0.commutation_mapper.config.use_index_gpio": False,
                              oc.NODE_KEY: 3}
    assert list(plan["writes"])[-1] == oc.NODE_KEY   # it takes effect at once


def test_clone_keeps_the_targets_own_calibration():
    plan = oc.plan_clone(KNEE, NEW, node_id=3)
    assert plan["kept"] == {"axis0.config.motor.phase_resistance": (0.196, 0.166),
                            "axis0.config.motor.direction": (-1.0, 1.0)}
    assert not set(plan["writes"]) & set(oc.CALIBRATION_KEYS)


def test_clone_skips_settings_the_target_firmware_lacks():
    plan = oc.plan_clone(KNEE, NEW, node_id=3)
    assert plan["missing"] == ["spi_encoder0.config.biss_c_bits"]
    assert "config.led_brightness" not in plan["writes"]


def test_clone_onto_an_identical_drive_only_moves_the_node_id():
    assert oc.plan_clone(KNEE, dict(KNEE), node_id=3)["writes"] == {oc.NODE_KEY: 3}
    assert oc.plan_clone(KNEE, dict(KNEE), node_id=7)["writes"] == {}


def test_same_is_float32_and_nan_aware():
    assert oc.same(math.nan, math.nan)
    assert oc.same(0.02, 0.019999999552965164)
    assert oc.same(10, 10.0)
    assert not oc.same(True, False) and not oc.same(1.0, -1.0)


def test_a_node_id_another_drive_has_is_refused():
    drives = {"NEW": 7, "KNEE_R": 7, "ANKLE": 4}
    assert oc.node_owner(drives, 7, except_serial="NEW") == "KNEE_R"
    assert oc.node_owner(drives, 3, except_serial="NEW") is None
    assert oc.node_owner({"NEW": 3}, 3, except_serial="NEW") is None   # already its own


BACKUPS = sorted(oc.CONFIG_DIR.glob("*.json"))


@pytest.mark.skipif(not BACKUPS, reason="no fleet backups in setup/odrive_config/")
@pytest.mark.parametrize("path", BACKUPS, ids=lambda p: p.stem)
def test_the_key_names_match_the_real_drives(path):
    d = json.loads(path.read_text())
    cfg = d["config"]
    for k in (oc.NODE_KEY, *oc.CALIBRATION_KEYS, *oc.SPOT_CHECK):
        assert k in cfg, k
    assert cfg[oc.NODE_KEY] == d["node_id"]
    assert path.stem.startswith(oc.joint_of(d["node_id"]))
    assert d["node_id"] in {c.node_id for c in fleet_config.OD_JOINTS}


ANKLES = [p for p in BACKUPS if p.stem.startswith("foot_")]


@pytest.mark.skipif(not ANKLES, reason="no ankle backups in setup/odrive_config/")
def test_the_ankle_settings_that_matter():
    """What the 2026-11-13 ankle backups say, which the service and the docs rely on: CAN
    Simple at 1 Mbit, no encoder index, vel_limit 100 (no velocity cap that binds), and
    the current limits (4.9 Nm at a knee / 5.5 Nm at an ankle; HANDOFF 2026-11-13). The
    right knee differs (3 A, index search, vel_limit 18): HANDOFF section 2."""
    for path in ANKLES:
        cfg = json.loads(path.read_text())["config"]
        assert cfg["can.config.protocol"] == 1 and cfg["can.config.baud_rate"] == 1_000_000
        assert cfg["axis0.commutation_mapper.config.use_index_gpio"] is False
        assert cfg["axis0.controller.config.vel_limit"] == 100.0
        assert cfg["axis0.config.motor.current_soft_max"] == 10.0
        assert cfg["config.inverter0.current_soft_max"] == 7.0
