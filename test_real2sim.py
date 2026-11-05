"""
test_real2sim.py — the reading <-> joint angle map, the gear ratio, and the tools on it.

real2sim is the one place that says what a motor reading means as a sim angle, so it
is pinned here: the formula, the ratio, refusing a calibration made with another
ratio, calibrate_joints' --ratio maths and its write/read round trip, motor_service's
unit scale, and the visualizer's meshed model. No hardware.
"""
from __future__ import annotations

import builtins
import math

import mujoco
import pytest

import calibrate_joints as cj
import fleet_config
import motor_service as ms
import real2sim
from real2sim import JointCal

TWO_PI = 2.0 * math.pi


@pytest.fixture(autouse=True)
def unmeasured_ratios(monkeypatch):
    """Every test starts from unmeasured ratios, whatever fleet_config holds today,
    and sets the ones it needs with `ratios`."""
    for j in list(fleet_config.GEAR_RATIO):
        monkeypatch.setitem(fleet_config.GEAR_RATIO, j, None)


@pytest.fixture
def ratios(monkeypatch):
    """Set gear ratios for one test: ratios(foot_left=32.0, ...)."""
    def set_(**kw):
        for j, r in kw.items():
            monkeypatch.setitem(fleet_config.GEAR_RATIO, j, r)
    return set_


class TestJointCal:
    def test_damiao_is_radians_with_sign_and_zero(self):
        c = JointCal("upper_leg_right", "upper_leg_right", sign=-1, zero=0.25)
        assert c.to_sim(0.25) == 0.0
        assert c.to_sim(0.35) == pytest.approx(-0.10)
        assert c.to_raw(c.to_sim(1.234)) == pytest.approx(1.234)
        assert c.vel_to_sim(2.0) == -2.0

    def test_odrive_goes_through_the_gear_ratio(self):
        c = JointCal("foot_left", "foot_left", sign=1, zero=3.0, ratio=32.0)
        # 32 motor turns = one joint turn
        assert c.to_sim(3.0 + 32.0) == pytest.approx(TWO_PI)
        assert c.to_sim(3.0 + 8.0) == pytest.approx(TWO_PI / 4)
        assert c.vel_to_sim(16.0) == pytest.approx(math.pi)          # turns/s -> rad/s
        assert c.to_raw(1.0) == pytest.approx(3.0 + 32.0 / TWO_PI)

    @pytest.mark.parametrize("sign", [1, -1])
    def test_zeroed_at(self, sign):
        c = JointCal("lower_leg_left", "lower_leg_left", sign=sign, zero=0.0, ratio=20.0)
        z = c.zeroed_at(raw_now=5.5, q_now=-0.5)
        assert z.to_sim(5.5) == pytest.approx(-0.5)
        assert z.sign == sign and z.ratio == 20.0


class TestLoad:
    def write(self, path, entries: str):
        path.write_text(f"JOINT_CALIB = {{{entries}}}\n")
        return path

    def test_missing_file_is_identity_and_says_so(self, tmp_path):
        cals, warnings = real2sim.load(tmp_path / "none.py")
        assert set(cals) == set(real2sim.JOINTS)
        assert all(not c.measured and c.motor == j and c.sign == 1 and c.zero == 0
                   for j, c in cals.items())
        # every ODrive joint is flagged while GEAR_RATIO is unmeasured
        assert {w.split(":")[0] for w in warnings} >= {"foot_left", "lower_leg_right"}

    def test_entry_used_with_matching_ratio(self, tmp_path, ratios):
        ratios(foot_left=32.0)
        p = self.write(tmp_path / "c.py",
                       '"foot_left": dict(motor="foot_left", sign=-1, ratio=32, zero=1.5),'
                       '"hip_left": dict(motor="hip_left", sign=1, zero=0.02),')
        cals, warnings = real2sim.load(p)
        assert cals["foot_left"].measured and cals["foot_left"].ratio == 32.0
        assert cals["foot_left"].to_sim(1.5 - 8.0) == pytest.approx(math.pi / 2)
        assert cals["hip_left"].measured and cals["hip_left"].zero == 0.02
        assert not any(w.startswith("foot_left") and "ratio" in w for w in warnings)

    def test_entry_made_with_another_ratio_is_refused(self, tmp_path, ratios):
        """July's file has no ratio (made assuming direct drive). Once the real ratio is
        in fleet_config, its knee/ankle zeros mean nothing: refuse, don't reuse."""
        ratios(foot_left=32.0)
        p = self.write(tmp_path / "c.py",
                       '"foot_left": dict(motor="foot_left", sign=-1, zero=0.3265),')
        cals, warnings = real2sim.load(p)
        assert not cals["foot_left"].measured
        assert any("calibrated with gear ratio 1" in w for w in warnings)

    def test_odrive_zeros_come_with_a_power_cycle_warning(self, tmp_path):
        p = self.write(tmp_path / "c.py",
                       '"lower_leg_left": dict(motor="lower_leg_left", sign=1, zero=0.6),')
        cals, warnings = real2sim.load(p)
        assert cals["lower_leg_left"].measured
        assert any(w.startswith("lower_leg_left") and "power" in w for w in warnings)

    def test_the_repo_file_loads(self):
        cals, _ = real2sim.load()            # the repo's joint_calib.py, if any
        assert set(cals) == set(real2sim.JOINTS)


class TestCalibrateJoints:
    def test_written_entries_carry_their_ratio_and_round_trip(self, tmp_path, ratios):
        ratios(lower_leg_right=19.5)
        path = str(tmp_path / "joint_calib.py")
        cj.write_calib({"lower_leg_right": {"motor": "lower_leg_right", "sign": -1,
                                            "zero": 2.0, "ratio": 19.5},
                        "hip_right": {"motor": "hip_right", "sign": 1, "zero": 0.1,
                                      "ratio": 1.0}}, path)
        back = cj.read_calib(path)
        assert back["lower_leg_right"]["ratio"] == 19.5
        cals, _ = real2sim.load(path)
        assert cals["lower_leg_right"].measured
        # the tools' own helpers agree with real2sim
        c = back["lower_leg_right"]
        assert cj.raw_to_sim(c, 2.0 - 19.5 / 4) == pytest.approx(TWO_PI / 4)
        assert cj.sim_to_raw(c, cj.raw_to_sim(c, 7.7)) == pytest.approx(7.7)

    def test_measure_ratio_from_a_phone_angle(self, monkeypatch, ratios):
        """Turn the ankle 60 deg on the phone while its motor turns 5.35: 32.1:1. The
        winner is picked by measurement, and the mapping is re-checked on the way."""
        fleet = cj.Fleet(dry=True)
        answers = iter(["10.0", "70.0", "n"])

        def fake_input(prompt=""):
            a = next(answers)
            if a == "70.0":                   # the joint has been turned
                fleet.pos["foot_left"] += 5.35
                fleet.pos["lower_leg_left"] += 0.02
            return a
        monkeypatch.setattr(builtins, "input", fake_input)
        monkeypatch.setattr(cj, "SETTLE_S", 0.0)
        assert cj.measure_ratio(fleet, "foot_left") == pytest.approx(5.35 * 360 / 60)

    def test_measure_ratio_refuses_a_damiao_joint(self):
        assert cj.measure_ratio(cj.Fleet(dry=True), "hip_left") is None


class TestMotorServiceUnits:
    def test_the_adapter_uses_the_gear_ratio(self, ratios):
        """Readings, commands and the ODrive gains all go through the ratio. (The
        clamp band and the velocity limit are in joint radians: ratio-free.)"""
        from odrive_can_joint import OdriveCanJoint, OdriveCanJointConfig
        from test_motor_calibration import FakeBus
        ratios(lower_leg_left=32.0)
        j = OdriveCanJoint(FakeBus(), OdriveCanJointConfig(name="lower_leg_left",
                                                           node_id=3))
        cal = real2sim.JointCal("lower_leg_left", "lower_leg_left", 1, 0.0,
                                real2sim.gear_ratio("lower_leg_left"), True)
        a = ms._JointAdapter(j, cal, None, (-1.0, 1.0))
        assert a.to_native(TWO_PI) == pytest.approx(32.0)
        assert a.to_rad(32.0) == pytest.approx(TWO_PI)
        _, vel_gain, _ = real2sim.odrive_gains("lower_leg_left", 50.0, 0.8)
        assert vel_gain == pytest.approx(0.8 * TWO_PI / 32.0 ** 2)


class TestViewModel:
    def test_meshes_shown_boxes_hidden(self):
        import visualize_hardware as viz
        m = viz.build_view_model()
        types = [m.geom_type[g] for g in range(m.ngeom)]
        meshes = [g for g, t in enumerate(types) if t == mujoco.mjtGeom.mjGEOM_MESH]
        others = [g for g, t in enumerate(types) if t != mujoco.mjtGeom.mjGEOM_MESH]
        assert len(meshes) >= 11 and m.nmesh >= 11
        assert all(m.geom_group[g] <= 2 for g in meshes)               # visible
        assert all(m.geom_group[g] == viz.HIDDEN_GROUP for g in others)
        assert set(real2sim.JOINTS) <= set(viz.hinge_map(m))
        lo, hi = viz.urdf_range(m, "foot_left")
        assert (lo, hi) == pytest.approx((-1.05, 1.57))

    def test_physics_model_unchanged(self):
        """The meshes are for looking at: sim_env's model keeps boxes only."""
        from sim_env import URDF_PATH, _build_model
        assert _build_model(str(URDF_PATH)).nmesh == 0
