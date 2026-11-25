"""
test_dodo_startup.py — tools/dodo_startup.py's stand-pose adjustments (adjust_pose and
the command-band check), and what they do to the COM over the feet in dodo_assets'
model: the numbers its help text quotes (HANDOFF log 2026-11-25). No sockets here; the
command line through sim_service is in test_sim_scene.py.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import mujoco
import numpy as np
import pytest

import assets
import dodo_ipc as ipc
import real2sim

_spec = importlib.util.spec_from_file_location(
    "dodo_startup", Path(__file__).resolve().parent / "tools" / "dodo_startup.py")
ds = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ds)

SIDES = ("left", "right")
HOME = assets.scene_pose("home")
# The pose the robot stands in on the floor (HANDOFF log 2026-11-25 (b)).
ROBOT_STAND = dict(legs_back=0.2, squat=0.175, stance=0.1)


def pitch_sum(pose, side):
    """hip pitch + knee + ankle: the sole's pitch relative to the body."""
    return pose[f"upper_leg_{side}"] + pose[f"lower_leg_{side}"] + pose[f"foot_{side}"]


class TestAdjustPose:
    def test_no_adjustment_is_the_pose(self):
        assert ds.adjust_pose(HOME) == HOME

    def test_input_is_not_modified(self):
        before = dict(HOME)
        ds.adjust_pose(HOME, legs_back=0.1, squat=0.1, stance=0.1)
        assert HOME == before

    @pytest.mark.parametrize("knobs", [dict(legs_back=0.13), dict(squat=-0.07),
                                       dict(stance=0.05),
                                       dict(legs_back=-0.2, squat=0.15, stance=-0.04)])
    def test_soles_stay_parallel_to_the_body(self, knobs):
        p = ds.adjust_pose(HOME, **knobs)
        for side in SIDES:
            assert pitch_sum(p, side) == pytest.approx(pitch_sum(HOME, side), abs=1e-12)

    def test_legs_back_swings_thigh_and_ankle_only(self):
        p = ds.adjust_pose(HOME, legs_back=0.1)
        for side in SIDES:
            assert p[f"upper_leg_{side}"] == pytest.approx(HOME[f"upper_leg_{side}"] + 0.1)
            assert p[f"foot_{side}"] == pytest.approx(HOME[f"foot_{side}"] - 0.1)
            assert p[f"lower_leg_{side}"] == HOME[f"lower_leg_{side}"]
            assert p[f"hip_{side}"] == HOME[f"hip_{side}"]

    def test_squat_bends_the_knee_twice_as_much(self):
        p = ds.adjust_pose(HOME, squat=0.1)
        for side in SIDES:
            assert p[f"upper_leg_{side}"] == pytest.approx(HOME[f"upper_leg_{side}"] + 0.1)
            assert p[f"lower_leg_{side}"] == pytest.approx(HOME[f"lower_leg_{side}"] - 0.2)
            assert p[f"foot_{side}"] == pytest.approx(HOME[f"foot_{side}"] + 0.1)

    def test_stance_rolls_both_hips_outward_mirrored(self):
        """hip roll + moves the foot LEFT: out for the left leg, in for the right one."""
        p = ds.adjust_pose(HOME, stance=0.05)
        assert p["hip_left"] == pytest.approx(HOME["hip_left"] + 0.05)
        assert p["hip_right"] == pytest.approx(HOME["hip_right"] - 0.05)
        assert {j: v for j, v in p.items() if not j.startswith("hip_")} == \
               {j: v for j, v in HOME.items() if not j.startswith("hip_")}


class TestCommandBand:
    def test_home_and_the_tried_adjustments_are_inside(self):
        bands = ds.command_bands({})
        assert set(bands) == set(ipc.JOINT_ORDER)
        for knobs in ({}, dict(legs_back=0.3), dict(squat=0.2), dict(stance=0.1)):
            assert ds.outside_band(ds.adjust_pose(HOME, **knobs), bands) == [], knobs

    def test_the_robots_floor_stand_is_inside(self):
        """The joint targets HANDOFF records for it, and dodo_startup accepts them
        (re-measured stops that cut into it would make it refuse the pose)."""
        stand = ds.adjust_pose(HOME, **ROBOT_STAND)
        for side in SIDES:
            assert stand[f"upper_leg_{side}"] == pytest.approx(0.825)
            assert stand[f"lower_leg_{side}"] == pytest.approx(-1.35)
            assert stand[f"foot_{side}"] == pytest.approx(0.525)
        assert stand["hip_left"] == pytest.approx(0.1) == -stand["hip_right"]
        assert ds.outside_band(stand, ds.command_bands({})) == []

    def test_too_far_is_flagged_per_joint(self):
        bad = ds.outside_band(ds.adjust_pose(HOME, legs_back=2.0), ds.command_bands({}))
        assert sorted(j for j, _, _ in bad) == ["foot_left", "foot_right",
                                                "upper_leg_left", "upper_leg_right"]
        for j, q, (lo, hi) in bad:
            assert not lo <= q <= hi

    def test_the_services_own_band_wins(self):
        """motor_service reports band_rad in status; without it, real2sim's (the band
        sim_service clamps to); a joint with neither isn't checked."""
        bands = ds.command_bands({"hip_left": {"band_rad": [-0.1, 0.1]}})
        assert bands["hip_left"] == [-0.1, 0.1]
        assert bands["hip_right"] == real2sim.command_band("hip_right")
        assert ds.outside_band({"hip_left": 0.2}, bands) == [("hip_left", 0.2, (-0.1, 0.1))]
        assert ds.outside_band({"x": 9.0}, {"x": None}) == []


@pytest.fixture(scope="module")
def com_mm():
    """pose -> the whole-robot COM in the rigid model with the soles flat on the floor,
    along the sole from the heel edge of foot_*_collision (what touches the floor) [mm]."""
    m = mujoco.MjModel.from_xml_path(str(assets.require(assets.SCENE_PATH)))
    d = mujoco.MjData(m)
    body = m.body("body").id
    feet = [m.geom(f"foot_{s}_collision").id for s in SIDES]

    def com(pose):
        mujoco.mj_resetData(m, d)
        for j, q in pose.items():
            d.qpos[m.jnt_qposadr[m.joint(j).id]] = q
        mujoco.mj_forward(m, d)
        out = []
        for g in feet:
            R = d.geom_xmat[g].reshape(3, 3)
            along_sole = R.T @ (d.subtree_com[body] - d.geom_xpos[g])
            out.append(along_sole[0] + m.geom_size[g][0])
        return 1e3 * float(np.mean(out))
    return com


class TestComOverTheFeet:
    def test_home_is_over_the_rear_half_of_the_foot(self, com_mm):
        # 40 mm of the 95 mm sole: the PD stand's sag (knees ~0.05 rad) carries it to
        # ~the middle once loaded. The robot is further back still: it tips backwards.
        assert com_mm(HOME) == pytest.approx(40.5, abs=1.0)

    def test_legs_back_moves_it_forward_3_5_mm_per_hundredth(self, com_mm):
        for x in (0.05, 0.1, 0.2):
            shift = com_mm(ds.adjust_pose(HOME, legs_back=x)) - com_mm(HOME)
            assert shift / (100 * x) == pytest.approx(3.5, abs=0.15), x

    def test_squat_and_stance_hardly_move_it(self, com_mm):
        for knobs in (dict(squat=0.1), dict(squat=0.2), dict(stance=0.05)):
            assert abs(com_mm(ds.adjust_pose(HOME, **knobs)) - com_mm(HOME)) < 4.0, knobs
