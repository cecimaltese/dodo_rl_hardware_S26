"""
test_pd_push_test.py — tools/pd_push_test.py's analysis on synthetic pushes.

A push is modelled the way the drives behave: an ODrive in position control gives
torque = N * vel_gain * min(pos_gain * e_motor, vel_limit) at standstill (the velocity
setpoint saturates), a Damiao kp * e. The analysis must recover the stiffness, find the
plateau and turn it into the drive's vel_limit, and flag the failure modes.
"""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

import fleet_config
import real2sim

_spec = importlib.util.spec_from_file_location(
    "pd_push_test", Path(__file__).resolve().parent / "tools" / "pd_push_test.py")
ppt = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ppt)

KNEE = "lower_leg_left"
RNG = np.random.default_rng(1)


def odrive_push(joint, scale, vel_limit, max_err=0.25, n=600, noise=0.01):
    """Slow pushes to max_err and back: (err, tau, vel) like the drive would report."""
    g = fleet_config.joint_group(joint)
    kp, kd = fleet_config.JOINT_KP[g] * scale, fleet_config.JOINT_KD[g] * math.sqrt(scale)
    pos_gain, vel_gain, _ = real2sim.odrive_gains(joint, kp, kd)
    nratio = real2sim.gear_ratio(joint)
    err = max_err * np.sin(np.linspace(0, 3 * np.pi, n)) ** 2
    e_motor = err * nratio / (2 * math.pi)
    tau = nratio * vel_gain * np.minimum(pos_gain * e_motor, vel_limit)
    tau = tau + RNG.normal(0, noise, n)
    vel = np.gradient(err) * 20          # slow: well under STILL_RAD_S
    return err, tau, vel, kp, kd


class TestOdrive:
    def test_plateau_gives_the_vel_limit(self):
        err, tau, vel, kp, kd = odrive_push(KNEE, 0.3, vel_limit=10.0)
        r = ppt.analyze(err, tau, vel, kp, kd, KNEE, 0.3)
        assert r["k_ratio"] == pytest.approx(1.0, abs=0.1)
        assert r["saturated"]
        assert r["vel_limit_if_velocity_cap"] == pytest.approx(10.0, rel=0.1)
        assert r["cap_full_gains"] == pytest.approx(2.0, rel=0.1)   # the 2.0 Nm worry
        assert "NOT enough" in r["verdict"]

    def test_high_vel_limit_is_enough(self):
        err, tau, vel, kp, kd = odrive_push(KNEE, 0.3, vel_limit=30.0, max_err=0.5)
        r = ppt.analyze(err, tau, vel, kp, kd, KNEE, 0.3)
        assert r["saturated"]
        assert r["cap_full_gains"] == pytest.approx(6.0, rel=0.1)
        assert "enough" in r["verdict"] and "NOT enough" not in r["verdict"]

    def test_no_plateau_reports_a_lower_bound(self):
        err, tau, vel, kp, kd = odrive_push(KNEE, 0.3, vel_limit=1e9, max_err=0.1)
        r = ppt.analyze(err, tau, vel, kp, kd, KNEE, 0.3)
        assert not r["saturated"]
        assert r["cap_full_gains_at_least"] == pytest.approx(
            r["tau_max"] / math.sqrt(0.3))

    def test_joint_side_torque_shows_as_gear_ratio_stiffness(self):
        err, tau, vel, kp, kd = odrive_push(KNEE, 1.0, vel_limit=1e9, max_err=0.05)
        r = ppt.analyze(err, tau * real2sim.gear_ratio(KNEE), vel, kp, kd, KNEE)
        assert r["k_ratio"] == pytest.approx(real2sim.gear_ratio(KNEE), rel=0.1)
        assert "STIFFER" in r["verdict"]

    def test_inverted_torque_sign_is_flagged(self):
        err, tau, vel, kp, kd = odrive_push(KNEE, 1.0, vel_limit=1e9, max_err=0.05)
        r = ppt.analyze(err, -tau, vel, kp, kd, KNEE)
        assert r["k_measured"] < 0 and "INVERTED" in r["verdict"]


class TestOther:
    def test_damiao_stiffness(self):
        err = 0.1 * np.sin(np.linspace(0, 4 * np.pi, 400))
        r = ppt.analyze(err, 150.0 * err + RNG.normal(0, 0.05, 400), np.zeros(400),
                        150.0, 1.5, "upper_leg_right")
        assert r["k_ratio"] == pytest.approx(1.0, abs=0.05)
        assert "OK" in r["verdict"] and "vel_limit" not in r

    def test_moving_samples_are_ignored(self):
        err = np.full(100, 0.1)
        r = ppt.analyze(err, np.full(100, 99.0), np.full(100, 2.0), 50.0, 0.8, KNEE)
        assert r["n_still"] == 0 and "not enough push" in r["verdict"]

    def test_no_push(self):
        r = ppt.analyze(np.zeros(100), np.zeros(100), np.zeros(100), 50.0, 0.8, KNEE)
        assert "not enough push" in r["verdict"]
