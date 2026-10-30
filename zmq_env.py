"""
zmq_env.py — ZmqEnv(Env): the deployment backend over ZMQ.

Replaces the ROS2 RealEnv (see legacy_ros2/real_env.py). Same Env interface as
SimEnv, so the identical policy runner deploys unchanged:

    SUB  motor_states   latest-value   joint pos/vel [rad] + flags
    SUB  imu            latest-value   gyro + accel (+ quat / sim lin_vel)
    PUB  motor_commands @ 1/control_dt absolute position targets [rad]

Crucially, ZmqEnv does not know (or care) whether the other end is
motor_service.py + imu_service.py on the real robot or sim_service.py running
MuJoCo — same topics, same schemas, same timing. That is the whole point.

Refuses to run (loudly) when:
  * no motor_states / imu traffic within the reset timeout,
  * the service is not enabled (run tools/dodo_ctl.py enable first),
  * joints are not calibrated (ODrive homing; pass allow_uncalibrated=True to
    override on the bench),
  * the service e-stops mid-run,
  * the obs layout needs base_lin_vel but the stream doesn't provide it
    (hardware never does; only sim_service publishes ground-truth lin_vel).

All policy knowledge (joint order, default pose, action scaling, obs assembly)
lives in the shared Env base — this file only supplies raw signals in policy
order and publishes resolved position targets in wire order.
"""

from __future__ import annotations

import logging
import time
from typing import Dict, Optional

import numpy as np

import dodo_ipc as ipc
from rl_env import (Env, ObsCfg, OBS_STAND, POLICY_JOINT_ORDER, DEFAULT_POS_VEC,
                    projected_gravity_from_quat)

log = logging.getLogger("zmq_env")


class ZmqEnv(Env):
    def __init__(self, obs_cfg: ObsCfg = OBS_STAND, command=(0.0, 0.0, 0.0),
                 control_dt: float = 0.02,
                 allow_uncalibrated: bool = False,
                 max_state_age_s: float = 0.25,
                 startup_ramp_s: float = 0.0,
                 disable_on_close: bool = True):
        super().__init__(obs_cfg, command=command, control_dt=control_dt)
        self.allow_uncalibrated = allow_uncalibrated
        self.max_state_age_s = max_state_age_s
        # Pre-start routine: reset() slowly ramps the joints from wherever they
        # ARE (on the real robot: unparked just inside the safety band after the
        # boot pose) to the policy's default pose, publishing at the control
        # rate the whole way — same commander process, so the service's command
        # watchdog stays fed and the policy takes over with zero discontinuity.
        # 0 disables (sim tests; robot already posed). run_policy defaults it ON
        # for --backend zmq.
        self.startup_ramp_s = startup_ramp_s
        self.disable_on_close = disable_on_close

        # policy index -> wire index (identity as long as POLICY_JOINT_ORDER
        # matches dodo_ipc.JOINT_ORDER, but mapped by NAME so an experimental
        # DODO_JOINT_ORDER override in rl_env can't scramble the wire).
        self._wire_idx = np.array([ipc.JOINT_INDEX[j] for j in POLICY_JOINT_ORDER])

        self._state_sub = ipc.Subscriber(ipc.TOPIC_MOTOR_STATES, conflate=True)
        self._imu_sub = ipc.Subscriber(ipc.TOPIC_IMU, conflate=True)
        self._cmd_pub = ipc.Publisher(ipc.TOPIC_MOTOR_COMMANDS)

        self._states: Optional[ipc.MotorStates] = None
        self._imu: Optional[ipc.ImuSample] = None
        # Arrival stamps on OUR clock (works cross-host, unlike msg.t_mono_ns).
        self._states_t = 0.0
        self._imu_t = 0.0

        self._loop = ipc.RateLoop(1.0 / control_dt, name="zmq_env")

    # ---- plumbing ----
    def _poll(self) -> None:
        m = self._state_sub.latest()
        if m is not None:
            self._states = m
            self._states_t = time.monotonic()
        m = self._imu_sub.latest()
        if m is not None:
            self._imu = m
            self._imu_t = time.monotonic()

    def _check_health(self) -> None:
        # NOTE: staleness is measured at drain time (arrival stamps are taken
        # when latest() is called), so one message parked in the conflated
        # queue can defer dead-stream detection by exactly one control step —
        # under continuous 50 Hz stepping that's 20 ms, which is fine.
        now = time.monotonic()
        if self._states is None or now - self._states_t > self.max_state_age_s:
            age = float("inf") if self._states is None else now - self._states_t
            raise RuntimeError(
                f"motor_states stale ({age:.2f}s > {self.max_state_age_s}s) — "
                f"is motor_service/sim_service still running?")
        if self._states.estopped:
            raise RuntimeError(
                "motor service is E-STOPPED — check its log for the reason, fix "
                "the cause, then tools/dodo_ctl.py estop_reset + enable")
        if self._imu is None or now - self._imu_t > self.max_state_age_s:
            age = float("inf") if self._imu is None else now - self._imu_t
            raise RuntimeError(
                f"imu stale ({age:.2f}s > {self.max_state_age_s}s) — "
                f"is imu_service/sim_service still running?")

    # ---- Env backend hooks ----
    def _read_signals(self) -> Dict[str, np.ndarray]:
        self._poll()
        self._check_health()
        st, imu = self._states, self._imu

        pos_wire = np.asarray(st.pos, dtype=np.float32)
        vel_wire = np.asarray(st.vel, dtype=np.float32)
        joint_pos = pos_wire[self._wire_idx]
        joint_vel = vel_wire[self._wire_idx]

        if imu.quat_valid:
            proj_g = projected_gravity_from_quat(imu.quat)
        else:
            a = np.asarray(imu.accel, dtype=np.float32)
            n = float(np.linalg.norm(a))
            # quasi-static fallback: gravity direction from the accelerometer
            proj_g = (-a / n) if n > 1e-3 else np.array([0, 0, -1], dtype=np.float32)

        lin_vel = (np.asarray(imu.lin_vel, dtype=np.float32) if imu.lin_vel_valid
                   else np.zeros(3, dtype=np.float32))

        return {
            "base_lin_vel": lin_vel,
            "base_ang_vel": np.asarray(imu.gyro, dtype=np.float32),
            "projected_gravity": proj_g.astype(np.float32),
            "joint_pos_rel": (joint_pos - DEFAULT_POS_VEC).astype(np.float32),
            "joint_vel": joint_vel.astype(np.float32),
        }

    def _apply_and_advance(self, targets: np.ndarray) -> None:
        wire = np.full(ipc.N_JOINTS, ipc.NO_COMMAND, dtype=np.float32)
        wire[self._wire_idx] = targets
        self._cmd_pub.send(ipc.MotorCommands(target_pos=[float(x) for x in wire]))
        self._loop.sleep()          # absolute-deadline pacing at 1/control_dt

    def _reset_backend(self) -> None:
        # Wait for live streams so the first observation is real.
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            self._poll()
            if self._states is not None and self._imu is not None:
                break
            time.sleep(0.05)
        if self._states is None:
            raise RuntimeError("no motor_states within 5s — start motor_service "
                               "or sim_service first")
        if self._imu is None:
            raise RuntimeError("no imu within 5s — start imu_service or "
                               "sim_service first")

        # Settle briefly and re-poll so the flag checks below act on a message
        # published AFTER any just-issued ctl command (enable/disable both
        # propagate through the state stream, not the ctl reply).
        time.sleep(0.1)
        self._poll()

        if self._states.estopped:
            raise RuntimeError("service is e-stopped — tools/dodo_ctl.py "
                               "estop_reset, then enable")
        if not self._states.enabled:
            raise RuntimeError("motors not enabled — tools/dodo_ctl.py enable "
                               "(after set_zero if needed)")
        if not self._states.all_calibrated:
            if self.allow_uncalibrated:
                log.warning("running UNCALIBRATED (positions may be wrong) — "
                            "bench only!")
            else:
                raise RuntimeError(
                    "joints not calibrated (ODrive zeros unset). Power on with "
                    "the legs folded at the stops, then tools/dodo_ctl.py "
                    "calibrate (or set_zero in the jig), then enable. "
                    "(allow_uncalibrated=True to override on the bench.)")
        if "base_lin_vel" in self.obs_cfg.terms and not self._imu.lin_vel_valid:
            raise RuntimeError(
                "this obs layout needs base_lin_vel, which only sim_service "
                "provides — a 'flat' policy cannot run on the real robot "
                "(use --task stand).")

        if self.startup_ramp_s > 0.0:
            self._startup_ramp()

    def _startup_ramp(self) -> None:
        """Pre-start routine: smoothstep every joint from its measured position
        to the default (policy home) pose over startup_ramp_s, then settle.
        Health-checked each step, so an e-stop mid-ramp aborts loudly."""
        q0 = np.asarray(self._states.pos, dtype=np.float32).copy()
        q1 = q0.copy()
        q1[self._wire_idx] = DEFAULT_POS_VEC
        n = max(1, int(round(self.startup_ramp_s / self.control_dt)))
        settle = max(1, int(round(0.3 / self.control_dt)))
        log.info("startup ramp: %.1fs to the default pose (max travel %.2f rad)",
                 self.startup_ramp_s, float(np.max(np.abs(q1 - q0))))
        for k in range(n + settle + 1):
            s = min(1.0, k / n)
            alpha = 3.0 * s * s - 2.0 * s * s * s
            wire = q0 + alpha * (q1 - q0)
            self._cmd_pub.send(ipc.MotorCommands(target_pos=[float(x) for x in wire]))
            self._loop.sleep()
            self._poll()
            self._check_health()
        err = np.abs(np.asarray(self._states.pos, dtype=np.float32) - q1)
        log.info("startup ramp done (max tracking error %.3f rad)", float(np.max(err)))

    def close(self) -> None:
        log.info(self._loop.stats_str())
        if self.disable_on_close:
            # The commander is going away; without this the service's command
            # watchdog would e-stop 0.5 s later. An explicit disable is the
            # clean end of a session (motors limp — catch the robot).
            try:
                ctl = ipc.CtlClient()
                ctl.request("disable", timeout_s=0.5)
                ctl.close()
                log.info("motors disabled (session over)")
            except Exception as e:
                log.warning("could not disable on close (%s) — the service's "
                            "watchdog will e-stop instead", e)
        for s in (self._state_sub, self._imu_sub, self._cmd_pub):
            try:
                s.close()
            except Exception:
                pass
