"""
real_env.py — RealEnv(Env): the real-robot backend, over ROS.

Same Env interface as SimEnv, so the identical policy runner deploys unchanged.
RealEnv is a thin ROS bridge:

    publish    /motor_commands  (sensor_msgs/JointState)  position targets [rad]
    subscribe  /motor_states    (sensor_msgs/JointState)  joint pos/vel/effort
    subscribe  /imu_states      (sensor_msgs/Imu)         base ang vel + orientation

All policy knowledge (joint order, default pose, action scaling, obs assembly)
lives in the shared Env base — RealEnv only supplies the raw signals and pushes
position targets. Joint names on the topics match the policy/URDF names, so
mapping into POLICY_JOINT_ORDER is by name.

NOTE on the policy obs: the deployable layout is OBS_STAND (no base_lin_vel — the
robot can't observe it). OBS_FLAT needs base_lin_vel and is sim-verification only;
if used here, base_lin_vel is published as zeros and the policy will misbehave.

Deployment tip: launch MotorController with `move_duration:=0.0` so streamed 50 Hz
targets are tracked directly instead of each one starting a 1 s ramp.
"""

from __future__ import annotations

import threading
import time
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

from rl_env import (Env, ObsCfg, OBS_STAND, POLICY_JOINT_ORDER, DEFAULT_POS_VEC,
                    projected_gravity_from_quat)


class RealEnv(Env):
    def __init__(self, obs_cfg: ObsCfg = OBS_STAND, command=(0.0, 0.0, 0.0),
                 control_dt: float = 0.02,
                 command_topic: str = "/motor_commands",
                 state_topic: str = "/motor_states",
                 imu_topic: str = "/imu_states"):
        super().__init__(obs_cfg, command=command, control_dt=control_dt)

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("real_env")
        self._lock = threading.Lock()

        # Latest cached feedback (policy order, radians).
        self._joint_pos = DEFAULT_POS_VEC.copy()
        self._joint_vel = np.zeros(self.n_joints, dtype=np.float32)
        self._base_ang_vel = np.zeros(3, dtype=np.float32)
        self._proj_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self._got_states = False
        self._got_imu = False

        self._idx = {name: i for i, name in enumerate(POLICY_JOINT_ORDER)}

        self.node.create_subscription(JointState, state_topic, self._on_states, 10)
        self.node.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self._cmd_pub = self.node.create_publisher(JointState, command_topic, 10)

        # Spin the node in the background so callbacks fill the caches.
        self._spin = True
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    # ---- ROS plumbing ----
    def _spin_loop(self):
        while self._spin and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _on_states(self, msg: JointState):
        with self._lock:
            for i, name in enumerate(msg.name):
                k = self._idx.get(name)
                if k is None:
                    continue
                if i < len(msg.position):
                    self._joint_pos[k] = msg.position[i]
                if i < len(msg.velocity):
                    self._joint_vel[k] = msg.velocity[i]
            self._got_states = True

    def _on_imu(self, msg: Imu):
        with self._lock:
            self._base_ang_vel = np.array(
                [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                dtype=np.float32)
            q = msg.orientation  # geometry_msgs/Quaternion (x,y,z,w)
            if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) > 1e-6:
                # Use fused orientation if the IMU provides it.
                self._proj_gravity = projected_gravity_from_quat([q.w, q.x, q.y, q.z])
            else:
                # Fallback: gravity direction from the accelerometer (quasi-static).
                a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                              msg.linear_acceleration.z], dtype=np.float32)
                n = np.linalg.norm(a)
                if n > 1e-3:
                    self._proj_gravity = (-a / n).astype(np.float32)
            self._got_imu = True

    # ---- Env backend hooks ----
    def _read_signals(self) -> Dict[str, np.ndarray]:
        with self._lock:
            return {
                "base_lin_vel": np.zeros(3, dtype=np.float32),  # unobservable on hardware
                "base_ang_vel": self._base_ang_vel.copy(),
                "projected_gravity": self._proj_gravity.copy(),
                "joint_pos_rel": (self._joint_pos - DEFAULT_POS_VEC).astype(np.float32),
                "joint_vel": self._joint_vel.copy(),
            }

    def _apply_and_advance(self, targets: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(POLICY_JOINT_ORDER)
        msg.position = [float(x) for x in targets]
        self._cmd_pub.publish(msg)
        time.sleep(self.control_dt)  # pace the control loop (~50 Hz)

    def _reset_backend(self):
        # Can't physically reset the robot; wait for first feedback so the initial
        # observation is real, not the default placeholder.
        t0 = time.time()
        while not (self._got_states and self._got_imu) and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if not self._got_states:
            self.node.get_logger().warn("RealEnv.reset: no /motor_states yet.")
        if not self._got_imu:
            self.node.get_logger().warn("RealEnv.reset: no /imu_states yet.")

    def close(self):
        self._spin = False
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self.node.destroy_node()
