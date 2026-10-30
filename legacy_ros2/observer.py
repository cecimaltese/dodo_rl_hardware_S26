"""
observer.py — Observer ROS2 node: publishes the robot's IMU/body state.

Companion to MotorController in the deployment stack:
    MotorController -> /motor_states   (joint pos/vel/effort)
    Observer        -> /imu_states     (sensor_msgs/Imu: ang vel, lin accel, orient)
RealEnv subscribes to both to assemble the policy observation.

The wire protocol (line formats, unit scaling, AXIS_REMAP, baud auto-detection)
lives in imu_protocol.py — ROS-free so imu_check.py and imu_sniff.py share it.
This file is just the ROS node: poll, wrap in sensor_msgs/Imu, publish.

BOTH firmware versions are supported — flashing is decoupled from this code:
  * v1 (`imu_firmware_v1/`)          — 'L' -> 8 fields, NO fused orientation.
    Gravity is derived from the accelerometer, which is valid only QUASI-STATIC:
    the accelerometer reads gravity PLUS linear acceleration, so during a shove
    the estimate is corrupted exactly when a balance policy needs it. Sensor
    ranges are BHI260AP defaults and must be GUESSED (see below).
  * v2 (`imu_firmware_v2/`)          — 'L' -> 12 fields, appends a fused
    game-rotation-vector quaternion (accel+gyro, no magnetometer, so motor fields
    can't poison it), and answers 'V' with its actual configured ranges. When v2
    is flashed this node auto-detects it, takes the ranges from the device, and
    RealEnv prefers the quaternion path — no edits needed here. v2 is preferred.

What the policy actually uses from here (see rl_env.OBS_STAND):
  * base_ang_vel       <- angular_velocity   (gyro, body frame, rad/s)
  * projected_gravity  <- orientation quat (preferred) or linear_acceleration
                          (RealEnv derives gravity direction from whichever is set)
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from imu_protocol import (parse_imu_line, query_version, open_imu,   # noqa: F401
                          GYRO_RANGE_DPS, ACCEL_RANGE_G, GYRO_LSB_TO_RAD_S,
                          ACCEL_LSB_TO_M_S2, AXIS_REMAP, BAUD_CANDIDATES)


class Observer(Node):
    def __init__(self):
        super().__init__("observer")
        self.declare_parameter("port", "/dev/ttyACM0")  # Nicla enumerates as ttyACM*
        self.declare_parameter("baud", 0)  # 0 = auto-detect (v2=921600, v1=115200)
        self.declare_parameter("imu_topic", "/imu_states")
        self.declare_parameter("rate", 100.0)          # Hz poll rate
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("gyro_range_dps", GYRO_RANGE_DPS)
        self.declare_parameter("accel_range_g", ACCEL_RANGE_G)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        topic = self.get_parameter("imu_topic").value
        rate = float(self.get_parameter("rate").value)
        self.frame_id = self.get_parameter("frame_id").value

        gyro_range = float(self.get_parameter("gyro_range_dps").value)
        accel_range = float(self.get_parameter("accel_range_g").value)

        if serial is None:
            raise RuntimeError("pyserial not installed (pip install pyserial).")
        self.ser, baud, ver = open_imu(port, baud or None)
        self.ser.timeout = 0.05          # tight per-poll timeout for the 100 Hz loop
        self._bad = 0

        # Prefer the ranges the firmware reports over our params: v2 self-reports and
        # is authoritative; v1 stays silent and we fall back to the (guessed) params.
        if ver:
            gyro_range = ver["gyro_range_dps"]
            accel_range = ver["accel_range_g"]
            src = f"firmware v{ver['proto']} (self-reported, authoritative)"
        else:
            src = "ROS params — GUESSED"
        self.gyro_scale = (gyro_range / 32768.0) * (math.pi / 180.0)
        self.accel_scale = (accel_range / 32768.0) * 9.80665

        self.pub = self.create_publisher(Imu, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._poll)
        self.get_logger().info(
            f"Observer up: polling IMU on {port}@{baud}, publishing {topic} at "
            f"{rate:.0f} Hz.")
        self.get_logger().info(
            f"Scales from {src}: gyro +/-{gyro_range:.0f} dps, accel +/-{accel_range:.0f} g.")
        if not ver:
            # A wrong gyro range scales base_ang_vel by a constant and the policy
            # never knows. Don't let that pass as a routine info line.
            self.get_logger().warn(
                "IMU did not answer 'V': sensor ranges are GUESSED, so base_ang_vel "
                "may be scaled wrong. Flash imu_firmware_v2 (self-reports its ranges) "
                "or verify with `python3 imu_check.py`.")

    def _poll(self):
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b"L")
            line = self.ser.readline().decode("ascii", errors="ignore")
        except Exception as e:
            self.get_logger().warn(f"serial read failed: {e}")
            return
        reading = parse_imu_line(line, self.gyro_scale, self.accel_scale)
        if reading is None:
            # "READY" at boot / a torn line is normal-ish; a STEADY stream of these
            # means the wire format doesn't match and the policy would fly blind on
            # a frozen default gravity. Say so instead of failing silently.
            self._bad += 1
            if self._bad in (20, 200, 2000):
                self.get_logger().error(
                    f"{self._bad} unparsable IMU lines in a row; last={line.strip()!r}. "
                    f"Expected 'ALL ax ay az gx gy gz pitch roll'. Run imu_sniff.py.")
            return
        self._bad = 0
        self.pub.publish(self._to_imu_msg(reading))

    def _to_imu_msg(self, r: dict) -> Imu:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        gx, gy, gz = r["gyro"]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx, gy, gz
        if "accel" in r:
            ax, ay, az = r["accel"]
            (msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z) = ax, ay, az
        if "quat" in r:
            w, x, y, z = r["quat"]
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = w, x, y, z
        else:
            # Leave orientation at all-zero so RealEnv falls back to the accelerometer.
            msg.orientation.w = 0.0
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = Observer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
