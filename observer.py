"""
observer.py — Observer ROS2 node: publishes the robot's IMU/body state.

Companion to MotorController in the deployment stack:
    MotorController -> /motor_states   (joint pos/vel/effort)
    Observer        -> /imu_states     (sensor_msgs/Imu: ang vel, lin accel, orient)
RealEnv subscribes to both to assemble the policy observation.

The IMU firmware (teammate's) streams a full reading over serial when sent the
single character 'L'. The exact wire format is firmware-specific, so the parsing
is isolated in `parse_imu_line()` — EDIT THAT ONE FUNCTION to match the firmware.
Everything else (polling, ROS publishing, framing) is generic.

What the policy actually uses from here (see rl_env.OBS_STAND):
  * base_ang_vel       <- angular_velocity   (gyro, body frame, rad/s)
  * projected_gravity  <- orientation quat (preferred) or linear_acceleration
                          (RealEnv derives gravity direction from whichever is set)
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import serial  # pyserial
except ImportError:
    serial = None


# --- EDIT THIS to match the IMU firmware's response to 'L' --------------------
def parse_imu_line(line: str) -> Optional[dict]:
    """Parse one IMU reading.

    Returns a dict with any of these keys (all optional except gyro):
        gyro   = (wx, wy, wz)        rad/s, body frame   [required]
        accel  = (ax, ay, az)        m/s^2, body frame
        quat   = (w, x, y, z)        orientation, if the firmware fuses it
    or None if the line is incomplete/garbage.

    PLACEHOLDER format assumed: comma-separated
        "gx,gy,gz,ax,ay,az[,qw,qx,qy,qz]"
    Replace the body to match the real firmware (units! deg/s -> rad/s, g -> m/s^2).
    """
    parts = line.strip().split(",")
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        return None
    if len(vals) < 6:
        return None
    out = {"gyro": (vals[0], vals[1], vals[2]),
           "accel": (vals[3], vals[4], vals[5])}
    if len(vals) >= 10:
        out["quat"] = (vals[6], vals[7], vals[8], vals[9])
    return out
# ------------------------------------------------------------------------------


class Observer(Node):
    def __init__(self):
        super().__init__("observer")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("imu_topic", "/imu_states")
        self.declare_parameter("rate", 100.0)          # Hz poll rate
        self.declare_parameter("frame_id", "imu_link")

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        topic = self.get_parameter("imu_topic").value
        rate = float(self.get_parameter("rate").value)
        self.frame_id = self.get_parameter("frame_id").value

        if serial is None:
            raise RuntimeError("pyserial not installed (pip install pyserial).")
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.pub = self.create_publisher(Imu, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._poll)
        self.get_logger().info(f"Observer up: polling IMU on {port}@{baud}, "
                               f"publishing {topic} at {rate:.0f} Hz.")

    def _poll(self):
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b"L")
            line = self.ser.readline().decode("ascii", errors="ignore")
        except Exception as e:
            self.get_logger().warn(f"serial read failed: {e}")
            return
        reading = parse_imu_line(line)
        if reading is None:
            return
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
