# legacy_ros2/ — frozen ROS2 versions (reference only)

These are the pre-ZMQ ROS2 nodes, kept for reference/provenance. They are NOT
part of the deployment stack anymore and need `rclpy` (a ROS 2 install) to run.

| old (here) | replaced by |
|---|---|
| `motor_controller.py` | `motor_service.py` (same safety layer, ZMQ topics, explicit enable) |
| `observer.py` | `imu_service.py` |
| `real_env.py` | `zmq_env.py` (`run_policy.py --backend zmq`) |

See `ZMQ_MIGRATION.md` for the topic spec and what changed.
