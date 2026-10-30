# IMU firmware (Arduino Nicla Sense ME)

Two firmwares for the robot's IMU; `imu_service.py` understands both formats
automatically:

| | `../imu.cpp` (flashed today) | `imu_stream_nicla/` (recommended) |
|---|---|---|
| Mode | poll: host sends `'L'`, gets one line | streams at 100 Hz on its own |
| Line | `ALL ax ay az gx gy gz pitch roll` | `IMU gx gy gz ax ay az qw qx qy qz` |
| Units | raw int16 LSB (host converts) | SI on-device (rad/s, m/s², unit quat) |
| Orientation | accel-only pitch/roll (discarded by the service) | **BHI260 on-chip fusion quaternion** |
| Run service | `imu_service.py --mode poll` (default) | `imu_service.py --mode stream` |

Why upgrade: the stream removes the poll round-trip latency/jitter, and the
fused quaternion gives `projected_gravity` that stays correct **while the robot
moves** — with the poll firmware the service falls back to the accelerometer,
which is only right when quasi-static. For balancing, that's a real difference.

Flashing: Arduino IDE → board *Nicla Sense ME* (Arduino Mbed OS Nicla boards) →
library *Arduino_BHY2* → open `imu_stream_nicla/imu_stream_nicla.ino` → upload.

Bench verification (both firmwares — the scale constants assume the BHI260AP
default ranges accel ±8 g / gyro ±2000 °/s):

1. `python3 imu_service.py --mode <poll|stream>` + `python3 tools/zmq_echo.py imu`
2. at rest: `|accel| ≈ 9.81 m/s²`, gyro ≈ 0 (and `|quat| ≈ 1` on stream)
3. rotate slowly 90° about each axis: that gyro axis integrates to ≈ 1.57 rad
4. check the MOUNTING axes against the body frame (x forward, y left, z up);
   remap in `imu_service.parse_imu_line()` if the Nicla is not axes-aligned.
