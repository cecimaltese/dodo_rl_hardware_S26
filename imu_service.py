"""
imu_service.py — publishes the robot's IMU over ZMQ (no ROS).

Port of the former ROS2 node (see legacy_ros2/observer.py). Companion to
motor_service.py in the deployment stack:

    motor_service  ->  motor_states   (joint pos/vel/torque)
    imu_service    ->  imu            (gyro, accel, optional fused quat)

ZmqEnv subscribes to both to assemble the policy observation. What the policy
actually uses from here (rl_env.OBS_STAND):
  * base_ang_vel       <- gyro (body frame, rad/s)
  * projected_gravity  <- quat if IMU_QUAT_VALID, else derived from accel

THE FIRMWARE (imu.cpp in this repo — Arduino Nicla Sense ME, BHY2/BHI260AP):
poll-based; prints "READY" at boot, then answers one line per command char:
    'L'  ->  "ALL ax ay az gx gy gz pitch roll"
accel/gyro are the RAW int16 sensor values (Arduino_BHY2 SensorXYZ is
unscaled), pitch/roll are accel-only tilt in degrees. We poll 'L' at --rate
and convert raw -> SI here:
    accel [m/s^2] = raw / ACC_LSB_PER_G   * 9.80665      (BHI260AP default ±8 g)
    gyro  [rad/s] = raw / GYRO_LSB_PER_DPS * pi/180      (default ±2000 °/s)
The pitch/roll fields are DISCARDED on purpose: they are the same quasi-static
accel tilt the Env already computes as its fallback, but yaw-less and noisy
under motion. A REAL orientation comes from the BHI260's on-chip sensor fusion
— flash firmware/imu_stream_nicla/ (recommended) to stream
    "IMU gx gy gz ax ay az qw qx qy qz"   (SI units, fused quat)
continuously; this service auto-detects that format, no flag needed
(--mode stream just skips sending the poll chars).

TODO(bench):
  * verify the scale constants: at rest |accel| must read ≈ 9.81 m/s^2, and a
    slow 90° rotation must integrate to ≈ 1.57 rad on the right gyro axis.
  * verify the MOUNTING orientation (body frame: x forward, y left, z up).
    If the Nicla is not mounted axes-aligned, remap axes/signs in
    parse_imu_line() — one place, both formats pass through it.

Run (robot):
    python3 imu_service.py --port /dev/ttyACM0 --baud 115200 --rate 100
"""

from __future__ import annotations

import argparse
import logging
import math
import time
from typing import Optional

import dodo_ipc as ipc

try:
    import serial  # pyserial
except ImportError:
    serial = None

log = logging.getLogger("imu_service")

# --- BHI260AP default full-scale ranges (Arduino_BHY2 SensorXYZ = raw int16) ---
# TODO(bench): verify (see module docstring). If the firmware ever configures a
# different range, update these two constants — nothing else.
ACC_LSB_PER_G = 4096.0        # ±8 g   -> 32768 / 8
GYRO_LSB_PER_DPS = 16.384     # ±2000 °/s -> 32768 / 2000
GRAVITY = 9.80665
DEG2RAD = math.pi / 180.0


# --- the one firmware-specific function ---------------------------------------
def parse_imu_line(line: str) -> Optional[dict]:
    """Parse one IMU line into SI units (body frame).

    Returns a dict with keys (quat optional):
        gyro   = (wx, wy, wz)   rad/s
        accel  = (ax, ay, az)   m/s^2
        quat   = (w, x, y, z)   fused orientation (stream firmware only)
    or None for anything unusable (banner, garbage, partial line).

    Understands BOTH firmware formats:
      poll   (imu.cpp, flashed today):  "ALL ax ay az gx gy gz pitch roll"
                                        raw int16 LSB; pitch/roll discarded
      stream (firmware/imu_stream_nicla): "IMU gx gy gz ax ay az qw qx qy qz"
                                        already SI; quat from BHY2 fusion
    """
    parts = line.split()
    if len(parts) < 7:
        return None
    tag = parts[0]
    try:
        vals = [float(p) for p in parts[1:]]
    except ValueError:
        return None

    if tag == "ALL" and len(vals) >= 6:
        ax, ay, az, gx, gy, gz = vals[:6]          # NOTE: accel first in ALL
        return {
            "gyro": (gx / GYRO_LSB_PER_DPS * DEG2RAD,
                     gy / GYRO_LSB_PER_DPS * DEG2RAD,
                     gz / GYRO_LSB_PER_DPS * DEG2RAD),
            "accel": (ax / ACC_LSB_PER_G * GRAVITY,
                      ay / ACC_LSB_PER_G * GRAVITY,
                      az / ACC_LSB_PER_G * GRAVITY),
        }

    if tag == "IMU" and len(vals) >= 6:            # stream firmware: SI already
        out = {"gyro": (vals[0], vals[1], vals[2]),
               "accel": (vals[3], vals[4], vals[5])}
        if len(vals) >= 10:
            quat = (vals[6], vals[7], vals[8], vals[9])
            # BHY2 fusion outputs a unit quat; all-zeros = not converged yet.
            if math.sqrt(sum(q * q for q in quat)) > 0.5:
                out["quat"] = quat
        return out

    return None
# ------------------------------------------------------------------------------


def reading_to_msg(r: dict) -> ipc.ImuSample:
    flags = 0
    quat = (0.0, 0.0, 0.0, 0.0)
    if "quat" in r:
        quat = tuple(r["quat"])
        flags |= ipc.IMU_QUAT_VALID
    # NOTE: IMU_LIN_VEL_VALID is deliberately NEVER set here — base linear
    # velocity is unobservable on hardware (sim_service only).
    return ipc.ImuSample(gyro=tuple(r["gyro"]),
                         accel=tuple(r.get("accel", (0.0, 0.0, 0.0))),
                         quat=quat, flags=flags)


def run(port: str, baud: int, rate_hz: float, mode: str):
    if serial is None:
        raise RuntimeError("pyserial not installed (pip install pyserial).")
    ser = serial.Serial(port, baud, timeout=0.05)
    pub = ipc.Publisher(ipc.TOPIC_IMU)
    loop = ipc.RateLoop(rate_hz, name="imu_service")
    log.info("imu_service up: %s mode on %s@%d, publishing '%s' at %.0f Hz.",
             mode, port, baud, ipc.TOPIC_IMU, rate_hz)

    ser.reset_input_buffer()      # drop the boot banner / stale replies
    n_bad = 0
    last_report = time.monotonic()
    try:
        while True:
            try:
                if mode == "poll":
                    # Lockstep: drop anything stale (late replies), one fresh
                    # request, one reply. Worst case a reply crosses the reset
                    # and we read a one-poll-old sample (10 ms at 100 Hz).
                    ser.reset_input_buffer()
                    ser.write(b"L")
                line = ser.readline().decode("ascii", errors="ignore")
            except Exception as e:
                log.warning("serial read failed: %s", e)
                loop.sleep()
                continue

            reading = parse_imu_line(line) if line else None
            if reading is not None:
                pub.send(reading_to_msg(reading))
            elif line:
                if line.startswith("READY"):
                    log.info("firmware banner: READY")
                else:
                    n_bad += 1

            if time.monotonic() - last_report > 5.0:
                if n_bad:
                    log.warning("%d unparseable lines in the last 5 s "
                                "(does parse_imu_line match the firmware?)", n_bad)
                    n_bad = 0
                log.debug(loop.stats_str())
                last_report = time.monotonic()

            if mode == "poll":
                loop.sleep()
            # stream mode: pace is set by the firmware; readline() blocks for us.
    except KeyboardInterrupt:
        pass
    finally:
        log.info(loop.stats_str())
        pub.close()
        ser.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0",
                    help="Nicla shows up as ttyACM* on Linux "
                         "(/dev/cu.usbmodem* on macOS)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=float, default=100.0,
                    help="poll rate [Hz] (poll mode only)")
    ap.add_argument("--mode", choices=["poll", "stream"], default="poll",
                    help="poll = imu.cpp as flashed today; stream = "
                         "firmware/imu_stream_nicla (recommended upgrade)")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s %(name)s %(levelname)s: %(message)s")
    run(args.port, args.baud, args.rate, args.mode)


if __name__ == "__main__":
    main()
