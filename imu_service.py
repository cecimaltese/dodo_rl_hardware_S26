"""
imu_service.py — publishes the robot's IMU over ZMQ (no ROS).

Companion to motor_service.py in the deployment stack:

    motor_service  ->  motor_states   (joint pos/vel/torque)
    imu_service    ->  imu            (gyro, accel, fused quat)

ZmqEnv subscribes to both to assemble the policy observation. What the policy
actually uses from here (rl_env.OBS_STAND):
  * base_ang_vel       <- gyro (body frame, rad/s)
  * projected_gravity  <- quat if IMU_QUAT_VALID, else derived from accel

The wire protocol (line formats, unit scaling, AXIS_REMAP into the body frame,
baud auto-detection) lives in imu_protocol.py, shared with imu_check.py and
imu_sniff.py. This file is only the service: poll, convert, publish.

The IMU is an Arduino Nicla Sense ME (BHI260AP). Both firmwares in firmware/ are
auto-detected:
  * v2 (flashed, preferred) — 921600 baud, reports its sensor ranges over 'V',
    and appends a fused accel+gyro quaternion (no magnetometer).
  * v1 (Jim's original, the rollback) — 115200 baud, no quaternion, and its
    sensor ranges are GUESSED (--gyro-range-dps / --accel-range-g).

Run (robot):
    python3 imu_service.py                       # /dev/ttyACM0, baud auto-detected
"""

from __future__ import annotations

import argparse
import logging
import math
import time
from typing import Optional, Tuple

import dodo_ipc as ipc
from imu_protocol import ACCEL_RANGE_G, GYRO_RANGE_DPS, open_imu, parse_imu_line

log = logging.getLogger("imu_service")


def sensor_scales(ver: Optional[dict], gyro_range_dps: float = GYRO_RANGE_DPS,
                  accel_range_g: float = ACCEL_RANGE_G) -> Tuple[float, float, str]:
    """Raw-count scales (gyro -> rad/s, accel -> m/s^2) and where they came from.

    The firmware's self-report (v2 'V') is authoritative; the given ranges are
    only used when it stays silent (v1), and are then a guess.
    """
    if ver:
        gyro_range_dps, accel_range_g = ver["gyro_range_dps"], ver["accel_range_g"]
        src = (f"firmware v{ver['proto']} (self-reported)" if ver.get("readback", True)
               else f"firmware v{ver['proto']}'s configured ranges (its readback said 0, "
                    f"so not verified on the chip)")
    else:
        src = "GUESSED (firmware did not answer 'V')"
    return ((gyro_range_dps / 32768.0) * (math.pi / 180.0),
            (accel_range_g / 32768.0) * 9.80665, src)


def reading_to_msg(r: dict) -> ipc.ImuSample:
    flags = 0
    quat = (0.0, 0.0, 0.0, 0.0)
    if "quat" in r:
        quat = tuple(r["quat"])
        flags |= ipc.IMU_QUAT_VALID
    # NOTE: IMU_LIN_VEL_VALID is deliberately NEVER set here — base linear
    # velocity is unobservable on hardware (sim_service only).
    return ipc.ImuSample(gyro=tuple(r["gyro"]), accel=tuple(r["accel"]),
                         quat=quat, flags=flags)


def run(port: str, baud: int, rate_hz: float, gyro_range_dps: float,
        accel_range_g: float):
    ser, baud, ver = open_imu(port, baud or None)
    ser.timeout = 0.05            # tight per-poll timeout for the poll loop
    gyro_scale, accel_scale, src = sensor_scales(ver, gyro_range_dps, accel_range_g)
    pub = ipc.Publisher(ipc.TOPIC_IMU)
    loop = ipc.RateLoop(rate_hz, name="imu_service")
    log.info("imu_service up: polling %s@%d, publishing '%s' at up to %.0f Hz.",
             port, baud, ipc.TOPIC_IMU, rate_hz)
    log.info("sensor ranges from %s", src)
    if ver and not ver.get("readback", True):
        log.warning("IMU range readback kept saying 0: using firmware v2's configured "
                    "ranges. `python3 imu_check.py` verifies them (|accel| = 1 g).")
    if not ver:
        # A wrong gyro range scales base_ang_vel by a constant and the policy never
        # knows. Don't let that pass as a routine info line.
        log.warning("IMU did not answer 'V': ranges are GUESSED, so base_ang_vel may "
                    "be scaled wrong. Flash firmware/imu_firmware_v2 or verify with "
                    "`python3 imu_check.py`.")

    n_bad = 0
    last_report = time.monotonic()
    try:
        while True:
            try:
                # Lockstep: drop anything stale (late replies), one fresh request,
                # one reply.
                ser.reset_input_buffer()
                ser.write(b"L")
                line = ser.readline().decode("ascii", errors="ignore")
            except Exception as e:
                log.warning("serial read failed: %s", e)
                loop.sleep()
                continue

            reading = parse_imu_line(line, gyro_scale, accel_scale) if line else None
            if reading is not None:
                pub.send(reading_to_msg(reading))
            elif line.startswith("READY"):
                log.info("firmware banner: READY")
            else:
                n_bad += 1

            if time.monotonic() - last_report > 5.0:
                if n_bad:
                    # A STEADY stream of these means the wire format doesn't match and
                    # ZmqEnv would trip on a stale imu stream. Say why.
                    log.warning("%d unparsable or empty IMU replies in the last 5 s "
                                "(run imu_sniff.py to see the raw lines)", n_bad)
                    n_bad = 0
                log.debug(loop.stats_str())
                last_report = time.monotonic()

            loop.sleep()
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
    ap.add_argument("--baud", type=int, default=0,
                    help="0 = auto-detect (v2: 921600, v1: 115200)")
    ap.add_argument("--rate", type=float, default=100.0, help="poll rate cap [Hz]")
    ap.add_argument("--gyro-range-dps", type=float, default=GYRO_RANGE_DPS,
                    help="v1 only: gyro full scale (v2 reports its own)")
    ap.add_argument("--accel-range-g", type=float, default=ACCEL_RANGE_G,
                    help="v1 only: accel full scale (v2 reports its own)")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(level=args.log_level.upper(),
                        format="%(asctime)s %(name)s %(levelname)s: %(message)s")
    run(args.port, args.baud, args.rate, args.gyro_range_dps, args.accel_range_g)


if __name__ == "__main__":
    main()
