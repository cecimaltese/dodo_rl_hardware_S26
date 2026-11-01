"""
imu_protocol.py — the Nicla Sense ME wire protocol.

Everything about talking to the IMU lives here: the 'ALL'/'V' line formats, the raw
int16 -> SI scaling, the board->base AXIS_REMAP, baud auto-detection. `imu_service.py`
(the ZMQ service), `imu_check.py` and `imu_sniff.py` all import from this one module.

Why it is its own file: this logic used to live inside observer.py (the former ROS2
node), which imports rclpy. imu_sniff.py was deliberately ROS-free, so it could not
share it and kept its OWN hardcoded baud — then the firmware moved to 921600 and
imu_sniff silently read a 921600 stream at 115200 and reported "no parsable 'ALL'
lines", as if the IMU were broken. Same failure as the duplicated fleet config (see
fleet_config.py): a constant that must agree in two places will eventually not. One
definition, imported.

The IMU is an Arduino Nicla Sense ME (BHI260AP); it answers the single character 'L'
with one full reading. The formats below are VERIFIED against the firmware source +
the Arduino_BHY2 library.

BOTH firmware versions are supported — flashing is decoupled from this code:
  * v1 (`firmware/imu_firmware_v1/`, = Jim's `~/Arduino/nicla_reader`) — 'L' -> 8 fields, NO fused
    orientation, 115200 baud. Gravity is derived from the accelerometer, valid only
    QUASI-STATIC: the accelerometer reads gravity PLUS linear acceleration, so during
    a shove the estimate is corrupted exactly when a balance policy needs it. Sensor
    ranges are BHI260AP defaults and must be GUESSED (see below).
  * v2 (`firmware/imu_firmware_v2/`) — 'L' -> 12 fields, appends a fused game-rotation-vector
    quaternion (accel+gyro, no magnetometer, so motor fields can't poison it), answers
    'V' with its actual configured ranges, and runs at 921600. Auto-detected. Preferred.

What the policy actually uses (see rl_env.OBS_STAND):
  * base_ang_vel       <- angular_velocity   (gyro, body frame, rad/s)
  * projected_gravity  <- orientation quat (preferred) or linear_acceleration
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

try:
    import serial  # pyserial
except ImportError:
    serial = None


# --- Wire format (VERIFIED against the firmware source) -----------------------
# Request 'L' -> exactly one line:
#   v1: "ALL <ax> <ay> <az> <gx> <gy> <gz> <pitch> <roll>\r\n"                (8 fields)
#   v2: "ALL <ax> <ay> <az> <gx> <gy> <gz> <pitch> <roll> <qw> <qx> <qy> <qz>" (12)
#   * SPACE-separated with a literal "ALL" tag — NOT comma-separated.
#   * ACCEL comes FIRST, then GYRO.
#   * pitch/roll are DEGREES, accel-derived, no yaw. Debug only — they are wrong
#     under acceleration for the same reason the accel gravity fallback is.
#   * v2's qw..qz are a unit quaternion, already scaled by the library (1/16384).
# The firmware also answers 'A'/'G'/'O'/'Q' and prints "READY" once at boot — all
# rejected below by the tag + field-count check.
#
# UNITS — the load-bearing part. Arduino_BHY2 `SensorXYZ::x()` returns **int16_t
# RAW COUNTS**, not physical units:
#   * sensors/SensorXYZ.h:11      -> `int16_t x() { return _data.x; }`
#   * sensors/DataParser.cpp:3-7  -> parse3DVector is a bare `data.getInt16(...)`,
#                                    no scaling applied
#   * sensors/SensorID.h:139/145  -> SENSOR_ID_ACC / SENSOR_ID_GYRO = VECTOR3D, 1.0
# The firmware assigns those ints to a float and prints "%.3f", so the wire carries
# e.g. "4096.000" — a sensor COUNT. Passing it through as rad/s would overstate the
# gyro by ~3 orders of magnitude. Counts scale by the configured dynamic range:
#     physical = raw * (range / 32768)
#
# Firmware v1 never calls setRange(), so its ranges are BHI260AP DEFAULTS and the
# values below are a GUESS that MUST BE VERIFIED — run `python3 imu_sniff.py`
# (measures the accel range directly) and `--gyro-cal` (pins the gyro range).
# Overridable with imu_service.py --gyro-range-dps / --accel-range-g.
#
# Firmware v2 makes all of this moot: it sets its ranges explicitly and reports them
# over 'V', so query_version() below takes them from the device and these constants
# are never used. That is the whole point of v2 — stop guessing.
GYRO_RANGE_DPS = 2000.0   # v1 only; VERIFY — gyro full scale, +/- dps
ACCEL_RANGE_G = 8.0       # v1 only; VERIFY — accel full scale, +/- g

GYRO_LSB_TO_RAD_S = (GYRO_RANGE_DPS / 32768.0) * (math.pi / 180.0)
ACCEL_LSB_TO_M_S2 = (ACCEL_RANGE_G / 32768.0) * 9.80665
# NOTE: the accel scale CANCELS for our only use of it — ZmqEnv normalizes
# (-a/|a|) to get the gravity direction — so an accel_range_g error is harmless to
# the policy. It matters only for the published sensor_msgs/Imu being honest m/s^2.
# The GYRO scale does NOT cancel: it feeds base_ang_vel directly. Get it right.

# Axis remap: IMU board frame -> robot base frame (x forward, y left, z up).
# Identity until measured on the assembled robot. A wrong remap is SILENT and has
# the same character as the L/R motor mirroring: the policy gets plausible-looking
# numbers that mean the wrong thing. Rows = base axes in IMU terms, so
# v_base = AXIS_REMAP @ v_board; the URDF's imu_joint carries the same rotation.
# WRITTEN BY `imu_locate.py` from description/imu_mount.json, together with the
# URDF: measure with `imu_locate.py measure`, don't edit by hand (`check` would flag it).
AXIS_REMAP = ((1, 0, 0),
              (0, 1, 0),
              (0, 0, 1))


def _remap(x: float, y: float, z: float) -> Tuple[float, float, float]:
    r = AXIS_REMAP
    return (r[0][0] * x + r[0][1] * y + r[0][2] * z,
            r[1][0] * x + r[1][1] * y + r[1][2] * z,
            r[2][0] * x + r[2][1] * y + r[2][2] * z)


def _det3(r) -> float:
    return (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))


def _matrix_to_quat(r) -> Tuple[float, float, float, float]:
    """Rotation matrix -> (w, x, y, z). Assumes r is a proper rotation."""
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return (0.25 * s, (r[2][1] - r[1][2]) / s,
                (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s)
    if r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        return ((r[2][1] - r[1][2]) / s, 0.25 * s,
                (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s)
    if r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        return ((r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s,
                0.25 * s, (r[1][2] + r[2][1]) / s)
    s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
    return ((r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s,
            (r[1][2] + r[2][1]) / s, 0.25 * s)


def _quat_mul(a, b) -> Tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


# The quaternion must be remapped TOO, not just accel/gyro — ZmqEnv PREFERS the quat,
# so remapping only the vectors would leave projected_gravity silently in the BOARD
# frame while accel/gyro were in the base frame. Derivation: the IMU quat q maps
# board->world (v_world = q v_board q*), and v_board = R^T v_base, so the base's
# orientation is q_base = q (x) conj(q_R), where R = AXIS_REMAP.
#
# A physical mounting is always a PROPER rotation (det = +1). A signed permutation
# with det = -1 is a reflection — not a rotation, not representable as a quaternion,
# and always a measurement mistake (e.g. imu_sniff --axes picking two axes with signs
# that mirror rather than rotate). Fail loudly at import rather than ship a wrong tilt.
_DET = _det3(AXIS_REMAP)
if abs(_DET - 1.0) > 1e-6:
    raise ValueError(
        f"AXIS_REMAP has determinant {_DET:+.3f}, expected +1. det=-1 means it's a "
        f"REFLECTION, not a rotation — a real IMU mounting can never be one, so a sign "
        f"is wrong. Re-run `python3 imu_sniff.py --axes`.")

_Q_REMAP_CONJ = _matrix_to_quat(AXIS_REMAP)
_Q_REMAP_CONJ = (_Q_REMAP_CONJ[0], -_Q_REMAP_CONJ[1],
                 -_Q_REMAP_CONJ[2], -_Q_REMAP_CONJ[3])


def parse_raw_line(line: str) -> Optional[list]:
    """Parse an 'ALL' line into RAW SENSOR COUNTS — unscaled AND unremapped.

    Returns [ax, ay, az, gx, gy, gz, pitch, roll]           (firmware v1, 8 fields)
         or [ax, ay, az, gx, gy, gz, pitch, roll, qw,qx,qy,qz] (v2, 12 fields)
         or None if the line isn't a well-formed 'ALL' response.

    THE single definition of the wire format — parse_imu_line() builds on it, and so do
    the diagnostics, which genuinely need the raw board frame:
      * inferring the sensor range needs UNSCALED counts (|accel| in LSB == LSB per g);
      * `imu_sniff --axes` needs the UNREMAPPED board axes, because it is what
        DETERMINES AXIS_REMAP — parsing through the remap would be circular.
    """
    parts = line.strip().split()
    if not parts or parts[0] != "ALL" or len(parts) not in (9, 13):
        return None                      # 9 = firmware v1, 13 = v2 (adds quaternion)
    try:
        return [float(p) for p in parts[1:]]
    except ValueError:
        return None


def parse_imu_line(line: str,
                   gyro_lsb_to_rad_s: float = GYRO_LSB_TO_RAD_S,
                   accel_lsb_to_m_s2: float = ACCEL_LSB_TO_M_S2) -> Optional[dict]:
    """Parse one 'ALL' response into SI units, in the ROBOT BASE frame.

    Returns:
        gyro      = (wx, wy, wz)  rad/s
        accel     = (ax, ay, az)  m/s^2
        pitch_deg / roll_deg      the firmware's accel-derived angles (debug only)
        quat      = (qw, qx, qy, qz)  fused orientation — FIRMWARE v2 ONLY
    or None if the line is not a well-formed "ALL" response.

    On v1 there is no 'quat' key, so ZmqEnv falls back to deriving gravity from
    `accel`. The firmware's own pitch/roll are NOT used for that: they are computed
    from the accelerometer alone, so they carry exactly the same information as the
    raw accel while adding trig and a degrees round-trip.
    """
    v = parse_raw_line(line)
    if v is None:
        return None
    out = {
        "accel": _remap(v[0] * accel_lsb_to_m_s2,
                        v[1] * accel_lsb_to_m_s2,
                        v[2] * accel_lsb_to_m_s2),
        "gyro": _remap(v[3] * gyro_lsb_to_rad_s,
                       v[4] * gyro_lsb_to_rad_s,
                       v[5] * gyro_lsb_to_rad_s),
        "pitch_deg": v[6],
        "roll_deg": v[7],
    }
    if len(v) == 12:
        # Firmware v2: fused game-rotation-vector quaternion (already unit-scaled by
        # the library). ZmqEnv prefers this over the accel fallback automatically.
        # Reject a zero/degenerate quat so a not-yet-converged fusion can't publish a
        # quaternion that ZmqEnv would trust over the (valid) accelerometer.
        qw, qx, qy, qz = v[8], v[9], v[10], v[11]
        if abs(qw) + abs(qx) + abs(qy) + abs(qz) > 1e-6:
            # Rotate board-frame orientation into the base frame (see _Q_REMAP_CONJ).
            # No-op while AXIS_REMAP is identity.
            out["quat"] = _quat_mul((qw, qx, qy, qz), _Q_REMAP_CONJ)
    return out


def query_version(ser, attempts: int = 4) -> Optional[dict]:
    """Ask the firmware for its protocol version + configured sensor ranges ('V').

    Firmware v2 self-reports the ranges it read back off the chip, which removes the
    guesswork entirely. v1 doesn't implement 'V' -> returns None and the caller keeps
    its (guessed) param values.

    RETRIES because the FIRST command after opening the port is reliably dropped: the
    Nicla is USB-CDC and does not reset on open, so the first write lands while the
    CDC endpoint is still settling. Measured: attempt 1 times out, attempt 2 answers.
    A one-shot query here would silently mis-scale every run.
    """
    for i in range(attempts):
        try:
            ser.reset_input_buffer()
            ser.write(b"V")
            parts = ser.readline().decode("ascii", errors="ignore").strip().split()
        except Exception:
            return None
        if len(parts) == 4 and parts[0] == "VER":
            try:
                proto, arange, grange = int(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                return None
            if arange <= 0 or grange <= 0:
                # v2 explicitly reporting "range readback failed, don't trust me".
                # That's a definitive answer, not a dropped byte — don't retry it.
                return None
            return {"proto": proto, "accel_range_g": arange, "gyro_range_dps": grange}
        time.sleep(0.05)   # empty/torn line: CDC still settling, or this is v1
    return None


# Firmware v2 runs at 921600 (115200 capped the poll at ~41 Hz — BELOW the 50 Hz
# control loop); v1 runs at 115200. Auto-detecting means flashing the board never
# strands the host config, and one command works against either firmware.
BAUD_CANDIDATES = (921600, 115200)


def open_imu(port: str, baud: Optional[int] = None, settle: float = 2.0):
    """Open the IMU, auto-detecting baud. Returns (ser, baud, version_or_None).

    Probes with 'L' BEFORE asking 'V', and the order is load-bearing. Right after a
    flash the board is still rebooting/re-enumerating; a 'V' that lands too early
    fails, and falling back to the guessed constants on a board that is actually v2
    produces a SILENT ~2x scale error (observed: |accel| = 19.5 m/s^2). Proving the
    link with 'L' first means 'V' is only asked once the board is definitely talking.
    """
    err = None
    for b in ([baud] if baud else BAUD_CANDIDATES):
        try:
            ser = serial.Serial(port, b, timeout=0.5)
        except Exception as e:            # wrong port / already open elsewhere
            err = e
            continue
        time.sleep(settle)
        alive = False
        for _ in range(6):                # 'L' is answered by BOTH v1 and v2
            ser.reset_input_buffer()
            ser.write(b"L")
            if parse_imu_line(ser.readline().decode("ascii", errors="ignore")):
                alive = True
                break
        if alive:
            return ser, b, query_version(ser)   # link is up -> 'V' is trustworthy now
        ser.close()
    raise RuntimeError(
        f"no IMU response on {port} at any of {BAUD_CANDIDATES}"
        + (f" (last error: {err})" if err else "")
        + ". Is the board flashed? Is something else holding the port? "
          "Run `python3 imu_sniff.py` to see the raw bytes.")
# ------------------------------------------------------------------------------
