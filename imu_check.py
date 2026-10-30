"""
imu_check.py — one-command acceptance test for the Dodo IMU. No ROS required.

Answers exactly one question: "is the IMU working, and is it giving the policy what
it needs?" Run it on the bench before trusting `--backend real`.

    python3 imu_check.py                    # auto-detects port baud
    python3 imu_check.py --port /dev/ttyACM0
    python3 imu_check.py --level            # also assert the board is level (z-up)

Checks, in order — each prints PASS/FAIL and the number it judged on:
  1. PORT      firmware responds (auto-detects baud: tries fast, falls back)
  2. FORMAT    'ALL' lines parse (8 fields = fw v1, 12 = v2 with quaternion)
  3. SCALE     |accel| == 1 g at rest -> the counts->m/s^2 scale is correct.
               This is THE check: it is what proves the raw-int16 range is right.
  4. GYRO      bias at rest is small (a standing bias feeds base_ang_vel directly
               and the policy will fight a rotation that isn't happening)
  5. QUAT      fused quaternion present and unit-norm (v2 only; v1 skips)
  6. GRAVITY   projected_gravity is unit-norm, and the accel path and the quat path
               AGREE. Disagreement means AXIS_REMAP is inconsistent between them.
  7. RATE      achieved sample rate vs the 50 Hz control loop

Exit 0 = every check passed (or skipped). Exit 1 = something is wrong.

NOT checked here: whether AXIS_REMAP matches the robot. That needs the IMU bolted to
the frame -- use `imu_sniff.py --axes`. This tool only proves the IMU itself is sane.
"""

import argparse
import math
import statistics
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  pip install pyserial")

import imu_protocol as proto   # protocol layer; alias keeps call sites short
from imu_protocol import open_imu
from rl_env import projected_gravity_from_quat

CONTROL_HZ = 50.0                    # rl_env control loop we must keep up with

_fails = []


def check(name, ok, detail, skip=False):
    tag = "SKIP" if skip else ("PASS" if ok else "FAIL")
    print(f"  [{tag}] {name:9s} {detail}")
    if not ok and not skip:
        _fails.append(name)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=None, help="skip auto-detect")
    ap.add_argument("--samples", type=int, default=100)
    ap.add_argument("--level", action="store_true",
                    help="also assert the board is level and z-up (base frame)")
    args = ap.parse_args()

    print(f"\nIMU acceptance test — {args.port}\n")

    # ---- 1. PORT --------------------------------------------------------------
    try:
        ser, baud, ver = open_imu(args.port, args.baud)
    except RuntimeError as e:
        print(f"  [FAIL] PORT      {e}\n")
        return 1
    if ver:
        gs = (ver["gyro_range_dps"] / 32768.0) * (math.pi / 180.0)
        as_ = (ver["accel_range_g"] / 32768.0) * 9.80665
        check("PORT", True, f"firmware v{ver['proto']} @ {baud} baud — ranges from "
                            f"device: accel +/-{ver['accel_range_g']:.0f} g, "
                            f"gyro +/-{ver['gyro_range_dps']:.0f} dps")
    else:
        gs, as_ = proto.GYRO_LSB_TO_RAD_S, proto.ACCEL_LSB_TO_M_S2
        check("PORT", True, f"firmware v1 (no 'V') @ {baud} baud — ranges are GUESSED "
                            f"from imu_protocol.py constants; SCALE below is the real test")

    # ---- collect --------------------------------------------------------------
    samples, t0 = [], time.time()
    for _ in range(args.samples):
        ser.reset_input_buffer()
        ser.write(b"L")
        r = proto.parse_imu_line(ser.readline().decode("ascii", "ignore"), gs, as_)
        if r:
            samples.append(r)
    elapsed = time.time() - t0
    ser.close()

    # ---- 2. FORMAT ------------------------------------------------------------
    if not check("FORMAT", len(samples) >= args.samples * 0.9,
                 f"{len(samples)}/{args.samples} lines parsed"):
        print("\n  -> run `python3 imu_sniff.py` to see the raw bytes.")
        return 1

    def mean3(key):
        return [statistics.fmean(s[key][i] for s in samples) for i in range(3)]

    def norm(v):
        return math.sqrt(sum(c * c for c in v))

    # ---- 3. SCALE -------------------------------------------------------------
    a = mean3("accel")
    mag = norm(a)
    err = abs(mag - 9.80665) / 9.80665 * 100
    check("SCALE", err < 3.0,
          f"|accel| = {mag:.3f} m/s^2 ({err:.2f}% off 1 g)   "
          f"accel=({a[0]:+.2f},{a[1]:+.2f},{a[2]:+.2f})")
    if err >= 3.0:
        print("           -> the counts->m/s^2 scale is WRONG (or the board is moving).")
        print("              v1: fix ACCEL_RANGE_G in imu_protocol.py (imu_sniff.py says which).")
        print("              v2: the device reported its range — suspect a moving board.")

    # ---- 4. GYRO bias ---------------------------------------------------------
    g = mean3("gyro")
    gmag = norm(g)
    check("GYRO", gmag < 0.05,
          f"bias at rest = {gmag:.4f} rad/s  ({math.degrees(gmag):.2f} deg/s)   "
          f"gyro=({g[0]:+.4f},{g[1]:+.4f},{g[2]:+.4f})")
    if gmag >= 0.05:
        print("           -> was the board moving? A real standing bias feeds")
        print("              base_ang_vel and the policy fights a phantom rotation.")

    # ---- 5. QUAT --------------------------------------------------------------
    has_quat = "quat" in samples[-1]
    if has_quat:
        q = samples[-1]["quat"]
        qn = math.sqrt(sum(c * c for c in q))
        check("QUAT", abs(qn - 1.0) < 0.05,
              f"fused quaternion present, |q| = {qn:.4f}  "
              f"(w={q[0]:+.3f} x={q[1]:+.3f} y={q[2]:+.3f} z={q[3]:+.3f})")
    else:
        check("QUAT", True, "firmware v1 — no fused orientation; gravity will come "
                            "from the accelerometer (valid only quasi-static)", skip=True)

    # ---- 6. GRAVITY -----------------------------------------------------------
    pg_a = [-c / mag for c in a]
    detail = f"from accel = ({pg_a[0]:+.3f},{pg_a[1]:+.3f},{pg_a[2]:+.3f})"
    ok = abs(norm(pg_a) - 1.0) < 1e-3
    if has_quat:
        pg_q = [float(c) for c in projected_gravity_from_quat(list(samples[-1]["quat"]))]
        dot = max(-1.0, min(1.0, sum(x * y for x, y in zip(pg_a, pg_q))))
        ang = math.degrees(math.acos(dot))
        ok = ok and ang < 5.0
        detail += f"\n                     from quat  = ({pg_q[0]:+.3f},{pg_q[1]:+.3f},{pg_q[2]:+.3f})" \
                  f"   paths differ by {ang:.2f} deg"
    check("GRAVITY", ok, detail)
    if has_quat and not ok:
        print("           -> the accel and quaternion paths DISAGREE. AXIS_REMAP is")
        print("              applied inconsistently, or the board was moving.")

    if args.level:
        check("LEVEL", abs(pg_a[2] + 1.0) < 0.1,
              f"projected_gravity z = {pg_a[2]:+.3f} (want -1.0 when level and z-up)")
        if abs(pg_a[2] + 1.0) >= 0.1:
            print("           -> board is not level, OR its axes don't match the base")
            print("              frame yet. Once mounted: `imu_sniff.py --axes`.")

    # ---- 7. RATE --------------------------------------------------------------
    hz = len(samples) / elapsed
    check("RATE", hz >= CONTROL_HZ,
          f"{hz:.1f} Hz achieved ({elapsed / len(samples) * 1e3:.1f} ms/sample) "
          f"vs {CONTROL_HZ:.0f} Hz control loop")
    if hz < CONTROL_HZ:
        print(f"           -> BELOW the control rate: the policy would reuse a stale")
        print(f"              tilt for some steps. Not fatal for standing (slow")
        print(f"              dynamics) but it is real sim2real latency.")

    print()
    if _fails:
        print(f"  RESULT: FAIL — {', '.join(_fails)}\n")
        return 1
    print("  RESULT: PASS — IMU is sane and delivering what the policy needs.")
    print("          (AXIS_REMAP still unverified — needs the robot: imu_sniff.py --axes)\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
