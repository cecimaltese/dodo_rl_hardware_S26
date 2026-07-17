"""
imu_sniff.py — bench diagnostic for the Nicla Sense ME IMU. No ROS, no deps but pyserial.

Answers the three questions observer.py cannot answer from source alone:

  1. WIRE FORMAT   — dumps the raw bytes the firmware actually sends, undecoded.
  2. SENSOR RANGE  — firmware v1 never calls setRange(), so its raw
                     int16 counts scale by an UNKNOWN BHI260AP default range. At rest
                     |accel| == exactly 1 g, so the measured magnitude in counts IS
                     the LSB-per-g scale — that pins the accel range directly.
                     --gyro-cal pins the gyro range (the one that actually matters:
                     it feeds base_ang_vel, where an error does NOT cancel).
  3. AXIS FRAME    — --axes builds the board -> robot-base remap from three poses.

Usage:
    python3 imu_sniff.py                       # dump + verdict on the format & accel range
    python3 imu_sniff.py --port /dev/ttyACM0   # Nicla usually enumerates as ttyACM*
    python3 imu_sniff.py --gyro-cal            # pin the gyro range (do this one!)
    python3 imu_sniff.py --axes                # build AXIS_REMAP for observer.py

Firmware v2 self-reports its ranges over 'V', so the range checks here are an
INDEPENDENT cross-check of the chip rather than the source of truth. On v2 the only
thing you must still set by hand is AXIS_REMAP (in imu_protocol.py) — and that needs
the IMU bolted to the robot. On v1, --gyro-cal IS the source of truth.
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

# Shared with observer.py / imu_check.py. In particular open_imu AUTO-DETECTS the baud
# (v2 = 921600, v1 = 115200). This file used to hardcode 115200; when the firmware moved
# to 921600 it silently read a 921600 stream at 115200, every line was garbage, and it
# reported "no parsable 'ALL' lines" — looking exactly like a broken IMU.
from imu_protocol import open_imu, parse_raw_line, BAUD_CANDIDATES  # noqa: F401

AXES = ("x", "y", "z")
CANDIDATE_ACCEL_RANGES_G = (2.0, 4.0, 8.0, 16.0)
CANDIDATE_GYRO_RANGES_DPS = (125.0, 250.0, 500.0, 1000.0, 2000.0)


def poll(ser, tag=b"L"):
    """Send one request, return the raw response line as bytes (no decoding)."""
    ser.reset_input_buffer()
    ser.write(tag)
    return ser.readline()


def parse_all(raw):
    """Raw counts from an 'ALL' line (v1: 8 fields, v2: 12), or None.

    Delegates to imu_protocol.parse_raw_line — the single definition of the format.
    This file used to carry its own copy that hardcoded `len(parts) != 9`, so when the
    v2 firmware started sending 13 parts every line was silently rejected and the tool
    reported "no parsable 'ALL' lines" as if the IMU were dead. Raw (not parse_imu_line)
    is deliberate: range inference needs unscaled counts and --axes needs the UNREMAPPED
    board frame.
    """
    try:
        return parse_raw_line(raw.decode("ascii", errors="ignore"))
    except Exception:
        return None


def collect(ser, n, label):
    """Collect n parsed samples, showing progress."""
    out = []
    print(f"  {label} ", end="", flush=True)
    deadline = time.time() + 10.0 + n * 0.05
    while len(out) < n and time.time() < deadline:
        v = parse_all(poll(ser))
        if v:
            out.append(v)
            if len(out) % max(1, n // 20) == 0:
                print(".", end="", flush=True)
    print(f" {len(out)}/{n}")
    if not out:
        sys.exit("  no parsable 'ALL' lines — run the plain dump first to see the format.")
    return out


def mean_axis(samples, base):
    """Mean of the 3 components starting at index `base` (0=accel, 3=gyro)."""
    return [statistics.fmean(s[base + i] for s in samples) for i in range(3)]


# ---------------------------------------------------------------------------
def cmd_dump(ser, count, ver=None):
    print("\n=== 1. RAW BYTES (exactly what the firmware sends) ===")
    print("Expecting v1:  b'ALL ax ay az gx gy gz pitch roll\\r\\n'          (8 fields)")
    print("      or v2:  b'ALL ax ay az gx gy gz pitch roll qw qx qy qz\\r\\n' (12)\n")
    good, nfields = 0, 0
    for i in range(count):
        raw = poll(ser)
        v = parse_all(raw)
        ok = "OK " if v else "BAD"
        print(f"  [{i:2d}] {ok} {raw!r}")
        if v:
            good += 1
            nfields = len(v)
            if good == 1:
                print(f"       -> accel=({v[0]:.3f}, {v[1]:.3f}, {v[2]:.3f})  "
                      f"gyro=({v[3]:.3f}, {v[4]:.3f}, {v[5]:.3f})  "
                      f"pitch={v[6]:.2f} deg  roll={v[7]:.2f} deg")
                if len(v) == 12:
                    print(f"       -> quat=({v[8]:+.4f}, {v[9]:+.4f}, {v[10]:+.4f}, {v[11]:+.4f})"
                          f"   (fused, v2)")
        time.sleep(0.05)

    print(f"\n=== VERDICT: format ===")
    if good == 0:
        print("  FAIL — nothing matched 'ALL ...'.")
        print("  * 'READY' only      -> firmware booted but isn't answering 'L'.")
        print("  * empty/timeout     -> wrong port (baud is auto-detected).")
        print("  * garbage bytes     -> baud mismatch.")
        return
    print(f"  OK — {good}/{count} lines parsed as 'ALL' + {nfields} fields "
          f"(firmware v{'2' if nfields == 12 else '1'}).")

    print("\n=== 2. ACCEL RANGE (robot LEVEL and STILL for this to be valid) ===")
    samples = collect(ser, 60, "sampling at rest:")
    a = mean_axis(samples, 0)
    mag = math.sqrt(sum(c * c for c in a))
    print(f"  mean accel = ({a[0]:+.1f}, {a[1]:+.1f}, {a[2]:+.1f})   |accel| = {mag:.1f}")

    if abs(mag - 9.80665) < 1.5:
        print("  -> |accel| ~= 9.81: firmware is ALREADY sending m/s^2, not raw counts.")
        print("     observer.py assumes RAW COUNTS — set accel_range_g so the scale is 1.0,")
        print("     or simplify the parser. Re-check SensorXYZ's return type.")
    elif abs(mag - 1.0) < 0.2:
        print("  -> |accel| ~= 1.0: firmware is already sending g.")
    elif mag < 100:
        print(f"  -> |accel| = {mag:.2f} is neither ~1, ~9.81, nor a plausible count.")
        print("     Is the robot actually level and still?")
    else:
        best = min(CANDIDATE_ACCEL_RANGES_G, key=lambda r: abs(mag - 32768.0 / r))
        implied = 32768.0 / mag
        print(f"  -> raw counts confirmed. 1 g = {mag:.0f} LSB  ->  implied full scale "
              f"= +/-{implied:.2f} g   (closest standard: +/-{best:.0f} g)")
        if abs(implied - best) / best > 0.1:
            print("     WARNING: >10% off the nearest standard range — recheck 'level and still'.")
        if ver:
            # v2 self-reports; this measurement is an INDEPENDENT cross-check of it.
            claimed = ver["accel_range_g"]
            agree = abs(implied - claimed) / claimed < 0.1
            print(f"\n  Device 'V' reports +/-{claimed:.0f} g; measured +/-{implied:.2f} g "
                  f"-> {'AGREE' if agree else 'DISAGREE !!'}")
            if agree:
                print("  Nothing to set — observer.py already takes the scale from the device.")
            else:
                print("  The chip's self-report and physics disagree. Do NOT trust 'V'; "
                      "investigate before deploying.")
        else:
            print(f"\n  SET IN imu_protocol.py:  ACCEL_RANGE_G = {best}   (firmware v1 only)")
        print("  (Reminder: an accel-range error CANCELS in projected_gravity, which")
        print("   normalizes. This mostly matters for /imu_states being honest m/s^2.)")

    g = mean_axis(samples, 3)
    gmag = math.sqrt(sum(c * c for c in g))
    print(f"\n  gyro bias at rest = ({g[0]:+.2f}, {g[1]:+.2f}, {g[2]:+.2f})  |g| = {gmag:.2f} counts")
    if gmag > 200:
        print("  WARNING: large gyro bias — was the robot moving? A standing bias feeds")
        print("           base_ang_vel directly and the policy will fight a phantom rotation.")
    print("\n  The gyro RANGE cannot be inferred at rest. Run:  python3 imu_sniff.py --gyro-cal")


# ---------------------------------------------------------------------------
def cmd_gyro_cal(ser, ver=None):
    """Integrate raw gyro counts through a known rotation -> implied full-scale range.

    Why this matters most: gyro counts feed base_ang_vel with NO normalization, so a
    wrong range scales 3 of the 30 obs dims by a constant. observer.py guesses 2000 dps.

    You do NOT need to know the board's axes. The BHI260AP range is one global setting
    for all three gyro axes, and for a rotation about any fixed axis the integral is
    `∫w dt = theta * n_hat`, so `|∫w dt| = theta` whatever the orientation. We integrate
    all three components and take the norm — the axis cancels out of the math.
    """
    print("\n=== GYRO RANGE CALIBRATION ===")
    print("Integrating raw counts through a rotation you measure by hand.\n")
    print("  You do NOT need to know which board axis is which:")
    print("   * the gyro range is ONE global setting for all 3 axes, and")
    print("   * |integral of w dt| = the total angle, whatever axis you turn about.")
    print("  So: rotate about ANY axis. Just keep it to a SINGLE axis (don't tumble it),")
    print("  and go smoothly over ~3-5 s. 90 deg against a table edge or a door frame is")
    print("  plenty — the candidate ranges are 2x apart, so even ~15% error is decisive.\n")

    try:
        truth = float(input("  Angle you will rotate through, in degrees [90]: ").strip() or "90")
    except ValueError:
        sys.exit("  not a number")
    if truth <= 0:
        sys.exit("  angle must be positive")

    input("  Hold the board STILL, then press Enter to measure gyro bias...")
    rest = collect(ser, 40, "bias:")
    bias = mean_axis(rest, 3)
    print(f"  bias = ({bias[0]:+.2f}, {bias[1]:+.2f}, {bias[2]:+.2f}) counts")

    input(f"\n  Press Enter, then rotate {truth:.0f} deg about ANY single axis...")
    print("  integrating — press Enter again when the rotation is COMPLETE.")

    import select
    integral = [0.0, 0.0, 0.0]
    t_prev, n = time.time(), 0
    while not select.select([sys.stdin], [], [], 0)[0]:
        v = parse_all(poll(ser))
        now = time.time()
        if v:
            dt = now - t_prev
            for i in range(3):
                integral[i] += (v[3 + i] - bias[i]) * dt   # counts * seconds
            n += 1
        t_prev = now
    sys.stdin.readline()

    counts_s = math.sqrt(sum(c * c for c in integral))
    dom = max(range(3), key=lambda k: abs(integral[k]))
    purity = abs(integral[dom]) / counts_s if counts_s else 0.0

    print(f"\n  integral = ({integral[0]:+.1f}, {integral[1]:+.1f}, {integral[2]:+.1f}) count-s"
          f"   |integral| = {counts_s:.1f}   ({n} samples)")
    if counts_s < 1e-6:
        sys.exit("  integral ~ 0 — did the board actually rotate? Is it still connected?")
    print(f"  -> you rotated mostly about board {AXES[dom]} ({purity * 100:.0f}% of the total)")
    if purity < 0.9:
        print("     (rotation was spread over several axes — still valid, the norm handles it,")
        print("      but a cleaner single-axis turn gives a tighter number)")

    # |integral| * (range/32768) = truth  ->  range = truth * 32768 / |integral|
    implied = truth * 32768.0 / counts_s
    best = min(CANDIDATE_GYRO_RANGES_DPS, key=lambda r: abs(r - implied))
    print(f"\n  implied full scale = +/-{implied:.0f} dps   "
          f"(closest standard: +/-{best:.0f} dps)")
    err = abs(implied - best) / best
    if err > 0.25:
        print(f"  WARNING: {err * 100:.0f}% off the nearest standard range — do NOT trust this.")
        print("           Likely: the real angle wasn't what you entered, the board tumbled,")
        print("           or the turn was fast enough to clip the range. Repeat it.")
        return
    print(f"  ({err * 100:.0f}% off the standard value — good.)")

    if ver:
        # The whole point on v2: an INDEPENDENT physical check of the chip's self-report.
        # The gyro scale is the one link nothing else validates — |accel|=1g proves the
        # ACCEL scale, but at rest the gyro reads ~0 regardless of its range.
        claimed = ver["gyro_range_dps"]
        agree = abs(implied - claimed) / claimed < 0.25
        print(f"\n  Device 'V' reports +/-{claimed:.0f} dps; measured +/-{implied:.0f} dps "
              f"-> {'AGREE' if agree else 'DISAGREE !!'}")
        if agree:
            print("  Nothing to set — the chip's self-report is confirmed by physics, and")
            print("  observer.py already takes the scale from the device. base_ang_vel is sound.")
        else:
            print(f"  The chip claims one range and the rotation says another (off by "
                  f"{claimed / implied:.1f}x). base_ang_vel would be scaled WRONG and nothing")
            print("  else would catch it. Do NOT deploy until this is understood.")
    else:
        print(f"\n  SET IN imu_protocol.py:  GYRO_RANGE_DPS = {best}   (firmware v1 only)")


# ---------------------------------------------------------------------------
def cmd_axes(ser):
    """Build AXIS_REMAP: IMU board frame -> robot base frame (x fwd, y left, z up).

    At rest the accelerometer reads +1 g along whichever board axis points UP (it
    measures proper acceleration, not gravity). So each pose reveals one base axis.
    """
    print("\n=== AXIS REMAP ===")
    print("Robot base frame = x FORWARD, y LEFT, z UP (URDF convention).")
    print("At rest the accelerometer reads +1 g along the board axis pointing UP.\n")

    def pose(prompt):
        input(f"  {prompt}\n    press Enter when held steady... ")
        a = mean_axis(collect(ser, 30, "reading:"), 0)
        mag = math.sqrt(sum(c * c for c in a)) or 1.0
        unit = [c / mag for c in a]
        i = max(range(3), key=lambda k: abs(unit[k]))
        print(f"    accel unit = ({unit[0]:+.2f}, {unit[1]:+.2f}, {unit[2]:+.2f})"
              f"  -> dominant board axis {AXES[i]}{'+' if unit[i] > 0 else '-'}"
              f" ({abs(unit[i]) * 100:.0f}%)")
        if abs(unit[i]) < 0.8:
            print("    WARNING: no axis clearly dominant — the pose is off, or the board")
            print("             is mounted at an angle (a pure remap can't fix that).")
        return i, (1 if unit[i] > 0 else -1)

    print("POSE 1/3 — identifies base +Z (up)")
    zi, zs = pose("Stand the robot LEVEL, in its normal standing pose.")

    print("\nPOSE 2/3 — identifies base +X (forward)")
    print("  Nose-down tips base +x downward, so gravity gains a +x component and")
    print("  the accelerometer's x reads NEGATIVE. The sign is handled for you.")
    xi, xs = pose("Pitch the robot NOSE-DOWN ~90 deg (front face toward the floor).")
    xs = -xs

    print("\nPOSE 3/3 — identifies base +Y (left)")
    print("  Same logic: roll left-side-down -> accel reads negative along base y.")
    yi, ys = pose("Roll the robot LEFT-SIDE-DOWN ~90 deg.")
    ys = -ys

    print("\n=== RESULT ===")
    if len({xi, yi, zi}) != 3:
        print("  FAIL — two poses picked the SAME board axis. The poses were ambiguous")
        print("  or the board is mounted skew. Repeat with cleaner 90 deg poses.")
        return

    rows = [[0, 0, 0] for _ in range(3)]
    rows[0][xi], rows[1][yi], rows[2][zi] = xs, ys, zs
    print("  base_x <- board", f"{'+-'[xs < 0]}{AXES[xi]}")
    print("  base_y <- board", f"{'+-'[ys < 0]}{AXES[yi]}")
    print("  base_z <- board", f"{'+-'[zs < 0]}{AXES[zi]}")
    print("\n  PASTE INTO observer.py:\n")
    print("AXIS_REMAP = ((%2d, %2d, %2d)," % tuple(rows[0]))
    print("              (%2d, %2d, %2d)," % tuple(rows[1]))
    print("              (%2d, %2d, %2d))" % tuple(rows[2]))
    print("\n  Then verify: level robot -> observer publishes accel ~ (0, 0, +9.81)")
    print("  and RealEnv's projected_gravity ~ (0, 0, -1). That is the acceptance test.")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0",
                    help="Nicla usually enumerates as /dev/ttyACM* (default: %(default)s)")
    ap.add_argument("--baud", type=int, default=None,
                    help=f"skip auto-detect (tries {BAUD_CANDIDATES})")
    ap.add_argument("--count", type=int, default=10, help="raw lines to dump")
    ap.add_argument("--gyro-cal", action="store_true", help="pin the gyro full-scale range")
    ap.add_argument("--axes", action="store_true", help="build the board->base AXIS_REMAP")
    args = ap.parse_args()

    try:
        ser, baud, ver = open_imu(args.port, args.baud)
    except RuntimeError as e:
        sys.exit(f"{e}\n"
                 f"  ls /dev/ttyACM* /dev/ttyUSB*   to find the port\n"
                 f"  a serial monitor or a running observer.py will also hold it")

    ser.timeout = 0.2
    if ver:
        print(f"connected: {args.port} @ {baud} — firmware v{ver['proto']}, "
              f"ranges from device: accel +/-{ver['accel_range_g']:.0f} g, "
              f"gyro +/-{ver['gyro_range_dps']:.0f} dps")
        print("NOTE: v2 self-reports its ranges, so --gyro-cal is only a cross-check now;\n"
              "      observer.py already takes the scale from the device.")
    else:
        print(f"connected: {args.port} @ {baud} — firmware v1 (no 'V'), ranges UNKNOWN. "
              f"This is what --gyro-cal is for.")

    try:
        if args.gyro_cal:
            cmd_gyro_cal(ser, ver)
        elif args.axes:
            cmd_axes(ser)
        else:
            cmd_dump(ser, args.count, ver)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
