/*
 * imu_firmware_v2.ino — Dodo IMU firmware for the Arduino Nicla Sense ME (BHI260AP).
 *
 * Supersedes v1 (../imu_firmware_v1). Host side: imu_service.py + imu_protocol.py.
 * Board: Arduino Nicla Sense ME     FQBN: arduino:mbed_nicla:nicla_sense
 *
 * WHAT CHANGED FROM v1, AND WHY
 * -----------------------------
 * 1. EXPLICIT SENSOR RANGES + A 'V' HANDSHAKE.
 *    v1 never called setRange(), so accel/gyro raw int16 counts scaled by an unknown
 *    BHI260AP default and the host had to GUESS. The gyro guess doesn't cancel — it
 *    feeds base_ang_vel straight into the policy. Now the ranges are set explicitly
 *    AND reported over 'V', so imu_service.py configures its scales from the device
 *    instead of a hardcoded constant. This deletes a whole class of silent bug.
 *
 * 2. FUSED ORIENTATION (game rotation vector).
 *    v1 had no fused orientation, so the host derived gravity from the accelerometer.
 *    That is only valid QUASI-STATIC: the accelerometer measures gravity PLUS linear
 *    acceleration, so during a shove the gravity estimate is corrupted exactly when a
 *    balance policy needs it. The BHI260AP fuses on-chip for free.
 *
 *    GAMERV, NOT RV — deliberate. SENSOR_ID_RV also fuses the MAGNETOMETER, which is
 *    worthless bolted next to 8 BLDC motors. SENSOR_ID_GAMERV is accel+gyro only.
 *    Its yaw drifts (no compass reference), but yaw is IRRELEVANT here:
 *    projected_gravity is gravity expressed in the body frame, gravity points along
 *    world -z, and that is yaw-invariant. Pitch/roll stay gravity-referenced and
 *    drift-free — exactly, and only, what the policy needs.
 *
 * BAUD: 921600, not 115200. Measured on the real board, the 'L' round-trip is
 * ~11 ms fixed (USB + one loop() -- BHY2.update() dominates) plus ~0.134 ms/char,
 * and that per-char term matches 115200's 0.087 ms/char almost exactly: the UART was
 * genuinely the bottleneck, capping the poll at ~41 Hz -- BELOW the 50 Hz control
 * loop. At 921600 the ~100-char line costs ~1 ms instead of ~13 ms.
 * The host (imu_service.py / imu_check.py) auto-detects, so old tools still work.
 *
 * WIRE PROTOCOL (ASCII lines, 921600 baud, request/response — one char in, one line out)
 *   'V' -> "VER <proto> <accel_range_g> <gyro_range_dps>"
 *   'L' -> "ALL <ax> <ay> <az> <gx> <gy> <gz> <pitch> <roll> <qw> <qx> <qy> <qz>"
 *   'A' -> "ACCEL <ax> <ay> <az>"          (v1 compat, debugging)
 *   'G' -> "GYRO <gx> <gy> <gz>"           (v1 compat, debugging)
 *   'O' -> "ORIENT <pitch> <roll>"         (v1 compat, debugging)
 *   'Q' -> "QUAT <qw> <qx> <qy> <qz> <accuracy>"
 *
 *   accel/gyro  = RAW int16 counts. physical = raw * (range / 32768). Ranges via 'V'.
 *   pitch/roll  = DEGREES, accel-derived (kept from v1 for eyeball debugging only;
 *                 prefer the quaternion — these two are wrong under acceleration).
 *   qw..qz      = unit quaternion, already scaled by the library (factor 1/16384).
 *   accuracy    = fusion accuracy estimate, radians.
 *
 *   'L' stays a superset of v1's line (same first 8 fields, 4 appended), so an old
 *   host still parses it if it ignores extra fields. imu_protocol.py accepts 9 or 13.
 */

#include <Arduino.h>
#include <Arduino_BHY2.h>

// --- Protocol ---------------------------------------------------------------
static const int PROTO_VERSION = 2;

// --- Sensor dynamic ranges --------------------------------------------------
// Valid values (Arduino_BHY2 sensors/SensorClass.h:24-42):
//   accel: 2 | 4 | 8 | 16            (g)
//   gyro : 125 | 250 | 500 | 1000 | 2000  (dps)
// Rationale: a standing/balancing biped sees ~1 g plus modest transients, and its
// body rates are slow. Tighter ranges = finer resolution per count. These are
// deliberately conservative; narrow them once you see real numbers on the rack.
// NOTE: changing the accel range also affects sensors sharing the physical accel
// (e.g. gravity, linear accel) — see BHI260AP datasheet "Change Sensor Dynamic Range".
static const uint16_t ACCEL_RANGE_G  = 4;
static const uint16_t GYRO_RANGE_DPS = 1000;

// Sample rate must comfortably exceed the host's poll rate (imu_service.py: 100 Hz) so
// every request is answered with a fresh sample. Latency 0 = don't sit in the FIFO.
static const float    SENSOR_RATE_HZ = 400.0f;
static const uint32_t SENSOR_LATENCY = 0;

SensorXYZ accel(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);
SensorQuaternion rv(SENSOR_ID_GAMERV);   // accel+gyro fusion, NO magnetometer

// 'V' reports the ranges read back LIVE from the chip, never a cached copy and never
// the constants above. Learned the hard way: reading the config in setup() — even
// after two BHY2.update()s and a 50 ms delay — returns an accel range of 0, because
// the BHI260 hasn't published the config that early. The same read from loop()
// returns 4. So: read it live, on demand. 0 means "don't trust me" and the host
// falls back to its own params rather than believing a wrong scale.
static bool is_valid_accel_range(uint16_t r) {
    return r == 2 || r == 4 || r == 8 || r == 16;
}
static bool is_valid_gyro_range(uint16_t r) {
    return r == 125 || r == 250 || r == 500 || r == 1000 || r == 2000;
}


void setup() {
    Serial.begin(921600);   // see BAUD note in the header — 115200 capped polling at 41 Hz
    delay(1000);

    BHY2.begin();

    // begin() BEFORE setRange(): the sensor must be subscribed before the BHI260
    // will accept a range command for it.
    accel.begin(SENSOR_RATE_HZ, SENSOR_LATENCY);
    gyro.begin(SENSOR_RATE_HZ, SENSOR_LATENCY);
    rv.begin(SENSOR_RATE_HZ, SENSOR_LATENCY);

    accel.setRange(ACCEL_RANGE_G);
    gyro.setRange(GYRO_RANGE_DPS);

    // Deliberately IGNORE setRange()'s return code. Its convention is INVERTED and
    // undocumented — BoschSensortec::configureSensorRange returns 1 on SUCCESS and 0
    // on failure (BoschSensortec.cpp:120-128), so the natural `!= 0` check reads
    // success as failure. We don't ask whether it worked; 'V' reads the chip back and
    // reports what it ACTUALLY has. Verify with 'V' (or 'D'), never with this call.

    // Note: no range in this banner. The Nicla is USB-CDC and does NOT reset when the
    // host opens the port, so nobody ever sees this line anyway — 'V' is the contract.
    Serial.println("READY");
}


void loop() {
    BHY2.update();   // must be called frequently

    if (!Serial.available()) return;
    char c = Serial.read();

    // Assign to float FIRST. Serial.print(int, N) prints in BASE N; only the float
    // overload treats N as decimal places. accel.x() returns int16_t, so
    // Serial.print(accel.x(), 3) would print in base 3. This bit us in v1's style.
    float ax = accel.x();
    float ay = accel.y();
    float az = accel.z();

    float gx = gyro.x();
    float gy = gyro.y();
    float gz = gyro.z();

    switch (c) {

        case 'V': {  // VERSION + ranges — the host configures its scales from this.
                     // Read LIVE from the chip (see the note above is_valid_*_range):
                     // a setup-time read returns 0 for accel. 0 => "don't trust me".
            uint16_t ar = accel.getConfiguration().range;
            uint16_t gr = gyro.getConfiguration().range;
            Serial.print("VER ");
            Serial.print(PROTO_VERSION); Serial.print(" ");
            Serial.print(is_valid_accel_range(ar) ? ar : 0); Serial.print(" ");
            Serial.println(is_valid_gyro_range(gr) ? gr : 0);
            break;
        }

        case 'A':   // ACCEL
            Serial.print("ACCEL ");
            Serial.print(ax, 3); Serial.print(" ");
            Serial.print(ay, 3); Serial.print(" ");
            Serial.println(az, 3);
            break;

        case 'G':   // GYRO
            Serial.print("GYRO ");
            Serial.print(gx, 3); Serial.print(" ");
            Serial.print(gy, 3); Serial.print(" ");
            Serial.println(gz, 3);
            break;

        case 'O': { // ORIENT (pitch, roll) — accel-only, debugging convenience
            float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958;
            float roll  = atan2(ay, az) * 57.2958;

            Serial.print("ORIENT ");
            Serial.print(pitch, 2); Serial.print(" ");
            Serial.println(roll, 2);
            break;
        }

        case 'D': { // DBG — raw config readback for both sensors, unvalidated.
                    // Diagnostic for exactly the case that bit us: a readback value
                    // that isn't in the documented set, masked to 0 by 'V'.
            SensorConfig ac = accel.getConfiguration();
            SensorConfig gc = gyro.getConfiguration();
            Serial.print("DBG accel range="); Serial.print(ac.range);
            Serial.print(" sens="); Serial.print(ac.sensitivity);
            Serial.print(" rate="); Serial.print(ac.sample_rate, 1);
            Serial.print(" lat="); Serial.print(ac.latency);
            Serial.print(" | gyro range="); Serial.print(gc.range);
            Serial.print(" sens="); Serial.print(gc.sensitivity);
            Serial.print(" rate="); Serial.print(gc.sample_rate, 1);
            Serial.print(" lat="); Serial.println(gc.latency);
            break;
        }

        case 'Q':   // QUAT — fused orientation + fusion accuracy
            Serial.print("QUAT ");
            Serial.print(rv.w(), 6); Serial.print(" ");
            Serial.print(rv.x(), 6); Serial.print(" ");
            Serial.print(rv.y(), 6); Serial.print(" ");
            Serial.print(rv.z(), 6); Serial.print(" ");
            Serial.println(rv.accuracy(), 4);
            break;

        case 'L': { // ALL — the line imu_service.py polls at 100 Hz
            float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958;
            float roll  = atan2(ay, az) * 57.2958;

            Serial.print("ALL ");
            Serial.print(ax, 3); Serial.print(" ");
            Serial.print(ay, 3); Serial.print(" ");
            Serial.print(az, 3); Serial.print(" ");
            Serial.print(gx, 3); Serial.print(" ");
            Serial.print(gy, 3); Serial.print(" ");
            Serial.print(gz, 3); Serial.print(" ");
            Serial.print(pitch, 2); Serial.print(" ");
            Serial.print(roll, 2); Serial.print(" ");
            Serial.print(rv.w(), 6); Serial.print(" ");
            Serial.print(rv.x(), 6); Serial.print(" ");
            Serial.print(rv.y(), 6); Serial.print(" ");
            Serial.println(rv.z(), 6);
            break;
        }

        default:
            break;
    }
}
