// imu_stream_nicla.ino — RECOMMENDED upgrade of imu.cpp (poll firmware).
//
// Board:   Arduino Nicla Sense ME (BHI260AP sensor hub)
// Library: Arduino_BHY2
// Flash:   Arduino IDE -> board "Nicla Sense ME" -> upload this sketch.
//
// Differences from imu.cpp (the poll firmware):
//   * STREAMS continuously at STREAM_HZ — no 'L' request round-trip, so no
//     polling latency/jitter on the host side.
//   * Adds the BHI260's ON-CHIP SENSOR FUSION quaternion (SENSOR_ID_RV).
//     That gives the policy a real orientation estimate under motion; the
//     accel-only pitch/roll of imu.cpp is only valid when quasi-static.
//   * Converts to SI UNITS on the device, so the host parser stays trivial:
//
//       IMU gx gy gz ax ay az qw qx qy qz\n
//            [rad/s]   [m/s^2]   [unit quat, w first]
//
// imu_service.py auto-detects this format ("IMU" tag) — run it with
// --mode stream so it doesn't send poll characters.
//
// VERIFY AFTER FLASHING (bench):
//   * at rest: sqrt(ax^2+ay^2+az^2) ≈ 9.81 and |q| ≈ 1
//   * rotate slowly 90° about one axis: that gyro integrates to ≈ 1.57 rad
// If the numbers are off, fix the two scale constants below (they assume the
// BHI260AP power-on default ranges: accel ±8 g, gyro ±2000 °/s).

#include <Arduino.h>
#include <Arduino_BHY2.h>

SensorXYZ accel(SENSOR_ID_ACC);           // raw int16, default FS ±8 g
SensorXYZ gyro(SENSOR_ID_GYRO);           // raw int16, default FS ±2000 °/s
SensorQuaternion rotation(SENSOR_ID_RV);  // fused rotation vector (unit quat)

// Keep these two in sync with imu_service.py (ACC_LSB_PER_G, GYRO_LSB_PER_DPS).
static const float ACC_LSB_PER_G   = 4096.0f;   // 32768 / 8 g
static const float GYRO_LSB_PER_DPS = 16.384f;  // 32768 / 2000 °/s
static const float GRAVITY = 9.80665f;
static const float DEG2RAD = 0.017453293f;

static const unsigned long STREAM_HZ = 100;
static const unsigned long PERIOD_US = 1000000UL / STREAM_HZ;
static unsigned long next_us = 0;

void setup() {
    Serial.begin(115200);   // USB CDC: baud is ignored, any value works
    delay(1000);

    BHY2.begin();
    accel.begin();
    gyro.begin();
    rotation.begin();

    Serial.println("READY");
    next_us = micros();
}

void loop() {
    BHY2.update();   // must be called frequently (feeds the sensor hub FIFO)

    unsigned long now = micros();
    if ((long)(now - next_us) < 0) {
        return;
    }
    next_us += PERIOD_US;

    float gx = gyro.x() / GYRO_LSB_PER_DPS * DEG2RAD;
    float gy = gyro.y() / GYRO_LSB_PER_DPS * DEG2RAD;
    float gz = gyro.z() / GYRO_LSB_PER_DPS * DEG2RAD;

    float ax = accel.x() / ACC_LSB_PER_G * GRAVITY;
    float ay = accel.y() / ACC_LSB_PER_G * GRAVITY;
    float az = accel.z() / ACC_LSB_PER_G * GRAVITY;

    // Arduino_BHY2 already scales the quaternion to unit floats.
    float qw = rotation.w();
    float qx = rotation.x();
    float qy = rotation.y();
    float qz = rotation.z();

    Serial.print("IMU ");
    Serial.print(gx, 4); Serial.print(" ");
    Serial.print(gy, 4); Serial.print(" ");
    Serial.print(gz, 4); Serial.print(" ");
    Serial.print(ax, 3); Serial.print(" ");
    Serial.print(ay, 3); Serial.print(" ");
    Serial.print(az, 3); Serial.print(" ");
    Serial.print(qw, 4); Serial.print(" ");
    Serial.print(qx, 4); Serial.print(" ");
    Serial.print(qy, 4); Serial.print(" ");
    Serial.println(qz, 4);
}
