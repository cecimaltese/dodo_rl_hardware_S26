#include <Arduino.h>
#include <Arduino_BHY2.h>

// This is the arduino code fror the NICLA with no camera (NICLA sense me)

SensorXYZ accel(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);

void setup() {
    Serial.begin(115200);
    delay(1000);

    BHY2.begin();
    accel.begin();
    gyro.begin();

    Serial.println("READY");
}

void loop() {
    BHY2.update();   // must be called frequently

    if (Serial.available()) {
        char c = Serial.read();

        float ax = accel.x();
        float ay = accel.y();
        float az = accel.z();

        float gx = gyro.x();
        float gy = gyro.y();
        float gz = gyro.z();

        switch (c) {

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

            case 'O': { // ORIENT (pitch, roll)
                float pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958;
                float roll  = atan2(ay, az) * 57.2958;

                Serial.print("ORIENT ");
                Serial.print(pitch, 2); Serial.print(" ");
                Serial.println(roll, 2);
                break;
            }

            case 'L': { // ALL
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
                Serial.println(roll, 2);
                break;
            }

            default:
                break;
        }
    }
}
