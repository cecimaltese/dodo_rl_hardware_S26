# IMU firmware (Arduino Nicla Sense ME)

The IMU is an Arduino Nicla Sense ME (Bosch BHI260AP) on USB serial. The host side is
`imu_service.py`, with the wire format in `imu_protocol.py`. Both firmwares are
auto-detected, so flashing never strands the host config.

| | `imu_firmware_v2/` (flashed) | `imu_firmware_v1/` (rollback) |
|---|---|---|
| Baud | 921600 | 115200 |
| `'L'` reply | `ALL ax ay az gx gy gz pitch roll qw qx qy qz` | `ALL ax ay az gx gy gz pitch roll` |
| Sensor ranges | set explicitly (±4 g, ±1000 °/s) and reported over `'V'` | chip defaults, never reported, so the host guesses |
| Orientation | on-chip accel+gyro fusion quaternion (`SENSOR_ID_GAMERV`, no magnetometer) | none; the host falls back to accelerometer tilt |

In both, accel and gyro are raw int16 counts that the host scales by the sensor range.
v1 is Jim's original firmware, kept verbatim.

## Flash

Needs `arduino-cli` with the `arduino:mbed_nicla` core. From the repo root:

```bash
arduino-cli compile --upload -p /dev/ttyACM0 --fqbn arduino:mbed_nicla:nicla_sense firmware/imu_firmware_v2
# rollback to v1:
arduino-cli compile --upload -p /dev/ttyACM0 --fqbn arduino:mbed_nicla:nicla_sense firmware/imu_firmware_v1
```

## Verify

```bash
python3 imu_check.py          # acceptance test: format, scale (|accel| = 1 g), gyro bias, rate
python3 imu_sniff.py          # raw lines + independent range cross-check
python3 imu_sniff.py --axes   # once mounted: prints AXIS_REMAP for imu_protocol.py
```
