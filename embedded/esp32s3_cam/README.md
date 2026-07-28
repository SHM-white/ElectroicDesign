# ESP32S3-Cam I2C reference

This project implements the documented ESP32S3-Cam I2C register interface
without vendor camera or inference code. Both firmware profiles return four
zero bytes until a separately licensed detection producer calls the native
service API.

## Protocol

- Slave address: `0x52`
- SDA: GPIO47
- SCL: GPIO48
- Bus frequency: 100000 Hz
- Receive behavior: the final received byte selects the register
- Response: exactly four unsigned bytes in `center_x`, `center_y`, `width`,
  `length` order

The `color_detection_reference` profile exposes red at register `0x00` and
blue at register `0x01`. The `face_detection_reference` profile exposes face
data at register `0x01`. Initial and no-detection values are `0, 0, 0, 0`.
Profiles are fixed at compile time and cannot be switched at runtime.

## Native tests

From WSL, configure and run the project-focused C++17 tests with:

```bash
cmake -S embedded/esp32s3_cam -B /tmp/ed-esp32s3-cam -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/ed-esp32s3-cam
ctest --test-dir /tmp/ed-esp32s3-cam --output-on-failure
```

The production core and core contract test do not include Arduino or Wire
headers. A separate native adapter contract target compiles the production
adapter against a minimal local ESP32 Wire API stub and verifies `slaveWrite`,
callback registration, hardware constants, and the exact four-byte response.

## Arduino firmware

PlatformIO defines two independent ESP32-S3 Arduino environments:

```bash
pio run -e color_detection_reference
pio run -e face_detection_reference
```

Both environments explicitly compile as GNU++17 with `-Wall`, `-Wextra`,
`-Werror`, and `-pedantic` after removing PlatformIO's GNU++11 default.

PlatformIO is not required for the native tests. Firmware builds were not run
as part of this reference implementation when PlatformIO was unavailable.
