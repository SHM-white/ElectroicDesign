# Embedded projects

This directory contains board-side projects that are separate from the ROS 2
workspace and the host-side `drone/` application.

- `ground_station_esp32s3`: Waveshare ESP32-S3 display-only ground-station
  firmware and its native Lingxiao V7 protocol tests.
- `esp32s3_cam`: original ESP32S3-Cam I2C register-reference firmware based on
  documented hardware interfaces. It does not copy vendor inference code.

Run host-native protocol tests from WSL:

```bash
cmake -S embedded -B /tmp/ed-embedded-native -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/ed-embedded-native
ctest --test-dir /tmp/ed-embedded-native --output-on-failure
```

The ground-station target uses ESP-IDF managed components. The camera reference
retains its own toolchain. Generated build files and managed component files are
ignored.
