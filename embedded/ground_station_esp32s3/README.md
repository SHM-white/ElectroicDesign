# ESP32-S3 Ground Station

This project targets the **Waveshare ESP32-S3-Touch-LCD-7 Rev1.2**, with an
ESP32-S3 N16R8, an 800x480 RGB565 display, and GT911 touch input. The firmware
is a display-only ground station with two screens, `Overview` and `Detail`.
Touch only changes the current screen. There is no flight-command transport and
there are no flight-command controls.

## Native core

The C++17 core is independent of the legacy display runtime. It retains:

- native V7 framing `AA ADDR ID LEN DATA SC AC`, checksum verification, and
  incremental resynchronization;
- authoritative signed little-endian centimeter position from V7 `0x08`;
- isolated V7 `0x51` diagnostics that cannot update position value, sequence, or
  freshness;
- independent 0.20 s position freshness and 0.50 s status and link freshness;
- generic navigation, view-model, and UI-layout tests.

Build and run the focused native tests from WSL:

```bash
cmake -S embedded/ground_station_esp32s3 -B /tmp/ed-ground-station-native \
  -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/ed-ground-station-native
ctest --test-dir /tmp/ed-ground-station-native --output-on-failure
```

The same target is exposed by `embedded/CMakeLists.txt` through
`add_subdirectory(ground_station_esp32s3)`.

## Board wiring

Use the board UART2 header at runtime:

- Board RX GPIO44 <- telemetry TX.
- Board TX GPIO43 -> telemetry RX, if needed.
- Common GND.
- 3.3V TTL only.
- 500000 baud, 8N1.
- Set the board switch to the UART2 position.

UART1 and CH343 cannot be used simultaneously with this UART2 connection. The
native USB Serial/JTAG interface is selected for the console. CAN is unavailable
in this firmware mode.

This documents direct 3.3V TTL UART V7 input only. It does not claim
compatibility with any radio module or specify a radio frequency, channel, or
air rate.

## ESP-IDF build

ESP-IDF manages the LVGL, display-port, and GT911 component dependencies from
`firmware/main/idf_component.yml`. From the repository root:

```bash
idf.py -C embedded/ground_station_esp32s3/firmware set-target esp32s3
idf.py -C embedded/ground_station_esp32s3/firmware build
```

`DESIGN.md` is the UI contract for the two screens, their layout, states, and
touch behavior.

## Validation limits

Native tests and static firmware checks run in this environment. No physical
board display, touch, sunlight, or electrical validation was performed here.
