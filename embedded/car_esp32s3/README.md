# ESP32-S3 Car Arduino Project

This Arduino CLI/IDE project contains the portable car controller core and a
clean-machine sketch at `arduino_sketch/car_esp32s3.ino`.

The controller is nonblocking: local line following and encoder integration
continue independently of UDP during a 1.0 s Wi-Fi grace period. Telemetry is
published at 20 Hz when a configured sink is present. `START`, `B`, `D`, `A`,
and `COMPLETE` are accepted only in order. Missed line, encoder disagreement,
PID overrun, brownout, motor fault, stuck button, or Wi-Fi loss beyond the
grace period brakes and latches `SAFE_STOP`; only `physical_reset()` permits a
new run.

`LineSensors`, `MotorDriver`, `Encoders`, `StartButton`, and `HealthMonitor`
are explicit ports. The sketch uses unwired implementations until a real board
adapter is supplied. No motor pins, encoder pins, display assumptions, or
credentials are guessed. Copy `config_local.h.example` to the ignored
`config_local.h` only in a local provisioning workspace.

## Pinned Arduino workflow

- Board package: `esp32:esp32@3.2.0`.
- FQBN: `esp32:esp32:esp32s3` (replace only with the verified board variant).
- Libraries: built-in `WiFi` and `WiFiUDP`; `EDSharedProtocol@1.0.0` and
  `EDCarController@1.0.0` are local libraries in this repository.

After installing Arduino CLI and the pinned core:

```text
arduino-cli compile --fqbn esp32:esp32:esp32s3 --libraries embedded/shared_protocol --libraries embedded/car_esp32s3 embedded/car_esp32s3/arduino_sketch
```

The clean build intentionally contains no usable network configuration. A
hardware build is blocked until local WPA2/MAC-DHCP provisioning and concrete
driver adapters are supplied.
