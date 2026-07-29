# ESP32-S3 Ground Station

This project targets the **Waveshare ESP32-S3-Touch-LCD-7 Rev1.2** N16R8
variant: ESP32-S3, 16 MB Flash, 8 MB PSRAM, an 800x480 RGB565 LCD, GT911
capacitive touch, and CH422G-controlled board functions.

The firmware is a display-only ground station with exactly two screens,
`Overview` and `Detail`. Touch only changes the current screen. It has no
flight-command transport, maps, charts, radio configuration, or flight-command
controls. `DESIGN.md` is the binding UI and behavior contract.

## Runtime behavior

The portable C++17 core provides:

- V7 framing `AA ADDR ID LEN DATA SC AC`, checksum verification, and stream
  resynchronization;
- authoritative signed little-endian position from V7 `0x08`;
- isolated V7 `0x51` diagnostics that cannot replace or refresh position;
- independent 0.20 s position freshness and 0.50 s status/link freshness;
- deterministic `UNKNOWN`, `STALE`, `LOST`, and `OK` display states.

At startup, `Overview` shows unknown states until verified V7 frames arrive.
`DETAIL` opens the diagnostic screen and `BACK` returns to `Overview`. Neither
screen sends data to the aircraft.

## Board and telemetry wiring

Use the board UART header in the `UART2` switch position for runtime telemetry:

- Board RX `GPIO44` <- telemetry TX.
- Board TX `GPIO43` -> telemetry RX, only if the external device needs it.
- Connect common GND.
- Use 3.3 V TTL only.
- Configure 500000 baud, 8N1, without hardware flow control.

The board's USB-to-UART Type-C connector and UART header share the switched UART
path. They cannot be used simultaneously: the runtime telemetry connection
requires the `UART2` header position. The firmware console uses the separate
native USB Serial/JTAG interface. The board support leaves the USB/CAN selector
in USB mode, so CAN is unavailable while this firmware is running.

The LCD consumes many ESP32-S3 pins. In particular, `GPIO17` and `GPIO18` are
RGB LCD data pins and must not be reused for telemetry. GT911 and CH422G share
I2C on `GPIO8`/`GPIO9`; GT911 interrupt is `GPIO4`, and touch reset is controlled
through CH422G.

This wiring describes direct UART V7 input. It does not define a radio
frequency, channel, air rate, or compatibility with a specific telemetry radio.

## Windows-native host validation

Run the portable tests from a normal Windows PowerShell in the repository root.
The commands below use the installed CMake, Ninja, and LLVM `g++`:

```powershell
$nativeBuild = Join-Path $env:TEMP 'ed-ground-station-native'
cmake -S embedded/ground_station_esp32s3 -B $nativeBuild `
  -G Ninja -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_CXX_COMPILER='C:\Program Files\LLVM\bin\g++.exe'
cmake --build $nativeBuild --parallel
ctest --test-dir $nativeBuild --output-on-failure
```

The suite covers V7 framing/resynchronization, `0x08` position authority,
isolated `0x51` diagnostics, freshness boundaries, board constants, fixed
800x480 layout, exactly two screens, and display-only restrictions.

## Windows ESP-IDF installation

Install Espressif Installation Manager CLI from Windows Package Manager:

```powershell
winget install --id Espressif.EIM-CLI --exact --source winget `
  --accept-package-agreements --accept-source-agreements
```

Install the Waveshare-documented ESP-IDF v5.5.2 release in a short ASCII path:

```powershell
eim install -i v5.5.2 -p 'D:\Espressif' -t esp32s3 -a true -n true `
  --esp-idf-json-path 'D:\Espressif\tools' `
  --activation-script-path-override 'D:\Espressif\tools'
```

If GitHub access fails, retry with Espressif's official China mirrors and a
reachable Python package mirror:

```powershell
eim remove v5.5.2
eim install -i v5.5.2 -p 'D:\Espressif' -t esp32s3 -a true -n true `
  --idf-mirror 'https://git.espressif.com.cn' `
  --mirror 'https://dl.espressif.cn/github_assets' `
  --pypi-mirror 'https://pypi.mirrors.ustc.edu.cn/simple' `
  --esp-idf-json-path 'D:\Espressif\tools' `
  --activation-script-path-override 'D:\Espressif\tools'
```

Activate and verify the environment in every new PowerShell session:

```powershell
. 'D:\Espressif\tools\Microsoft.v5.5.2.PowerShell_profile.ps1'
where.exe idf.py
$env:IDF_PATH
idf.py --version
python --version
```

Expected versions for the validated setup are ESP-IDF v5.5.2 and Python 3.11.
The project manifest resolves `esp_lvgl_port 2.8.0~1`, LVGL 8.4.0, and
`esp_lcd_touch_gt911 1.1.1~1` through the IDF Component Manager.

## Build and artifacts

From an activated ESP-IDF PowerShell in the repository root:

```powershell
idf.py -C embedded/ground_station_esp32s3/firmware set-target esp32s3
idf.py -C embedded/ground_station_esp32s3/firmware build
```

A successful build produces:

- `embedded/ground_station_esp32s3/firmware/build/ed_ground_station_waveshare.bin`
- `embedded/ground_station_esp32s3/firmware/build/bootloader/bootloader.bin`
- `embedded/ground_station_esp32s3/firmware/build/partition_table/partition-table.bin`

The validated application image is approximately 661 kB and fits the default
1 MB application partition with approximately 37% free space. Rebuild locally
before relying on these figures because the binary changes with source and
toolchain revisions.

## COM port, flashing, and recovery

List Windows serial ports before connecting the board, then repeat after
connecting it. Use the newly appearing Waveshare port only:

```powershell
Get-CimInstance Win32_SerialPort |
  Select-Object DeviceID, Name, PNPDeviceID
```

Do not select a Bluetooth serial port. The machine used for this validation had
only Bluetooth `COM4`, `COM5`, `COM7`, and `COM8`, so no flash, monitor, or erase
operation was performed.

After verifying a non-Bluetooth board port, replace `COM12` below with that
device and flash through the board's download connection:

```powershell
idf.py -C embedded/ground_station_esp32s3/firmware -p COM12 flash
```

The board has an automatic download circuit. If Windows cannot detect the
download port, hold `BOOT`, reconnect USB, and release `BOOT` after the port
appears. Press `RESET` after downloading. For runtime telemetry, return the UART
selector to `UART2`. Use `idf.py -p <verified-native-USB-COM> monitor` only after
separately identifying the native USB Serial/JTAG console port.

Erasing Flash is not part of the normal project workflow. Use the vendor's
recovery instructions only when a verified board requires it.

## Official Waveshare references

- Board overview, specifications, pin mapping, and interface sharing:
  https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/
- Windows ESP-IDF setup and official board examples:
  https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/Development-Environment-Setup-ESP-IDF
- Usage notes, BOOT recovery, CH422G/I2C restrictions, and LVGL guidance:
  https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/Instructions-For-Use
- Vendor binary flashing and recovery procedure:
  https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/Firmware-Flashing
- Schematic, datasheets, board drawings, and demo archive:
  https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/Resources-And-Documents

## Validation limits

Windows-native host tests and the Windows ESP-IDF v5.5.2 target build have been
run. Physical LCD output, GT911 accuracy, touch navigation, sunlight
readability, UART electrical behavior, and flashing remain hardware validation
items until a verified non-Bluetooth board port is connected.
