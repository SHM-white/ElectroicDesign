# ESP32-S3 地面站

此项目面向 **Waveshare ESP32-S3-Touch-LCD-7 Rev1.2** N16R8 变体：ESP32-S3、
16 MB Flash、8 MB PSRAM、800x480 RGB565 LCD、GT911 电容触摸，以及由 CH422G
控制的板卡功能。

固件是仅显示的地面站，严格包含两个屏幕：
`Overview` 和 `Detail`。触摸只改变当前屏幕。它没有飞行命令传输、地图、图表、
无线电配置或飞行命令控件。`DESIGN.md` 是具有约束力的 UI 和行为契约。

## 运行时行为

可移植的 C++17 核心提供：

- V7 帧格式 `AA ADDR ID LEN DATA SC AC`、校验和验证及流重新同步；
- 来自 V7 `0x08` 的权威有符号小端位置；
- 隔离的 V7 `0x51` 诊断，不能替换或刷新位置；
- 独立的 0.20 s 位置新鲜度和 0.50 s 状态/链路新鲜度；
- 确定性的 `UNKNOWN`、`STALE`、`LOST` 和 `OK` 显示状态。

启动时，`Overview` 在收到经过验证的 V7 帧之前显示未知状态。`DETAIL` 打开诊断屏幕，
`BACK` 返回 `Overview`。两个屏幕都不会向飞行器发送数据。

## 板卡和遥测接线

运行时遥测请将板卡 UART 接口置于 `UART2` 开关位置：

- 板卡 RX `GPIO44` <- 遥测 TX。
- 板卡 TX `GPIO43` -> 遥测 RX，仅在外部设备需要时连接。
- 连接公共 GND。
- 只能使用 3.3 V TTL。
- 配置为 500000 baud、8N1，不使用硬件流控。

板卡的 USB-to-UART Type-C 接口和 UART 接口共用切换后的 UART 路径，不能同时使用：
运行时遥测连接要求接口处于 `UART2` 位置。固件控制台使用独立的原生 USB Serial/JTAG
接口。板卡支持将 USB/CAN 选择器保持在 USB 模式，因此运行此固件时 CAN 不可用。

LCD 占用许多 ESP32-S3 引脚。特别是，`GPIO17` 和 `GPIO18` 是 RGB LCD 数据引脚，
不得重新用于遥测。GT911 和 CH422G 在 `GPIO8`/`GPIO9` 上共用 I2C；GT911 中断为
`GPIO4`，触摸复位通过 CH422G 控制。

此接线描述直接的 UART V7 输入。它不定义无线电频率、信道、空中速率，也不保证与
特定遥测无线电兼容。

## Windows 原生主机验证

在仓库根目录的普通 Windows PowerShell 中运行可移植测试。以下命令使用已安装的
CMake、Ninja 和 LLVM `g++`：

```powershell
$nativeBuild = Join-Path $env:TEMP 'ed-ground-station-native'
cmake -S embedded/ground_station_esp32s3 -B $nativeBuild `
  -G Ninja -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_CXX_COMPILER='C:\Program Files\LLVM\bin\g++.exe'
cmake --build $nativeBuild --parallel
ctest --test-dir $nativeBuild --output-on-failure
```

测试套件覆盖 V7 帧格式/重新同步、`0x08` 位置权威性、隔离的 `0x51` 诊断、新鲜度
边界、板卡常量、固定的 800x480 布局、严格的两个屏幕以及仅显示限制。

## Windows ESP-IDF 安装

从 Windows Package Manager 安装 Espressif Installation Manager CLI：

```powershell
winget install --id Espressif.EIM-CLI --exact --source winget `
  --accept-package-agreements --accept-source-agreements
```

将 Waveshare 记录的 ESP-IDF v5.5.2 版本安装到较短的 ASCII 路径：

```powershell
eim install -i v5.5.2 -p 'D:\Espressif' -t esp32s3 -a true -n true `
  --esp-idf-json-path 'D:\Espressif\tools' `
  --activation-script-path-override 'D:\Espressif\tools'
```

如果 GitHub 访问失败，请使用 Espressif 官方中国镜像和可访问的 Python 软件包镜像
重试：

```powershell
eim remove v5.5.2
eim install -i v5.5.2 -p 'D:\Espressif' -t esp32s3 -a true -n true `
  --idf-mirror 'https://git.espressif.com.cn' `
  --mirror 'https://dl.espressif.cn/github_assets' `
  --pypi-mirror 'https://pypi.mirrors.ustc.edu.cn/simple' `
  --esp-idf-json-path 'D:\Espressif\tools' `
  --activation-script-path-override 'D:\Espressif\tools'
```

每个新的 PowerShell 会话都要激活并验证环境：

```powershell
. 'D:\Espressif\tools\Microsoft.v5.5.2.PowerShell_profile.ps1'
where.exe idf.py
$env:IDF_PATH
idf.py --version
python --version
```

经过验证的设置应使用 ESP-IDF v5.5.2 和 Python 3.11。项目清单通过 IDF Component
Manager 解析 `esp_lvgl_port 2.8.0~1`、LVGL 8.4.0 和 `esp_lcd_touch_gt911 1.1.1~1`。

## 构建和产物

在仓库根目录已激活 ESP-IDF 的 PowerShell 中：

```powershell
idf.py -C embedded/ground_station_esp32s3/firmware set-target esp32s3
idf.py -C embedded/ground_station_esp32s3/firmware build
```

成功构建会生成：

- `embedded/ground_station_esp32s3/firmware/build/ed_ground_station_waveshare.bin`
- `embedded/ground_station_esp32s3/firmware/build/bootloader/bootloader.bin`
- `embedded/ground_station_esp32s3/firmware/build/partition_table/partition-table.bin`

经过验证的应用镜像约为 661 kB，适合默认的 1 MB 应用分区，剩余空间约 37%。由于
二进制文件会随源代码和工具链版本变化，依赖这些数值前请先在本地重新构建。

## COM 端口、烧录和恢复

连接板卡前列出 Windows 串行端口，连接后再次列出。只能使用新出现的 Waveshare 端口：

```powershell
Get-CimInstance Win32_SerialPort |
  Select-Object DeviceID, Name, PNPDeviceID
```

不要选择 Bluetooth 串行端口。执行此验证的机器只有 Bluetooth `COM4`、`COM5`、
`COM7` 和 `COM8`，因此没有执行烧录、监视或擦除操作。

确认板卡端口不是 Bluetooth 后，将下面的 `COM12` 替换为该设备，并通过板卡下载连接进行烧录：

```powershell
idf.py -C embedded/ground_station_esp32s3/firmware -p COM12 flash
```

板卡带有自动下载电路。如果 Windows 无法检测下载端口，请按住 `BOOT`，重新连接 USB，
端口出现后松开 `BOOT`。下载后按 `RESET`。使用运行时遥测时，将 UART 选择器恢复到
`UART2`。只有单独确认原生 USB Serial/JTAG 控制台端口后，才能使用
`idf.py -p <verified-native-USB-COM> monitor`。

擦除 Flash 不属于项目正常工作流。只有在已确认的板卡确实需要时，才使用厂商的恢复说明。

## Waveshare 官方参考

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

## 验证限制

Windows 原生主机测试和 Windows ESP-IDF v5.5.2 目标构建已经运行。在连接已确认的非
Bluetooth 板卡端口之前，物理 LCD 输出、GT911 精度、触摸导航、阳光下可读性、UART
电气行为和烧录仍属于硬件验证项目。

## Todo 6 Arduino HMI

Arduino 状态机库位于 `arduino_hmi/`，干净草图位于
`arduino_sketch/ground_station_esp32s3.ino`。它定义严格的
`BOOT_LOCKED -> PRESTART -> SELECT_PENDING -> SELECTED/ARMED_READY ->
CAR_RUNNING/FAULT` 流程。选择在收到
权威 ACK 之前不会显示为已提交；小车启动后地面站变为只读。视图模型保留 AP、小车、
ROS 和视觉的新鲜度时长。重启和失去权威会使 HMI 返回锁定状态。

显示和触摸是明确的 `DisplayPort` 和 `TouchPort` 接口。草图使用仅串行的未接线显示，
不猜测面板控制器或触摸引脚。确定性的浏览器参考是
[`arduino_preview/index.html`](arduino_preview/index.html); it mirrors the
800x480 几何布局，并包含用于受限 CJK 换行检查的双语标签。

此同级项目的 Arduino CLI 固定项为 `esp32:esp32@3.2.0`、FQBN
`esp32:esp32:esp32s3`、内置 `WiFi`/`WiFiUDP`，以及本地
`EDSharedProtocol@1.0.0` plus `EDGroundStationHmi@1.0.0` libraries:

```text
arduino-cli compile --fqbn esp32:esp32:esp32s3 --libraries embedded/shared_protocol --libraries embedded/ground_station_esp32s3/arduino_hmi embedded/ground_station_esp32s3/arduino_sketch
```
