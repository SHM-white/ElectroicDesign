# ESP32-S3 小车 Arduino 项目

此 Arduino CLI/IDE 项目包含可移植的小车控制器核心，以及位于
`arduino_sketch/car_esp32s3.ino` 的干净机器草图。

控制器不阻塞：在 1.0 s Wi-Fi 宽限期内，本地循线和编码器积分独立于 UDP 继续运行。
存在已配置的接收端时，以 20 Hz 发布遥测。仅按顺序接受 `START`、`B`、`D`、`A`
和 `COMPLETE`。循线丢失、编码器不一致、PID 超时、欠压、马达故障、按钮卡住，或
Wi-Fi 超过宽限期丢失，都会制动并锁存 `SAFE_STOP`；只有 `physical_reset()` 才能
允许新的一轮运行。

`LineSensors`、`MotorDriver`、`Encoders`、`StartButton` 和 `HealthMonitor` 是明确的
端口。提供真实板卡适配器之前，草图使用未接线实现。不猜测马达引脚、编码器引脚、
显示屏假设或凭据。仅在本地配置工作区中将 `config_local.h.example` 复制为会被忽略
的 `config_local.h`。

## 固定版本的 Arduino 工作流

- 板卡包：`esp32:esp32@3.2.0`。
- FQBN：`esp32:esp32:esp32s3`（仅替换为已验证的板卡变体）。
- 库：内置 `WiFi` 和 `WiFiUDP`；`EDSharedProtocol@1.0.0` 和
  `EDCarController@1.0.0` 是本仓库中的本地库。

安装 Arduino CLI 和固定版本核心后：

```text
arduino-cli compile --fqbn esp32:esp32:esp32s3 --libraries embedded/shared_protocol --libraries embedded/car_esp32s3 embedded/car_esp32s3/arduino_sketch
```

干净构建刻意不包含可用的网络配置。在提供本地 WPA2/MAC-DHCP 配置和具体驱动适配器
之前，硬件构建无法进行。
