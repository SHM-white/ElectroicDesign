# ESP32S3-Cam I2C 参考

此项目实现已记录的 ESP32S3-Cam I2C 寄存器接口，不包含厂商摄像头或推理代码。在
获得单独许可的检测程序调用原生服务 API 之前，两个固件配置都返回四个零字节。

## 协议

- 从机地址：`0x52`
- SDA：GPIO47
- SCL：GPIO48
- 总线频率：100000 Hz
- 接收行为：最后接收的字节选择寄存器
- 响应：严格按 `center_x`、`center_y`、`width`、`length` 顺序返回四个无符号字节

`color_detection_reference` 配置在寄存器 `0x00` 暴露红色数据，在寄存器 `0x01`
暴露蓝色数据。`face_detection_reference` 配置在寄存器 `0x01` 暴露人脸数据。初始
值和未检测到目标时的值为 `0, 0, 0, 0`。配置在编译时固定，不能在运行时切换。

## 原生测试

在 WSL 中使用以下命令配置并运行项目专用的 C++17 测试：

```bash
cmake -S embedded/esp32s3_cam -B /tmp/ed-esp32s3-cam -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/ed-esp32s3-cam
ctest --test-dir /tmp/ed-esp32s3-cam --output-on-failure
```

生产核心和核心契约测试不包含 Arduino 或 Wire 头文件。单独的原生适配器契约目标会
使用本地最小 ESP32 Wire API 存根编译生产适配器，并验证 `slaveWrite`、回调注册、
硬件常量以及严格的四字节响应。

## Arduino 固件

PlatformIO 定义两个相互独立的 ESP32-S3 Arduino 环境：

```bash
pio run -e color_detection_reference
pio run -e face_detection_reference
```

移除 PlatformIO 默认的 GNU++11 后，两个环境都明确使用 GNU++17，并启用 `-Wall`、
`-Wextra`、`-Werror` 和 `-pedantic` 编译。

原生测试不需要 PlatformIO。PlatformIO 不可用时，本参考实现不会运行固件构建。
