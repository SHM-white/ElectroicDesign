# 嵌入式项目

此目录包含板端项目，与 ROS 2 工作区和主机端 `drone/` 应用相互独立。

- `ground_station_esp32s3`：Waveshare ESP32-S3 仅显示地面站固件及其原生
  Lingxiao V7 协议测试。
- `esp32s3_cam`：基于已记录硬件接口的原始 ESP32S3-Cam I2C 寄存器参考固件，
  不复制厂商推理代码。

在 WSL 中运行主机原生协议测试：

```bash
cmake -S embedded -B /tmp/ed-embedded-native -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/ed-embedded-native
ctest --test-dir /tmp/ed-embedded-native --output-on-failure
```

地面站目标使用 ESP-IDF 托管组件。摄像头参考项目保留自己的工具链。生成的
构建文件和托管组件文件会被忽略。
