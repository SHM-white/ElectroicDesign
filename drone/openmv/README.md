# OpenMV 视觉后端

OpenMV 完成绿色占比和区块编号识别，上位机只接收识别结果，不传图像。

## 部署

1. 使用 OpenMV IDE 连接设备，把 [main.py](main.py) 复制到 OpenMV 根目录。
2. 在 OpenMV 文件系统创建 `/templates`，放入 `1.pgm` 到 `28.pgm`。模板应在实际飞行高度、镜头焦距和现场光照下截取，只保留区块数字及少量边缘背景。
3. 在 OpenMV IDE 的阈值工具中标定 `GREEN_LAB_THRESHOLD`，再调整 `TEMPLATE_THRESHOLD`。模板误识别时提高阈值，漏识别时降低阈值。
4. 默认使用 `UART(3)`、115200 baud。按具体 OpenMV 型号确认 UART TX 引脚，将 OpenMV TX 接到上位机 USB-TTL RX，并连接 GND。结果为单向传输，不需要连接上位机 TX。

OpenMV 与 USB-TTL 使用 3.3V 逻辑电平。不要把 5V TTL 信号直接接入 OpenMV IO。

## 上位机启动

```bash
python3 -m drone.main \
  --vision-backend openmv \
  --openmv-port /dev/ttyUSB1 \
  --openmv-baudrate 115200
```

工业相机方案仍是默认值，原命令无需改动：

```bash
python3 -m drone.main --vision-backend industrial
```

如果同时连接飞控、H7 GPIO 板和 OpenMV，三者必须使用不同串口。建议用 udev 规则建立稳定设备名，避免 `/dev/ttyUSB0`、`/dev/ttyUSB1` 的枚举顺序变化。

## 串口协议

每行一个 ASCII 帧：

```text
$OMV1,<sequence>,<green_per_mille>,<digit>*<xor>\r\n
```

- `sequence`: `0..65535`，循环递增。
- `green_per_mille`: 绿色像素占比的千分数，`0..1000`。
- `digit`: 区块编号 `1..28`，未识别时为 `-1`。
- `xor`: 对 `$` 与 `*` 之间所有 ASCII 字节进行 XOR，使用两位十六进制表示。

上位机会拒绝校验错误和越界数据，并在 0.5 秒内复用最近一次有效结果；超过该时间后结果失效。
