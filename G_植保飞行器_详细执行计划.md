# G_植保飞行器 — 详细执行计划

> 2021年全国大学生电子设计竞赛 G题 | 凌霄飞控 + 树莓派4B + 海康工业相机

---

## 目录

1. [系统架构](#1-系统架构)
2. [硬件清单](#2-硬件清单)
3. [树莓派与MCU串口通信协议](#3-树莓派与mcu串口通信协议)
4. [凌霄IMU API指令速查](#4-凌霄imu-api指令速查)
5. [树莓派环境搭建](#5-树莓派环境搭建)
6. [MCU固件开发](#6-mcu固件开发)
7. [视觉识别开发](#7-视觉识别开发)
8. [三层定位融合算法](#8-三层定位融合算法)
9. [全覆盖路径规划](#9-全覆盖路径规划)
10. [状态机设计](#10-状态机设计)
11. [激光笔与LED控制](#11-激光笔与led控制)
12. [调试步骤（分速度档位）](#12-调试步骤)
13. [异常处理策略](#13-异常处理策略)
14. [比赛测试流程](#14-比赛测试流程)
15. [风险清单与对策](#15-风险清单与对策)
16. [附录：文件路径索引](#16-附录文件路径索引)

---

## 1. 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      无人机机载系统                           │
│                                                              │
│  ┌──────────────────────┐         ┌──────────────────────┐   │
│  │  凌霄 STM32F4 底板     │  UART   │    树莓派4B 1GB       │   │
│  │                      │←───────→│                      │   │
│  │  ├─ UART1 → 凌霄IMU   │GPIO14/15│  ├─ USB3.0 → 海康相机  │   │
│  │  ├─ UART4 ← 光流模块   │ 115200  │  ├─ GPIO → 激光笔     │   │
│  │  ├─ UART1 ← SBUS接收机│         │  ├─ GPIO → LED指示灯  │   │
│  │  ├─ PWM1-4 → 电调×4   │         │  └─ 主程序            │   │
│  │  ├─ ADC → 电池监测     │         │     ├─ 图像采集(30fps) │   │
│  │  └─ GPIO → (备用IO)    │         │     ├─ 颜色识别        │   │
│  │                      │         │     ├─ 区块检测        │   │
│  └──────────────────────┘         │     ├─ 数字OCR         │   │
│           │  │                    │     ├─ 定位融合        │   │
│           │  │                    │     ├─ 路径规划        │   │
│     ┌─────┘  └─────┐              │     └─ 指令发送        │   │
│     │              │              └──────────────────────┘   │
│  ┌──┴──┐      ┌───┴───┐                                      │
│  │光流  │      │激光测距│                                      │
│  │模块  │      │模块    │                                      │
│  └─────┘      └───────┘                                      │
└─────────────────────────────────────────────────────────────┘
```

**核心设计**：

- **树莓派 = 主控大脑**：图像采集 → 颜色/区块/数字识别 → 定位融合 → 路径规划 → 发送API指令
- **STM32F4 MCU = 通信中继 + 物理执行**：接收树莓派指令 → 转发至凌霄IMU；采集光流数据 → 回传树莓派；输出PWM至电调
- 树莓派与MCU之间通过 **UART (GPIO14=TX, GPIO15=RX) 115200bps** 通信

---

## 2. 硬件清单


| 序号 | 模块          | 规格/型号                       | 数量 | 用途                 | 接口             |
| ------ | --------------- | --------------------------------- | ------ | ---------------------- | ------------------ |
| 1    | 飞控底板      | 凌霄 STM32F407 底板             | 1    | MCU主控，通信中继    | -                |
| 2    | IMU模块       | 凌霄 IMU (BMI088+IST8310+SPL06) | 1    | 姿态解算、PID控制    | UART1            |
| 3    | 机架          | 333mm轴距四旋翼                 | 1    | 载机平台             | -                |
| 4    | 电机+电调     | 配套无刷电机+电调               | 4    | 动力                 | PWM1-4           |
| 5    | 螺旋桨        | 配套桨叶                        | 4对  | 升力                 | -                |
| 6    | 桨叶防护罩    | 全防护型                        | 1套  | 安全（赛题强制要求） | -                |
| 7    | 电池          | 3S/4S LiPo ≥2200mAh            | 2块  | 供电（备一块换电）   | BAT端子          |
| 8    | 光流模块      | 匿名光流                        | 1    | 水平位置/速度估计    | UART4, 500000bps |
| 9    | 激光测距      | VL53L0X / TFmini                | 1    | 精准定高             | 与光流配套       |
| 10   | 遥控器+接收机 | FS-i6S + SBUS接收机             | 1    | 手动飞行/紧急接管    | SBUS             |
| 11   | 树莓派        | RPi 4B 1GB                      | 1    | 图像处理+主控逻辑    | GPIO UART        |
| 12   | 工业相机      | 海康 USB3.0 工业相机            | 1    | 下视图像采集         | USB3.0           |
| 13   | 激光笔        | 3.3V/5V 红色激光模组            | 1    | 模拟撒药             | GPIO             |
| 14   | LED灯条       | 高亮LED                         | 1    | 显示条码数字（发挥） | GPIO             |
| 15   | 电源模块      | 5V BEC / 降压模块               | 1    | 给树莓派+相机供电    | -                |
| 16   | USB转TTL      | CP2102/CH340                    | 1    | 树莓派↔MCU 串口调试 | USB              |

---

## 3. 树莓派与MCU串口通信协议

### 3.1 物理连接

```
树莓派 GPIO14 (TXD)  ────  MCU UARTx (RX)
树莓派 GPIO15 (RXD)  ────  MCU UARTx (TX)
树莓派 GND           ────  MCU GND
```

> 注意：树莓派GPIO为3.3V电平，确认MCU串口也是3.3V。若MCU串口为5V，需加电平转换。

### 3.2 数据帧格式（树莓派 → MCU）

两类帧：

**A. 飞控指令转发帧（树莓派→MCU→IMU）**

```
帧结构（变长）：
┌──────┬────────┬──────┬────────┬───────┬──────┐
│ 0xAA │ CMD_LEN│ TYPE │ PAYLOAD│ SUM_LO│SUM_HI│
│ 帧头  │ 载荷长度│ 0x01  │(变长)  │  双字节校验和  │
│ 1B    │ 1B     │ 1B   │ nB     │     2B       │
└──────┴────────┴──────┴────────┴───────┴──────┘

TYPE=0x01 表示"转发至IMU的API指令帧"
PAYLOAD: 完整的凌霄API帧（不含外围校验，MCU会加上SC/AC）
CMD_LEN: PAYLOAD字节数（最大250）
SUM = (所有字节之和) & 0xFFFF
```

**B. 光流/状态查询帧（树莓派→MCU）**

```
帧结构（固定2字节）：
┌──────┬──────┐
│ 0xBB │ CMD  │
│ 帧头  │ 命令  │
└──────┴──────┘

CMD:
  0x01 = 请求光流位置（MCU回传0xCC 0x01 + POS_X(4B) + POS_Y(4B)）
  0x02 = 请求飞行状态（MCU回传0xCC 0x02 + MODE(1B) + LOCKED(1B) + ALT(4B)）
  0x03 = 重置光流积分零点（起飞点归零）
```

### 3.3 数据帧格式（MCU → 树莓派）

**光流位置回传帧**

```
0xCC 0x01 [POS_X_Lo..Hi 4B s32 cm] [POS_Y_Lo..Hi 4B s32 cm] [QUALITY 1B]
```

**飞行状态回传帧**

```
0xCC 0x02 [MODE 1B] [LOCKED 1B] [ALT_Lo..Hi 4B s32 cm]
  MODE:  1=自稳+定高, 2=定点, 3=程控
  LOCKED: 0=已解锁, 1=已加锁
  ALT: 当前高度(cm)
```

---

## 4. 凌霄IMU API指令速查

> **来源**：匿名通信协议V7 + 源码验证。**注意**：源码中控制类指令使用 **CID=0x10**，非协议文档中的CID=0x00。

### 4.1 帧格式

```
AA [D_ADDR] [ID] [LEN] [DATA...] [SC] [AC]
│   │        │    │     │          │    └── 累加校验
│   │        │    │     │          └── 和校验
│   │        │    │     └── 载荷数据(LEN字节)
│   │        │    └── 载荷长度
│   │        └── 帧ID (0xE0=命令帧)
│   └── 目标地址 (0xFF=广播)
└── 帧头 (固定)
```

总帧长 = 6 + LEN 字节

### 4.2 校验和计算

```c
// 伪代码
u8 sumcheck = 0, addcheck = 0;
for (i = 0; i < data_len - 2; i++) {
    sumcheck += data_buf[i];
    addcheck += sumcheck;
}
// data_buf末尾两字节应为 sumcheck 和 addcheck
```

### 4.3 常用命令

#### 模式切换

```
AA FF E0 0B  01 01 01 [MODE] 00 00 00 00 00 00 00  SC AC
                      ^^^^^^
MODE: 0=自稳, 1=自稳+定高, 2=定点(GPS/光流), 3=程控模式
```

#### 解锁 / 加锁

```
解锁: AA FF E0 0B  10 00 01 00 00 00 00 00 00 00 00  SC AC
加锁: AA FF E0 0B  10 00 02 00 00 00 00 00 00 00 00  SC AC
       ^^ ^^ ^^
       CID=0x10 (控制类), CMD0=0x00, CMD1=子命令
```

#### 一键起飞

```
AA FF E0 0B  10 00 05 [H_LO] [H_HI] 00 00 00 00 00 00  SC AC
                      ^^^^^^^^^^^^^^
高度(u16小端, cm): 0=使用默认值(约150cm), 非0则按指定值
```

#### 一键降落

```
AA FF E0 0B  10 00 06 00 00 00 00 00 00 00 00  SC AC
```

#### 水平移动（最关键指令）

```
AA FF E0 0B  10 02 03 [D_LO] [D_HI] [S_LO] [S_HI] [A_LO] [A_HI] 00 00  SC AC
      ^^ ^^ ^^ ^^
      CID C0 C1 C2
          D: 距离(u16小端, cm) 范围 0~10000
          S: 速度(u16小端, cm/s) 范围 10~300
          A: 方向(u16小端, 0~359度)  0=机头方向, 顺时针增加
```

#### 高度升降

```
上升: AA FF E0 0B  10 02 01 [H_LO] [H_HI] [S_LO] [S_HI] 00 00 00 00  SC AC
下降: AA FF E0 0B  10 02 02 [H_LO] [H_HI] [S_LO] [S_HI] 00 00 00 00  SC AC
      H: 升降高度(u16小端, cm)  S: 速度(u16小端, cm/s)
```

### 4.4 常用命令 Python 构建函数

```python
import struct

def build_lx_frame(d_addr, frame_id, data: bytes) -> bytes:
    """构建凌霄API帧，自动计算校验"""
    buf = bytes([0xAA, d_addr, frame_id])
    buf += bytes([len(data)])  # LEN
    buf += data
    # 计算 SC(和校验) 和 AC(累加校验)
    sumcheck = 0
    addcheck = 0
    for b in buf:
        sumcheck = (sumcheck + b) & 0xFF
        addcheck = (addcheck + sumcheck) & 0xFF
    buf += bytes([sumcheck, addcheck])
    return buf

def cmd_unlock() -> bytes:
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))

def cmd_lock() -> bytes:
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))

def cmd_takeoff(height_cm: int = 0) -> bytes:
    h = struct.pack('<H', height_cm)
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x00, 0x05]) + h + bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))

def cmd_land() -> bytes:
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))

def cmd_mode(mode: int) -> bytes:
    # mode: 0=自稳, 1=自稳+定高, 2=定点, 3=程控
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x01, 0x01, 0x01, mode]) + bytes([0x00]*7))

def cmd_move(distance_cm: int, speed_cmps: int, direction_deg: int) -> bytes:
    d = struct.pack('<H', distance_cm)
    s = struct.pack('<H', speed_cmps)
    a = struct.pack('<H', direction_deg)
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x02, 0x03]) + d + s + a + bytes([0x00, 0x00]))

def cmd_ascend(height_cm: int, speed_cmps: int) -> bytes:
    h = struct.pack('<H', height_cm)
    s = struct.pack('<H', speed_cmps)
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x02, 0x01]) + h + s + bytes([0x00, 0x00, 0x00, 0x00]))

def cmd_descend(height_cm: int, speed_cmps: int) -> bytes:
    h = struct.pack('<H', height_cm)
    s = struct.pack('<H', speed_cmps)
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x02, 0x02]) + h + s + bytes([0x00, 0x00, 0x00, 0x00]))
```

---

## 5. 树莓派环境搭建

### 5.1 系统安装

```bash
# 刷写 Raspberry Pi OS Lite (64-bit, Bookworm)
# 使用 Raspberry Pi Imager，配置：
#   - 主机名: pixhawk-pi
#   - 开启SSH
#   - 设置用户密码
#   - 配置WiFi(用于开发调试)

# 首次SSH登录后
sudo apt update && sudo apt upgrade -y
sudo raspi-config
# → Interface Options → Serial Port:
#   "Would you like a login shell over serial?" → No
#   "Would you like the serial port hardware to be enabled?" → Yes
# 这会禁用蓝牙串口占用，释放 /dev/serial0 (GPIO14/15) 给树莓派↔MCU通信
sudo reboot
```

### 5.2 安装依赖

```bash
# OpenCV (预编译版，无需从源码编译)
sudo apt install -y python3-opencv

# 串口库
sudo apt install -y python3-serial

# 其它工具
sudo apt install -y python3-numpy python3-pip git vim tmux

# 海康相机SDK (如果用MVS)
# 从海康官网下载 Linux ARM64 版 MVS SDK
# 或直接用 OpenCV VideoCapture（海康USB相机通常支持UVC协议可直接用）
```

### 5.3 验证设备

```bash
# 检查串口
ls -la /dev/serial0    # 应指向 /dev/ttyAMA0 或 /dev/ttyS0

# 检查相机
lsusb                  # 应看到海康相机设备
v4l2-ctl --list-devices

# 测试相机抓图
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
cv2.imwrite('/tmp/test.jpg', frame)
print('OK' if ret else 'FAIL')
cap.release()
"
```

### 5.4 项目目录结构

```
/home/pi/drone/
├── main.py                 # 主程序入口 + 状态机
├── config.py               # 配置参数（速度、阈值、路径等）
├── lx_protocol.py          # 凌霄API帧构建（见第四章）
├── mcu_serial.py           # MCU串口通信（发送指令+读取光流）
├── vision.py               # 图像采集 + 颜色识别 + 区块检测 + OCR
├── localization.py         # 三层定位融合算法
├── path_plan.py            # 全覆盖路径规划
├── state_machine.py        # 状态机实现
├── laser_led.py            # 激光笔+LED GPIO控制
├── utils.py                # 工具函数
├── test/
│   ├── test_camera.py      # 相机测试
│   ├── test_serial.py      # 串口测试
│   ├── test_move.py        # 水平移动测试
│   └── test_color.py       # 颜色阈值调试
└── logs/                   # 运行日志（调试用）
```

### 5.5 开机自启

```bash
# /etc/systemd/system/drone.service
sudo tee /etc/systemd/system/drone.service << 'EOF'
[Unit]
Description=Drone Auto Mission
After=multi-user.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/drone
ExecStart=/usr/bin/python3 /home/pi/drone/main.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable drone.service
```

---

## 6. MCU固件开发

### 6.1 开发环境

- **IDE**: Keil MDK V5
- **芯片包**: Keil.STM32F4xx_DFP.2.2.0 (已在资料包中)
- **源码工程**: `凌霄/5.飞控MCU源码工程/ANO_LX_FC-2021-7-18 121043.rar`
- **底层驱动**: 沿用官方 `DriversMcu/STM32F407/Drivers/` 目录
- **用户代码**: 修改 `FcSrc/User_Task.c` 和新增串口通信模块

### 6.2 需修改/新增的文件

#### 6.2.1 `FcSrc/User_Task.c` — 主业务逻辑

```c
// 树莓派串口指令解析 (在50Hz任务中调用)
#include "lx_protocol.h"   // IMU API帧构建

static u8 pi_rx_buf[256];
static u8 pi_rx_len = 0;

// 50Hz: 处理树莓派下发的指令
void UserTask_OneKeyCmd(void)
{
    // 1. 读取树莓派串口数据
    pi_rx_len = DrvUartReadBuffer(UART_PI, pi_rx_buf, sizeof(pi_rx_buf));
    if (pi_rx_len > 0) {
        PiCmd_Parse(pi_rx_buf, pi_rx_len);  // 解析指令
    }
  
    // 2. 发送光流数据给树莓派(每200ms)
    static u32 last_of_send = 0;
    if (GetSysTimeMs() - last_of_send > 200) {
        Send_OF_Position();   // 发送0xCC 0x01 + POS_X/Y
        last_of_send = GetSysTimeMs();
    }
}

void PiCmd_Parse(u8 *data, u8 len)
{
    if (len < 3) return;
  
    if (data[0] == 0xAA) {
        // TYPE=0x01: 转发IMU指令
        // 数据格式: AA [LEN] 01 [IMU_API_FRAME]
        u8 frame_len = data[1];
        if (frame_len > 0 && data[2] == 0x01) {
            // 将 data[3..3+frame_len-1] 发送给IMU (UART1)
            ANO_DT_LX_Send_Data(&data[3], frame_len);
        }
    }
    else if (data[0] == 0xBB) {
        // TYPE=0xBB: 控制/查询命令
        if (len >= 2) {
            switch (data[1]) {
                case 0x01:  // 请求光流位置
                    Send_OF_Position();
                    break;
                case 0x02:  // 请求飞行状态
                    Send_Flight_Status();
                    break;
                case 0x03:  // 重置光流零点
                    Reset_OF_Zero();
                    break;
            }
        }
    }
}

void Send_OF_Position(void)
{
    u8 buf[12];
    buf[0] = 0xCC; buf[1] = 0x01;
    // POS_X: 光流积分X位置(s32, cm)
    buf[2] = ano_of.of_pos_x & 0xFF;
    buf[3] = (ano_of.of_pos_x >> 8) & 0xFF;
    buf[4] = (ano_of.of_pos_x >> 16) & 0xFF;
    buf[5] = (ano_of.of_pos_x >> 24) & 0xFF;
    // POS_Y: 光流积分Y位置(s32, cm)
    buf[6] = ano_of.of_pos_y & 0xFF;
    buf[7] = (ano_of.of_pos_y >> 8) & 0xFF;
    buf[8] = (ano_of.of_pos_y >> 16) & 0xFF;
    buf[9] = (ano_of.of_pos_y >> 24) & 0xFF;
    // QUALITY
    buf[10] = ano_of.of_quality;
    PiUart_Send(buf, 11);
}

void Send_Flight_Status(void)
{
    u8 buf[8];
    buf[0] = 0xCC; buf[1] = 0x02;
    buf[2] = fc_sta.mode;       // 当前飞行模式
    buf[3] = fc_sta.locked;     // 锁定状态
    // 高度 (s32, cm)
    s32 alt = fc_sta.alt_cm;
    buf[4] = alt & 0xFF;
    buf[5] = (alt >> 8) & 0xFF;
    buf[6] = (alt >> 16) & 0xFF;
    buf[7] = (alt >> 24) & 0xFF;
    PiUart_Send(buf, 8);
}

void Reset_OF_Zero(void)
{
    ano_of.of_pos_x = 0;
    ano_of.of_pos_y = 0;
}
```

#### 6.2.2 串口配置

```c
// 在 SysConfig.h 中新增树莓派串口定义
#define UART_PI    USART3    // 或 USART2，选一个空闲串口
// 波特率: 115200
// 在初始化代码中调用 DrvUartInit(UART_PI, 115200)
```

### 6.3 编译与烧录

```bash
# 在Keil中打开工程
ProjectSTM32F407/ANO_LX_FC.uvprojx

# 编译 (F7)
# 烧录 (F8) 使用 J-Link / ST-Link
```

---

## 7. 视觉识别开发

### 7.1 相机参数配置

```python
# vision.py

import cv2
import numpy as np

class Camera:
    def __init__(self, device_id=0, width=640, height=480, fps=30):
        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # 海康相机可能需要额外设置：
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 手动曝光
    
    def read(self):
        ret, frame = self.cap.read()
        return ret, frame
  
    def release(self):
        self.cap.release()
```

### 7.2 颜色识别 — 区块检测

```python
# vision.py (续)

class BlockDetector:
    # 颜色HSV阈值（需现场调试微调！）
    # 赛题标准颜色：
    #   淡绿色播撒区: RGB(150,250,150)
    #   淡灰色非播撒区: RGB(240,240,240)
    #   黑色标志线: 0.5cm宽
  
    def __init__(self):
        # 绿色阈值 (HSV) — 先以标准值为参考，现场调整
        self.green_lower = np.array([35, 40, 40])
        self.green_upper = np.array([85, 255, 255])
    
        # 灰色阈值 (HSV)
        self.gray_lower = np.array([0, 0, 180])
        self.gray_upper = np.array([180, 30, 255])
    
        # 黑色阈值 (HSV) — 用于边界线检测
        self.black_lower = np.array([0, 0, 0])
        self.black_upper = np.array([180, 255, 50])
  
    def detect_green_mask(self, hsv):
        """返回绿色区域二值mask"""
        return cv2.inRange(hsv, self.green_lower, self.green_upper)
  
    def calc_green_ratio(self, hsv):
        """计算画面中绿色像素占比（用于边界跳变检测）"""
        mask = self.detect_green_mask(hsv)
        return np.count_nonzero(mask) / mask.size
  
    def find_green_blocks(self, hsv):
        """
        在画面中找到所有绿色区块轮廓
        返回: [(cx, cy, w, h), ...]  各区块的中心和尺寸
        """
        mask = self.detect_green_mask(hsv)
        # 形态学去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        blocks = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:  # 过小忽略（噪点）
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w//2, y + h//2
            blocks.append((cx, cy, w, h, area))
    
        # 按面积从大到小排
        blocks.sort(key=lambda b: b[4], reverse=True)
        return blocks
  
    def find_block_boundary_lines(self, hsv):
        """检测黑色边界线（备选方案）"""
        mask = cv2.inRange(hsv, self.black_lower, self.black_upper)
        lines = cv2.HoughLinesP(mask, 1, np.pi/180, 
                                threshold=50, minLineLength=80, maxLineGap=20)
        return lines
```

### 7.3 数字识别 (OCR)

```python
# vision.py (续)
import pytesseract  # 需要 sudo apt install tesseract-ocr

class DigitReader:
    """
    识别区块上的数字编号
    区块数字是25cm高的加粗黑体，灰色(RGB 240,240,240)
    与灰色非播撒区颜色相同
    """
    def __init__(self):
        # 灰色数字的HSV阈值
        self.digit_lower = np.array([0, 0, 180])
        self.digit_upper = np.array([180, 25, 255])
  
    def extract_digits(self, frame, block_roi=None):
        """
        从画面中提取数字
        block_roi: (x, y, w, h) 可选，限定识别区域
        返回: 识别到的数字(int) 或 None
        """
        if block_roi is not None:
            x, y, w, h = block_roi
            roi = frame[y:y+h, x:x+w]
        else:
            roi = frame
    
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # 二值化（数字是亮的灰色，背景是暗的绿色）
        # 反转：让数字变成白字黑底
        _, thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    
        # OCR识别
        config = '--psm 6 -c tessedit_char_whitelist=0123456789'
        text = pytesseract.image_to_string(thresh, config=config).strip()
    
        if text and text.isdigit():
            return int(text)
        return None
  
    def find_A_marker(self, frame):
        """
        检测"A"标记
        A标记：加粗黑体，字符高25cm (赛题描述)
        在区块21的位置
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    
        # 模板匹配（需要先截取A字符图片作为模板）
        # template = cv2.imread('templates/A_marker.png', 0)
        # result = cv2.matchTemplate(thresh, template, cv2.TM_CCOEFF_NORMED)
        # 或使用轮廓特征检测
    
        # 简化方案：找大面积的黑色连通区域
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if h > 30 and w > 15 and h/w > 1.5:  # 字高>字宽的竖长形状
                # 这个区域可能是字母
                roi = thresh[y:y+h, x:x+w]
                # tesseract 识别
                text = pytesseract.image_to_string(roi, 
                    config='--psm 10 -c tessedit_char_whitelist=A').strip()
                if text == 'A':
                    return (x + w//2, y + h//2)  # 中心坐标
        return None
```

### 7.4 视觉偏移计算（用于区块居中）

```python
# vision.py (续)

def calc_offset_to_block(frame_center, block_center, altitude_cm, focal_length_px):
    """
    根据像素偏差计算实际距离偏差(cm)
  
    frame_center: (cx, cy) 画面中心
    block_center: (bx, by) 区块中心
    altitude_cm: 飞行高度(cm)
    focal_length_px: 相机焦距(像素) 需标定
    """
    dx_px = block_center[0] - frame_center[0]
    dy_px = block_center[1] - frame_center[1]
  
    # 相似三角形：实际偏差 = 像素偏差 * (高度 / 焦距)
    scale = altitude_cm / focal_length_px
    dx_cm = dx_px * scale
    dy_cm = dy_px * scale
  
    return dx_cm, dy_cm
```

### 7.5 颜色阈值现场调试工具

```python
# test/test_color.py  — 运行时放在地面，手动调整HSV阈值

import cv2
import numpy as np

def nothing(x): pass

cv2.namedWindow('Threshold Tuner')

# 创建滑动条
cv2.createTrackbar('H Low', 'Threshold Tuner', 35, 179, nothing)
cv2.createTrackbar('S Low', 'Threshold Tuner', 40, 255, nothing)
cv2.createTrackbar('V Low', 'Threshold Tuner', 40, 255, nothing)
cv2.createTrackbar('H High', 'Threshold Tuner', 85, 179, nothing)
cv2.createTrackbar('S High', 'Threshold Tuner', 255, 255, nothing)
cv2.createTrackbar('V High', 'Threshold Tuner', 255, 255, nothing)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret: break
  
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  
    h_low = cv2.getTrackbarPos('H Low', 'Threshold Tuner')
    s_low = cv2.getTrackbarPos('S Low', 'Threshold Tuner')
    v_low = cv2.getTrackbarPos('V Low', 'Threshold Tuner')
    h_high = cv2.getTrackbarPos('H High', 'Threshold Tuner')
    s_high = cv2.getTrackbarPos('S High', 'Threshold Tuner')
    v_high = cv2.getTrackbarPos('V High', 'Threshold Tuner')
  
    lower = np.array([h_low, s_low, v_low])
    upper = np.array([h_high, s_high, v_high])
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)
  
    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)
  
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"Final thresholds: H({h_low},{h_high}) S({s_low},{s_high}) V({v_low},{v_high})")
```

---

## 8. 三层定位融合算法

### 8.1 整体思路

```
Layer 1 (光流死推算):  持续运行，每50ms更新位置 → 积分漂移
Layer 2 (颜色跳变检测): 检测绿色→灰色过渡 → 确认跨过边界，重置光流
Layer 3 (数字OCR):     读取区块编号 → 绝对定位，校准所有误差
```

### 8.2 实现代码

```python
# localization.py

import numpy as np
from enum import Enum
from path_plan import PATH

class Localizer:
    """
    三层定位融合
    - Layer 1: 光流积分位置跟踪
    - Layer 2: 颜色占比跳变检测边界
    - Layer 3: OCR数字绝对校准
    """
    def __init__(self):
        self.path = PATH                    # 预设路径 [21, 20, 18, ...]
        self.path_index = 0                 # 当前在路径中的位置
        self.current_block = self.path[0]   # 当前区块号
    
        # 光流位置积分 (cm)，起飞点为原点
        self.pos_x = 0.0
        self.pos_y = 0.0
    
        # Layer 2 相关
        self.prev_green_ratio = 0.0
        self.green_drop_threshold = 0.4      # 绿色占比下降超过此值=跨边界
        self.green_high = 0.6                # 绿色占比高=在区块内
        self.green_low = 0.2                 # 绿色占比低=在灰色区域
    
        # Layer 3 相关
        self.last_ocr_block = None
        self.since_last_ocr = 0             # 距上次OCR读数经过的区块数
        self.ocr_interval = 4               # 每4个区块尝试OCR一次
    
        # 移动方向追踪
        self.move_direction = 0             # 当前移动方向(度)
        self.block_size_cm = 50             # 每个区块50cm
    
        # 微调状态
        self.fine_tuning = False
        self.fine_tune_dx = 0
        self.fine_tune_dy = 0
  
    def update_optical_flow(self, dx_cm, dy_cm):
        """Layer 1: 更新光流积分位置"""
        self.pos_x += dx_cm
        self.pos_y += dy_cm
  
    def check_boundary_crossed(self, green_ratio):
        """
        Layer 2: 检测是否跨越了区块边界
        返回: True 如果刚刚跨过了边界
        """
        crossed = False
    
        if self.prev_green_ratio > self.green_high and green_ratio < self.green_low:
            # 从绿色区域进入了灰色区域 → 刚刚离开了一块
            crossed = True
    
        if self.prev_green_ratio < self.green_low and green_ratio > self.green_high:
            # 从灰色区域进入了绿色区域 → 刚刚进入了一块
            crossed = True
    
        self.prev_green_ratio = green_ratio
        return crossed
  
    def apply_ocr(self, block_number):
        """
        Layer 3: OCR读取到了区块编号
        绝对校准，重置所有漂移
        """
        if block_number is None:
            return False
    
        if block_number in self.path:
            self.current_block = block_number
            self.path_index = self.path.index(block_number)
            self.last_ocr_block = block_number
            self.since_last_ocr = 0
            self.pos_x = 0.0  # 相对此区块的漂移清零
            self.pos_y = 0.0
            return True
        return False
  
    def advance_block(self):
        """
        确认进入下一区块，推进路径索引
        """
        self.path_index += 1
        if self.path_index >= len(self.path):
            self.path_index = len(self.path) - 1  # 兜底
        self.current_block = self.path[self.path_index]
        self.pos_x = 0.0  # 重置相对漂移
        self.pos_y = 0.0
        self.since_last_ocr += 1
    
    def get_current_target(self):
        """获取当前目标区块编号"""
        return self.current_block
  
    def is_mission_complete(self):
        """所有区块是否已覆盖"""
        return self.path_index >= len(self.path) - 1
  
    def should_do_ocr(self):
        """是否该尝试OCR校准了"""
        return self.since_last_ocr >= self.ocr_interval
  
    def get_position_for_return(self):
        """
        返回起飞点的位置
        用于降落导航
        """
        # 光流累积位置取反即是从起飞点到当前位置的向量
        # 注意：需要在整个飞行过程中保持全局积分（不重置）
        return -self._global_pos_x, -self._global_pos_y
  
    def init_global_position(self):
        """起飞时记录全局零点"""
        self._global_pos_x = 0.0
        self._global_pos_y = 0.0
  
    def update_global_position(self, dx_cm, dy_cm):
        """更新全局位置（用于返航）"""
        self._global_pos_x += dx_cm
        self._global_pos_y += dy_cm
```

---

## 9. 全覆盖路径规划

### 9.1 作业区布局还原

根据赛题图1，作业区 400cm×500cm，区块尺寸 50cm×50cm：

```
         Col0  Col1  Col2  Col3  Col4  Col5  Col6
         ────  ────  ────  ────  ────  ────  ────
Row0(top) 28    26    25    24    23    --    22
Row1      21    20    18    16    15    19    17
Row2      12    14    13    11    --    --    --
Row3      10     9    --    --     8     7    --
Row4      --    --    --    --     5     6    --
Row5      --    --    --    --     4     3    --
Row6(bot) --    --    --    --     1     2    --
```

起降点 "十" 在左下角外（距作业区75cm+50cm=125cm处）
A标记在区块21

### 9.2 蛇形全覆盖路径

```python
# path_plan.py

# 每个区块在世界坐标系的中心位置 (x_cm, y_cm)
# 原点: 起降点 "十" 字
# X轴正方向: 飞机机头方向（指向作业区）
# Y轴正方向: 飞机右侧（指向Col增加方向）

# 区块坐标映射: block_id -> (col_index, row_index)
BLOCK_GRID = {}
BLOCK_POSITIONS = {}  # block_id -> (x_cm, y_cm)  世界坐标

def init_grid():
    """
    初始化区块网格坐标
    col: 列索引 (0=左, 6=右)
    row: 行索引 (0=顶, 6=底)
    每块50cm×50cm，坐标以区块中心计
    """
    layout = {
        # (row, col): block_id
        (0, 0): 28, (0, 1): 26, (0, 2): 25, (0, 3): 24, (0, 4): 23, (0, 6): 22,
        (1, 0): 21, (1, 1): 20, (1, 2): 18, (1, 3): 16, (1, 4): 15, (1, 5): 19, (1, 6): 17,
        (2, 0): 12, (2, 1): 14, (2, 2): 13, (2, 3): 11,
        (3, 0): 10, (3, 1):  9,                       (3, 4):  8, (3, 5):  7,
        (4, 4):  5, (4, 5):  6,
        (5, 4):  4, (5, 5):  3,
        (6, 4):  1, (6, 5):  2,
    }
  
    # 起降点坐标: 在左下角外
    # 作业区底部边缘y=0，起降点在y=-50cm左右
    # 作业区左下角(第6行第4列)中心x=4*50+25=225cm(从左边), y=6*50+25=325cm(从顶)
    # 简化：令起降点处x=0
  
    ORIGIN_OFFSET_X = 100  # 从起降点到作业区左边界的距离(cm)
    ORIGIN_OFFSET_Y = 100  # 从起降点到作业区底部边界的距离(cm)
  
    for (row, col), bid in layout.items():
        # 在grid坐标中 (col增=X轴正方向, row增=Y轴负方向)
        x = ORIGIN_OFFSET_X + col * 50 + 25  # 区块中心X
        y = ORIGIN_OFFSET_Y + (6 - row) * 50 + 25  # 区块中心Y (翻转axis)
    
        BLOCK_GRID[bid] = (col, row)
        BLOCK_POSITIONS[bid] = (x, y)
  
    return BLOCK_POSITIONS

# 预设蛇形路径 (从A=21开始)
# 路径决定飞行顺序和移动方向
PATH = [21,20,18,16,15,19,17,   # Row1: 从左到右，跳过col5
        22,23,24,25,26,28,      # Row0: 从右到左(上行)
        12,14,13,11,             # Row2: 从左到右
        10,9,8,7,5,6,4,3,1,2]   # Row3-6: 蛇形

def generate_move_commands(path, positions, speed_cmps=30):
    """
    根据路径生成移动指令列表
    返回: [(distance_cm, direction_deg, target_block_id), ...]
    """
    commands = []
    for i in range(len(path) - 1):
        cur_id = path[i]
        nxt_id = path[i + 1]
        cur_pos = positions[cur_id]
        nxt_pos = positions[nxt_id]
    
        dx = nxt_pos[0] - cur_pos[0]
        dy = nxt_pos[1] - cur_pos[1]
    
        distance = np.sqrt(dx**2 + dy**2)
        direction = np.degrees(np.arctan2(dy, dx)) % 360
    
        commands.append({
            'from': cur_id,
            'to': nxt_id,
            'distance': int(distance),
            'direction': int(direction),
            'speed': speed_cmps
        })
    return commands


def get_return_to_home_command(current_block_id, positions, speed_cmps=30):
    """
    从当前区块返回起降点(0,0)的移动指令
    """
    cur_pos = positions[current_block_id]
    home_pos = (0, 0)
  
    dx = home_pos[0] - cur_pos[0]
    dy = home_pos[1] - cur_pos[1]
  
    distance = np.sqrt(dx**2 + dy**2)
    direction = np.degrees(np.arctan2(dy, dx)) % 360
  
    return {
        'from': current_block_id,
        'to': 'HOME',
        'distance': int(distance),
        'direction': int(direction),
        'speed': speed_cmps
    }
```

### 9.3 路径可视化（调试用）

```python
def print_path_map():
    """打印路径地图，方便目视验证"""
    init_grid()
  
    # 创建7×7网格
    grid = [['  ' for _ in range(7)] for _ in range(7)]
    for bid, (col, row) in BLOCK_GRID.items():
        grid[row][col] = f'{bid:2d}'
  
    print("    Col: 0   1   2   3   4   5   6")
    for row_idx, row in enumerate(grid):
        print(f"Row{row_idx}: " + "  ".join(row))
  
    print(f"\n飞行路径: {'→'.join(map(str, PATH))}")
```

输出示例：

```
    Col: 0   1   2   3   4   5   6
Row0: 28  26  25  24  23      22
Row1: 21  20  18  16  15  19  17
Row2: 12  14  13  11      
Row3: 10   9           8   7  
Row4:                 5   6  
Row5:                 4   3  
Row6:                 1   2  

飞行路径: 21→20→18→16→15→19→17→22→23→24→25→26→28→12→14→13→11→10→9→8→7→5→6→4→3→1→2
```

---

## 10. 状态机设计

### 10.1 状态定义

```python
# state_machine.py

from enum import Enum, auto

class FlightState(Enum):
    IDLE            = auto()   # 待命（收到启动信号前）
    ARM_UNLOCK      = auto()   # 解锁电机
    SET_PROGRAM_MODE = auto()  # 切换到程控模式
    TAKEOFF         = auto()   # 起飞至150cm
    FIND_START      = auto()   # 寻找A标记和区块21
    SPRAY           = auto()   # 撒药（激光闪烁）
    NAVIGATE        = auto()   # 移动到下一区块
    RETURN_HOME     = auto()   # 返回起降点
    LAND            = auto()   # 着陆
    LOCK            = auto()   # 加锁
    EMERGENCY       = auto()   # 紧急状态
    DONE            = auto()   # 任务完成
```

### 10.2 状态转换

```
IDLE ──(启动信号)──→ ARM_UNLOCK
                       │
                       ↓
                  SET_PROGRAM_MODE
                       │
                       ↓
                     TAKEOFF ──(高度<140或>160)──→ TAKEOFF (重试上升)
                       │ (高度到达150±10)
                       ↓
                    FIND_START ──(找到A/区块21)──→ SPRAY
                       │
                       ↓
                      SPRAY ──(闪烁完成, visited[21]=true)──→ NAVIGATE
                       │
                       ↓
                    NAVIGATE ──(到达下一区块上方)──→ SPRAY
                       │              ↑
                       │              │ (还有未访问区块)
                       │              └────────────────┘
                       │
                       │ (全部28块完成)
                       ↓
                   RETURN_HOME ──(到达起降点上方)──→ LAND
                                                       │
                                                       ↓
                                                      LOCK → DONE
```

### 10.3 状态机主循环

```python
# state_machine.py (续)

class DroneStateMachine:
    def __init__(self, mcu, camera, localizer, laser, config):
        self.state = FlightState.IDLE
        self.mcu = mcu            # MCU串口通信对象
        self.camera = camera      # 相机对象
        self.localizer = localizer  # 定位融合对象
        self.laser = laser        # 激光控制对象
        self.visited = [False] * 29  # visited[1..28]
        self.visited[0] = True    # 不用下标0
    
        self.cfg = config         # 配置参数
    
        # 状态计时器
        self.state_start_time = 0
        self.spray_start_time = 0
        self.takeoff_start_time = 0
    
        # 路径相关
        self.path = PATH
        self.move_commands = None
        self.cmd_index = 0
    
        # 重试计数
        self.retry_count = 0
        self.max_retries = 3
  
    def run_iteration(self):
        """每个循环周期调用一次（20-50Hz）"""
        frame, green_ratio, ocr_result = self._get_vision_data()
    
        # 更新光流
        of_dx, of_dy = self.mcu.read_optical_flow()
        self.localizer.update_optical_flow(of_dx, of_dy)
    
        # 检查颜色跳变
        if self.localizer.check_boundary_crossed(green_ratio):
            self.localizer.advance_block()
    
        # 尝试OCR校准
        if self.localizer.should_do_ocr():
            self.localizer.apply_ocr(ocr_result)
    
        # 执行当前状态
        {
            FlightState.IDLE:              self._state_idle,
            FlightState.ARM_UNLOCK:        self._state_arm_unlock,
            FlightState.SET_PROGRAM_MODE:  self._state_set_program_mode,
            FlightState.TAKEOFF:           self._state_takeoff,
            FlightState.FIND_START:        self._state_find_start,
            FlightState.SPRAY:             self._state_spray,
            FlightState.NAVIGATE:          self._state_navigate,
            FlightState.RETURN_HOME:       self._state_return_home,
            FlightState.LAND:              self._state_land,
            FlightState.LOCK:              self._state_lock,
        }[self.state](frame, green_ratio, ocr_result)
  
    def _state_idle(self, frame, green_ratio, ocr_result):
        # 等待启动信号（可以是GPIO按钮 或 CH6开关）
        if self._check_start_signal():
            self._transition(FlightState.ARM_UNLOCK)
  
    def _state_arm_unlock(self, frame, green_ratio, ocr_result):
        self.mcu.send_cmd_unlock()
        time.sleep(2)
        self._transition(FlightState.SET_PROGRAM_MODE)
  
    def _state_set_program_mode(self, frame, green_ratio, ocr_result):
        self.mcu.send_cmd_mode(3)  # 程控模式
        time.sleep(1)
        # 重置光流零点
        self.mcu.send_of_zero_reset()
        self.localizer.init_global_position()
        self._transition(FlightState.TAKEOFF)
  
    def _state_takeoff(self, frame, green_ratio, ocr_result):
        self.mcu.send_cmd_takeoff(150)  # 起飞至150cm
    
        # 检查高度是否到达
        alt = self.mcu.read_altitude()
        if 140 <= alt <= 160:
            self._transition(FlightState.FIND_START)
        elif time.time() - self.state_start_time > 15:
            # 超时重试
            self.retry_count += 1
            if self.retry_count > self.max_retries:
                self._transition(FlightState.EMERGENCY)
            else:
                self.mcu.send_cmd_takeoff(150)
  
    def _state_find_start(self, frame, green_ratio, ocr_result):
        # 飞到区块21的大致位置
        # 用路径规划的移动指令
        cmd = generate_move_commands(PATH[:1], BLOCK_POSITIONS, 
                                     self.cfg['debug_speed'])[0]
        self.mcu.send_cmd_move(cmd['distance'], cmd['speed'], cmd['direction'])
    
        # 等待移动完成 + 视觉确认
        if ocr_result == 21:  # 或检测到A标记
            # 微调居中
            dx, dy = calc_offset_to_block(frame_center, block_center, 
                                          self.mcu.read_altitude(), 800)
            if abs(dx) < 5 and abs(dy) < 5:
                self._transition(FlightState.SPRAY)
            else:
                # 发送微调指令(<10cm移动)
                self.mcu.send_cmd_move(min(abs(dx), 10), 15, 
                    np.degrees(np.arctan2(dy, dx)) % 360)
  
    def _state_spray(self, frame, green_ratio, ocr_result):
        cur_block = self.localizer.get_current_target()
    
        if not self.visited[cur_block]:
            # 激光闪烁
            self.laser.blink(count=2, period_ms=1500)
            self.visited[cur_block] = True
            print(f"[SPRAY] Block {cur_block} sprayed")
    
        # 检查是否完成
        if all(self.visited[1:]):
            self._transition(FlightState.RETURN_HOME)
        else:
            self._transition(FlightState.NAVIGATE)
  
    def _state_navigate(self, frame, green_ratio, ocr_result):
        # 找下一个未访问区块
        next_block = None
        for bid in self.path:
            if not self.visited[bid]:
                next_block = bid
                break
    
        if next_block is None:
            self._transition(FlightState.RETURN_HOME)
            return
    
        # 从当前位置移动到next_block
        cur_block = self.localizer.get_current_target()
        cur_pos = BLOCK_POSITIONS[cur_block]
        nxt_pos = BLOCK_POSITIONS[next_block]
    
        dx = nxt_pos[0] - cur_pos[0]
        dy = nxt_pos[1] - cur_pos[1]
        distance = int(np.sqrt(dx**2 + dy**2))
        direction = int(np.degrees(np.arctan2(dy, dx)) % 360)
    
        self.mcu.send_cmd_move(distance, self.cfg['move_speed'], direction)
    
        # 等待移动完成（颜色跳变检测会自动触发advance_block）
        # 或超时继续
        if time.time() - self.state_start_time > 5:
            self._transition(FlightState.SPRAY)
  
    def _state_return_home(self, frame, green_ratio, ocr_result):
        # 发送返航指令 (使用凌霄一键返航 或 自行计算)
        gpx, gpy = self.localizer.get_position_for_return()
        distance = int(np.sqrt(gpx**2 + gpy**2))
        direction = int(np.degrees(np.arctan2(gpy, gpx)) % 360)
    
        self.mcu.send_cmd_move(distance, self.cfg['move_speed'], direction)
        time.sleep(distance / self.cfg['move_speed'] + 1)
    
        self._transition(FlightState.LAND)
  
    def _state_land(self, frame, green_ratio, ocr_result):
        self.mcu.send_cmd_land()
    
        # 等待降落完成
        alt = self.mcu.read_altitude()
        if alt < 10:  # 距离地面10cm以内
            time.sleep(1)
            self._transition(FlightState.LOCK)
        elif time.time() - self.state_start_time > 20:
            self._transition(FlightState.EMERGENCY)
  
    def _state_lock(self, frame, green_ratio, ocr_result):
        self.mcu.send_cmd_lock()
        print("==> MISSION COMPLETE <==")
        self._transition(FlightState.DONE)
  
    def _transition(self, new_state):
        print(f"[STATE] {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_start_time = time.time()
        self.retry_count = 0
  
    def _get_vision_data(self):
        ret, frame = self.camera.read()
        if not ret:
            return None, 0.0, None
    
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_ratio = self.camera.detector.calc_green_ratio(hsv)
        ocr_result = self.camera.digit_reader.extract_digits(frame)
    
        return frame, green_ratio, ocr_result
  
    def _check_start_signal(self):
        # 通过MCU读取CH6/AUX2通道值
        # >1700 视为启动
        return self.mcu.read_aux2() > 1700
```

---

## 11. 激光笔与LED控制

```python
# laser_led.py

import RPi.GPIO as GPIO
import time

class LaserController:
    """
    激光笔控制 — GPIO输出
    激光笔通过三极管/MOSFET驱动，树莓派GPIO作为开关信号
    """
    def __init__(self, pin=17):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
  
    def on(self):
        GPIO.output(self.pin, GPIO.HIGH)
  
    def off(self):
        GPIO.output(self.pin, GPIO.LOW)
  
    def blink(self, count=2, period_ms=1500):
        """
        闪烁激光笔模拟撒药
        count: 闪烁次数(1-3)
        period_ms: 闪烁周期(1-2秒)
        """
        on_time = period_ms / 2 / 1000.0  # 50%占空比
        off_time = period_ms / 2 / 1000.0
    
        for i in range(count):
            self.on()
            time.sleep(on_time)
            self.off()
            time.sleep(off_time)
  
    def cleanup(self):
        GPIO.cleanup()


class LEDController:
    """
    LED指示灯 — 用于显示条码数字(发挥部分)
    """
    def __init__(self, pin=27):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
  
    def show_number(self, number):
        """闪烁LED显示数字"""
        for _ in range(number):
            GPIO.output(self.pin, GPIO.HIGH)
            time.sleep(0.3)
            GPIO.output(self.pin, GPIO.LOW)
            time.sleep(0.3)
        time.sleep(2)  # 间隔数秒
  
    def cleanup(self):
        GPIO.cleanup()
```

---

## 12. 调试步骤（分速度档位）

### 12.1 三档速度配置

```python
# config.py

# 调试速度配置 — 逐步加速
SPEED_PROFILES = {
    'debug': {       # 阶段1: 验证逻辑
        'move_speed': 15,       # cm/s
        'ascent_speed': 15,
        'descent_speed': 10,
        'block_timeout': 10,    # 每块超时(秒)
        'laser_period': 2000,   # 激光闪烁周期(ms)
    },
    'tuning': {      # 阶段2: 联调测试
        'move_speed': 30,
        'ascent_speed': 25,
        'descent_speed': 15,
        'block_timeout': 5,
        'laser_period': 1500,
    },
    'competition': { # 阶段3: 比赛
        'move_speed': 45,
        'ascent_speed': 30,
        'descent_speed': 25,
        'block_timeout': 3,
        'laser_period': 1200,
    }
}

# 当前使用的配置
CURRENT = 'debug'

def get_config():
    return SPEED_PROFILES[CURRENT]
```

### 12.2 分阶段测试脚本

```python
# test/test_move.py — 测试单次移动

import sys
sys.path.append('..')
from mcu_serial import MCUSerial
from lx_protocol import cmd_unlock, cmd_mode, cmd_takeoff, cmd_move, cmd_land, cmd_lock
import time

def test_single_move(distance_cm, direction_deg, speed_cmps):
    """测试单次水平移动"""
    mcu = MCUSerial(port='/dev/serial0', baudrate=115200)
  
    print("1. Unlocking...")
    mcu.send(cmd_unlock())
    time.sleep(2)
  
    print("2. Setting program mode...")
    mcu.send(cmd_mode(3))
    time.sleep(1)
  
    print("3. Taking off...")
    mcu.send(cmd_takeoff(150))
    time.sleep(5)  # 等待起飞完成
  
    print(f"4. Moving {distance_cm}cm at {direction_deg}deg, speed {speed_cmps}cm/s")
    mcu.send(cmd_move(distance_cm, speed_cmps, direction_deg))
    time.sleep(distance_cm / speed_cmps + 2)  # 等待移动完成
  
    print("5. Landing...")
    mcu.send(cmd_land())
    time.sleep(5)
  
    print("6. Locking...")
    mcu.send(cmd_lock())
  
    print("Test complete!")

if __name__ == '__main__':
    # 测试前进50cm
    test_single_move(50, 0, 20)
```

### 12.3 调试阶段划分


| 阶段    | 速度        | 测试内容                                 | 通过标准                        |
| --------- | ------------- | ------------------------------------------ | --------------------------------- |
| **G1**  | debug       | 机上不装视觉，遥控手动飞行验证硬件       | 悬停稳定，定点模式OK            |
| **G2**  | debug       | 串口通信：树莓派→MCU→IMU，验证单条指令 | unlock/takeoff/land/lock 全部OK |
| **G3**  | debug       | 移动测试：前进50cm再退回                 | 实际位移与指令误差<15%          |
| **G4**  | debug       | 光流数据读取                             | 树莓派能正确解析POS_X/Y         |
| **G5**  | debug       | 相机画面传输+颜色阈值调试                | 绿色/灰色正确分割               |
| **G6**  | debug       | 边界跳变检测                             | 手拿飞机模拟移动，能检测跳变    |
| **G7**  | debug       | 数字OCR调试                              | 辅助灯光，检测OCR准确率         |
| **G8**  | debug       | 全流程自主飞行（慢速）                   | 完成一次完整的28块全覆盖        |
| **G9**  | tuning      | 全流程（中速）                           | 完成时间<300秒                  |
| **G10** | competition | 全流程（高速）                           | 完成时间<200秒，降落<10cm       |

---

## 13. 异常处理策略

```python
# 异常处理框架

class EmergencyHandler:
    """
    处理各类异常情况
    """
    def __init__(self, mcu):
        self.mcu = mcu
        self.fail_count = 0
        self.max_fails = 3
  
    def handle_comm_timeout(self):
        """MCU通信超时"""
        self.fail_count += 1
        print(f"[WARN] MCU communication timeout ({self.fail_count}/{self.max_fails})")
        if self.fail_count > self.max_fails:
            print("[EMERGENCY] Lost communication with MCU!")
            # 触发降落（MCU侧应有关断保护）
            return True
        return False
  
    def handle_altitude_anomaly(self, alt_cm):
        """高度异常（过低或过高）"""
        if alt_cm < 50:
            print("[WARN] Altitude too low! Attempting ascent...")
            self.mcu.send(cmd_ascend(100, 25))
        elif alt_cm > 300:
            print("[WARN] Altitude too high! Descending...")
            self.mcu.send(cmd_descend(100, 25))
        elif alt_cm < 20:
            print("[EMERGENCY] Critically low altitude!")
            self.mcu.send(cmd_land())
  
    def handle_touchdown(self):
        """触地检测 — 5秒内不能恢复=失败"""
        # MCU侧检测到电机堵转/高度异常低
        pass
  
    def handle_lost_optical_flow(self):
        """光流失锁"""
        print("[WARN] Optical flow lost! Hovering in place...")
        # 悬停等待恢复
        self.mcu.send(cmd_ascend(0, 0))  # 或者发悬停指令
  
    def emergency_land(self):
        """紧急降落"""
        print("[EMERGENCY] Initiating emergency landing!")
        self.mcu.send(cmd_land())
        time.sleep(5)
        self.mcu.send(cmd_lock())


# 在状态机主循环中集成异常检测
def check_exceptions(state_machine, emergency):
    alt = state_machine.mcu.read_altitude()
  
    # 低电量
    voltage = state_machine.mcu.read_voltage()
    if voltage < cfg['low_voltage_threshold']:
        emergency.emergency_land()
  
    # 高度异常
    if alt < 30 or alt > 250:
        emergency.handle_altitude_anomaly(alt)
  
    # 任务超时 (360秒)
    if state_machine.mission_time > 360:
        print("[TIMEOUT] Mission time exceeded 360 seconds!")
        emergency.emergency_land()
```

### 13.1 异常场景与对策表


| 异常              | 检测方式               | 对策                          |
| ------------------- | ------------------------ | ------------------------------- |
| 光流失锁          | 连续1秒无光流数据更新  | 悬停等待，超5秒则触发紧急降落 |
| 通信超时          | 连续500ms无MCU响应     | 重试3次，仍失败则紧急降落     |
| 高度过高          | >250cm                 | 发送下降指令至150cm           |
| 高度过低          | <30cm                  | 发送上升指令，<10cm紧急降落   |
| 触地              | 高度<5cm + 光流速率为0 | 立即加锁（其实已落地）        |
| 360秒超时         | 任务计时器             | 放弃剩余区块，直接返航降落    |
| 电池低压          | 电压<阈值              | 紧急降落在当前位置            |
| 连续3次边界未检测 | visited超过预期        | 降低速度，重试OCR绝对定位     |

---

## 14. 比赛测试流程

### 14.1 测试前检查清单

```
□ 电池满电（3S≥11.1V 或 4S≥14.8V）
□ 桨叶防护罩安装牢固
□ 激光笔垂直朝下，方向固定
□ 光流模块镜头清洁
□ 相机镜头清洁，USB连接牢固
□ 树莓派开机自启服务正常（SSH验证）
□ 凌霄上位机确认IMU状态正常（姿态角、气压高度）
□ 光流定点模式悬停稳定（遥控验证）
□ 佩戴护目镜和防护手套
```

### 14.2 测试步骤

```
1. 放置飞行器在起降点 "十" 字上，机头指向作业区方向
2. 上电（先给飞控上电，等树莓派启动约30秒）
3. 检查树莓派状态（SSH登录查看日志 / LED指示灯）
4. 遥控器AUX1切到自稳+定高模式（调试阶段可先手动飞）
   ── 进入程控模式（正式测试）
5. 遥控器CH6/按钮触发启动信号
6. 观察飞行器自主执行：
   a. 解锁 → 程控模式 → 起飞至150cm
   b. 飞到区块21上方 → 激光闪烁
   c. 按路径依次全覆盖28块 → 激光每块闪烁
   d. 返回起降点 → 缓慢下降 → 加锁
7. 测量降落偏差（几何中心到"十"字中心）
8. 记录总耗时
```

### 14.3 结果记录表


| 项目         | 要求         | 实测值    | 达标 |
| -------------- | -------------- | ----------- | ------ |
| 起飞高度     | 150±10cm    | ___cm     | □   |
| 起点区块     | 从A(21)开始  | 区块___   | □   |
| 全覆盖       | 28块全部闪烁 | 完成___块 | □   |
| 完成任务时间 | <360秒       | ___秒     | □   |
| 降落偏差     | ≤±10cm     | ___cm     | □   |
| 漏撒         | 0            | ___块     | □   |
| 重撒         | 0            | ___块     | □   |
| 非播撒区播撒 | 0            | ___块     | □   |

---

## 15. 风险清单与对策


| 编号 | 风险                       | 概率 | 影响 | 对策                                               |
| ------ | ---------------------------- | ------ | ------ | ---------------------------------------------------- |
| R1   | 光照不均导致颜色识别不稳定 | 高   | 中   | 动态阈值 + 颜色跳变检测不依赖绝对阈值 + 黑色线辅助 |
| R2   | 边界线太浅无法检测         | 中   | 中   | 用颜色跳变替代线检测，OCR做绝对定位兜底            |
| R3   | 光流漂移导致路径偏航       | 高   | 高   | 每块用颜色跳变/OCR重置漂移，最多偏半个格子         |
| R4   | 海康相机帧率不足/延迟      | 低   | 中   | 降分辨率至640x480，启用MJPG编码                    |
| R5   | 树莓派供电不足导致重启     | 中   | 高   | 独立BEC供电(5V 3A+)，加电容滤波                    |
| R6   | 螺旋桨触安全网             | 中   | 高   | 桨叶全防护罩 + 定高选150cm(低于安全网)             |
| R7   | 电池电量不足360秒          | 中   | 高   | 测试续航，备换电池，优化的飞行速度减少悬停         |
| R8   | API指令响应超时            | 低   | 中   | 每条指令有超时+重试机制                            |
| R9   | 起飞后树莓派死机           | 低   | 高   | MCU侧检测心跳超时→自动触发紧急降落                |
| R10  | 降落偏差>10cm              | 中   | 中   | 光流全程跟踪全局位置 + 下降时光流保持水平位置      |

---

## 16. 附录：文件路径索引

### 资料包文件


| 用途                | 路径                                                              |
| --------------------- | ------------------------------------------------------------------- |
| 通信协议V7 PDF      | `凌霄/1.用户手册_通信协议/匿名通信协议V7.pdf`                     |
| 凌霄飞控手册        | `凌霄/1.用户手册_通信协议/匿名--凌霄--飞控手册.V1.07pdf.pdf`      |
| 到手飞手册          | `凌霄/1.用户手册_通信协议/匿名--凌霄到手飞手册.pdf`               |
| PID调参参考         | `凌霄/1.用户手册_通信协议/匿名凌霄FC姿态单参数控制参考配置.txt`   |
| MCU源码主工程       | `凌霄/5.飞控MCU源码工程/ANO_LX_FC-2021-7-18 121043.rar`           |
| 例程1(起飞降落)     | `凌霄/5.飞控MCU源码工程/例程1.一键起飞_降落/`                     |
| 例程2(完整任务)     | `凌霄/5.飞控MCU源码工程/例程2.一键任务_起飞+悬停+前进+右移+降落/` |
| 底板原理图          | `凌霄/6.原理图_PCB/凌霄整机底板ANO-LX-PCB2-20200716.pdf`          |
| STM32F4核心板原理图 | `凌霄/6.原理图_PCB/STM32F407核心板原理图.pdf`                     |
| 最新IMU固件         | `凌霄/7.凌霄IMU固件/ANO_LX-hw122-sw135.ano`                       |
| STM32F4 Keil包      | `凌霄/3.开发环境安装/Keil.STM32F4xx_DFP.2.2.0.pack`               |
| STM32F4 参考手册    | `凌霄/4.相关芯片Datasheet/STM32F4xx中文参考手册.pdf`              |

### 本项目文件


| 用途       | 路径                            |
| ------------ | --------------------------------- |
| 赛题原文   | `G_植保飞行器.pdf`              |
| 本执行计划 | `G_植保飞行器_详细执行计划.md`  |
| 主程序     | `drone/main.py` (待创建)        |
| 配置参数   | `drone/config.py`               |
| 协议库     | `drone/lx_protocol.py`          |
| MCU串口    | `drone/mcu_serial.py`           |
| 视觉处理   | `drone/vision.py`               |
| 定位融合   | `drone/localization.py`         |
| 路径规划   | `drone/path_plan.py`            |
| 状态机     | `drone/state_machine.py`        |
| 激光LED    | `drone/laser_led.py`            |
| MCU固件    | `drone/fc/User_Task.c` (待修改) |

---

> **文档版本**: v1.0
> **最后更新**: 2026年7月
> **团队**: 3人组
> **平台**: 凌霄IMU + STM32F407 + 树莓派4B + 海康USB3.0工业相机
