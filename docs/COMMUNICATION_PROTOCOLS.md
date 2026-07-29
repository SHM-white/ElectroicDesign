# 当前实现说明（2026-07）

当前树莓派通过 USB-TTL 以 500000 bps 直连凌霄 IMU 串口 2，使用匿名通信
协议 V7 原生帧 `AA D_ADDR ID LEN DATA SC AC`。本文中早期设计的 STM32F4
桥接帧 `0xAA/0xBB/0xCC`、查询帧和自定义心跳不再由 `drone/` 主程序使用，
仅供历史固件设计参考。

- 水平移动：`ID=0xE0, CID=0x10, CMD0=0x02, CMD1=0x03`；
- `0x08`：相对起飞点位置偏移，`S32 POS_X/POS_Y`，单位cm，作为主导航源；
- `0x51`：匿名光流模块信息，独立诊断/备用，禁止与`0x08`交替写入同一连续
    位置缓存后计算增量；
- 导航约完成66%后，MVS/工业相机可使用灰色数字中心执行有限小步校准；这不
    改变飞控协议，只会额外发送标准水平移动命令；
- 定位器将`0x08`原始位置和任务世界坐标分开保存，使用
    `world = position_0x08 + offset`映射。灰色数字中心稳定对准目标区块后，以
    该区块已知世界坐标重新估计`offset`；后续`0x08`增量继续沿用新的坐标映射，
    从而修正累计漂移，而不是只完成一次物理微调；
- 《匿名通信协议V7》控制指令说明明确规定：平移和高度控制属于动作互斥指令，
    只执行最新指令；移动指令与所有一键控制命令也互斥。中途灰色小步修正会
    覆盖原平移，因此状态机在修正结束后必须重新发送到原目标的剩余路线。
    真机启用前仍需无桨核对所用固件版本与文档行为一致。

# G_植保飞行器 通信协议与接口参考手册

> 版本: 1.1
> 日期: 2026-07-14
> 适用固件: 凌霄飞控 STM32F4 / GPIO扩展板 STM32H7 / OpenMV

---

## 目录

1. [系统架构概述](#1-系统架构概述)
2. [凌霄IMU API 协议 (lx_protocol.py)](#2-凌霄imu-api-协议-lx_protocolpy)
3. [主控↔MCU 串口协议 (mcu_serial.py)](#3-主控mcu-串口协议-mcu_serialpy)
4. [STM32H7 GPIO 协议 (h7_gpio_protocol.py)](#4-stm32h7-gpio-协议-h7_gpio_protocolpy)
5. [GPIO 后端抽象接口 (gpio_backend.py)](#5-gpio-后端抽象接口-gpio_backendpy)
6. [软件接口速查](#6-软件接口速查)
7. [OpenMV识别结果协议](#7-openmv识别结果协议)
8. [附录: MCU固件接口要求](#8-附录-mcu固件接口要求)

---

## 1. 系统架构概述

### 1.1 硬件连接拓扑

```
┌─────────────────────────────────────────────────────────┐
│                  x86迷你主机 / 树莓派4B                    │
│                                                         │
│  USB1 ──USB-TTL──→ STM32F4 MCU (凌霄飞控)               │
│                       ├─ /dev/ttyUSB0 @ 115200           │
│                       ├─ UART1 ──→ 凌霄IMU (姿态/控制)    │
│                       ├─ UART4 ──→ 光流模块 @ 500000bps   │
│                       └─ SBUS ←── 遥控器接收机            │
│                                                         │
│  USB2 ──USB-TTL──→ STM32H7 GPIO开发板                    │
│                       └─ /dev/ttyUSB1 @ 115200           │
│                                                         │
│  USB3 ──────────→ 海康工业相机 (UVC协议，上位机识别)      │
│       或 USB-TTL←── OpenMV UART (板端识别，只回传结果)    │
└─────────────────────────────────────────────────────────┘
```

### 1.2 串口映射


| 接口 | 设备         | 串口路径       | 波特率 | 用途               |
| ------ | -------------- | ---------------- | -------- | -------------------- |
| USB1 | STM32F4 MCU  | `/dev/ttyUSB0` | 115200 | 飞控指令/状态      |
| USB2 | STM32H7 GPIO | `/dev/ttyUSB1` | 115200 | GPIO控制(激光/LED) |
| USB3 | 海康工业相机 | UVC直连        | N/A    | 图像采集           |
| USB3(可选) | OpenMV | `/dev/ttyUSB1` | 115200 | 识别结果回传       |

工业相机与 OpenMV 是两种可切换视觉方案。若同时使用 H7 GPIO 板和 OpenMV，必须为它们分配不同串口，例如 H7 使用 `/dev/ttyUSB1`、OpenMV 使用 `/dev/ttyUSB2`。

### 1.3 MCU内部串口


| UART  | 连接       | 波特率 | 方向 | 用途              |
| ------- | ------------ | -------- | ------ | ------------------- |
| UART1 | 凌霄IMU    | 115200 | 双向 | 姿态解算/飞行控制 |
| UART4 | 光流模块   | 500000 | 双向 | 位置估计          |
| SBUS  | 遥控接收机 | 100000 | 输入 | 手动控制          |

---

## 2. 凌霄IMU API 协议 (lx_protocol.py)

### 2.1 帧格式

所有帧以固定帧头 `0xAA` 开始，总长度可变。

```
 0    1    2    3    4..4+LEN-1  4+LEN   5+LEN
┌────┬────┬────┬────┬───────────┬───────┬───────┐
│ 0xAA│D_ADDR│ ID │ LEN │ DATA[0..N] │  SC   │  AC   │
└────┴────┴────┴────┴───────────┴───────┴───────┘
  帧头   地址   命令ID  数据长度  数据域      和校验   累加校验
```

**字段说明:**


| 字段   | 长度  | 说明                       |
| -------- | ------- | ---------------------------- |
| 帧头   | 1B    | 固定`0xAA`                 |
| D_ADDR | 1B    | 目标地址,`0xFF` = 广播     |
| ID     | 1B    | 命令标识, 固定`0xE0`       |
| LEN    | 1B    | DATA域字节数 (不含校验)    |
| DATA   | N字节 | 命令载荷, 不足部分补`0x00` |
| SC     | 1B    | 和校验                     |
| AC     | 1B    | 累加校验                   |

### 2.2 校验算法 (Fletcher-8)

校验覆盖整帧所有字节, 包括帧头 `0xAA`。逐字节遍历, 使用双累加器:

```
sumcheck = 0
addcheck = 0
for b in frame_bytes:
    sumcheck = (sumcheck + b) & 0xFF
    addcheck = (addcheck + sumcheck) & 0xFF
SC = sumcheck
AC = addcheck
```

- **SC (和校验):** 所有字节的累加和, 取低8位。
- **AC (累加校验):** 每一步的运行累加和再累加, 取低8位。

> AC 不是简单的 `SC & 0xFF`, 而是逐步累加 `sumcheck` 的中间值。

### 2.3 命令列表

所有命令使用固定参数: `D_ADDR=0xFF` (广播), `ID=0xE0`。

#### 2.3.1 模式切换 cmd_mode(mode)


| 字段 | 值                                    |
| ------ | --------------------------------------- |
| CID  | `0x01`                                |
| CMD  | `0x01`                                |
| LEN  | `0x0B` (11字节)                        |
| DATA | `01 01 01 [MODE] 00 00 00 00 00 00 00` |

**MODE 取值:**


| 值 | 模式 | 说明              |
| ---- | ------ | ------------------- |
| 0  | 自稳 | 飞手直接控制姿态  |
| 1  | 定高 | 气压计+超声波定高 |
| 2  | 定点 | GPS定点悬停       |
| 3  | 程控 | 自主航线飞行      |

帧示例 (模式=3, 程控):

```
AA FF E0 0B 01 01 01 03 00 00 00 00 00 00 [SC] [AC]
```

#### 2.3.2 解锁电机 cmd_unlock()


| 字段 | 值                                  |
| ------ | ------------------------------------- |
| CID  | `0x10`                              |
| CMD  | `0x00`                              |
| LEN  | `0x0B` (11字节)                      |
| DATA | `10 00 01 00 00 00 00 00 00 00 00`  |

帧: `AA FF E0 0B 10 00 01 00 00 00 00 00 00 00 [SC] [AC]`

#### 2.3.3 加锁电机 cmd_lock()


| 字段 | 值                                  |
| ------ | ------------------------------------- |
| CID  | `0x10`                              |
| CMD  | `0x00`                              |
| LEN  | `0x0B` (11字节)                      |
| DATA | `10 00 02 00 00 00 00 00 00 00 00`  |

帧: `AA FF E0 0B 10 00 02 00 00 00 00 00 00 00 [SC] [AC]`

#### 2.3.4 起飞 cmd_takeoff(h)


| 字段 | 值                                               |
| ------ | -------------------------------------------------- |
| CID  | `0x10`                                           |
| CMD  | `0x00`                                           |
| LEN  | `0x0B` (11字节)                                   |
| DATA | `10 00 05 [H_LO] [H_HI] 00 00 00 00 00 00`                |

- `h`: 目标高度, 单位 cm, u16 小端序 (LE)
- `h=0` 或省略时默认 150cm

示例 (起飞至200cm):

```
h = 200 → H_LO=0xC8, H_HI=0x00
AA FF E0 0B 10 00 05 C8 00 00 00 00 00 00 00 [SC] [AC]
```

#### 2.3.5 降落 cmd_land()


| 字段 | 值                                  |
| ------ | ------------------------------------- |
| CID  | `0x10`                              |
| CMD  | `0x00`                              |
| LEN  | `0x0B` (11字节)                      |
| DATA | `10 00 06 00 00 00 00 00 00 00 00`  |

帧: `AA FF E0 0B 10 00 06 00 00 00 00 00 00 00 [SC] [AC]`

#### 2.3.6 水平移动 cmd_move(d, s, a)


| 字段 | 值                                                                          |
| ------ | ----------------------------------------------------------------------------- |
| CID  | `0x10`                                                                      |
| CMD  | `0x02`                                                                      |
| LEN  | `0x0B` (11字节)                                                              |
| DATA | `10 02 03 [D_LO] [D_HI] [S_LO] [S_HI] [A_LO] [A_HI] 00 00`               |


| 参数     | 范围      | 单位 | 编码   |
| ---------- | ----------- | ------ | -------- |
| d (距离) | 0 ~ 10000 | cm   | u16 LE |
| s (速度) | 10 ~ 300  | cm/s | u16 LE |
| a (航向) | 0 ~ 359   | 度   | u16 LE |

示例 (向45°方向移动500cm, 速度100cm/s):

```
d=500  → 0xF4 0x01
s=100  → 0x64 0x00
a=45   → 0x2D 0x00
AA FF E0 0B 10 02 03 F4 01 64 00 2D 00 00 00 [SC] [AC]
```

#### 2.3.7 上升 cmd_ascend(h, s)


| 字段 | 值                                                              |
| ------ | ----------------------------------------------------------------- |
| CID  | `0x10`                                                          |
| CMD  | `0x02`                                                          |
| LEN  | `0x0B` (11字节)                                                  |
| DATA | `10 02 01 [H_LO] [H_HI] [S_LO] [S_HI] 00 00 00 00`         |


| 参数       | 范围      | 单位 | 编码   |
| ------------ | ----------- | ------ | -------- |
| h (高度差) | 0 ~ 10000 | cm   | u16 LE |
| s (速度)   | 10 ~ 300  | cm/s | u16 LE |

#### 2.3.8 下降 cmd_descend(h, s)


| 字段 | 值                                                              |
| ------ | ----------------------------------------------------------------- |
| CID  | `0x10`                                                          |
| CMD  | `0x02`                                                          |
| LEN  | `0x0B` (11字节)                                                  |
| DATA | `10 02 02 [H_LO] [H_HI] [S_LO] [S_HI] 00 00 00 00`         |

参数同上升。

---

## 3. 主控↔MCU 串口协议 (mcu_serial.py)

### 3.1 物理层


| 属性   | 值                     |
| -------- | ------------------------ |
| 接口   | USB-TTL (CP2102/CH340) |
| 波特率 | 115200 bps             |
| 数据位 | 8                      |
| 停止位 | 1                      |
| 校验   | 无                     |
| 电平   | 3.3V CMOS              |

### 3.2 主控 → MCU 帧

#### 类型 A：IMU 指令转发帧

主控将凌霄IMU API帧封装后发送给MCU, 由MCU转发至IMU。

```
 0    1       2    3..3+CMD_LEN-1  3+CMD_LEN  4+CMD_LEN
┌────┬────────┬────┬───────────────┬──────────┬──────────┐
│ 0xAA│CMD_LEN │ 0x01│ IMU_API_FRAME  │ SUM_LO   │ SUM_HI   │
└────┴────────┴────┴───────────────┴──────────┴──────────┘
  帧头   长度     标识    IMU帧(含校验)      校验低字节   校验高字节
```


| 字段          | 长度  | 说明                          |
| --------------- | ------- | ------------------------------- |
| 帧头          | 1B    | 固定`0xAA`                    |
| CMD_LEN       | 1B    | IMU_API_FRAME 的字节长度      |
| 标识          | 1B    | 固定`0x01`, 表示IMU转发       |
| IMU_API_FRAME | N字节 | 完整的凌霄IMU帧 (含SC/AC校验) |
| SUM           | 2B    | u16 LE, 所有字节之和 & 0xFFFF |

**SUM 计算:**

```
SUM = (0xAA + CMD_LEN + 0x01 + IMU_API_FRAME[0] + ... + IMU_API_FRAME[N-1]) & 0xFFFF
```

#### 类型 B：查询帧

```
 0    1
┌────┬────┐
│ 0xBB│CMD │
└────┴────┘
```


| CMD    | 含义         | 说明                 |
| -------- | -------------- | ---------------------- |
| `0x01` | 请求光流位置 | MCU回复 0xCC 0x01 帧 |
| `0x02` | 请求飞行状态 | MCU回复 0xCC 0x02 帧 |
| `0x03` | 重置光流零点 | MCU执行后回复确认    |
| `0x04` | 心跳         | 维持连接, 间隔 500ms |

### 3.3 MCU → 主控 回传帧

所有回传帧以 `0xCC` 开头。

```
 0    1    2..2+LEN-1
┌────┬────┬──────────┐
│ 0xCC│SUB_CMD│ PAYLOAD   │
└────┴────┴──────────┘
  帧头   子命令   载荷数据
```

#### 0x01: 光流位置


| 偏移 | 字段    | 类型 | 大小 | 说明                 |
| ------ | --------- | ------ | ------ | ---------------------- |
| 0    | POS_X   | s32  | 4B   | X轴位移, 单位 cm, LE |
| 4    | POS_Y   | s32  | 4B   | Y轴位移, 单位 cm, LE |
| 8    | QUALITY | u8   | 1B   | 光流质量 (0~255)     |

总帧长度: 11B

```
CC 01 [X0] [X1] [X2] [X3] [Y0] [Y1] [Y2] [Y3] [QUALITY]
       ←── s32 LE ──→    ←── s32 LE ──→
```

#### 0x02: 飞行状态


| 偏移 | 字段   | 类型 | 大小 | 说明                          |
| ------ | -------- | ------ | ------ | ------------------------------- |
| 0    | MODE   | u8   | 1B   | 当前飞行模式                  |
| 1    | LOCKED | u8   | 1B   | 电机锁定状态 (0=锁定, 1=解锁) |
| 2    | ALT    | s32  | 4B   | 当前高度, 单位 cm, LE         |

总帧长度: 8B

MODE取值: 0=自稳, 1=定高, 2=定点, 3=程控

#### 0x03: 电池电压


| 偏移 | 字段    | 类型 | 大小 | 说明                  |
| ------ | --------- | ------ | ------ | ----------------------- |
| 0    | VOLTAGE | u16  | 2B   | 电池电压, 单位 mV, LE |

总帧长度: 4B

示例 (电压 12.4V):

```
VOLTAGE = 12400 → 0x3064 → LE: 0x64 0x30
CC 03 64 30
```

---

## 4. STM32H7 GPIO 协议 (h7_gpio_protocol.py)

### 4.1 帧格式

#### 命令帧 (主控 → H7)

```
 0    1    2    3    4..4+LEN-1  4+LEN
┌────┬────┬────┬────┬───────────┬──────┐
│ 0xAA│PIN │ CMD │ LEN │ PAYLOAD    │ XOR  │
└────┴────┴────┴────┴───────────┴──────┘
  帧头   引脚  命令  长度    载荷数据     异或校验
```

#### 响应帧 (H7 → 主控)

```
 0    1    2    3    4
┌────┬────┬────┬────┬────┐
│ 0xBB│PIN │ CMD │STATUS│ XOR  │
└────┴────┴────┴────┴────┘
  帧头   引脚  命令  状态    异或校验
```

**XOR 校验覆盖范围:** `PIN ⊕ CMD ⊕ LEN ⊕ PAYLOAD[0] ⊕ ... ⊕ PAYLOAD[N-1]`
**不包含帧头** (`0xAA` 或 `0xBB`)。

### 4.2 命令列表

#### CMD 0x01: SET_OUTPUT — 设置输出值


| 方向 | 字段    | 值                                      |
| ------ | --------- | ----------------------------------------- |
| 命令 | PIN     | 目标引脚编号                            |
| 命令 | CMD     | `0x01`                                  |
| 命令 | LEN     | `0x01`                                  |
| 命令 | PAYLOAD | `[VALUE: 0x00或0x01]`                   |
| 响应 | STATUS  | `0x00`=OK, `0x01`=ERROR, `0x02`=TIMEOUT |

帧示例 (PIN=17, 输出高):

```
AA 11 01 01 01 [XOR]
    ↕  ↕  ↕  ↕
   PIN CMD LEN VALUE
```

XOR = `0x11 ⊕ 0x01 ⊕ 0x01 ⊕ 0x01` = `0x10`

响应: `BB 11 01 00 [XOR]`

#### CMD 0x02: CONFIGURE — 配置引脚方向


| 方向 | 字段    | 值                                              |
| ------ | --------- | ------------------------------------------------- |
| 命令 | PIN     | 目标引脚编号                                    |
| 命令 | CMD     | `0x02`                                          |
| 命令 | LEN     | `0x01`                                          |
| 命令 | PAYLOAD | `[MODE: 0x00=输入, 0x01=输出]`                 |
| 响应 | STATUS  | `0x00`=OK, `0x01`=ERROR, `0x02`=TIMEOUT         |

帧示例 (PIN=17, 设为输出):

```
AA 11 02 01 01 [XOR]
```

XOR = `0x11 ⊕ 0x02 ⊕ 0x01 ⊕ 0x01` = `0x13`

#### CMD 0x03: PULSE — 硬件定时脉冲


| 方向 | 字段    | 值                                                 |
| ------ | --------- | ---------------------------------------------------- |
| 命令 | PIN     | 目标引脚编号                                       |
| 命令 | CMD     | `0x03`                                             |
| 命令 | LEN     | `0x03`                                             |
| 命令 | PAYLOAD | `[COUNT] [PERIOD_LO] [PERIOD_HI]`                  |
| 响应 | **无**  | 发出后不等待响应                         |


| 参数   | 说明                      |
| -------- | --------------------------- |
| COUNT  | 脉冲次数                  |
| PERIOD | 脉冲周期, u16 LE, 单位 ms |

帧示例 (PIN=17, 10次脉冲, 周期50ms):

```
AA 11 03 03 0A 32 00 [XOR]
    ↕  ↕  ↕  ↕  ← u16 LE ─→
   PIN CMD LEN CNT  PERIOD
```

XOR = `0x11 ⊕ 0x03 ⊕ 0x03 ⊕ 0x0A ⊕ 0x32` = `0x29`

### 4.3 STATUS 码


| 值     | 含义    | 说明               |
| -------- | --------- | -------------------- |
| `0x00` | OK      | 命令执行成功       |
| `0x01` | ERROR   | 引脚无效或参数错误 |
| `0x02` | TIMEOUT | 命令执行超时       |

---

## 5. GPIO 后端抽象接口 (gpio_backend.py)

### 5.1 GpioBackend 抽象基类

```python
class GpioBackend(ABC):
    """GPIO后端抽象接口, 所有硬件后端必须实现以下方法。"""

    @abstractmethod
    def setup(self, pin: int, mode: GpioMode) -> None:
        """配置引脚模式。
      
        Args:
            pin: 引脚编号 (BCM/物理编号取决于后端)
            mode: GpioMode.IN (输入) 或 GpioMode.OUT (输出)
        """

    @abstractmethod
    def output(self, pin: int, value: GpioValue) -> None:
        """设置引脚输出值。
      
        Args:
            pin: 引脚编号
            value: GpioValue.LOW (低电平) 或 GpioValue.HIGH (高电平)
        """

    @abstractmethod
    def cleanup(self, pin: int) -> None:
        """清理引脚资源。
      
        Args:
            pin: 引脚编号
        """
```

### 5.2 后端实现对比


| 后端        | 类名               | 适用平台       | 硬件脉冲     | 依赖     |
| ------------- | -------------------- | ---------------- | -------------- | ---------- |
| 树莓派 GPIO | `RpiGpioBackend`   | Raspberry Pi   | 软件模拟     | RPi.GPIO |
| FT232H      | `Ft232hBackend`    | 任意 (USB)     | 软件模拟     | pyftdi   |
| STM32H7     | `H7GpioBackend`    | 任意 (USB串口) | **硬件定时** | 串口通信 |
| 模拟        | `DummyGpioBackend` | 任意           | 软件模拟     | 无       |

### 5.3 各后端详情

#### RpiGpioBackend

使用 BCM 编号方案。直接操作树莓派 GPIO 引脚。

```python
backend = RpiGpioBackend()
backend.setup(17, 'out')
backend.output(17, 1)
backend.cleanup(17)
```

#### Ft232hBackend

通过 USB 连接 FT232H 芯片, 使用 pyftdi 库。适用于非树莓派平台。

```python
backend = Ft232hBackend(ftdi_url='ftdi://ftdi:232h/1')
backend.setup(0, 'out')   # ADBUS0
backend.output(0, 1)
backend.cleanup(0)
```

#### H7GpioBackend

通过串口连接 STM32H7, 使用第4节定义的协议。支持硬件定时脉冲。

```python
backend = H7GpioBackend(port='/dev/ttyUSB1')
backend.setup(17, 'out')
backend.output(17, 1)
# 硬件脉冲: 引脚17, 10次, 周期50ms
backend.pulse(17, count=10, period_ms=50)
backend.cleanup(17)
```

**pulse 方法:**

```python
def pulse(self, pin: int, count: int, period_ms: int) -> None:
    """发送硬件定时脉冲。
  
    通过 STM32H7 定时器产生精确脉冲, CPU不参与计时。
  
    Args:
        pin: 引脚编号
        count: 脉冲次数
        period_ms: 脉冲周期 (ms)
    """
```

#### DummyGpioBackend

用于测试和模拟。所有操作仅记录日志, 不操作硬件。

```python
backend = DummyGpioBackend()
backend.setup(17, 'out')
backend.output(17, 1)   # 仅打印日志
```

---

## 6. 软件接口速查

### 6.1 MCUSerial (mcu_serial.py)

主控与 STM32F4 MCU 的通信封装。


| 方法                     | 参数                                          | 返回                   | 说明                  |
| -------------------------- | ----------------------------------------------- | ------------------------ | ----------------------- |
| `connect()`              | —                                            | `bool`                 | 打开串口连接          |
| `disconnect()`           | —                                            | `None`                 | 关闭串口连接          |
| `poll()`                 | —                                            | `dict`                 | 读取并解析MCU回传数据 |
| `send_cmd_unlock()`      | —                                            | `bool`                 | 发送解锁电机指令      |
| `send_cmd_lock()`        | —                                            | `bool`                 | 发送加锁电机指令      |
| `send_cmd_mode(mode)`    | `mode: int` (0~3)                             | `bool`                 | 切换飞行模式          |
| `send_cmd_takeoff(h)`    | `h: int` (cm, 0=默认150)                      | `bool`                 | 起飞                  |
| `send_cmd_land()`        | —                                            | `bool`                 | 降落                  |
| `send_cmd_move(d, s, a)` | `d: int` (cm), `s: int` (cm/s), `a: int` (度) | `bool`                 | 水平移动              |
| `send_cmd_ascend(h, s)`  | `h: int` (cm), `s: int` (cm/s)                | `bool`                 | 上升                  |
| `send_cmd_descend(h, s)` | `h: int` (cm), `s: int` (cm/s)                | `bool`                 | 下降                  |
| `send_heartbeat()`       | —                                            | `bool`                 | 发送心跳 (500ms间隔)  |
| `send_of_zero_reset()`   | —                                            | `bool`                 | 重置光流零点          |
| `read_optical_flow()`    | —                                            | `tuple[float, float]`  | 返回增量 (dx, dy) cm  |
| `read_optical_flow_position()` | —                                        | `tuple[float, float, int]` | 返回 (x, y, quality) |
| `read_altitude()`        | —                                            | `int`                  | 返回当前高度 (cm)     |
| `read_voltage()`         | —                                            | `float`                | 返回电池电压 (V)      |
| `is_communication_ok(timeout_ms)` | `timeout_ms: float` (默认500)          | `bool`                 | 通信状态检查          |

### 6.2 H7GpioSerial (h7_gpio_protocol.py)

与 STM32H7 GPIO 开发板的串口通信封装。


| 方法                            | 参数                                     | 返回   | 说明                       |
| --------------------------------- | ------------------------------------------ | -------- | ---------------------------- |
| `connect()`                     | —                                       | `bool` | 打开串口连接               |
| `disconnect()`                  | —                                       | `None` | 关闭串口连接               |
| `send_frame(frame)`             | `frame: bytes` (预构建帧)               | `bool` | 发送命令帧                 |
| `read_response(timeout_s)`      | `timeout_s: float`                       | `dict` | 读取响应帧, 含 status 字段 |

### 6.3 LaserController (laser_led.py)

激光/LED 控制器, 自动检测硬件脉冲支持。


| 方法                      | 参数                                      | 返回   | 说明                              |
| --------------------------- | ------------------------------------------- | -------- | ----------------------------------- |
| `__init__(pin, backend)`  | `pin: int`, `backend: GpioBackend` (可选) | —     | 构造器, 默认 auto_detect_backend() (自动检测 RPi → FT232H → Dummy) |
| `on()`                    | —                                        | `None` | 开启激光/LED                      |
| `off()`                   | —                                        | `None` | 关闭激光/LED                      |
| `blink(count, period_ms)` | `count: int`, `period_ms: int`            | `None` | 闪烁, 自动检测硬件脉冲            |
| `enable()`                | —                                        | `None` | 使能控制器 (调用 setup)           |
| `disable()`               | —                                        | `None` | 禁用控制器                        |
| `cleanup()`               | —                                        | `None` | 清理 GPIO 资源                    |

**blink 自动检测逻辑:**

```
if backend 支持 pulse():
    调用 backend.pulse(pin, count, period_ms)   # 硬件定时
else:
    软件循环 on()/off() + sleep()               # 软件模拟
```

### 6.3 LEDController (laser_led.py)

LED指示灯控制器, 用于显示条码数字 (通过闪烁次数表示数字)。


| 方法                     | 参数                              | 返回   | 说明                    |
| -------------------------- | ----------------------------------- | -------- | ------------------------- |
| `__init__(pin, backend)` | `pin: int`, `backend: GpioBackend` (可选) | —     | 构造器, 默认 auto_detect_backend() |
| `show_number(number)`     | `number: int`                     | `None` | 闪烁LED显示数字         |
| `on()`                    | —                                | `None` | 点亮LED                 |
| `off()`                   | —                                | `None` | 熄灭LED                 |
| `cleanup()`               | —                                | `None` | 清理 GPIO 资源          |

### 6.4 配置入口 (config.py)


| 配置项           | 默认值           | 说明                |
| ------------------ | ------------------ | --------------------- |
| `SERIAL_PORT`    | `'/dev/ttyUSB0'` | MCU (STM32F4) 串口  |
| `H7_SERIAL_PORT` | `'/dev/ttyUSB1'` | GPIO (STM32H7) 串口 |
| `VISION_BACKEND` | `'industrial'` | 视觉后端选择       |
| `OPENMV_SERIAL_PORT` | `'/dev/ttyUSB1'` | OpenMV结果串口  |
| `OPENMV_SERIAL_BAUDRATE` | `115200` | OpenMV波特率     |
| `LASER_PIN`      | `17`             | 激光引脚编号        |
| `LED_PIN`        | `27`             | LED引脚编号         |

### 6.5 CLI 参数 (main.py)

```
python main.py [OPTIONS]
```


| 参数                 | 值                                 | 默认         | 说明                     |
| ---------------------- | ------------------------------------ | -------------- | -------------------------- |
| `--profile`          | `debug` / `tuning` / `competition` | —           | 运行配置档               |
| `--serial-port PORT` | 串口路径                           | config.py 值 | MCU串口覆盖              |
| `--h7-serial PORT`   | 串口路径                           | config.py 值 | H7串口覆盖               |
| `--vision-backend`   | `industrial` / `openmv`           | config.py 值 | 选择视觉后端             |
| `--openmv-port PORT` | 串口路径                           | config.py 值 | OpenMV串口覆盖           |
| `--openmv-baudrate N` | 波特率                            | config.py 值 | OpenMV波特率覆盖         |
| `--no-camera`        | —                                  | `False`      | 禁用所有视觉后端         |
| `--dry-run`          | —                                 | `False`      | 模拟模式, 不发送实际指令 |
| `--verbose`          | —                                 | `False`      | 详细日志输出             |
| `--no-save-logs`     | —                                 | `False`      | 不保存日志文件           |

---

## 7. OpenMV识别结果协议

OpenMV 完成图像采集、绿色区域检测和区块数字识别。上位机不接收图像，只接收以下 ASCII 结果帧：

```text
$OMV1,<SEQUENCE>,<GREEN_PER_MILLE>,<DIGIT>*<XOR>\r\n
```

| 字段 | 范围 | 说明 |
| ---- | ---- | ---- |
| `SEQUENCE` | `0..65535` | 帧序号，溢出后回到 0 |
| `GREEN_PER_MILLE` | `0..1000` | 绿色像素占比千分数 |
| `DIGIT` | `-1` 或 `1..28` | 区块编号，`-1` 表示未识别 |
| `XOR` | `00..FF` | `$` 和 `*` 之间 ASCII 字节的异或校验 |

示例：

```text
$OMV1,42,731,21*79\r\n
```

上位机以非阻塞方式解析数据，拒绝校验错误、字段缺失和数值越界的帧。最新有效结果可缓存 `OPENMV_STALE_TIMEOUT_S`，默认 0.5 秒；超时后视为无有效视觉结果。

OpenMV 端参考程序和模板标定说明位于 `drone/openmv/`。

---

## 8. 附录: MCU固件接口要求

### 8.1 STM32F4 (凌霄飞控) 固件要求

#### 8.1.1 串口接收配置


| 属性     | 值       |
| ---------- | ---------- |
| UART名称 | UART_PI  |
| 波特率   | 115200   |
| 数据位   | 8        |
| 停止位   | 1        |
| 校验     | 无       |
| 中断     | 接收中断 |

#### 8.1.2 帧解析

固件需解析两种帧类型:

- **`0xAA` 帧**: IMU指令转发帧, 提取内嵌的凌霄IMU API帧并转发至 UART1 (IMU)
- **`0xBB` 帧**: 查询帧, 根据 CMD 字段执行对应操作并回复

#### 8.1.3 光流回传


| 属性     | 值                                                           |
| ---------- | -------------------------------------------------------------- |
| 发送间隔 | 200ms (5Hz)                                                  |
| 帧格式   | `0xCC 0x01 [POS_X 4B s32 LE] [POS_Y 4B s32 LE] [QUALITY 1B]` |
| 数据源   | UART4 光流模块数据经解算后填入                               |

#### 8.1.4 心跳检测


| 属性     | 值                              |
| ---------- | --------------------------------- |
| 检测间隔 | 2秒                             |
| 超时触发 | 自动紧急降落                    |
| 心跳帧   | `0xBB 0x04` (由主控每500ms发送) |

固件内部维护心跳计时器。若连续 2 秒未收到 `0xBB 0x04` 帧, 触发以下流程:

1. 停止所有水平移动指令
2. 执行原地降落
3. 加锁电机
4. 上报心跳丢失错误

### 8.2 STM32H7 (GPIO扩展板) 固件要求

#### 8.2.1 串口接收配置


| 属性   | 值           |
| -------- | -------------- |
| UART   | 任意可用UART |
| 波特率 | 115200       |
| 数据位 | 8            |
| 停止位 | 1            |
| 校验   | 无           |
| 中断   | 接收中断     |

#### 8.2.2 帧解析

接收 `0xAA` 帧, 验证 XOR 校验后执行对应命令:


| 命令              | 操作                     | 响应                           |
| ------------------- | -------------------------- | -------------------------------- |
| SET_OUTPUT (0x01) | 将指定引脚设为高/低电平  | 回复`0xBB` 帧, STATUS=OK/ERROR |
| CONFIGURE (0x02)  | 设置引脚方向 (输入/输出) | 回复`0xBB` 帧, STATUS=OK/ERROR |
| PULSE (0x03)      | 启动硬件定时器脉冲输出   | **无回复** (点火即忘)          |

#### 8.2.3 SET_OUTPUT 实现要求

- 解析 PAYLOAD 中的 VALUE 字段 (0x00 或 0x01)
- 直接操作对应 GPIO 引脚输出电平
- 引脚未配置为输出模式时, 回复 STATUS=0x01 (ERROR)

#### 8.2.4 CONFIGURE 实现要求

- 解析 PAYLOAD 中的 MODE 字段 (0x00=输入, 0x01=输出)
- 调用 HAL GPIO 初始化函数设置引脚方向
- 引脚编号超出范围时, 回复 STATUS=0x01 (ERROR)

#### 8.2.5 PULSE 实现要求

- 使用硬件定时器 (TIM) 产生脉冲
- COUNT: 脉冲次数
- PERIOD: u16 LE, 单位 ms, 为半个周期 (高电平+低电平各占 PERIOD ms)
- 脉冲期间引脚电平翻转由硬件自动完成, CPU不参与
- 执行完毕后引脚保持当前电平

```
时序示意:
        ┌──┐  ┌──┐  ┌──┐
PIN:  ──┘  └──┘  └──┘  └──
        ←T→  ←T→  ←T→
        COUNT=3, T=PERIOD
```

#### 8.2.6 响应帧格式

所有命令 (除 PULSE) 必须回复 `0xBB` 帧:

```
BB [PIN] [CMD] [STATUS] [XOR]
```

XOR = `PIN ⊕ CMD ⊕ STATUS`

---

## 字节序汇总


| 数据类型 | 编码        | 说明                                |
| ---------- | ------------- | ------------------------------------- |
| u16      | 小端序 (LE) | 低字节在前, 如 0x01C8 →`C8 01`     |
| s32      | 小端序 (LE) | 低字节在前, 如 -500 →`0C FE FF FF` |
| u8       | 单字节      | 无需字节序                          |

所有多字节整数均采用 **小端序 (Little-Endian)** 编码。

---

> 文档结束
