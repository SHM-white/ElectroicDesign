"""
h7_gpio_protocol.py — STM32H7 GPIO 串口通信协议模块

协议格式:
  Command:  0xAA [PIN] [CMD] [LEN] [PAYLOAD...] [XOR]
  Response: 0xBB [PIN] [CMD] [STATUS] [XOR]

CMD:
  0x01 = SET_OUTPUT  —  [VALUE: 1=HIGH, 0=LOW]
  0x02 = CONFIGURE   —  [MODE: 1=OUTPUT, 0=INPUT]
  0x03 = PULSE       —  [COUNT] [PERIOD_LO] [PERIOD_HI]
"""

import struct
from typing import Optional


def build_h7_command(pin: int, cmd: int, payload: bytes) -> bytes:
    """
    构建 H7 GPIO 命令帧

    帧格式:
        AA [PIN] [CMD] [LEN] [PAYLOAD...] [XOR]

    XOR 校验覆盖: PIN + CMD + LEN + PAYLOAD (不含帧头 0xAA)

    Args:
        pin: GPIO 引脚 0-15
        cmd: 命令类型 (0x01=SET_OUTPUT, 0x02=CONFIGURE, 0x03=PULSE)
        payload: 载荷数据 (最多 8 字节)

    Returns:
        完整命令帧字节序列

    Raises:
        ValueError: pin 不在 0-15 范围内, 或 payload 长度超过 8
    """
    if not (0 <= pin <= 15):
        raise ValueError(f"pin must be 0-15, got {pin}")
    if len(payload) > 8:
        raise ValueError(f"payload length must be <= 8, got {len(payload)}")

    buf = bytes([0xAA, pin, cmd, len(payload)]) + payload

    # XOR over pin + cmd + len + payload (skip 0xAA header)
    xor = 0
    for b in buf[1:]:
        xor ^= b

    buf += bytes([xor])
    return buf


def parse_h7_response(buf: bytes | None) -> Optional[dict]:
    """
    解析 H7 响应帧

    帧格式:
        BB [PIN] [CMD] [STATUS] [XOR]

    XOR 校验覆盖: PIN + CMD + STATUS (不含帧头 0xBB)

    Args:
        buf: 响应帧字节序列

    Returns:
        {'pin': int, 'cmd': int, 'status': int} 或 None
    """
    if buf is None:
        return None
    if len(buf) < 5:
        return None
    if buf[0] != 0xBB:
        return None

    # XOR over PIN + CMD + STATUS
    xor = 0
    for b in buf[1:4]:
        xor ^= b

    if xor != buf[4]:
        return None

    return {
        'pin': buf[1],
        'cmd': buf[2],
        'status': buf[3],
    }


def cmd_set_output(pin: int, high: bool) -> bytes:
    """
    构建 SET_OUTPUT 命令帧

    Args:
        pin: GPIO 引脚 0-15
        high: True=HIGH, False=LOW

    Returns:
        完整命令帧字节序列
    """
    return build_h7_command(pin, 0x01, b'\x01' if high else b'\x00')


def cmd_configure(pin: int, as_output: bool) -> bytes:
    """
    构建 CONFIGURE 命令帧

    Args:
        pin: GPIO 引脚 0-15
        as_output: True=OUTPUT, False=INPUT

    Returns:
        完整命令帧字节序列
    """
    return build_h7_command(pin, 0x02, b'\x01' if as_output else b'\x00')


def cmd_pulse(pin: int, count: int, period_ms: int) -> bytes:
    """
    构建 PULSE 命令帧

    载荷: [COUNT 1B] [PERIOD 2B u16 LE]

    Args:
        pin: GPIO 引脚 0-15
        count: 脉冲计数 (0-255)
        period_ms: 脉冲周期 (ms, 0-65535, 超出则钳位)

    Returns:
        完整命令帧字节序列
    """
    period = max(0, min(period_ms, 65535))
    payload = bytes([count & 0xFF]) + struct.pack('<H', period)
    return build_h7_command(pin, 0x03, payload)


def verify_h7_frame(frame: bytes) -> bool:
    """
    验证 H7 帧的 XOR 校验和

    覆盖命令帧 (0xAA) 和响应帧 (0xBB)。

    XOR 覆盖: frame[1:-1] 的逐字节异或 → frame[-1]

    Args:
        frame: 待验证的帧字节序列

    Returns:
        True if checksum is correct
    """
    if len(frame) < 5:
        return False

    xor = 0
    for b in frame[1:-1]:
        xor ^= b

    return xor == frame[-1]


def print_frame(frame: bytes, label: str = "") -> None:
    """以十六进制格式打印帧（调试用）"""
    hex_str = " ".join(f"{b:02X}" for b in frame)
    if label:
        print(f"[{label}] ({len(frame)}B) {hex_str}")
    else:
        print(f"({len(frame)}B) {hex_str}")
