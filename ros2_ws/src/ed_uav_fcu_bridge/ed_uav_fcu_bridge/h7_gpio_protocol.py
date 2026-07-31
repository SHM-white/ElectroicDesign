"""Pure encoder and decoder for the H7 GPIO board 0xAA protocol.

STM32H7 大疆电机开发板C (C 板) 沿用旧激光头控制协议:
  Command:  0xAA [PIN] [CMD] [LEN] [PAYLOAD...] [XOR]
  Response: 0xBB [PIN] [CMD] [STATUS] [XOR]

XOR 校验覆盖除帧头外的全部字节 (pin + cmd + len + payload, 响应为 pin + cmd + status)。
C 板固件无需改动, 该模块仅提供帧编解码, 不依赖任何 ROS API。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Final

FRAME_HEADER: Final = 0xAA
RESPONSE_HEADER: Final = 0xBB
MAX_PAYLOAD_BYTES: Final = 8
MAX_PIN: Final = 15

CMD_SET_OUTPUT: Final = 0x01  # 载荷: [VALUE 1B] 1=HIGH, 0=LOW
CMD_CONFIGURE: Final = 0x02   # 载荷: [MODE 1B] 1=OUTPUT, 0=INPUT
CMD_PULSE: Final = 0x03       # 载荷: [COUNT 1B] [PERIOD 2B u16 LE]


class H7GpioFrameError(ValueError):
    """Raised when frame bytes are malformed or checksum-invalid."""


@dataclass(frozen=True, slots=True)
class H7GpioResponse:
    """A checksum-verified 5-byte H7 GPIO response frame."""

    pin: int
    command: int
    status: int
    raw: bytes


def build_command(pin: int, command: int, payload: bytes) -> bytes:
    """Encode one H7 GPIO command frame: AA [PIN] [CMD] [LEN] [PAYLOAD...] [XOR]."""
    if not 0 <= pin <= MAX_PIN:
        raise ValueError(f"pin must be 0-{MAX_PIN}, got {pin}")
    if not 0 <= command <= 0xFF:
        raise ValueError(f"command must fit in one byte, got {command}")
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError(
            f"payload length must be <= {MAX_PAYLOAD_BYTES}, got {len(payload)}"
        )
    prefix = bytes((FRAME_HEADER, pin, command, len(payload))) + payload
    checksum = _xor_checksum(prefix[1:])
    return prefix + bytes((checksum,))


def cmd_set_output(pin: int, high: bool) -> bytes:
    """Encode SET_OUTPUT command; high=True 为高电平 (吸合), False 为低电平 (释放)."""
    return build_command(pin, CMD_SET_OUTPUT, b"\x01" if high else b"\x00")


def cmd_configure(pin: int, as_output: bool) -> bytes:
    """Encode CONFIGURE command; as_output=True 配置为输出."""
    return build_command(pin, CMD_CONFIGURE, b"\x01" if as_output else b"\x00")


def cmd_pulse(pin: int, count: int, period_ms: int) -> bytes:
    """Encode PULSE command: [COUNT 1B] [PERIOD 2B u16 LE]."""
    period = max(0, min(period_ms, 65535))
    payload = bytes((count & 0xFF,)) + struct.pack("<H", period)
    return build_command(pin, CMD_PULSE, payload)


def parse_response(frame: bytes) -> H7GpioResponse | None:
    """Parse and verify one H7 GPIO response frame (BB [PIN] [CMD] [STATUS] [XOR]).

    Returns None when the buffer is not a valid 5-byte response frame.
    """
    if frame is None or len(frame) != 5 or frame[0] != RESPONSE_HEADER:
        return None
    if _xor_checksum(frame[1:4]) != frame[4]:
        return None
    return H7GpioResponse(
        pin=frame[1],
        command=frame[2],
        status=frame[3],
        raw=bytes(frame),
    )


def verify_checksum(frame: bytes) -> bool:
    """Return True when the trailing XOR byte matches the payload bytes.

    XOR 覆盖除帧头 (0xAA/0xBB) 外的全部内容字节, 即 frame[1:-1]。
    """
    if len(frame) < 2:
        return False
    return _xor_checksum(frame[1:-1]) == frame[-1]


def _xor_checksum(data: bytes) -> int:
    checksum = 0
    for value in data:
        checksum ^= value
    return checksum


def format_frame(frame: bytes) -> str:
    """Render a frame as space-separated uppercase hex (debug/tests)."""
    return " ".join(f"{b:02X}" for b in frame)
