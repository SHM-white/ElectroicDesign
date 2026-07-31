"""Serial transport for the H7 GPIO board using the package's exclusive TTY owner.

复用 ed_uav_fcu_bridge.ExclusiveSerialPort 的独占串口管理 (flock + TIOCEXCL + raw TTY),
为 H7 GPIO 0xAA 协议提供帧发送与超时响应读取。

C 板 (STM32H7 大疆电机开发板C) 通过 USB-TTL 连接, 默认 /dev/ttyUSB1 @ 115200。
"""

from __future__ import annotations

import logging
import select
import time
from pathlib import Path
from typing import Final

from .h7_gpio_protocol import (
    H7GpioResponse,
    RESPONSE_HEADER,
    parse_response,
)
from .serial_port import ExclusiveSerialPort, SerialOpenError

logger = logging.getLogger("ed_uav_fcu_bridge.h7_gpio")

RESPONSE_LENGTH: Final = 5  # BB [PIN] [CMD] [STATUS] [XOR]
RESPONSE_TAIL_LENGTH: Final = RESPONSE_LENGTH - 1


class H7GpioTransport:
    """Own one H7 GPIO serial endpoint and send/read 0xAA frames."""

    def __init__(
        self,
        device: str = "/dev/ttyUSB1",
        baudrate: int = 115200,
        lock_dir: Path = Path("/tmp"),
    ) -> None:
        self._port = ExclusiveSerialPort(device, baudrate, lock_dir)
        self._buffer = bytearray()

    def open(self) -> None:
        """Open the TTY exclusively; raises SerialOpenError on failure."""
        self._port.open()
        logger.info("H7 GPIO serial open: %s @ %d", self._port.device, self._port.baudrate)

    def close(self) -> None:
        self._port.close()
        self._buffer.clear()

    @property
    def is_open(self) -> bool:
        return self._port.is_open

    def send(self, frame: bytes) -> None:
        """Write one command frame; raises SerialOpenError on short write."""
        logger.debug("H7 GPIO TX: %s", frame.hex(" ").upper())
        self._port.write(frame)

    def read_response(self, timeout_s: float = 0.5) -> H7GpioResponse | None:
        """Read one complete response frame within timeout, or None on timeout."""
        deadline = time.monotonic() + timeout_s
        while True:
            response = self._take_response()
            if response is not None:
                return response
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            readable, _, _ = select.select([self._port.fileno], [], [], remaining)
            if not readable:
                return None
            try:
                chunk = self._port.read()
            except BlockingIOError:
                continue
            if chunk:
                self._buffer.extend(chunk)

    def _take_response(self) -> H7GpioResponse | None:
        """Parse a complete 0xBB frame from the buffer, dropping stale bytes."""
        while self._buffer:
            header = self._buffer.find(bytes((RESPONSE_HEADER,)))
            if header < 0:
                self._buffer.clear()
                return None
            if header > 0:
                del self._buffer[:header]
            if len(self._buffer) < RESPONSE_LENGTH:
                return None
            frame = bytes(self._buffer[:RESPONSE_LENGTH])
            del self._buffer[:RESPONSE_LENGTH]
            parsed = parse_response(frame)
            if parsed is not None:
                logger.debug("H7 GPIO RX: %s", frame.hex(" ").upper())
                return parsed
            # 校验失败: 丢弃该字节继续寻找下一个 0xBB 帧头。
            del self._buffer[:1]
        return None


def open_h7_gpio(device: str = "/dev/ttyUSB1", baudrate: int = 115200) -> H7GpioTransport:
    """Convenience factory: open the H7 GPIO transport or raise SerialOpenError."""
    transport = H7GpioTransport(device=device, baudrate=baudrate)
    transport.open()
    return transport
