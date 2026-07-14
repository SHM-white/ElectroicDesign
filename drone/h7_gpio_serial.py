"""
h7_gpio_serial.py — STM32H7 GPIO 串口通信模块

轻量级同步串口封装，用于与 H7 GPIO 板通信。

协议格式:
  Command:  0xAA [PIN] [CMD] [LEN] [PAYLOAD...] [XOR]
  Response: 0xBB [PIN] [CMD] [STATUS] [XOR]  (共5字节)

使用方式:
    ser = H7GpioSerial(dry_run=False, port='/dev/ttyUSB1')
    ser.connect()
    ser.send_frame(cmd_set_output(3, True))
    resp = ser.read_response(timeout_s=0.5)
    ser.disconnect()
"""

import logging
import time
from typing import Optional

try:
    from .mcu_serial import DummySerial, RealSerial
    from .h7_gpio_protocol import parse_h7_response, print_frame
except ImportError:
    from mcu_serial import DummySerial, RealSerial
    from h7_gpio_protocol import parse_h7_response, print_frame

logger = logging.getLogger('drone.h7_gpio')


class H7GpioSerial:
    """
    H7 GPIO 串口通信管理

    职责:
    - 发送命令帧到 H7 GPIO 板
    - 接收并解析响应帧
    - 连接管理 (支持 dry_run 模拟)
    """

    # 响应帧固定长度: 0xBB + PIN + CMD + STATUS + XOR
    RESPONSE_LEN = 5
    # 响应帧头
    RESPONSE_HEADER = 0xBB

    def __init__(self, dry_run: bool = False,
                 port: str = '/dev/ttyUSB1',
                 baudrate: int = 115200):
        """
        初始化 H7 GPIO 串口

        Args:
            dry_run: True 则使用模拟串口，不连接真实硬件
            port: 串口设备路径
            baudrate: 波特率
        """
        self.dry_run = dry_run
        self.port = port
        self.baudrate = baudrate
        self._ser = None

        logger.info(f"H7GpioSerial initialized (dry_run={dry_run}, port={port}, baud={baudrate})")

    def connect(self) -> bool:
        """
        建立串口连接

        dry_run 模式下使用 DummySerial，否则尝试打开真实串口。
        连接失败时回退到 DummySerial 并返回 False。

        Returns:
            True 连接成功，False 连接失败
        """
        if self.dry_run:
            self._ser = DummySerial()
            logger.info("Using dummy serial (dry run mode)")
            return True

        try:
            self._ser = RealSerial(self.port, self.baudrate)
            logger.info(f"Connected to {self.port} @ {self.baudrate}bps")
            return True
        except Exception as e:
            logger.error(f"Failed to open serial port: {e}")
            self._ser = DummySerial()
            return False

    def disconnect(self):
        """断开串口连接"""
        if self._ser:
            self._ser.close()
            self._ser = None
        logger.info("Disconnected")

    def send_frame(self, frame: bytes) -> bool:
        """
        发送命令帧到 H7 GPIO 板

        Args:
            frame: 完整命令帧字节序列

        Returns:
            True 发送成功，False 发送失败或未连接
        """
        if not self._ser:
            logger.error("Serial not connected")
            return False
        try:
            logger.debug("H7 TX (%dB): %s", len(frame), frame.hex().upper())
            print_frame(frame, 'H7 TX')
            written = self._ser.write(frame)
            try:
                self._ser.flush()
            except AttributeError:
                pass  # DummySerial may not have flush
            return written == len(frame)
        except Exception as e:
            logger.error(f"TX error: {e}")
            return False

    def read_response(self, timeout_s: float = 0.5) -> Optional[dict]:
        """
        同步阻塞读取 H7 响应帧

        逐字节读取，寻找 0xBB 帧头。找到后继续读取剩余 4 字节，
        凑齐 5 字节后调用 parse_h7_response() 解析。

        Args:
            timeout_s: 读取超时时间（秒）

        Returns:
            解析后的响应字典 {'pin': int, 'cmd': int, 'status': int}，
            超时或解析失败返回 None
        """
        if not self._ser:
            logger.error("Serial not connected")
            return None

        deadline = time.monotonic() + timeout_s

        # 第一阶段: 寻找 0xBB 帧头
        while time.monotonic() < deadline:
            try:
                byte = self._ser.read(1)
                if not byte:
                    continue
                if byte[0] == self.RESPONSE_HEADER:
                    break
            except Exception as e:
                logger.warning(f"RX error while searching header: {e}")
                return None
        else:
            # 超时，未找到帧头
            logger.debug(f"Response timeout ({timeout_s}s), no 0xBB header")
            return None

        # 第二阶段: 读取剩余 4 字节 (PIN + CMD + STATUS + XOR)
        buf = bytearray([self.RESPONSE_HEADER])
        remaining_deadline = time.monotonic() + max(timeout_s * 0.3, 0.05)

        while len(buf) < self.RESPONSE_LEN and time.monotonic() < remaining_deadline:
            try:
                byte = self._ser.read(1)
                if not byte:
                    continue
                buf.extend(byte)
            except Exception as e:
                logger.warning(f"RX error while reading payload: {e}")
                return None

        if len(buf) < self.RESPONSE_LEN:
            logger.warning(f"Incomplete response: got {len(buf)}B, need {self.RESPONSE_LEN}B")
            return None

        # 解析响应
        result = parse_h7_response(bytes(buf))
        if result is None:
            logger.warning(f"Invalid response frame: {buf.hex().upper()}")
        return result

    def is_connected(self) -> bool:
        """
        检查串口是否已连接

        Returns:
            True 已连接，False 未连接
        """
        return self._ser is not None
