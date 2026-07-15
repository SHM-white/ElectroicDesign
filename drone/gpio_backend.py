"""
gpio_backend.py — GPIO 硬件抽象层
Section: 跨平台 GPIO 控制 (树莓派 / FT232H / 模拟)

提供统一的 GPIO 后端接口，支持自动检测硬件平台:
- RpiGpioBackend: 树莓派原生 GPIO (RPi.GPIO)
- Ft232hBackend:  FT232H USB-to-GPIO 适配器 (pyftdi)
- H7GpioBackend:   STM32H7 GPIO 开发板 (大疆电机开发板C, 串口协议)
- DummyGpioBackend: 无硬件时的模拟后端 (测试用)

注意: USB-TTL 串口模块 (CH340/CP2102) 仅提供 TX/RX，不支持 GPIO。
如需激光/LED 控制，请选择以下方案之一:
  方案A: STM32H7 GPIO 开发板 (已实现, 使用 --h7-serial 指定串口)
  方案B: FT232H USB-to-GPIO 转接板 (如 Adafruit FT232H)

用法:
    backend = auto_detect_backend()
    backend.setup(17, GpioMode.OUT)
    backend.output(17, GpioValue.HIGH)
"""

import logging
import time
from abc import ABC, abstractmethod
from enum import Enum, auto
from typing import ClassVar

logger = logging.getLogger('drone.gpio')


class GpioMode(Enum):
    """GPIO 引脚模式"""
    IN = auto()   # 输入模式
    OUT = auto()  # 输出模式


class GpioValue(Enum):
    """GPIO 电平值"""
    LOW = 0   # 低电平
    HIGH = 1  # 高电平


class GpioBackend(ABC):
    """GPIO 后端抽象基类"""

    @abstractmethod
    def setup(self, pin: int, mode: GpioMode) -> None:
        """配置引脚模式"""
        ...

    @abstractmethod
    def output(self, pin: int, value: GpioValue) -> None:
        """设置引脚输出电平"""
        ...

    @abstractmethod
    def cleanup(self, pin: int) -> None:
        """释放引脚资源"""
        ...


class RpiGpioBackend(GpioBackend):
    """树莓派 GPIO 后端 (RPi.GPIO, BCM 引脚编号)"""

    def __init__(self, **kwargs: object):
        """初始化 RPi.GPIO，设置 BCM 模式"""
        try:
            import RPi.GPIO as GPIO
        except ImportError:
            raise ImportError(
                "RPi.GPIO 未安装。请在树莓派上运行: pip install RPi.GPIO"
            )
        self._gpio = GPIO
        self._gpio.setmode(self._gpio.BCM)
        logger.info("RPi.GPIO backend initialized (BCM mode)")

    def setup(self, pin: int, mode: GpioMode) -> None:
        direction = self._gpio.IN if mode == GpioMode.IN else self._gpio.OUT
        self._gpio.setup(pin, direction)
        logger.debug("RPi GPIO setup: pin=%d, mode=%s", pin, mode.name)

    def output(self, pin: int, value: GpioValue) -> None:
        level = self._gpio.LOW if value == GpioValue.LOW else self._gpio.HIGH
        self._gpio.output(pin, level)
        logger.debug("RPi GPIO output: pin=%d, value=%s", pin, value.name)

    def cleanup(self, pin: int) -> None:
        self._gpio.cleanup(pin)
        logger.debug("RPi GPIO cleanup: pin=%d", pin)


class Ft232hBackend(GpioBackend):
    """FT232H USB-to-GPIO 适配器后端 (pyftdi)

    使用 ADBUS 端口 (CS=0) 进行 GPIO 控制。
    适合 x86 迷你 PC 等没有原生 GPIO 的平台。
    """

    # FT232H 默认设备 URL
    DEFAULT_URL: ClassVar[str] = 'ftdi://ftdi:232h/1'

    def __init__(self, url: str = '', **kwargs: object):
        """初始化 FT232H GPIO 控制器

        Args:
            url: FTDI 设备 URL, 默认使用 DEFAULT_URL
        """
        try:
            from pyftdi.gpio import GpioController  # type: ignore[import-untyped]
        except ImportError:
            raise ImportError(
                "pyftdi 未安装。请运行: pip install pyftdi"
            )

        self._url = url or self.DEFAULT_URL
        self._gpio = GpioController()
        # 初始化: 配置所有 ADBUS 引脚为输出 (高阻态默认)
        self._gpio.configure(self._url, direction=0xFF)
        # 记录需要配置方向的引脚集合
        self._pin_dirs: dict[int, GpioMode] = {}
        logger.info("FT232H backend initialized: url=%s", self._url)

    def setup(self, pin: int, mode: GpioMode) -> None:
        if not (0 <= pin <= 7):
            raise ValueError(f"FT232H ADBUS 引脚必须在 0-7 之间, 收到: {pin}")
        self._pin_dirs[pin] = mode
        # pyftdi 的 set_direction 使用位掩码: pin=5 → 1<<5 = 0b100000
        pin_mask = 1 << pin
        dir_mask = pin_mask if mode == GpioMode.OUT else 0
        self._gpio.set_direction(pin_mask, dir_mask)
        logger.debug("FT232H setup: pin=%d, mode=%s", pin, mode.name)

    def output(self, pin: int, value: GpioValue) -> None:
        # pyftdi 没有 write_port 方法 — 使用 read-modify-write
        current = self._gpio.read()
        pin_mask = 1 << pin
        if value == GpioValue.HIGH:
            current |= pin_mask
        else:
            current &= ~pin_mask
        self._gpio.write(current & self._gpio.direction)
        logger.debug("FT232H output: pin=%d, value=%s", pin, value.name)

    def cleanup(self, pin: int) -> None:
        self._pin_dirs.pop(pin, None)
        logger.debug("FT232H cleanup: pin=%d", pin)


class DummyGpioBackend(GpioBackend):
    """模拟 GPIO 后端 — 无硬件操作，仅记录调用日志

    适用于单元测试和没有 GPIO 硬件的开发环境。
    通过 last_calls 属性读取调用历史进行断言。
    """

    def __init__(self, **kwargs: object):
        self._log: list[tuple[str, int, object, float]] = []
        logger.info("Dummy GPIO backend initialized (no hardware)")

    def setup(self, pin: int, mode: GpioMode) -> None:
        self._log.append(('setup', pin, mode.name, time.monotonic()))
        logger.debug("Dummy GPIO setup: pin=%d, mode=%s", pin, mode.name)

    def output(self, pin: int, value: GpioValue) -> None:
        self._log.append(('output', pin, value.name, time.monotonic()))
        logger.debug("Dummy GPIO output: pin=%d, value=%s", pin, value.name)

    def cleanup(self, pin: int) -> None:
        self._log.append(('cleanup', pin, None, time.monotonic()))
        logger.debug("Dummy GPIO cleanup: pin=%d", pin)

    def reset_log(self) -> None:
        """清空调用日志 (测试用)"""
        self._log.clear()

    @property
    def last_calls(self) -> list[tuple[str, int, object, float]]:
        """返回调用历史 — 每个元素为 (method, pin, value, timestamp)"""
        return list(self._log)


class H7GpioBackend(GpioBackend):
    """STM32H7 GPIO 开发板后端 (串口协议)

    通过 USB-TTL 串口连接 STM32H7 开发板 (大疆电机开发板C),
    使用 h7_gpio_protocol 协议帧控制 GPIO 引脚。

    当前桥接板固件仅支持 0x01 SET_OUTPUT；激光闪烁由
    LaserController 在上位机后台线程中发送 HIGH/LOW 实现，setup
    不发送协议帧。
    """

    def __init__(self, serial: 'H7GpioSerial', **kwargs: object):
        """初始化 H7 GPIO 后端

        Args:
            serial: H7GpioSerial 实例 (已连接)
        """
        # 延迟导入避免循环依赖
        try:
            from .h7_gpio_protocol import cmd_set_output
        except ImportError:
            from h7_gpio_protocol import cmd_set_output

        self._serial = serial
        self._cmd_set_output = cmd_set_output
        logger.info("H7 GPIO backend initialized")

    def setup(self, pin: int, mode: GpioMode) -> None:
        """桥接板固件预配置输出引脚，此处不发送未实现的 0x02。"""
        logger.debug("H7 GPIO setup skipped: pin=%d, mode=%s", pin, mode.name)

    def output(self, pin: int, value: GpioValue) -> None:
        """设置引脚输出电平"""
        high = (value == GpioValue.HIGH)
        frame = self._cmd_set_output(pin, high)
        self._serial.send_frame(frame)
        # 桥接板的激光响应属于正常回执，不再校验其状态。
        self._serial.read_response(timeout_s=0.1)
        logger.debug("H7 GPIO output: pin=%d, value=%s", pin, value.name)

    def cleanup(self, pin: int) -> None:
        """释放引脚 (设为低电平)"""
        frame = self._cmd_set_output(pin, False)
        self._serial.send_frame(frame)
        # 返回帧仅作尽力消费，不参与清理结果判定。
        self._serial.read_response(timeout_s=0.1)
        logger.debug("H7 GPIO cleanup: pin=%d", pin)


def auto_detect_backend() -> GpioBackend:
    """自动检测并返回合适的 GPIO 后端

    检测顺序:
        1. RPi.GPIO → RpiGpioBackend
        2. pyftdi    → Ft232hBackend
        3. 降级      → DummyGpioBackend

    Returns:
        可用的 GpioBackend 实例
    """
    # 1) 尝试树莓派 GPIO
    try:
        import RPi.GPIO  # noqa: F401
        backend = RpiGpioBackend()
        logger.info("auto_detect: selected RpiGpioBackend")
        return backend
    except ImportError:
        logger.debug("auto_detect: RPi.GPIO not available")

    # 2) 尝试 FT232H USB GPIO
    try:
        from pyftdi.gpio import GpioController  # noqa: F401
        backend = Ft232hBackend()
        # 快速验证设备是否真的存在
        backend._gpio.read()
        logger.info("auto_detect: selected Ft232hBackend")
        return backend
    except (ImportError, Exception):
        logger.debug("auto_detect: pyftdi unavailable or FT232H not connected")

    # 3) 降级到模拟后端
    logger.info("auto_detect: falling back to DummyGpioBackend")
    return DummyGpioBackend()
