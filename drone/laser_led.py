"""
laser_led.py — 激光笔与LED控制
Section 11: 激光笔与LED控制

硬件:
- 激光笔: GPIO 17 (BCM), 通过三极管/MOSFET驱动
- LED指示灯: GPIO 27 (BCM), 显示条码数字(发挥部分)
"""

import time
import threading
import logging
from typing import Optional

try:
    from gpio_backend import (
        GpioBackend, GpioMode, GpioValue,
        auto_detect_backend, DummyGpioBackend,
    )
except ImportError:
    from .gpio_backend import (
        GpioBackend, GpioMode, GpioValue,
        auto_detect_backend, DummyGpioBackend,
    )

logger = logging.getLogger('drone.gpio')


class LaserController:
    """
    激光笔控制

    激光闪烁模拟撒药动作:
    - 频率: 1-2秒周期 (根据速度档位)
    - 次数: 1-3次
    - 占空比: 50%
    """

    def __init__(self, pin: int = 17, backend: GpioBackend | None = None):
        self.pin = pin
        self._enabled = True
        self._backend = backend if backend is not None else auto_detect_backend()
        self._blink_thread: Optional[threading.Thread] = None
        self._blink_stop = threading.Event()
        self._backend.setup(self.pin, GpioMode.OUT)
        self._backend.output(self.pin, GpioValue.LOW)
        logger.info(f"Laser initialized on GPIO{pin}")

    def on(self):
        """开启激光"""
        logger.debug("Laser ON")
        if self._enabled:
            self._backend.output(self.pin, GpioValue.HIGH)

    def off(self):
        """关闭激光"""
        logger.debug("Laser OFF")
        if self._enabled:
            self._backend.output(self.pin, GpioValue.LOW)

    def blink(self, count: int = 2, period_ms: int = 1500):
        """
        闪烁激光笔模拟撒药

        如果后端支持硬件脉冲 (如 H7GpioBackend.pulse()),
        则发送单条协议帧由 STM32H7 精确控制时序 (非阻塞)。
        否则回退到软件 time.sleep() 闪烁。

        Args:
            count: 闪烁次数 (1-3)
            period_ms: 闪烁周期(ms) (1000-2000)
        """
        # 硬件脉冲路径 (非阻塞, MCU 控制时序)
        if hasattr(self._backend, 'pulse') and callable(self._backend.pulse):
            logger.info(f"Laser hardware pulse: count={count}, period={period_ms}ms")
            self._backend.pulse(self.pin, count, period_ms)
            return

        # 软件闪烁路径 (后台线程, 兼容 RPi/FT232H/Dummy 后端)
        half_period_s = period_ms / 1000.0 / 2.0  # 50%占空比
        if self._blink_thread is not None and self._blink_thread.is_alive():
            logger.warning("Laser blink request ignored because a blink is already active")
            return

        self._blink_stop.clear()

        def run_blink() -> None:
            logger.info(f"Laser blinking: count={count}, period={period_ms}ms")
            for _ in range(count):
                if self._blink_stop.is_set():
                    break
                self.on()
                if self._blink_stop.wait(half_period_s):
                    break
                self.off()
                if self._blink_stop.wait(half_period_s):
                    break
            self.off()

        self._blink_thread = threading.Thread(
            target=run_blink,
            name='laser-blink',
            daemon=True,
        )
        self._blink_thread.start()

    def enable(self):
        self._blink_stop.clear()
        self._enabled = True

    def disable(self):
        self._blink_stop.set()
        self._backend.output(self.pin, GpioValue.LOW)
        self._enabled = False

    def cleanup(self):
        """清理GPIO资源"""
        self._blink_stop.set()
        if self._blink_thread is not None and self._blink_thread.is_alive():
            self._blink_thread.join(timeout=1.0)
        self.off()
        self._backend.cleanup(self.pin)
        logger.info("Laser cleanup complete")


class LEDController:
    """
    LED指示灯控制

    用于显示条码数字(发挥部分):
    通过闪烁次数表示数字
    """

    def __init__(self, pin: int = 27, backend: GpioBackend | None = None):
        self.pin = pin
        self._backend = backend if backend is not None else auto_detect_backend()
        self._backend.setup(self.pin, GpioMode.OUT)
        self._backend.output(self.pin, GpioValue.LOW)
        logger.info(f"LED initialized on GPIO{pin}")

    def show_number(self, number: int):
        """
        闪烁LED显示数字

        Args:
            number: 要显示的数字 (>0)
        """
        if number <= 0:
            return

        logger.info(f"LED showing number: {number}")

        for _ in range(number):
            self._backend.output(self.pin, GpioValue.HIGH)
            time.sleep(0.3)
            self._backend.output(self.pin, GpioValue.LOW)
            time.sleep(0.3)

        time.sleep(2)  # 间隔

    def on(self):
        self._backend.output(self.pin, GpioValue.HIGH)

    def off(self):
        self._backend.output(self.pin, GpioValue.LOW)

    def cleanup(self):
        """清理GPIO资源"""
        self.off()
        self._backend.cleanup(self.pin)
        logger.info("LED cleanup complete")


class DummyLaser(LaserController):
    """模拟激光 (测试/演示用)"""

    def __init__(self):
        super().__init__(pin=17, backend=DummyGpioBackend())
        self._blink_count = 0
        self._log = []

    def on(self):
        self._log.append(('ON', time.time()))

    def off(self):
        self._log.append(('OFF', time.time()))

    def blink(self, count: int = 2, period_ms: int = 1500):
        self._blink_count += 1
        self._log.append(('BLINK', time.time(), count, period_ms))
        logger.info(f"[DUMMY] Laser blink: {count}x @ {period_ms}ms")

    @property
    def total_blinks(self):
        return self._blink_count
