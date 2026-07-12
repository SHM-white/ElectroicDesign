"""
laser_led.py — 激光笔与LED控制
Section 11: 激光笔与LED控制

硬件:
- 激光笔: GPIO 17 (BCM), 通过三极管/MOSFET驱动
- LED指示灯: GPIO 27 (BCM), 显示条码数字(发挥部分)
"""

import time
import logging
from typing import Optional

logger = logging.getLogger('drone.gpio')

# 条件导入: 树莓派上才有RPi.GPIO
try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False
    logger.warning("RPi.GPIO not available, using dummy GPIO")


class LaserController:
    """
    激光笔控制

    激光闪烁模拟撒药动作:
    - 频率: 1-2秒周期 (根据速度档位)
    - 次数: 1-3次
    - 占空比: 50%
    """

    def __init__(self, pin: int = 17):
        self.pin = pin
        self._initialized = False
        self._enabled = True

        if _GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.pin, GPIO.OUT)
                GPIO.output(self.pin, GPIO.LOW)
                self._initialized = True
                logger.info(f"Laser initialized on GPIO{pin}")
            except Exception as e:
                logger.error(f"Failed to init laser GPIO: {e}")
        else:
            logger.info(f"Laser (dummy) on GPIO{pin}")

    def on(self):
        """开启激光"""
        logger.debug("Laser ON")
        if self._initialized and self._enabled:
            GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        """关闭激光"""
        logger.debug("Laser OFF")
        if self._initialized and self._enabled:
            GPIO.output(self.pin, GPIO.LOW)

    def blink(self, count: int = 2, period_ms: int = 1500):
        """
        闪烁激光笔模拟撒药

        Args:
            count: 闪烁次数 (1-3)
            period_ms: 闪烁周期(ms) (1000-2000)
        """
        half_period_s = period_ms / 1000.0 / 2.0  # 50%占空比
        on_time = half_period_s
        off_time = half_period_s

        logger.info(f"Laser blinking: count={count}, period={period_ms}ms")

        for i in range(count):
            self.on()
            time.sleep(on_time)
            self.off()
            time.sleep(off_time)

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False
        self.off()

    def cleanup(self):
        """清理GPIO资源"""
        if self._initialized:
            self.off()
            # 注意: 不调用 GPIO.cleanup() 因为可能有其他GPIO在使用
        logger.info("Laser cleanup complete")


class LEDController:
    """
    LED指示灯控制

    用于显示条码数字(发挥部分):
    通过闪烁次数表示数字
    """

    def __init__(self, pin: int = 27):
        self.pin = pin
        self._initialized = False

        if _GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.pin, GPIO.OUT)
                GPIO.output(self.pin, GPIO.LOW)
                self._initialized = True
                logger.info(f"LED initialized on GPIO{pin}")
            except Exception as e:
                logger.error(f"Failed to init LED GPIO: {e}")
        else:
            logger.info(f"LED (dummy) on GPIO{pin}")

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
            if self._initialized:
                GPIO.output(self.pin, GPIO.HIGH)
            time.sleep(0.3)
            if self._initialized:
                GPIO.output(self.pin, GPIO.LOW)
            time.sleep(0.3)

        time.sleep(2)  # 间隔

    def on(self):
        if self._initialized:
            GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        if self._initialized:
            GPIO.output(self.pin, GPIO.LOW)

    def cleanup(self):
        """清理GPIO资源"""
        if self._initialized:
            self.off()
        logger.info("LED cleanup complete")


class DummyLaser(LaserController):
    """模拟激光 (测试/演示用)"""

    def __init__(self):
        super().__init__(pin=17)
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
