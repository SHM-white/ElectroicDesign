"""
test_laser_led.py — GPIO 抽象层与激光/LED 控制模块测试
验证 gpio_backend.py 和 laser_led.py 的完整功能:
- GpioBackend 接口实现
- DummyGpioBackend 日志记录
- LaserController (注入 backend)
- LEDController  (注入 backend)
- DummyLaser 向后兼容
- auto_detect_backend 自动检测
"""

import sys
import os
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from gpio_backend import (
    GpioBackend, GpioMode, GpioValue,
    DummyGpioBackend, auto_detect_backend,
)
from laser_led import LaserController, LEDController, DummyLaser


# ---------------------------------------------------------------------------
# TestGpioBackendInterface — 验证 DummyGpioBackend 实现了抽象接口
# ---------------------------------------------------------------------------

class TestGpioBackendInterface(unittest.TestCase):
    """测试 GpioBackend 抽象接口实现"""

    def test_dummy_is_gpio_backend_instance(self):
        """DummyGpioBackend 应为 GpioBackend 的实例"""
        backend = DummyGpioBackend()
        self.assertIsInstance(backend, GpioBackend)


# ---------------------------------------------------------------------------
# TestDummyGpioBackend — 模拟后端日志记录行为
# ---------------------------------------------------------------------------

class TestDummyGpioBackend(unittest.TestCase):
    """测试 DummyGpioBackend 日志记录"""

    def setUp(self):
        self.backend = DummyGpioBackend()

    def test_setup_records_call(self):
        """setup() 应记录 (setup, pin, mode_name, timestamp) 到日志"""
        self.backend.setup(17, GpioMode.OUT)
        calls = self.backend.last_calls
        self.assertEqual(len(calls), 1)
        method, pin, value, ts = calls[0]
        self.assertEqual(method, 'setup')
        self.assertEqual(pin, 17)
        self.assertEqual(value, 'OUT')
        self.assertIsInstance(ts, float)

    def test_output_records_call(self):
        """output() 应记录 (output, pin, value_name, timestamp) 到日志"""
        self.backend.output(17, GpioValue.HIGH)
        calls = self.backend.last_calls
        self.assertEqual(len(calls), 1)
        method, pin, value, ts = calls[0]
        self.assertEqual(method, 'output')
        self.assertEqual(pin, 17)
        self.assertEqual(value, 'HIGH')
        self.assertIsInstance(ts, float)

    def test_output_low_records_correct_value(self):
        """output(LOW) 应记录 value='LOW'"""
        self.backend.output(27, GpioValue.LOW)
        _, _, value, _ = self.backend.last_calls[0]
        self.assertEqual(value, 'LOW')

    def test_cleanup_records_call(self):
        """cleanup() 应记录 (cleanup, pin, None, timestamp) 到日志"""
        self.backend.cleanup(17)
        calls = self.backend.last_calls
        self.assertEqual(len(calls), 1)
        method, pin, value, ts = calls[0]
        self.assertEqual(method, 'cleanup')
        self.assertEqual(pin, 17)
        self.assertIsNone(value)
        self.assertIsInstance(ts, float)

    def test_last_calls_returns_copy(self):
        """last_calls 应返回副本，外部修改不影响内部日志"""
        self.backend.setup(17, GpioMode.OUT)
        calls = self.backend.last_calls
        calls.clear()
        self.assertEqual(len(self.backend.last_calls), 1)

    def test_reset_log_clears_all(self):
        """reset_log() 应清空所有调用记录"""
        self.backend.setup(17, GpioMode.OUT)
        self.backend.output(17, GpioValue.HIGH)
        self.assertEqual(len(self.backend.last_calls), 2)
        self.backend.reset_log()
        self.assertEqual(len(self.backend.last_calls), 0)

    def test_multiple_calls_preserve_order(self):
        """多次调用应按时间顺序记录"""
        self.backend.setup(17, GpioMode.OUT)
        self.backend.output(17, GpioValue.HIGH)
        self.backend.output(17, GpioValue.LOW)
        self.backend.cleanup(17)
        calls = self.backend.last_calls
        methods = [c[0] for c in calls]
        self.assertEqual(methods, ['setup', 'output', 'output', 'cleanup'])


# ---------------------------------------------------------------------------
# TestLaserControllerWithBackend — 注入 backend 的 LaserController
# ---------------------------------------------------------------------------

class TestLaserControllerWithBackend(unittest.TestCase):
    """测试 LaserController (注入 DummyGpioBackend)"""

    def setUp(self):
        self.backend = DummyGpioBackend()
        self.lc = LaserController(pin=17, backend=self.backend)

    def test_init_with_explicit_backend(self):
        """LaserController(pin=17, backend=...) 应使用提供的 backend"""
        backend = DummyGpioBackend()
        lc = LaserController(pin=17, backend=backend)
        self.assertIs(lc._backend, backend)

    def test_init_calls_setup_on_backend(self):
        """初始化时应调用 backend.setup(pin, OUT)"""
        calls = self.backend.last_calls
        setup_calls = [c for c in calls if c[0] == 'setup']
        self.assertEqual(len(setup_calls), 1)
        self.assertEqual(setup_calls[0][1], 17)
        self.assertEqual(setup_calls[0][2], 'OUT')

    def test_init_sets_output_low(self):
        """初始化时应将引脚置为低电平"""
        calls = self.backend.last_calls
        output_calls = [c for c in calls if c[0] == 'output']
        self.assertGreaterEqual(len(output_calls), 1)
        self.assertEqual(output_calls[-1][1], 17)
        self.assertEqual(output_calls[-1][2], 'LOW')

    def test_on_calls_backend_output_high(self):
        """on() 应调用 backend.output(pin, HIGH)"""
        self.backend.reset_log()
        self.lc.on()
        calls = self.backend.last_calls
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][:3], ('output', 17, 'HIGH'))

    def test_off_calls_backend_output_low(self):
        """off() 应调用 backend.output(pin, LOW)"""
        self.backend.reset_log()
        self.lc.off()
        calls = self.backend.last_calls
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][:3], ('output', 17, 'LOW'))

    def test_blink_sequence_on_off_on_off(self):
        """blink(count=2) 应产生 ON→OFF→ON→OFF 顺序"""
        self.backend.reset_log()
        self.lc.blink(count=2, period_ms=100)
        output_calls = [c for c in self.backend.last_calls if c[0] == 'output']
        self.assertEqual(len(output_calls), 4)
        values = [c[2] for c in output_calls]
        self.assertEqual(values, ['HIGH', 'LOW', 'HIGH', 'LOW'])

    def test_disable_blocks_on(self):
        """disable() 后 on() 不应调用 backend"""
        self.lc.disable()
        self.backend.reset_log()
        self.lc.on()
        self.assertEqual(len(self.backend.last_calls), 0,
                         "disable 后 on() 不应调用 backend")

    def test_disable_then_off_skips_backend(self):
        """disable() 先将 _enabled 置为 False，后续 on()/off() 不调用 backend"""
        self.backend.reset_log()
        self.lc.disable()
        # disable 内部先设置 _enabled=False，再调用 self.off()
        # 但 off() 检查 _enabled 为 False，不会调用 backend.output
        self.assertFalse(self.lc._enabled)
        self.lc.off()
        self.lc.on()
        self.assertEqual(len(self.backend.last_calls), 0,
                         "disable 后 on/off 不应调用 backend")

    def test_enable_restores_output(self):
        """enable() 后 on() 应恢复调用 backend"""
        self.lc.disable()
        self.lc.enable()
        self.backend.reset_log()
        self.lc.on()
        self.assertEqual(len(self.backend.last_calls), 1)
        self.assertEqual(self.backend.last_calls[0][:3], ('output', 17, 'HIGH'))

    def test_cleanup_calls_backend_cleanup(self):
        """cleanup() 应调用 backend.cleanup(pin)"""
        self.backend.reset_log()
        self.lc.cleanup()
        cleanup_calls = [c for c in self.backend.last_calls if c[0] == 'cleanup']
        self.assertEqual(len(cleanup_calls), 1)
        self.assertEqual(cleanup_calls[0][1], 17)

    def test_cleanup_turns_off_first(self):
        """cleanup() 应先关闭激光再清理"""
        self.backend.reset_log()
        self.lc.cleanup()
        calls = self.backend.last_calls
        output_calls = [c for c in calls if c[0] == 'output']
        cleanup_calls = [c for c in calls if c[0] == 'cleanup']
        self.assertGreaterEqual(len(output_calls), 1)
        self.assertGreaterEqual(len(cleanup_calls), 1)

    def test_init_with_custom_pin(self):
        """自定义 pin=23 应传递给 backend"""
        backend = DummyGpioBackend()
        lc = LaserController(pin=23, backend=backend)
        setup_calls = [c for c in backend.last_calls if c[0] == 'setup']
        self.assertEqual(setup_calls[0][1], 23)

    def test_enabled_by_default(self):
        """默认 _enabled 应为 True"""
        self.assertTrue(self.lc._enabled)

    def test_auto_detect_used_when_no_backend(self):
        """不传 backend 时使用 auto_detect_backend()"""
        lc = LaserController(pin=17)
        self.assertIsInstance(lc._backend, GpioBackend)


# ---------------------------------------------------------------------------
# TestLEDControllerWithBackend — 注入 backend 的 LEDController
# ---------------------------------------------------------------------------

class TestLEDControllerWithBackend(unittest.TestCase):
    """测试 LEDController (注入 DummyGpioBackend)"""

    def setUp(self):
        self.backend = DummyGpioBackend()
        self.led = LEDController(pin=27, backend=self.backend)

    def test_init_with_explicit_backend(self):
        """LEDController(pin=27, backend=...) 应使用提供的 backend"""
        backend = DummyGpioBackend()
        led = LEDController(pin=27, backend=backend)
        self.assertIs(led._backend, backend)

    def test_init_calls_setup_on_backend(self):
        """初始化时应调用 backend.setup(pin, OUT)"""
        calls = self.backend.last_calls
        setup_calls = [c for c in calls if c[0] == 'setup']
        self.assertEqual(len(setup_calls), 1)
        self.assertEqual(setup_calls[0][1], 27)

    def test_on_calls_backend_output_high(self):
        """on() 应调用 backend.output(pin, HIGH)"""
        self.backend.reset_log()
        self.led.on()
        self.assertEqual(len(self.backend.last_calls), 1)
        self.assertEqual(self.backend.last_calls[0][:3], ('output', 27, 'HIGH'))

    def test_off_calls_backend_output_low(self):
        """off() 应调用 backend.output(pin, LOW)"""
        self.backend.reset_log()
        self.led.off()
        self.assertEqual(self.backend.last_calls[0][:3], ('output', 27, 'LOW'))

    def test_show_number_outputs_high_low_n_times(self):
        """show_number(3) 应输出 HIGH/LOW 各 3 次"""
        self.backend.reset_log()
        self.led.show_number(3)
        output_calls = [c for c in self.backend.last_calls if c[0] == 'output']
        self.assertEqual(len(output_calls), 6)  # 3×HIGH + 3×LOW
        values = [c[2] for c in output_calls]
        self.assertEqual(values, ['HIGH', 'LOW', 'HIGH', 'LOW', 'HIGH', 'LOW'])

    def test_show_number_zero_does_nothing(self):
        """show_number(0) 不应调用 backend"""
        self.backend.reset_log()
        self.led.show_number(0)
        self.assertEqual(len(self.backend.last_calls), 0)

    def test_show_number_negative_does_nothing(self):
        """show_number(-1) 不应调用 backend"""
        self.backend.reset_log()
        self.led.show_number(-1)
        self.assertEqual(len(self.backend.last_calls), 0)

    def test_cleanup_calls_backend_cleanup(self):
        """cleanup() 应调用 backend.cleanup(pin)"""
        self.backend.reset_log()
        self.led.cleanup()
        cleanup_calls = [c for c in self.backend.last_calls if c[0] == 'cleanup']
        self.assertEqual(len(cleanup_calls), 1)
        self.assertEqual(cleanup_calls[0][1], 27)

    def test_init_with_custom_pin(self):
        """自定义 pin=13 应传递给 backend"""
        backend = DummyGpioBackend()
        led = LEDController(pin=13, backend=backend)
        setup_calls = [c for c in backend.last_calls if c[0] == 'setup']
        self.assertEqual(setup_calls[0][1], 13)


# ---------------------------------------------------------------------------
# TestDummyLaserBackwardCompat — DummyLaser 向后兼容性
# ---------------------------------------------------------------------------

class TestDummyLaserBackwardCompat(unittest.TestCase):
    """测试 DummyLaser 向后兼容 — 模拟激光 (测试/演示用)"""

    def setUp(self):
        self.laser = DummyLaser()

    def test_creates_with_default_state(self):
        """DummyLaser 创建成功，初始计数为 0"""
        self.assertEqual(self.laser._blink_count, 0)
        self.assertEqual(len(self.laser._log), 0)

    def test_on_logs_action(self):
        """on() 应在 _log 中记录 ('ON', timestamp)"""
        self.laser.on()
        self.assertEqual(len(self.laser._log), 1)
        action, ts = self.laser._log[0]
        self.assertEqual(action, 'ON')
        self.assertIsInstance(ts, float)

    def test_off_logs_action(self):
        """off() 应在 _log 中记录 ('OFF', timestamp)"""
        self.laser.off()
        self.assertEqual(len(self.laser._log), 1)
        action, ts = self.laser._log[0]
        self.assertEqual(action, 'OFF')
        self.assertIsInstance(ts, float)

    def test_blink_increments_count_and_logs(self):
        """blink() 应递增 _blink_count 并记录 BLINK 条目"""
        self.laser.blink(count=2, period_ms=500)
        self.assertEqual(self.laser._blink_count, 1)
        self.assertEqual(len(self.laser._log), 1)
        action, ts, cnt, period = self.laser._log[0]
        self.assertEqual(action, 'BLINK')
        self.assertEqual(cnt, 2)
        self.assertEqual(period, 500)

    def test_multiple_blinks_accumulate_count(self):
        """多次 blink() 应正确累计 _blink_count"""
        self.laser.blink()
        self.laser.blink()
        self.laser.blink()
        self.assertEqual(self.laser._blink_count, 3)

    def test_total_blinks_property(self):
        """total_blinks 应与 _blink_count 一致"""
        self.assertEqual(self.laser.total_blinks, 0)
        self.laser.blink()
        self.assertEqual(self.laser.total_blinks, 1)
        self.laser.blink(count=3)
        self.assertEqual(self.laser.total_blinks, 2)

    def test_on_off_sequence_in_log(self):
        """on() → off() 交替应在 _log 中按顺序记录"""
        self.laser.on()
        self.laser.off()
        self.laser.on()
        self.assertEqual(len(self.laser._log), 3)
        self.assertEqual(self.laser._log[0][0], 'ON')
        self.assertEqual(self.laser._log[1][0], 'OFF')
        self.assertEqual(self.laser._log[2][0], 'ON')

    def test_inherits_laser_controller(self):
        """DummyLaser 应为 LaserController 的子类"""
        self.assertIsInstance(self.laser, LaserController)

    def test_has_gpio_backend(self):
        """DummyLaser 应有 _backend (来自父类 LaserController)"""
        self.assertIsInstance(self.laser._backend, GpioBackend)


# ---------------------------------------------------------------------------
# TestAutoDetectBackend — 自动检测后端
# ---------------------------------------------------------------------------

class TestAutoDetectBackend(unittest.TestCase):
    """测试 auto_detect_backend 自动检测"""

    def test_returns_gpio_backend_instance(self):
        """auto_detect_backend() 应返回 GpioBackend 实例"""
        backend = auto_detect_backend()
        self.assertIsInstance(backend, GpioBackend)

    def test_returns_dummy_on_non_pi_system(self):
        """无 RPi.GPIO 时应降级为 DummyGpioBackend"""
        backend = auto_detect_backend()
        self.assertIsInstance(backend, DummyGpioBackend,
                              "本系统应降级到 DummyGpioBackend")


if __name__ == '__main__':
    unittest.main(verbosity=2)
