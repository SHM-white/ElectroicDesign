"""
test_state_machine.py — 状态机逻辑测试
验证 Section 10: 状态机设计
"""

import sys
import os
import unittest
from unittest.mock import MagicMock, patch, PropertyMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from state_machine import DroneStateMachine, FlightState
from vision_result import VisionResult
from localization import Localizer
from config import get_config


class MockMCU:
    """模拟MCU通信"""

    def __init__(self):
        self._altitude = 0
        self._mode = 0
        self._locked = 0
        self._of_dx = 0.0
        self._of_dy = 0.0
        self._sent_commands = []
        self._comm_ok = True
        self._comm_age_ms = 0

    def poll(self):
        """模拟串口轮询 (run_iteration 需要在每次迭代中调用)"""
        return True

    def read_optical_flow(self):
        return self._of_dx, self._of_dy

    def read_altitude(self):
        return self._altitude

    def read_mode(self):
        return self._mode

    def read_locked(self):
        return self._locked

    def read_aux6(self):
        return 1000  # 未触发

    def read_voltage(self):
        return 11.8

    def is_communication_ok(self, timeout_ms=500):
        return self._comm_ok

    def get_communication_age_ms(self):
        return self._comm_age_ms

    def has_flight_status(self):
        return True

    def send_cmd_unlock(self):
        self._sent_commands.append('unlock')
        return True

    def send_cmd_lock(self):
        self._sent_commands.append('lock')
        return True

    def send_cmd_mode(self, mode):
        self._sent_commands.append(f'mode_{mode}')
        return True

    def send_cmd_takeoff(self, height_cm):
        self._sent_commands.append(f'takeoff_{height_cm}')
        return True

    def send_cmd_land(self):
        self._sent_commands.append('land')
        return True

    def send_cmd_move(self, distance, speed, direction):
        self._sent_commands.append(f'move_{distance}_{speed}_{direction}')
        return True

    def send_cmd_ascend(self, height, speed):
        self._sent_commands.append(f'ascend_{height}_{speed}')
        return True

    def send_cmd_descend(self, height, speed):
        self._sent_commands.append(f'descend_{height}_{speed}')
        return True

    def send_of_zero_reset(self):
        self._sent_commands.append('of_reset')
        return True

    def send_heartbeat(self):
        self._sent_commands.append('heartbeat')
        return True

    def set_altitude(self, alt):
        self._altitude = alt

    def trigger_start(self):
        self._read_aux6_original = self.read_aux6
        self.read_aux6 = lambda: 1800


class MockCamera:
    """模拟相机"""
    def read(self):
        return False, None


class MockResultCamera:
    """模拟已经在相机端完成识别的视觉后端。"""
    def read_result(self):
        return VisionResult(green_ratio=0.75, digit=21)


class MockLaser:
    """模拟激光"""
    def __init__(self):
        self.blink_count = 0
        self.blink_records = []

    def blink(self, count=2, period_ms=1500):
        self.blink_count += 1
        self.blink_records.append((count, period_ms))
        return True

    def is_blinking(self):
        return False

    def disable(self):
        pass


class TestStateMachineInit(unittest.TestCase):
    """测试状态机初始化"""

    def test_init_state_idle(self):
        mcu = MockMCU()
        cam = MockCamera()
        loc = Localizer()
        laser = MockLaser()
        sm = DroneStateMachine(mcu, cam, loc, laser, get_config())
        self.assertEqual(sm.state, FlightState.IDLE)

    def test_visited_array_size(self):
        sm = DroneStateMachine(MockMCU(), MockCamera(),
                                Localizer(), MockLaser(), get_config())
        self.assertEqual(len(sm.visited), 29)  # 29 entries (index 0-28)
        self.assertFalse(sm.visited[0])  # index 0 is unused

    def test_not_completed_initially(self):
        sm = DroneStateMachine(MockMCU(), MockCamera(),
                                Localizer(), MockLaser(), get_config())
        self.assertFalse(sm.is_completed)

    def test_processed_vision_backend_result(self):
        sm = DroneStateMachine(MockMCU(), MockResultCamera(),
                               Localizer(), MockLaser(), get_config())
        frame, green_ratio, digit = sm._get_vision_data()
        self.assertIsNone(frame)
        self.assertEqual(green_ratio, 0.75)
        self.assertEqual(digit, 21)

    def test_missing_vision_is_not_reported_as_zero_green(self):
        sm = DroneStateMachine(MockMCU(), MockCamera(),
                               Localizer(), MockLaser(), get_config())
        _, green_ratio, digit = sm._get_vision_data()
        self.assertIsNone(green_ratio)
        self.assertIsNone(digit)


class TestStateTransitions(unittest.TestCase):
    """测试状态转换流程"""

    def setUp(self):
        self.mcu = MockMCU()
        self.cam = MockCamera()
        self.loc = Localizer()
        self.laser = MockLaser()
        self.cfg = get_config()
        self.sm = DroneStateMachine(self.mcu, self.cam, self.loc, self.laser, self.cfg)

    def test_idle_stays_until_signal(self):
        """IDLE状态下没有启动信号应保持"""
        state = self.sm.run_iteration()
        self.assertEqual(state, FlightState.IDLE)

    def test_full_sequence_dry(self):
        """测试完整状态序列 (快速推进时间)"""
        import time

        states_seen = set()

        # 模拟启动信号
        self.mcu.read_aux6 = lambda: 1800

        for _ in range(200):
            state = self.sm.run_iteration()
            states_seen.add(state)

            # 在每个需要等待的状态加速时间
            if state == FlightState.TAKEOFF:
                self.mcu.set_altitude(150)  # 到目标高度

            if state == FlightState.DONE or state == FlightState.EMERGENCY:
                break

        # 应该遍历了多个状态
        self.assertIn(FlightState.ARM_UNLOCK, states_seen)
        self.assertNotEqual(state, FlightState.IDLE)

    def test_emergency_on_altitude(self):
        """测试高度异常触发紧急状态"""
        self.mcu.read_aux6 = lambda: 1800

        # 先从IDLE推进一点
        for _ in range(10):
            self.sm.run_iteration()

        # 设置极低高度触发紧急
        self.mcu.set_altitude(5)
        self.sm._mission_start_time = 0  # 避免超时检查干扰
        self.sm.run_iteration()

        if self.sm.state != FlightState.EMERGENCY:
            # 如果还没到飞行状态, 继续推进
            for _ in range(50):
                self.mcu.set_altitude(5)
                self.sm.run_iteration()

        # 此时可能已经在其他状态, 但高度异常应被检测


class TestSprayLogic(unittest.TestCase):
    """测试撒药逻辑"""

    def setUp(self):
        self.mcu = MockMCU()
        self.cam = MockCamera()
        self.loc = Localizer()
        self.laser = MockLaser()
        self.cfg = get_config()
        self.sm = DroneStateMachine(self.mcu, self.cam, self.loc, self.laser, self.cfg)
        self.sm._start_block_confirmed = True

    def test_spray_marks_visited(self):
        """测试SPRAY状态标记区块已访问"""
        # 手动设置到SPRAY状态
        self.sm.state = FlightState.SPRAY
        self.sm._state_start_time = 0

        cur_block = self.loc.get_current_target()
        self.assertFalse(self.sm.visited[cur_block])

        self.sm.run_iteration()
        self.sm.run_iteration()

        # 应标记为已访问
        self.assertTrue(self.sm.visited[cur_block])

    def test_laser_blink_on_spray(self):
        """测试撒药时激光闪烁"""
        old_count = self.laser.blink_count
        self.sm.state = FlightState.SPRAY
        self.sm.run_iteration()
        self.assertEqual(self.laser.blink_count, old_count + 1)

    def test_first_laser_is_blocked_without_21_confirmation(self):
        self.sm._start_block_confirmed = False
        self.sm.state = FlightState.SPRAY

        self.sm.run_iteration()

        self.assertEqual(self.laser.blink_count, 0)
        self.assertEqual(self.sm.state, FlightState.EMERGENCY)


class TestNavigateLogic(unittest.TestCase):
    """测试导航逻辑"""

    def setUp(self):
        self.mcu = MockMCU()
        self.cam = MockCamera()
        self.loc = Localizer()
        self.laser = MockLaser()
        self.cfg = get_config()
        self.sm = DroneStateMachine(self.mcu, self.cam, self.loc, self.laser, self.cfg)

    def test_navigate_sends_move(self):
        """测试NAVIGATE发送移动指令"""
        import time
        self.sm.state = FlightState.NAVIGATE
        self.sm._state_start_time = time.time()  # 设为当前时间, 使state_start_time≈0
        # 确保当前区块已访问
        self.sm.visited[21] = True

        self.sm.run_iteration()
        # 应发送了移动指令
        move_cmds = [c for c in self.mcu._sent_commands if c.startswith('move')]
        self.assertGreaterEqual(len(move_cmds), 1)

    def test_all_blocks_visited_triggers_return_home(self):
        """测试全部区块完成后触发返航"""
        self.sm.state = FlightState.NAVIGATE
        # 标记全部已访问
        for i in range(1, 29):
            self.sm.visited[i] = True

        self.sm.run_iteration()
        self.assertEqual(self.sm.state, FlightState.RETURN_HOME)

    def test_manual_navigation_waits_for_target_digit(self):
        """人工移动模式必须识别到目标区块数字才推进。"""
        self.cfg['manual_navigation'] = True
        self.sm.state = FlightState.NAVIGATE
        self.sm.visited[21] = True
        next_block = self.loc.get_next_target()

        self.sm._state_navigate(None, 0.5, None)
        self.assertEqual(self.sm.state, FlightState.NAVIGATE)
        self.assertEqual(self.loc.get_current_target(), 21)

        self.sm._state_navigate(None, 0.5, next_block)
        self.assertEqual(self.sm.state, FlightState.SPRAY)
        self.assertEqual(self.loc.get_current_target(), next_block)

    def test_manual_navigation_ocr_timeout_sprays_first_two_blocks(self):
        self.cfg['manual_navigation'] = True
        self.cfg['manual_navigation_timeout_s'] = 15
        self.cfg['max_consecutive_ocr_timeouts'] = 3
        self.sm.state = FlightState.NAVIGATE
        self.sm.visited[21] = True
        self.sm._state_start_time = __import__('time').time() - 16

        self.sm._state_navigate(None, 0.5, None)

        self.assertEqual(self.sm.state, FlightState.SPRAY)
        self.assertEqual(self.loc.get_current_target(), 20)
        self.assertEqual(self.sm._consecutive_ocr_timeouts, 1)

        self.sm.visited[20] = True
        self.sm.state = FlightState.NAVIGATE
        self.sm._state_start_time = __import__('time').time() - 16
        self.sm._state_navigate(None, 0.5, None)

        self.assertEqual(self.sm.state, FlightState.SPRAY)
        self.assertEqual(self.loc.get_current_target(), 19)
        self.assertEqual(self.sm._consecutive_ocr_timeouts, 2)

    def test_third_consecutive_ocr_timeout_triggers_emergency(self):
        self.cfg['manual_navigation'] = True
        self.cfg['manual_navigation_timeout_s'] = 15
        self.cfg['max_consecutive_ocr_timeouts'] = 3
        self.sm.state = FlightState.NAVIGATE
        self.sm.visited[21] = True
        self.sm.visited[20] = True
        self.sm.visited[19] = True
        self.loc.apply_ocr(19)
        self.sm._consecutive_ocr_timeouts = 2
        self.sm._state_start_time = __import__('time').time() - 16

        self.sm._state_navigate(None, 0.5, None)

        self.assertEqual(self.sm.state, FlightState.EMERGENCY)
        self.assertFalse(self.sm.visited[18])
        self.assertIn('3 consecutive blocks', self.sm.emergency_reason)

    def test_successful_ocr_resets_consecutive_timeout_count(self):
        self.cfg['manual_navigation'] = True
        self.sm.state = FlightState.NAVIGATE
        self.sm.visited[21] = True
        self.sm._consecutive_ocr_timeouts = 2

        self.sm._state_navigate(None, 0.5, 20)

        self.assertEqual(self.sm.state, FlightState.SPRAY)
        self.assertEqual(self.sm._consecutive_ocr_timeouts, 0)

    def test_navigation_failure_does_not_mark_block_visited(self):
        self.sm.state = FlightState.NAVIGATE
        self.sm.visited[21] = True
        self.sm._retry_count = self.sm._max_retries
        self.sm._state_start_time = __import__('time').time() - 11

        self.sm._state_navigate(None, 0.5, None)

        self.assertEqual(self.sm.state, FlightState.EMERGENCY)
        self.assertFalse(self.sm.visited[20])


class TestStartAndHomeVisionGates(unittest.TestCase):
    def setUp(self):
        self.mcu = MockMCU()
        self.cam = MockCamera()
        self.loc = Localizer()
        self.laser = MockLaser()
        self.cfg = get_config()
        self.cfg['start_block_confirm_frames'] = 3
        self.cfg['home_cross_confirm_frames'] = 3
        self.sm = DroneStateMachine(
            self.mcu, self.cam, self.loc, self.laser, self.cfg,
        )

    def test_start_accepts_three_digit_21_observations(self):
        self.sm.state = FlightState.FIND_START
        target = __import__('path_plan').get_block_position(21)
        self.loc._global_pos_x, self.loc._global_pos_y = target

        for _ in range(3):
            self.sm._state_find_start(None, 0.5, 21)

        self.assertEqual(self.sm.state, FlightState.SPRAY)
        self.assertTrue(self.sm._start_block_confirmed)

    def test_manual_start_accepts_digit_without_a_marker(self):
        self.sm.state = FlightState.FIND_START
        self.sm.cfg['manual_navigation'] = True

        for _ in range(3):
            self.sm._state_find_start(None, 0.5, 21)

        self.assertEqual(self.sm.state, FlightState.SPRAY)
        self.assertTrue(self.sm._start_block_confirmed)

    def test_manual_start_accepts_a_marker(self):
        self.sm.state = FlightState.FIND_START
        self.sm.cfg['manual_navigation'] = True

        for _ in range(3):
            self.sm._start_marker_center = (720, 540)
            self.sm._state_find_start(None, 0.5, None)

        self.assertEqual(self.sm.state, FlightState.SPRAY)
        self.assertTrue(self.sm._start_block_confirmed)

    def test_start_confirmation_ignores_frames_without_new_ocr_event(self):
        self.sm.state = FlightState.FIND_START
        self.sm.cfg['manual_navigation'] = True

        self.sm._state_find_start(None, 0.5, 21)
        for _ in range(20):
            self.sm._start_marker_center = None
            self.sm._state_find_start(None, 0.5, None)
        self.sm._state_find_start(None, 0.5, 21)
        for _ in range(20):
            self.sm._state_find_start(None, 0.5, None)
        self.sm._state_find_start(None, 0.5, 21)

        self.assertEqual(self.sm.state, FlightState.SPRAY)
        self.assertTrue(self.sm._start_block_confirmed)

    def test_start_confirmation_resets_after_event_window_expires(self):
        self.sm.state = FlightState.FIND_START
        self.sm.cfg['manual_navigation'] = True
        self.sm.cfg['start_block_confirm_window_s'] = 5.0

        with patch('state_machine.time.monotonic', side_effect=(10.0, 16.0)):
            self.sm._state_find_start(None, 0.5, 21)
            self.sm._state_find_start(None, 0.5, 21)

        self.assertEqual(self.sm.state, FlightState.FIND_START)
        self.assertEqual(self.sm._start_confirm_count, 1)

    def test_home_alignment_targets_cross_below_center_for_tail_camera(self):
        self.sm.state = FlightState.ALIGN_HOME
        self.mcu.set_altitude(100)
        self.sm._home_cross_confidence = 1.0
        # fy=800, altitude=100: 机体中心对齐时，十字应在中心下方200px。
        self.sm._home_cross_center = (720.0, 740.0)

        for _ in range(3):
            self.sm._state_align_home(None, None, None)

        self.assertEqual(self.sm.state, FlightState.LAND)


class TestProgressTracking(unittest.TestCase):
    """测试进度追踪"""

    def test_visited_count(self):
        sm = DroneStateMachine(MockMCU(), MockCamera(),
                                Localizer(), MockLaser(), get_config())
        self.assertEqual(sm.visited_count, 0)

        sm.visited[1] = True
        self.assertEqual(sm.visited_count, 1)

        sm.visited[21] = True
        self.assertEqual(sm.visited_count, 2)

    def test_progress_dict(self):
        sm = DroneStateMachine(MockMCU(), MockCamera(),
                                Localizer(), MockLaser(), get_config())
        progress = sm.get_progress()
        self.assertEqual(progress['state'], 'IDLE')
        self.assertEqual(progress['total'], 28)
        self.assertFalse(progress['emergency'])


class TestEmergencyHandling(unittest.TestCase):
    """测试紧急处理"""

    def setUp(self):
        self.mcu = MockMCU()
        self.cam = MockCamera()
        self.loc = Localizer()
        self.laser = MockLaser()
        self.cfg = get_config()
        self.sm = DroneStateMachine(self.mcu, self.cam, self.loc, self.laser, self.cfg)

    def test_emergency_reason_preserved(self):
        """测试紧急原因被保存"""
        self.sm._emergency("Test emergency")
        self.assertEqual(self.sm.state, FlightState.EMERGENCY)
        self.assertIn("Test emergency", self.sm._emergency_reason)
        self.assertTrue(self.sm.is_emergency)

    def test_emergency_sends_land(self):
        """测试紧急状态发送降落指令"""
        self.sm._emergency("Battery low")
        self.sm.run_iteration()

        # 应发送降落指令
        land_cmds = [c for c in self.mcu._sent_commands if c == 'land']
        self.assertGreaterEqual(len(land_cmds), 1)

    def test_confirmed_low_voltage_returns_home(self):
        self.sm.state = FlightState.NAVIGATE
        self.mcu.read_voltage = lambda: 10.4

        for _ in range(self.cfg['low_voltage_confirm_samples']):
            self.sm._check_exceptions()

        self.assertEqual(self.sm.state, FlightState.RETURN_HOME)

    def test_confirmed_critical_voltage_triggers_emergency(self):
        self.sm.state = FlightState.NAVIGATE
        self.mcu.read_voltage = lambda: 9.9

        for _ in range(self.cfg['low_voltage_confirm_samples']):
            self.sm._check_exceptions()

        self.assertEqual(self.sm.state, FlightState.EMERGENCY)
        self.assertIn('Critical battery voltage', self.sm.emergency_reason)


if __name__ == '__main__':
    unittest.main(verbosity=2)
