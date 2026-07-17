"""
test_localization.py — 定位融合算法测试
验证 Section 8: 三层定位融合
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from localization import Localizer


class TestLocalizerInit(unittest.TestCase):
    """测试定位器初始化"""

    def test_init_defaults(self):
        loc = Localizer()
        self.assertEqual(loc.current_block, 21)  # 路径从21开始
        self.assertEqual(loc.path_index, 0)
        self.assertFalse(loc.is_mission_complete())
        self.assertFalse(loc.fine_tuning)

    def test_init_custom_params(self):
        loc = Localizer(
            green_drop_threshold=0.5,
            green_high=0.7,
            green_low=0.1,
            ocr_interval=2
        )
        self.assertEqual(loc.green_drop_threshold, 0.5)
        self.assertEqual(loc.green_high, 0.7)
        self.assertEqual(loc.green_low, 0.1)
        self.assertEqual(loc.ocr_interval, 2)


class TestOpticalFlowIntegration(unittest.TestCase):
    """测试Layer 1: 光流积分"""

    def setUp(self):
        self.loc = Localizer()

    def test_update_optical_flow(self):
        """测试光流更新"""
        self.loc.update_optical_flow(10.0, -5.0)
        self.assertEqual(self.loc.pos_x, 10.0)
        self.assertEqual(self.loc.pos_y, -5.0)

        self.loc.update_optical_flow(3.0, 7.0)
        self.assertEqual(self.loc.pos_x, 13.0)
        self.assertEqual(self.loc.pos_y, 2.0)

    def test_global_position_tracking(self):
        """测试全局位置追踪"""
        self.loc.update_optical_flow(50.0, 0.0)
        gx, gy = self.loc.get_global_position()
        self.assertEqual(gx, 50.0)
        self.assertEqual(gy, 0.0)

    def test_visual_anchor_corrects_flow_to_world_offset(self):
        self.loc.update_optical_flow(92.0, 47.0)

        correction = self.loc.calibrate_world_position(100.0, 50.0)

        self.assertEqual(correction, (8.0, 3.0))
        self.assertEqual(self.loc.get_flow_position(), (92.0, 47.0))
        self.assertEqual(self.loc.get_world_offset(), (8.0, 3.0))
        self.assertEqual(self.loc.get_global_position(), (100.0, 50.0))

        self.loc.update_optical_flow(10.0, -5.0)
        self.assertEqual(self.loc.get_flow_position(), (102.0, 42.0))
        self.assertEqual(self.loc.get_global_position(), (110.0, 45.0))

    def test_total_travel(self):
        """测试总飞行距离统计"""
        self.loc.update_optical_flow(10.0, 0.0)
        self.loc.update_optical_flow(0.0, 10.0)
        self.assertAlmostEqual(self.loc.total_travel_cm, 20.0)

    def test_reset_on_advance(self):
        """测试推进区块时重置相对位置"""
        self.loc.update_optical_flow(15.0, 15.0)
        self.loc.advance_block()
        self.assertEqual(self.loc.pos_x, 0.0)
        self.assertEqual(self.loc.pos_y, 0.0)
        # 但全局位置不应重置
        gx, gy = self.loc.get_global_position()
        self.assertEqual(gx, 15.0)
        self.assertEqual(gy, 15.0)


class TestBoundaryDetection(unittest.TestCase):
    """测试Layer 2: 颜色跳变边界检测"""

    def setUp(self):
        self.loc = Localizer(
            green_drop_threshold=0.4,
            green_high=0.6,
            green_low=0.2,
        )

    def test_no_boundary_same_level(self):
        """测试无跳变: 保持在绿色区域"""
        for _ in range(10):
            result = self.loc.check_boundary_crossed(0.65)
            self.assertFalse(result)

    def test_detect_green_to_gray(self):
        """测试检测到绿色→灰色跳变 (窗口比较法, 3绿+3灰=6帧触发)"""
        for _ in range(3):
            self.loc.check_boundary_crossed(0.65)
        crossed = False
        for _ in range(3):
            crossed = self.loc.check_boundary_crossed(0.15)
        self.assertTrue(crossed, "Should detect green→gray boundary")

    def test_detect_gray_to_green(self):
        """测试检测到灰色→绿色跳变"""
        for _ in range(3):
            self.loc.check_boundary_crossed(0.15)
        crossed = False
        for _ in range(3):
            crossed = self.loc.check_boundary_crossed(0.65)
        self.assertTrue(crossed, "Should detect gray→green boundary")

    def test_no_false_positive_on_noise(self):
        """测试噪声不触发误检"""
        # 在绿色区波动但不低于阈值
        for _ in range(10):
            result = self.loc.check_boundary_crossed(0.55)
            self.assertFalse(result, f"Noise triggered false positive")

    def test_debounce_works(self):
        """测试防抖机制"""
        # 单帧跳变不应触发
        self.loc.check_boundary_crossed(0.65)
        # 1帧低不应触发
        self.assertFalse(self.loc.check_boundary_crossed(0.15))
        # 再回到高也不触发
        self.assertFalse(self.loc.check_boundary_crossed(0.65))

    def test_boundary_counter(self):
        """测试跳变计数器"""
        for _ in range(3):
            self.loc.check_boundary_crossed(0.65)
        for _ in range(3):
            self.loc.check_boundary_crossed(0.15)
        self.assertEqual(self.loc.total_boundary_crossings, 1)


class TestOCRCalibration(unittest.TestCase):
    """测试Layer 3: OCR绝对校准"""

    def setUp(self):
        self.loc = Localizer()

    def test_ocr_match(self):
        """测试OCR识别到正确区块"""
        result = self.loc.apply_ocr(21)  # 应该在路径上
        self.assertTrue(result)
        self.assertEqual(self.loc.current_block, 21)
        self.assertEqual(self.loc.path_index, 0)
        self.assertEqual(self.loc.since_last_ocr, 0)

    def test_ocr_match_mid_path(self):
        """测试OCR识别到路径中间区块"""
        result = self.loc.apply_ocr(12)  # 在路径上
        self.assertTrue(result)
        self.assertEqual(self.loc.current_block, 12)
        # 路径索引应更新
        self.assertEqual(self.loc.path_index, self.loc.path.index(12))
        self.assertEqual(self.loc.pos_x, 0.0)
        self.assertEqual(self.loc.pos_y, 0.0)

    def test_ocr_no_match(self):
        """测试OCR识别到路径外区块"""
        result = self.loc.apply_ocr(999)  # 不在路径上
        self.assertFalse(result)
        self.assertEqual(self.loc.current_block, 21)  # 保持不变

    def test_ocr_none(self):
        """测试OCR返回None"""
        result = self.loc.apply_ocr(None)
        self.assertFalse(result)

    def test_ocr_resets_position(self):
        """测试OCR校准重置位置漂移"""
        self.loc.update_optical_flow(100.0, 100.0)
        self.loc.apply_ocr(21)
        self.assertEqual(self.loc.pos_x, 0.0)
        self.assertEqual(self.loc.pos_y, 0.0)
        self.assertEqual(self.loc.get_global_position(), (200, 50))

    def test_ocr_counter(self):
        """测试OCR校准计数器"""
        self.assertEqual(self.loc.ocr_calibrations, 0)
        self.loc.apply_ocr(21)
        self.assertEqual(self.loc.ocr_calibrations, 1)


class TestPathAdvancement(unittest.TestCase):
    """测试路径推进"""

    def setUp(self):
        self.loc = Localizer()

    def test_advance_block(self):
        """测试正常推进"""
        self.loc.advance_block()
        self.assertEqual(self.loc.current_block, 20)  # 路径[1]
        self.assertEqual(self.loc.path_index, 1)
        self.assertEqual(self.loc.since_last_ocr, 1)

    def test_advance_resets_pos(self):
        """测试推进重置相对位置"""
        self.loc.update_optical_flow(50.0, 0.0)
        self.loc.advance_block()
        self.assertEqual(self.loc.pos_x, 0.0)

    def test_is_complete(self):
        """测试任务完成判定"""
        self.assertFalse(self.loc.is_mission_complete())

        # 推进到最后一个区块
        for _ in range(len(self.loc.path) - 1):
            self.loc.advance_block()

        self.assertTrue(self.loc.is_mission_complete())

    def test_path_index_capped(self):
        """测试路径索引不超过上限"""
        for _ in range(len(self.loc.path) + 5):
            self.loc.advance_block()
        self.assertLessEqual(self.loc.path_index, len(self.loc.path) - 1)


class TestPositionQueries(unittest.TestCase):
    """测试位置查询"""

    def setUp(self):
        self.loc = Localizer()

    def test_get_current_target(self):
        self.assertEqual(self.loc.get_current_target(), 21)

    def test_get_next_target(self):
        nxt = self.loc.get_next_target()
        self.assertEqual(nxt, 20)

    def test_distance_to_home(self):
        self.loc.update_optical_flow(300.0, 400.0)
        d = self.loc.get_distance_to_home()
        self.assertAlmostEqual(d, 500.0, delta=0.5)

    def test_homing_direction(self):
        self.loc.update_optical_flow(0.0, 100.0)
        deg = self.loc.get_homing_direction_deg()
        # 在Y轴正方向, 返航方向应为270度 (指向Y轴负方向)
        self.assertAlmostEqual(deg, 270.0, delta=1.0)

    def test_should_do_ocr(self):
        """测试OCR间隔判断"""
        loc = Localizer(ocr_interval=3)
        self.assertFalse(loc.should_do_ocr())
        loc.advance_block()
        self.assertFalse(loc.should_do_ocr())
        loc.advance_block()
        self.assertFalse(loc.should_do_ocr())
        loc.advance_block()
        self.assertTrue(loc.should_do_ocr())


class TestFineTuning(unittest.TestCase):
    """测试微调功能"""

    def setUp(self):
        self.loc = Localizer()

    def test_fine_tuning_toggle(self):
        self.assertFalse(self.loc.fine_tuning)
        self.loc.start_fine_tuning()
        self.assertTrue(self.loc.fine_tuning)
        self.loc.stop_fine_tuning()
        self.assertFalse(self.loc.fine_tuning)

    def test_fine_tune_target(self):
        self.loc.set_fine_tune_target(5.0, -3.0)
        self.assertEqual(self.loc.fine_tune_dx, 5.0)
        self.assertEqual(self.loc.fine_tune_dy, -3.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
