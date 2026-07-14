"""
localization.py — 三层定位融合算法
Section 8: 三层定位融合

Layer 1 (光流死推算):  持续运行, 每50ms更新位置 → 积分漂移
Layer 2 (颜色跳变检测): 检测绿色→灰色过渡 → 确认跨过边界, 重置光流
Layer 3 (数字OCR):     读取区块编号 → 绝对定位, 校准所有误差
"""

import time
import logging
from typing import Optional, Tuple, List
from collections import deque

try:
    from .path_plan import PATH, BLOCK_POSITIONS, get_block_position, get_home_position
except ImportError:
    from path_plan import PATH, BLOCK_POSITIONS, get_block_position, get_home_position

logger = logging.getLogger('drone.loc')


class Localizer:
    """
    三层定位融合

    状态:
    - 光流积分位置 (相对当前区块)
    - 全局位置追踪 (相对起降点)
    - 颜色跳变检测
    - OCR绝对校准
    """

    def __init__(self, green_drop_threshold: float = 0.4,
                 green_high: float = 0.6,
                 green_low: float = 0.2,
                 ocr_interval: int = 4,
                 block_size_cm: float = 50.0):
        """
        Args:
            green_drop_threshold: 绿色占比下降阈值, 超过此值判定跨边界
            green_high: 高绿色占比=在区块内
            green_low: 低绿色占比=在灰色区域(非播撒区)
            ocr_interval: 每隔多少个区块执行一次OCR校准
            block_size_cm: 区块尺寸(cm)
        """
        self.path = PATH
        self.path_index = 0
        self.current_block = self.path[0] if self.path else 1

        # Layer 1: 光流位置积分
        self.pos_x = 0.0       # 相对当前区块原点(cm)
        self.pos_y = 0.0

        # 全局位置 (用于返航)
        self._init_global_position()

        # Layer 2: 颜色跳变
        self.prev_green_ratio = 0.0
        self.green_drop_threshold = green_drop_threshold
        self.green_high = green_high
        self.green_low = green_low
        self.green_history = deque(maxlen=20)
        self._boundary_debounce_threshold = 3  # 连续N帧确认

        # Layer 3: OCR校准
        self.last_ocr_block = None
        self.since_last_ocr = 0
        self.ocr_interval = ocr_interval
        self.block_size_cm = block_size_cm

        # 统计数据
        self.total_boundary_crossings = 0
        self.ocr_calibrations = 0
        self.total_travel_cm = 0.0

        # 移动方向追踪
        self.move_direction = 0

        # 微调状态
        self.fine_tuning = False
        self.fine_tune_dx = 0.0
        self.fine_tune_dy = 0.0

        logger.info(f"Localizer initialized: path={self.path[:3]}..., "
                    f"green_threshold={green_drop_threshold}")

    def _init_global_position(self):
        """起飞时记录全局零点"""
        self._global_pos_x = 0.0
        self._global_pos_y = 0.0

    # ── Layer 1: 光流积分 ─────────────────────────────────

    def update_optical_flow(self, dx_cm: float, dy_cm: float):
        """
        更新光流积分位置

        应在每次读取光流数据后调用
        """
        self.pos_x += dx_cm
        self.pos_y += dy_cm
        self._global_pos_x += dx_cm
        self._global_pos_y += dy_cm
        self.total_travel_cm += abs(dx_cm) + abs(dy_cm)

    # ── Layer 2: 颜色跳变 ─────────────────────────────────

    def check_boundary_crossed(self, green_ratio: float) -> bool:
        """
        检测是否跨越了区块边界

        使用窗口比较法: 比较最近N帧 vs 之前N帧的绿色占比,
        若全部一致从高区间跳到低区间(或反之), 则判定为跨边界

        Args:
            green_ratio: 当前帧绿色像素占比 [0.0, 1.0]

        Returns:
            True if boundary crossed
        """
        self.green_history.append(green_ratio)
        self.prev_green_ratio = green_ratio

        window = self._boundary_debounce_threshold
        if len(self.green_history) < window * 2:
            return False

        recent = list(self.green_history)[-window:]       # 最近N帧
        older = list(self.green_history)[-window*2:-window]  # 之前N帧

        # 绿色→灰色: 之前都在高区间, 最近都在低区间
        if all(r > self.green_high for r in older) and \
           all(r < self.green_low for r in recent):
            self.total_boundary_crossings += 1
            logger.debug(f"Boundary crossed: GREEN→GRAY "
                         f"(older={older[-1]:.3f}, recent={recent[-1]:.3f})")
            return True

        # 灰色→绿色: 之前都在低区间, 最近都在高区间
        if all(r < self.green_low for r in older) and \
           all(r > self.green_high for r in recent):
            self.total_boundary_crossings += 1
            logger.debug(f"Boundary crossed: GRAY→GREEN "
                         f"(older={older[-1]:.3f}, recent={recent[-1]:.3f})")
            return True

        return False

    # ── Layer 3: OCR绝对校准 ──────────────────────────────

    def apply_ocr(self, block_number: Optional[int]) -> bool:
        """
        OCR读取到了区块编号, 进行绝对校准

        完全重置漂移: 设置当前区块, 重新定位

        Args:
            block_number: OCR识别到的数字

        Returns:
            True if calibration was applied
        """
        if block_number is None:
            return False

        if block_number in self.path:
            self.current_block = block_number
            self.path_index = self.path.index(block_number)
            self.last_ocr_block = block_number
            self.since_last_ocr = 0
            self.pos_x = 0.0
            self.pos_y = 0.0
            block_pos = get_block_position(block_number)
            if block_pos is not None:
                self._global_pos_x, self._global_pos_y = block_pos
            self.ocr_calibrations += 1
            logger.info(f"OCR calibration: block={block_number}, "
                        f"path_index={self.path_index}")
            return True
        else:
            logger.warning(f"OCR read block {block_number}, not in path!")
            return False

    # ── 路径推进 ──────────────────────────────────────────

    def advance_block(self):
        """确认进入下一区块, 推进路径索引"""
        self.path_index += 1
        if self.path_index >= len(self.path):
            self.path_index = len(self.path) - 1
            logger.warning("Path index exceeded path length!")

        self.current_block = self.path[self.path_index]
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.since_last_ocr += 1

        logger.debug(f"Advanced to block {self.current_block} "
                     f"(index {self.path_index}/{len(self.path)})")

    # ── 查询接口 ──────────────────────────────────────────

    def get_current_target(self) -> int:
        """获取当前目标区块编号"""
        return self.current_block

    def get_next_target(self) -> Optional[int]:
        """获取下一个目标区块"""
        if self.path_index + 1 < len(self.path):
            return self.path[self.path_index + 1]
        return None

    def get_path_index(self) -> int:
        return self.path_index

    def is_mission_complete(self) -> bool:
        """所有区块是否已覆盖"""
        return self.path_index >= len(self.path) - 1

    def should_do_ocr(self) -> bool:
        """是否该尝试OCR校准了"""
        return self.since_last_ocr >= self.ocr_interval

    def get_position(self) -> Tuple[float, float]:
        """获取当前位置 (相对起飞点, cm)"""
        return self._global_pos_x, self._global_pos_y

    def get_global_position(self) -> Tuple[float, float]:
        """获取全局位置 (同get_position)"""
        return self._global_pos_x, self._global_pos_y

    def get_distance_to_home(self) -> float:
        """获取到起降点的直线距离(cm)"""
        x, y = self._global_pos_x, self._global_pos_y
        return (x ** 2 + y ** 2) ** 0.5

    def get_homing_direction_deg(self) -> float:
        """获取指向起降点的方向(度)"""
        import math
        x, y = self._global_pos_x, self._global_pos_y
        return math.degrees(math.atan2(-y, -x)) % 360

    # ── 位置误差计算 ──────────────────────────────────────

    def calc_offset_to_block(self, block_id: int) -> Tuple[float, float]:
        """
        计算当前位置到目标区块中心的偏移

        Returns:
            (dx_cm, dy_cm) 需要移动的距离
        """
        target_pos = get_block_position(block_id)
        if target_pos is None:
            return 0.0, 0.0

        current_pos = self.get_global_position()
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        return dx, dy

    def is_above_target(self, block_id: int, tolerance_cm: float = 10.0) -> bool:
        """检查是否已到达目标区块上方"""
        dx, dy = self.calc_offset_to_block(block_id)
        return (dx ** 2 + dy ** 2) ** 0.5 < tolerance_cm

    # ── 微调控制 ──────────────────────────────────────────

    def start_fine_tuning(self):
        """进入微调模式"""
        self.fine_tuning = True

    def stop_fine_tuning(self):
        """退出微调模式"""
        self.fine_tuning = False
        self.fine_tune_dx = 0.0
        self.fine_tune_dy = 0.0

    def set_fine_tune_target(self, dx_cm: float, dy_cm: float):
        """设置微调目标偏移"""
        self.fine_tune_dx = dx_cm
        self.fine_tune_dy = dy_cm

    # ── 统计 ──────────────────────────────────────────────

    def get_stats(self) -> dict:
        return {
            'current_block': self.current_block,
            'path_index': self.path_index,
            'total_path': len(self.path),
            'progress_pct': self.path_index / max(len(self.path) - 1, 1) * 100,
            'global_x': round(self._global_pos_x, 1),
            'global_y': round(self._global_pos_y, 1),
            'distance_to_home': round(self.get_distance_to_home(), 1),
            'boundary_crossings': self.total_boundary_crossings,
            'ocr_calibrations': self.ocr_calibrations,
            'total_travel_cm': round(self.total_travel_cm, 1),
        }
