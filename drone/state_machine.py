"""
state_machine.py — 无人机状态机
Section 10: 状态机设计

状态转换图:
  IDLE → ARM_UNLOCK → SET_PROGRAM_MODE → TAKEOFF → FIND_START
       → SPRAY ↔ NAVIGATE (循环直到全部完成)
       → RETURN_HOME → LAND → LOCK → DONE

异常状态: EMERGENCY (任意状态可触发)
"""

import math
import time
import logging
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Optional

try:
    from .path_plan import get_block_position, get_home_position
except ImportError:
    from path_plan import get_block_position, get_home_position

logger = logging.getLogger('drone.sm')


class FlightState(Enum):
    """飞行状态枚举"""
    IDLE = auto()               # 待命 (等待启动信号)
    ARM_UNLOCK = auto()         # 解锁电机
    SET_PROGRAM_MODE = auto()   # 切换到程控模式
    TAKEOFF = auto()            # 起飞至指定高度
    FIND_START = auto()         # 寻找起始区块 (A标记/区块21)
    SPRAY = auto()              # 撒药 (激光闪烁)
    NAVIGATE = auto()           # 移动到下一区块
    RETURN_HOME = auto()        # 返回起降点
    ALIGN_HOME = auto()         # 识别十字并将机体中心对准起降点
    LAND = auto()               # 着陆
    LOCK = auto()               # 加锁
    EMERGENCY = auto()          # 紧急状态
    DONE = auto()               # 任务完成


class NavigationPhase(Enum):
    """NAVIGATE内部子阶段。"""
    ROUTE = auto()
    ACQUIRE_GRAY = auto()
    WAIT_CORRECTION = auto()
    RESUME_ROUTE = auto()
    ARRIVAL = auto()


@dataclass
class ActiveRoute:
    target_block: int
    start_x: float
    start_y: float
    target_x: float
    target_y: float
    initial_distance_cm: float
    direction_deg: int
    started_at: float
    command_id: int


class DroneStateMachine:
    """
    无人机任务状态机

    职责:
    - 管理飞行状态转换
    - 每个状态的执行逻辑
    - 超时检测与重试
    - 任务进度跟踪
    """

    def __init__(self, mcu, camera, localizer, laser, config: dict):
        """
        Args:
            mcu: MCUSerial实例
            camera: Camera或OpenMVVision实例
            localizer: Localizer实例
            laser: LaserController实例 (可为None用于测试)
            config: 配置字典 (from config.get_config())
        """
        self.state = FlightState.IDLE
        self.mcu = mcu
        self.camera = camera
        self.localizer = localizer
        self.laser = laser
        self.cfg = config

        # 28个区块的访问记录 (下标1~28)
        self.visited = [False] * 29

        # 计时器
        self._state_start_time = time.time()
        self._mission_start_time = 0.0
        self._retry_count = 0
        self._max_retries = 3
        self._state_command_sent = False
        self._start_confirm_count = 0
        self._last_start_observation_time = 0.0
        self._start_block_confirmed = bool(config.get('dry_run', False))
        self._start_marker_center = None
        self._home_cross_center = None
        self._home_cross_confidence = 0.0
        self._home_cross_confirm_count = 0
        self._last_home_align_command = 0.0
        self._spray_started = False
        self._low_voltage_count = 0
        self._consecutive_ocr_timeouts = 0
        self._landing_low_alt_count = 0
        self._last_landing_alt_sequence = None
        self._gray_marker_center = None
        self._gray_marker_box = None
        self._gray_marker_confidence = 0.0
        self._gray_marker_sequence = None

        # 导航路线与灰色中心辅助校准。
        self._nav_phase = NavigationPhase.ROUTE
        self._active_route: Optional[ActiveRoute] = None
        self._move_command_id = 0
        self._nav_progress = 0.0
        self._nav_remaining_cm = 0.0
        self._nav_cross_track_cm = 0.0
        self._calibration_started_at = 0.0
        self._calibration_attempts = 0
        self._calibration_corrections = 0
        self._calibration_total_distance_cm = 0.0
        self._calibration_centers = deque(maxlen=max(
            1, int(config.get('gray_calibration_confirm_frames', 3)),
        ))
        self._last_gray_sequence = None
        self._last_calibration_command = 0.0
        self._last_calibration_error_cm = None
        self._calibration_error_forward_cm = 0.0
        self._calibration_error_right_cm = 0.0
        self._calibration_world_anchor_applied = False
        self._calibration_status = 'idle'
        self._calibration_last_action = 'WAITING'
        self._calibration_completed = False
        self._calibration_stats = {
            'commands': 0, 'successes': 0, 'degraded': 0,
            'timeouts': 0, 'skipped': 0, 'distance_cm': 0.0,
        }

        # 异常检测
        self._emergency_reason = ""

        # 性能统计
        self._state_history = []
        self._total_states = 0

        logger.info(f"State machine initialized: initial_state={self.state.name}")

    # ── 状态属性 ──────────────────────────────────────────

    @property
    def state_start_time(self) -> float:
        return time.time() - self._state_start_time

    @property
    def mission_time(self) -> float:
        if self._mission_start_time == 0:
            return 0.0
        return time.time() - self._mission_start_time

    @property
    def retry_count(self) -> int:
        return self._retry_count

    @property
    def is_completed(self) -> bool:
        return self.state == FlightState.DONE

    @property
    def is_emergency(self) -> bool:
        return self.state == FlightState.EMERGENCY

    @property
    def emergency_reason(self) -> str:
        return self._emergency_reason

    @property
    def visited_count(self) -> int:
        return sum(1 for v in self.visited[1:] if v)

    @property
    def total_blocks(self) -> int:
        return len(self.localizer.path)

    # ── 主循环 ────────────────────────────────────────────

    def run_iteration(self) -> FlightState:
        """
        每个循环周期调用一次。主程序限频20Hz；相机目标采集30fps，
        实际视觉处理帧率由预览与日志中的 FPS 指标给出。

        Returns:
            当前状态
        """
        # 读取串口数据 (必须最先调用, 否则所有传感器读数为0)
        self.mcu.poll()

        # 前往21号块同时找A和数字21；作业途中OCR仅作为非阻塞定位辅助；
        # 28号块完成后才启用起降十字识别。
        set_modes = getattr(self.camera, 'set_processing_modes', None)
        if callable(set_modes):
            expected_digit = None
            if self.state == FlightState.FIND_START:
                expected_digit = 21
            elif self.state == FlightState.NAVIGATE:
                expected_digit = next(
                    (bid for bid in self.localizer.path if not self.visited[bid]),
                    None,
                )
            set_modes(
                ocr=self.state in (FlightState.FIND_START, FlightState.NAVIGATE),
                start_marker=self.state == FlightState.FIND_START,
                expected_digit=expected_digit,
                state_label=self.state.name,
                home_cross=self.state in (
                    FlightState.RETURN_HOME, FlightState.ALIGN_HOME,
                ),
                gray_marker=(
                    self.state == FlightState.NAVIGATE
                    and self.cfg.get('gray_calibration_enabled', True)
                    and self._nav_progress >= self.cfg.get(
                        'gray_calibration_start_progress', 0.66,
                    )
                    and not self._calibration_completed
                ),
                navigation_details=self._navigation_details(),
            )

        # 获取视觉数据
        frame, green_ratio, ocr_result = self._get_vision_data()

        # 更新光流
        of_dx, of_dy = self.mcu.read_optical_flow()
        if of_dx != 0.0 or of_dy != 0.0:
            self.localizer.update_optical_flow(of_dx, of_dy)

        # 颜色跳变只作为区域边界观测，不直接推进离散路径索引。
        if green_ratio is not None and self.state not in (
                FlightState.IDLE, FlightState.ARM_UNLOCK,
                FlightState.SET_PROGRAM_MODE, FlightState.TAKEOFF,
                FlightState.EMERGENCY, FlightState.DONE):
            self.localizer.check_boundary_crossed(green_ratio)

        # OCR只在作业路径中低频辅助校准，不参与A点或返航判定。
        if (self.state == FlightState.NAVIGATE
            and self.localizer.should_do_ocr()
            and ocr_result is not None):
            self.localizer.apply_ocr(ocr_result)

        # 异常检测
        if self.state not in (FlightState.IDLE, FlightState.EMERGENCY,
                              FlightState.DONE):
            self._check_exceptions()

        # 执行当前状态
        state_handlers = {
            FlightState.IDLE: self._state_idle,
            FlightState.ARM_UNLOCK: self._state_arm_unlock,
            FlightState.SET_PROGRAM_MODE: self._state_set_program_mode,
            FlightState.TAKEOFF: self._state_takeoff,
            FlightState.FIND_START: self._state_find_start,
            FlightState.SPRAY: self._state_spray,
            FlightState.NAVIGATE: self._state_navigate,
            FlightState.RETURN_HOME: self._state_return_home,
            FlightState.ALIGN_HOME: self._state_align_home,
            FlightState.LAND: self._state_land,
            FlightState.LOCK: self._state_lock,
            FlightState.EMERGENCY: self._state_emergency,
            FlightState.DONE: self._state_done,
        }

        handler = state_handlers.get(self.state)
        if handler:
            handler(frame, green_ratio, ocr_result)

        return self.state

    # ── 状态处理函数 ──────────────────────────────────────

    def _state_idle(self, frame, green_ratio, ocr_result):
        """等待启动信号"""
        # 检查启动信号: AUX6开关 >1700us（AUX1~AUX5不可用）
        has_rc_channels = getattr(self.mcu, 'has_rc_channels', lambda: True)
        if self.cfg.get('auto_start', False) or self.cfg.get('dry_run', False) or \
            (has_rc_channels() and self.mcu.read_aux6() > 1700):
            logger.info("Start signal received!")
            self._transition(FlightState.ARM_UNLOCK)

    def _state_arm_unlock(self, frame, green_ratio, ocr_result):
        """解锁电机"""
        if not self._state_command_sent:
            logger.info("Sending unlock command...")
            if not self.mcu.send_cmd_unlock():
                self._emergency("Failed to send motor unlock command")
                return
            self._state_command_sent = True

        # V7 0x06 状态帧中 LOCKED: 1=已解锁，0=已锁定。
        # 真机优先等待遥测确认；旧测试桩或未提供新鲜状态接口时保留延时兼容。
        has_lock_status = getattr(self.mcu, 'has_recent_lock_status', None)
        status_is_recent = (
            has_lock_status() if callable(has_lock_status)
            else self._has_flight_status()
        )
        unlock_wait_s = self.cfg.get('unlock_wait_s', 2)
        if (status_is_recent and self.mcu.read_locked() == 1
                and self.state_start_time >= unlock_wait_s):
            self._transition(FlightState.SET_PROGRAM_MODE)
        elif self.state_start_time > self.cfg.get('unlock_timeout_s', 5):
            self._emergency("Motor unlock could not be confirmed")

    def _state_set_program_mode(self, frame, green_ratio, ocr_result):
        """切换到程控模式"""
        if not self._state_command_sent:
            logger.info("Setting program control mode...")
            if not self.mcu.send_cmd_mode(3):  # 程控模式
                self._emergency("Failed to send program mode command")
                return
            self._state_command_sent = True

        # C类移动指令只能在程控模式使用，必须确认模式3后才允许起飞。
        mode_switch_wait_s = self.cfg.get('mode_switch_wait_s', 1)
        if (self._has_flight_status() and self.mcu.read_mode() == 3
            and self.state_start_time >= mode_switch_wait_s):
            # 重置光流零点
            if not self.mcu.send_of_zero_reset():
                self._emergency("Failed to reset optical flow origin")
                return
            self.localizer._init_global_position()
            self._mission_start_time = time.time()
            self._transition(FlightState.TAKEOFF)
        elif self.state_start_time > self.cfg.get('mode_switch_timeout_s', 5):
            self._emergency("Program control mode could not be confirmed")

    def _state_takeoff(self, frame, green_ratio, ocr_result):
        """起飞至目标高度"""
        takeoff_height = self.cfg['takeoff_height_cm']
        tolerance = self.cfg['takeoff_height_tolerance']

        # 仅第一次进入时发送指令
        if not self._state_command_sent:
            logger.info(f"Taking off to {takeoff_height}cm...")
            if not self.mcu.send_cmd_takeoff(takeoff_height):
                self._emergency("Failed to send takeoff command")
                return
            self._state_command_sent = True

        # 检查高度
        alt = self.mcu.read_altitude()
        if abs(alt - takeoff_height) <= tolerance and alt > 0:
            logger.info(f"Takeoff complete: altitude={alt}cm")
            self._transition(FlightState.FIND_START)
        elif self.state_start_time > self.cfg['takeoff_timeout_s']:
            # 超时重试
            self._retry_count += 1
            if self._retry_count > self._max_retries:
                self._emergency("Takeoff failed after max retries")
            else:
                logger.warning(f"Takeoff timeout, retry {self._retry_count}/{self._max_retries}")
                self.mcu.send_cmd_takeoff(takeoff_height)
                self._state_start_time = time.time()

    def _state_find_start(self, frame, green_ratio, ocr_result):
        """按飞控位置前往21号块，用A标记或数字21确认起点。"""
        # 相机装在机尾：机体需越过区块中心25cm，才能让相机位于A点正上方。
        target_pos = get_block_position(21)
        if target_pos is None:
            self._emergency("Cannot find block 21 position")
            return
        target_pos = self._camera_centered_body_target(*target_pos)

        distance, direction = self._calc_move_to_target(target_pos[0], target_pos[1])

        position_ready = distance < 10 or self.cfg.get('manual_navigation', False)
        if position_ready:
            start_seen = (
                self._start_marker_center is not None or ocr_result == 21
            )
            if start_seen:
                now = time.monotonic()
                confirmation_window = self.cfg.get(
                    'start_block_confirm_window_s', 5.0,
                )
                if (self._last_start_observation_time > 0.0
                        and now - self._last_start_observation_time
                        > confirmation_window):
                    logger.info(
                        "Start confirmation window expired after %.1fs; "
                        "restarting observations",
                        now - self._last_start_observation_time,
                    )
                    self._start_confirm_count = 0
                self._start_confirm_count += 1
                self._last_start_observation_time = now
                logger.info(
                    "Start observation %d/%d (%s)",
                    self._start_confirm_count,
                    self.cfg.get('start_block_confirm_frames', 3),
                    "A marker" if self._start_marker_center is not None
                    else "digit 21",
                )
            elif (self._start_confirm_count > 0
                  and time.monotonic() - self._last_start_observation_time
                  > self.cfg.get('start_block_confirm_window_s', 5.0)):
                logger.info("Start confirmation window expired; observations reset")
                self._start_confirm_count = 0
                self._last_start_observation_time = 0.0

            if self.cfg.get('dry_run', False) or self._start_confirm_count >= \
                    self.cfg.get('start_block_confirm_frames', 3):
                if self.cfg.get('dry_run', False) or self.localizer.apply_ocr(21):
                    self._start_block_confirmed = True
                    logger.info("Start block 21 confirmed by vision")
                    self._transition(FlightState.SPRAY)
        else:
            # 发送移动指令
            if not self._state_command_sent:
                self.mcu.send_cmd_move(distance, self.cfg['move_speed'], direction)
                self._state_command_sent = True

        if self.state_start_time > self.cfg.get('start_block_timeout_s', 30):
            self._emergency("Block 21 was not confirmed; laser remains disabled")

    def _state_spray(self, frame, green_ratio, ocr_result):
        """撒药 (激光闪烁)"""
        cur_block = self.localizer.get_current_target()

        if not self._start_block_confirmed:
            laser_off = getattr(self.laser, 'off', None)
            if callable(laser_off):
                laser_off()
            self._emergency("Laser interlock: block 21 not confirmed")
            return

        if not self.visited[cur_block] and not self._spray_started:
            if self.laser is not None:
                started = self.laser.blink(
                    count=self.cfg.get('laser_count', 2),
                    period_ms=self.cfg.get('laser_period', 1500),
                )
                if started is False:
                    self._emergency("Laser was busy; spray cycle did not start")
                    return
            self._spray_started = True
            return

        if not self.visited[cur_block]:
            is_blinking = getattr(self.laser, 'is_blinking', None)
            if callable(is_blinking) and is_blinking():
                return
            self.visited[cur_block] = True
            logger.info(f"[SPRAY] Block {cur_block} sprayed "
                        f"({self.visited_count}/{self.total_blocks})")

        # 检查是否全部完成
        if self.visited_count >= self.total_blocks:
            logger.info("All blocks sprayed! Returning home.")
            self._transition(FlightState.RETURN_HOME)
        else:
            self._transition(FlightState.NAVIGATE)

    def _start_route(self, target_block: int, target_pos: tuple[float, float]) -> None:
        cur_x, cur_y = self.localizer.get_global_position()
        distance, direction = self._calc_move_to_target(*target_pos)
        self._move_command_id += 1
        self._active_route = ActiveRoute(
            target_block=target_block,
            start_x=cur_x,
            start_y=cur_y,
            target_x=target_pos[0],
            target_y=target_pos[1],
            initial_distance_cm=max(float(distance), 1.0),
            direction_deg=direction,
            started_at=time.time(),
            command_id=self._move_command_id,
        )
        self._nav_phase = NavigationPhase.ROUTE
        self._nav_progress = 0.0
        self._reset_gray_calibration()
        logger.info(
            "[NAV] route=%d target=%d start=(%.1f,%.1f) target_pos=(%.1f,%.1f) "
            "distance=%dcm direction=%d° speed=%dcm/s",
            self._active_route.command_id, target_block, cur_x, cur_y,
            target_pos[0], target_pos[1], distance, direction,
            self.cfg['move_speed'],
        )

    def _reset_gray_calibration(self) -> None:
        self._calibration_started_at = 0.0
        self._calibration_attempts = 0
        self._calibration_corrections = 0
        self._calibration_total_distance_cm = 0.0
        self._calibration_centers.clear()
        self._last_gray_sequence = None
        self._last_calibration_command = 0.0
        self._last_calibration_error_cm = None
        self._calibration_error_forward_cm = 0.0
        self._calibration_error_right_cm = 0.0
        self._calibration_world_anchor_applied = False
        self._calibration_status = 'waiting'
        self._calibration_last_action = 'WAITING FOR 66%'
        self._calibration_completed = False

    def _update_route_metrics(self) -> None:
        route = self._active_route
        if route is None:
            self._nav_progress = 0.0
            self._nav_remaining_cm = 0.0
            self._nav_cross_track_cm = 0.0
            return
        cur_x, cur_y = self.localizer.get_global_position()
        vx = route.target_x - route.start_x
        vy = route.target_y - route.start_y
        length_sq = vx * vx + vy * vy
        if length_sq <= 1e-6:
            self._nav_progress = 1.0
            self._nav_cross_track_cm = 0.0
        else:
            px = cur_x - route.start_x
            py = cur_y - route.start_y
            projected = (px * vx + py * vy) / length_sq
            self._nav_progress = max(0.0, min(1.0, projected))
            self._nav_cross_track_cm = abs(px * vy - py * vx) / math.sqrt(length_sq)
        self._nav_remaining_cm = math.hypot(
            route.target_x - cur_x, route.target_y - cur_y,
        )
        if self.cfg.get('manual_navigation', False):
            expected_s = route.initial_distance_cm / max(1.0, self.cfg['move_speed'])
            elapsed_progress = (time.time() - route.started_at) / max(1.0, expected_s)
            self._nav_progress = max(
                self._nav_progress, min(0.95, elapsed_progress),
            )

    def _calibration_tolerance(self) -> float:
        if self._calibration_corrections == 0:
            return self.cfg.get('gray_calibration_first_tolerance_cm', 7.0)
        return self.cfg.get('gray_calibration_later_tolerance_cm', 11.0)

    def _finish_gray_calibration(self, outcome: str, reason: str) -> None:
        if self._calibration_completed:
            return
        self._calibration_completed = True
        self._calibration_status = outcome
        self._calibration_last_action = reason
        if outcome == 'aligned':
            self._calibration_stats['successes'] += 1
        elif outcome == 'skipped':
            self._calibration_stats['skipped'] += 1
        else:
            self._calibration_stats['degraded'] += 1
            if outcome == 'timeout':
                self._calibration_stats['timeouts'] += 1
        logger.info(
            "[GRAY_CAL] target=%s event=completed outcome=%s progress=%.1f%% "
            "attempts=%d corrections=%d total=%.1fcm reason=%s",
            self._active_route.target_block if self._active_route else '-',
            outcome, self._nav_progress * 100.0, self._calibration_attempts,
            self._calibration_corrections,
            self._calibration_total_distance_cm, reason,
        )

    def _anchor_gray_target_to_world(self, route: ActiveRoute) -> None:
        """确认数字中心对准后，用该区块真实坐标校准0x08→世界坐标偏置。"""
        if self._calibration_world_anchor_applied:
            return
        correction = self.localizer.calibrate_world_position(
            route.target_x, route.target_y, source='visual',
        )
        self._calibration_world_anchor_applied = True
        self._calibration_last_action = (
            f'WORLD ANCHOR {correction[0]:+.1f},{correction[1]:+.1f}cm'
        )
        logger.info(
            "[GRAY_CAL] target=%d event=world-anchor correction_x=%+.1fcm "
            "correction_y=%+.1fcm flow=%s offset=%s",
            route.target_block, correction[0], correction[1],
            self.localizer.get_flow_position(),
            self.localizer.get_world_offset(),
        )

    def _run_gray_calibration(self) -> bool:
        """执行导航中段校准；返回True表示本轮应等待校准。"""
        route = self._active_route
        if route is None:
            return False
        if self._calibration_completed:
            return False
        if not self.cfg.get('gray_calibration_enabled', True):
            self._finish_gray_calibration('skipped', 'disabled')
            return False
        if self.camera is None:
            self._finish_gray_calibration(
                'skipped', 'no camera / simulated calibration completed',
            )
            return False
        if self._nav_progress < self.cfg.get('gray_calibration_start_progress', 0.66):
            return False

        now = time.time()
        if self._calibration_started_at == 0.0:
            self._calibration_started_at = now
            self._nav_phase = NavigationPhase.ACQUIRE_GRAY
            self._calibration_status = 'acquiring'
            self._calibration_last_action = 'ACQUIRING GRAY CENTER'
            logger.info(
                "[GRAY_CAL] target=%d event=armed progress=%.1f%% remaining=%.1fcm",
                route.target_block, self._nav_progress * 100.0,
                self._nav_remaining_cm,
            )

        if now - self._calibration_started_at >= self.cfg.get(
                'gray_calibration_total_timeout_s', 6.0):
            self._finish_gray_calibration('timeout', 'total timeout; continue route')
            return False

        if self._nav_phase == NavigationPhase.WAIT_CORRECTION:
            if now - self._last_calibration_command < self.cfg.get(
                    'gray_calibration_command_interval_s', 0.8):
                return True
            self._nav_phase = NavigationPhase.ACQUIRE_GRAY
            self._calibration_centers.clear()

        if (self._gray_marker_center is None
                or self._gray_marker_confidence < self.cfg.get(
                    'gray_calibration_min_confidence', 0.55)):
            if now - self._calibration_started_at >= self.cfg.get(
                    'gray_calibration_acquire_timeout_s', 2.5):
                self._finish_gray_calibration(
                    'degraded', 'gray center unavailable; continue route',
                )
                return False
            return True

        if self._gray_marker_sequence == self._last_gray_sequence:
            return True
        self._last_gray_sequence = self._gray_marker_sequence
        self._calibration_attempts += 1
        self._calibration_centers.append(self._gray_marker_center)
        logger.info(
            "[GRAY_CAL] target=%d event=observed progress=%.1f%% center=(%.1f,%.1f) "
            "box=%s confidence=%.2f stable=%d/%d",
            route.target_block, self._nav_progress * 100.0,
            self._gray_marker_center[0], self._gray_marker_center[1],
            self._gray_marker_box, self._gray_marker_confidence,
            len(self._calibration_centers),
            self.cfg.get('gray_calibration_confirm_frames', 3),
        )
        required = self.cfg.get('gray_calibration_confirm_frames', 3)
        if len(self._calibration_centers) < required:
            return True

        centers = list(self._calibration_centers)
        median_x = sorted(point[0] for point in centers)[len(centers) // 2]
        median_y = sorted(point[1] for point in centers)[len(centers) // 2]
        max_spread = max(
            math.hypot(point[0] - median_x, point[1] - median_y)
            for point in centers
        )
        if max_spread > self.cfg.get('gray_calibration_stability_px', 28.0):
            self._calibration_centers.clear()
            self._calibration_last_action = f'UNSTABLE {max_spread:.0f}px'
            logger.warning(
                "[GRAY_CAL] target=%d event=rejected reason=unstable spread=%.1fpx",
                route.target_block, max_spread,
            )
            return True

        altitude = max(float(self.mcu.read_altitude()), 1.0)
        cx = self.cfg.get('camera_principal_x_px', 720.0)
        cy = self.cfg.get('camera_principal_y_px', 540.0)
        fx = self.cfg.get('camera_focal_x_px', 800.0)
        fy = self.cfg.get('camera_focal_y_px', 800.0)
        observed_forward = (cy - median_y) * altitude / fy
        observed_right = (median_x - cx) * altitude / fx
        error_forward = observed_forward - self.cfg.get(
            'work_point_forward_offset_cm',
            self.cfg.get('camera_tail_forward_offset_cm', 20.0),
        )
        error_right = observed_right - self.cfg.get(
            'work_point_right_offset_cm', 0.0,
        )
        error_distance = math.hypot(error_forward, error_right)
        self._calibration_error_forward_cm = error_forward
        self._calibration_error_right_cm = error_right
        tolerance = self._calibration_tolerance()
        if error_distance <= tolerance:
            self._anchor_gray_target_to_world(route)
            self._finish_gray_calibration(
                'aligned', f'aligned within {tolerance:.1f}cm',
            )
            return False

        max_corrections = self.cfg.get('gray_calibration_max_corrections', 3)
        if self._calibration_corrections >= max_corrections:
            self._finish_gray_calibration(
                'degraded', 'correction limit reached; continue route',
            )
            return False
        if (self._last_calibration_error_cm is not None
                and error_distance > self._last_calibration_error_cm
                / self.cfg.get('gray_calibration_min_improvement_ratio', 0.90)):
            self._finish_gray_calibration(
                'degraded', 'error did not improve; continue route',
            )
            return False

        max_step = self.cfg.get(
            'gray_calibration_first_max_step_cm', 15.0,
        ) if self._calibration_corrections == 0 else self.cfg.get(
            'gray_calibration_later_max_step_cm', 10.0,
        )
        remaining_budget = self.cfg.get(
            'gray_calibration_max_total_distance_cm', 35.0,
        ) - self._calibration_total_distance_cm
        step = min(error_distance, max_step, remaining_budget)
        if step < 1.0:
            self._finish_gray_calibration(
                'degraded', 'correction distance budget exhausted',
            )
            return False
        direction = int(math.degrees(math.atan2(error_right, error_forward)) % 360)
        auto_command = self.cfg.get('gray_calibration_auto_command', True)
        if self.cfg.get('manual_navigation', False):
            auto_command = self.cfg.get(
                'manual_gray_calibration_auto_command', True,
            )
        source = 'manual-simulated' if self.cfg.get(
            'manual_navigation', False,
        ) else ('simulated' if getattr(self.mcu, 'dry_run', False) else 'real')
        if auto_command:
            self.mcu.send_cmd_move(
                max(1, int(round(step))), self.cfg['move_speed'], direction,
            )
        self._calibration_corrections += 1
        self._calibration_total_distance_cm += step
        self._calibration_stats['commands'] += int(auto_command)
        self._calibration_stats['distance_cm'] += step
        self._last_calibration_error_cm = error_distance
        self._last_calibration_command = now
        self._nav_phase = NavigationPhase.WAIT_CORRECTION
        self._calibration_status = 'correcting'
        self._calibration_last_action = (
            f"{'MOVE' if auto_command else 'SUGGEST'} {step:.1f}cm @{direction}°"
        )
        logger.info(
            "[GRAY_CAL] target=%d event=commanded source=%s progress=%.1f%% "
            "center=(%.1f,%.1f) confidence=%.2f error_forward=%+.1fcm "
            "error_right=%+.1fcm error=%.1fcm tolerance=%.1fcm correction=%d/%d "
            "command=%s distance=%.1fcm direction=%d° total=%.1fcm",
            route.target_block, source, self._nav_progress * 100.0,
            median_x, median_y, self._gray_marker_confidence,
            error_forward, error_right, error_distance, tolerance,
            self._calibration_corrections, max_corrections,
            'sent' if auto_command else 'suggested', step, direction,
            self._calibration_total_distance_cm,
        )
        return True

    def _resume_route_after_gray_calibration(self) -> bool:
        """灰色修正会覆盖原平移指令，完成后重新下发剩余主路线。"""
        route = self._active_route
        if route is None or self._nav_phase != NavigationPhase.RESUME_ROUTE:
            return False

        distance, direction = self._calc_move_to_target(
            route.target_x, route.target_y,
        )
        if distance < 10:
            self._nav_phase = NavigationPhase.ARRIVAL
            return False

        logger.info(
            "[NAV] route=%d event=resume-after-gray target=%d "
            "remaining=%dcm direction=%d°",
            route.command_id, route.target_block, distance, direction,
        )
        if not self.mcu.send_cmd_move(
                distance, self.cfg['move_speed'], direction):
            self._emergency("Failed to resume route after gray calibration")
            return True
        route.started_at = time.time()
        route.initial_distance_cm = max(float(distance), 1.0)
        route.start_x, route.start_y = self.localizer.get_global_position()
        route.direction_deg = direction
        self._nav_phase = NavigationPhase.ROUTE
        self._state_command_sent = True
        return True

    def _state_navigate(self, frame, green_ratio, ocr_result):
        """移动到下一未访问区块"""
        # 找下一个未访问区块 (按预设路径)
        next_block = None
        for bid in self.localizer.path:
            if not self.visited[bid]:
                next_block = bid
                break

        if next_block is None:
            logger.info("No unvisited blocks remaining, returning home")
            self._transition(FlightState.RETURN_HOME)
            return

        # 计算移动指令
        target_pos = get_block_position(next_block)
        if target_pos is None:
            self._transition(FlightState.RETURN_HOME)
            return

        if (self._active_route is None
                or self._active_route.target_block != next_block):
            self._start_route(next_block, target_pos)

        self._update_route_metrics()

        # 人工移动场测不使用模拟光流判定到达。程序只记录飞控移动
        # 指令，并在真实相机连续流程中识别到目标编号后推进。
        if self.cfg.get('manual_navigation', False):
            if not self._state_command_sent:
                distance, direction = self._calc_move_to_target(
                    target_pos[0], target_pos[1]
                )
                logger.info(
                    "[FIELD] 请人工移动到区块 %d (建议方向=%d°, 距离=%dcm)",
                    next_block, direction, distance,
                )
                self.mcu.send_cmd_move(
                    distance, self.cfg['move_speed'], direction
                )
                self._state_command_sent = True

            if self._run_gray_calibration():
                return

            if ocr_result == next_block:
                logger.info("[FIELD] OCR确认到达区块 %d", next_block)
                self._consecutive_ocr_timeouts = 0
                self.localizer.apply_ocr(next_block)
                if not self._calibration_completed:
                    self._finish_gray_calibration(
                        'skipped', 'OCR confirmed before gray calibration',
                    )
                self._transition(FlightState.SPRAY)
            elif self.state_start_time > self.cfg.get(
                    'manual_navigation_timeout_s', 15):
                self._consecutive_ocr_timeouts += 1
                max_timeouts = self.cfg.get(
                    'max_consecutive_ocr_timeouts', 3,
                )
                if self._consecutive_ocr_timeouts >= max_timeouts:
                    self._emergency(
                        f"OCR timed out for {self._consecutive_ocr_timeouts} "
                        "consecutive blocks"
                    )
                    return

                logger.warning(
                    "[FIELD] 区块 %d 等待OCR超过%.0fs，按预定位置播撒 "
                    "(连续超时 %d/%d)",
                    next_block,
                    self.cfg.get('manual_navigation_timeout_s', 15),
                    self._consecutive_ocr_timeouts,
                    max_timeouts,
                )
                self.localizer.apply_ocr(next_block)
                if not self._calibration_completed:
                    self._finish_gray_calibration(
                        'degraded', 'manual timeout; continue mission',
                    )
                self._transition(FlightState.SPRAY)
            return

        distance, direction = self._calc_move_to_target(target_pos[0], target_pos[1])

        if self._run_gray_calibration():
            return

        # V7协议规定移动指令与一键控制互斥、同类动作只执行最新指令。
        # 灰色小步修正会中断原主路线，因此校准结束后必须显式恢复剩余路线。
        if (self._calibration_completed
                and self._calibration_corrections > 0
                and self._nav_phase in (
                    NavigationPhase.ACQUIRE_GRAY,
                    NavigationPhase.WAIT_CORRECTION,
                )):
            self._nav_phase = NavigationPhase.RESUME_ROUTE
        if self._resume_route_after_gray_calibration():
            return

        if distance < 10:
            # 到达前保证灰色校准已经完成、跳过或降级。
            if not self._calibration_completed:
                self._finish_gray_calibration(
                    'skipped', 'route reached target before calibration',
                )
            self._nav_phase = NavigationPhase.ARRIVAL
            # OCR可能已把离散路径索引校准到目标块；此时不能再次推进，
            # 否则会跳过一个区块。未校准到目标时才正常推进。
            if self.localizer.get_current_target() != next_block:
                self.localizer.advance_block()
            self._transition(FlightState.SPRAY)
        else:
            # 发送移动指令
            if not self._state_command_sent:
                self.mcu.send_cmd_move(distance, self.cfg['move_speed'], direction)
                self._state_command_sent = True

            # 超时处理
            if self.state_start_time > self.cfg['block_timeout']:
                logger.warning(f"Navigate timeout to block {next_block}")
                self._retry_count += 1
                if self._retry_count > self._max_retries:
                    self._emergency(
                        f"Failed to reach block {next_block} after max retries"
                    )
                else:
                    self.mcu.send_cmd_move(distance, self.cfg['move_speed'], direction)
                    self._state_start_time = time.time()

    def _state_return_home(self, frame, green_ratio, ocr_result):
        """返回起降点"""
        hx, hy = get_home_position()
        # 与A点一致，粗返航先把机体前移到使机尾相机位于十字上方的位置；
        # ALIGN_HOME 随后用视觉对同一目标关系做精确闭环。
        hx, hy = self._camera_centered_body_target(hx, hy)
        distance, direction = self._calc_move_to_target(hx, hy)

        # 28号块完成后一路识别十字。稳定检出时直接转精对准，可减少
        # 光流累计误差；否则到地图估计位置附近再进入精对准。
        cross_seen = (
            self._home_cross_center is not None
            and self._home_cross_confidence >= self.cfg.get(
                'home_cross_min_confidence', 0.58,
            )
        )
        if cross_seen or distance < 20:
            logger.info(
                "Starting visual home alignment (cross=%s, estimate=%dcm)",
                cross_seen, distance,
            )
            self._transition(FlightState.ALIGN_HOME)
        else:
            if not self._state_command_sent:
                self.mcu.send_cmd_move(
                    distance, self.cfg['return_home_speed_cmps'], direction
                )
                self._state_command_sent = True

            # 超时: 仍然降落
            if self.state_start_time > distance / self.cfg['move_speed'] + 10:
                logger.warning("Return estimate timeout; searching for home cross")
                self._transition(FlightState.ALIGN_HOME)

    def _state_align_home(self, frame, green_ratio, ocr_result):
        """识别起降十字，并补偿机尾相机偏移后对准机体几何中心。"""
        center = self._home_cross_center
        confidence = self._home_cross_confidence
        min_confidence = self.cfg.get('home_cross_min_confidence', 0.58)
        if center is None or confidence < min_confidence:
            self._home_cross_confirm_count = 0
            if self.state_start_time > self.cfg.get('home_align_timeout_s', 30):
                self._emergency("Home cross not found; landing inhibited")
            return

        alt = max(float(self.mcu.read_altitude()), 1.0)
        cx = self.cfg.get('camera_principal_x_px', 720.0)
        cy = self.cfg.get('camera_principal_y_px', 540.0)
        fx = self.cfg.get('camera_focal_x_px', 800.0)
        fy = self.cfg.get('camera_focal_y_px', 800.0)

        # 图像已旋转180°且画面上方为机头方向。机体中心对准十字时，
        # 机尾相机看到的十字应位于画面主点上方对应25cm的位置。
        observed_forward = (cy - center[1]) * alt / fy
        # 移动机体向前会让地面十字在画面中向下移动，因此控制误差为
        # “观测值-目标值”。十字位于画面中心时 observed=0，需后退25cm。
        error_forward = observed_forward \
            - self.cfg.get('home_target_up_offset_cm', 25.0)
        error_right = (center[0] - cx) * alt / fx
        distance = math.hypot(error_forward, error_right)
        tolerance = self.cfg.get('home_align_tolerance_cm', 8.0)

        if distance <= tolerance:
            self._home_cross_confirm_count += 1
            if self._home_cross_confirm_count >= self.cfg.get(
                    'home_cross_confirm_frames', 3):
                logger.info(
                    "Home cross aligned: forward=%.1fcm right=%.1fcm",
                    error_forward, error_right,
                )
                self._transition(FlightState.LAND)
            return

        self._home_cross_confirm_count = 0
        now = time.time()
        if now - self._last_home_align_command < 1.0:
            return
        step = min(distance, self.cfg.get('home_align_max_step_cm', 30))
        direction = int(math.degrees(math.atan2(error_right, error_forward)) % 360)
        logger.info(
            "Home align correction: %.1fcm @ %d° (forward=%.1f right=%.1f)",
            step, direction, error_forward, error_right,
        )
        self.mcu.send_cmd_move(
            max(1, int(round(step))), self.cfg['move_speed'], direction,
        )
        self._last_home_align_command = now

    def _state_land(self, frame, green_ratio, ocr_result):
        """着陆"""
        if not self._state_command_sent:
            logger.info("Landing...")
            if not self.mcu.send_cmd_land():
                self._emergency("Failed to send landing command")
                return
            self._state_command_sent = True

        alt = self.mcu.read_altitude()
        if self._landing_confirmed():
            logger.info(f"Land complete: altitude={alt}cm")
            self._transition(FlightState.LOCK)
        elif self.state_start_time > self.cfg['land_timeout_s']:
            self._emergency("Land timeout")

    def _state_lock(self, frame, green_ratio, ocr_result):
        """加锁"""
        if not self._state_command_sent:
            logger.info("Locking motors...")
            if not self.mcu.send_cmd_lock():
                self._emergency("Failed to send motor lock command")
                return
            self._state_command_sent = True

        has_lock_status = getattr(self.mcu, 'has_recent_lock_status', None)
        status_is_recent = (
            has_lock_status() if callable(has_lock_status)
            else self._has_flight_status()
        )
        # V7 状态语义：0=已锁定，1=已解锁。
        if status_is_recent and self.mcu.read_locked() == 0:
            logger.info("=== MISSION COMPLETE ===")
            self._transition(FlightState.DONE)
        elif self.state_start_time > self.cfg.get('lock_timeout_s', 5):
            self._emergency("Motor lock could not be confirmed")

    def _state_emergency(self, frame, green_ratio, ocr_result):
        """
        紧急状态处理

        执行紧急降落流程
        """
        if not self._state_command_sent:
            logger.critical(f"EMERGENCY: {self._emergency_reason}")
            if not self.mcu.send_cmd_land():
                logger.critical("Emergency landing command could not be sent")
            self._state_command_sent = True

        if self._landing_confirmed():
            logger.critical("Emergency landing confirmed; locking motors")
            self._transition(FlightState.LOCK)
        elif self.state_start_time > self.cfg['land_timeout_s']:
            logger.critical("Emergency landing could not be confirmed; leaving motors unlocked")
            self._transition(FlightState.DONE)

    def _state_done(self, frame, green_ratio, ocr_result):
        """任务完成, 静止"""
        pass

    # ── 状态转换 ──────────────────────────────────────────

    def _transition(self, new_state: FlightState):
        """执行状态转换"""
        old_state = self.state
        logger.info(f"[STATE] {old_state.name} → {new_state.name} "
                    f"(elapsed: {self.state_start_time:.1f}s)")

        self.state = new_state
        self._state_start_time = time.time()
        self._retry_count = 0
        self._state_command_sent = False
        self._spray_started = False
        if old_state == FlightState.NAVIGATE and new_state != FlightState.NAVIGATE:
            self._active_route = None
            self._nav_phase = NavigationPhase.ROUTE
            self._nav_progress = 0.0
            self._nav_remaining_cm = 0.0
        if new_state in (FlightState.LAND, FlightState.EMERGENCY):
            self._landing_low_alt_count = 0
            self._last_landing_alt_sequence = None
        self._total_states += 1
        self._state_history.append((old_state, new_state, time.time()))

        if new_state in (
            FlightState.RETURN_HOME, FlightState.ALIGN_HOME,
            FlightState.LAND, FlightState.LOCK,
            FlightState.EMERGENCY, FlightState.DONE):
            disable_laser = getattr(self.laser, 'disable', None)
            if callable(disable_laser):
                disable_laser()

    def _emergency(self, reason: str):
        """触发紧急状态"""
        if self.state in (FlightState.EMERGENCY, FlightState.DONE):
            return
        self._emergency_reason = reason
        logger.critical(f"Emergency triggered: {reason}")
        self._transition(FlightState.EMERGENCY)

    # ── 移动计算 ──────────────────────────────────────────

    def _camera_centered_body_target(
            self, target_x: float, target_y: float) -> tuple[float, float]:
        """将地面视觉目标转换为机体目标，使机尾相机位于目标正上方。"""
        return (
            target_x + self.cfg.get('camera_tail_forward_offset_cm', 25.0),
            target_y,
        )

    def _calc_move_to_target(self, target_x: float, target_y: float) -> tuple[int, int]:
        """计算从当前位置到目标位置的距离(cm)和方向(度)"""
        cur_x, cur_y = self.localizer.get_global_position()
        dx = target_x - cur_x
        dy = target_y - cur_y
        distance = int(math.sqrt(dx ** 2 + dy ** 2))
        direction = int(math.degrees(math.atan2(dy, dx)) % 360)
        return distance, direction

    def _navigation_details(self) -> dict:
        return {
            'target_block': (
                self._active_route.target_block if self._active_route else '-'
            ),
            'phase': self._nav_phase.name,
            'progress': self._nav_progress,
            'remaining_cm': self._nav_remaining_cm,
            'cross_track_cm': self._nav_cross_track_cm,
            'calibration_attempts': self._calibration_corrections,
            'calibration_limit': self.cfg.get(
                'gray_calibration_max_corrections', 3,
            ),
            'calibration_tolerance_cm': self._calibration_tolerance(),
            'error_forward_cm': self._calibration_error_forward_cm,
            'error_right_cm': self._calibration_error_right_cm,
            'calibration_status': self._calibration_status,
            'last_action': self._calibration_last_action,
        }

    def _has_flight_status(self) -> bool:
        checker = getattr(self.mcu, 'has_flight_status', None)
        return bool(checker()) if callable(checker) else True

    def _landing_confirmed(self) -> bool:
        """仅用降落后的多个新鲜独立高度样本确认已经落地。"""
        if self.state_start_time < self.cfg.get('land_min_wait_s', 3):
            return False

        read_sample = getattr(self.mcu, 'read_altitude_sample', None)
        if callable(read_sample):
            sample: Any = read_sample()
            altitude = sample[0]
            sequence = sample[1]
            age = sample[2]
            if age > self.cfg.get('altitude_max_age_s', 1.0):
                self._landing_low_alt_count = 0
                return False
            # 模拟飞控的降落命令只生成一次0cm样本，不会持续产生遥测；
            # 最短等待仍保留，但仅真实飞控要求三个独立新样本。
            if getattr(self.mcu, 'dry_run', False):
                return altitude <= self.cfg['land_alt_threshold_cm']
            if sequence == self._last_landing_alt_sequence:
                return False
            self._last_landing_alt_sequence = sequence
        else:
            # 兼容旧测试桩；真实 MCUSerial 始终提供带序号的样本。
            altitude = self.mcu.read_altitude()

        if altitude <= self.cfg['land_alt_threshold_cm']:
            self._landing_low_alt_count += 1
        else:
            self._landing_low_alt_count = 0
        return self._landing_low_alt_count >= self.cfg.get(
            'land_confirm_samples', 3,
        )

    # ── 视觉数据获取 ──────────────────────────────────────

    def _get_vision_data(self):
        """获取视觉处理数据"""
        if self.camera is None:
            return None, None, None

        # 新视觉后端统一返回处理结果。OpenMV 在板端识别，因此 frame=None；
        # 工业相机仍由 Camera.read_result() 在上位机执行 OpenCV/Tesseract。
        read_result = getattr(self.camera, 'read_result', None)
        if callable(read_result):
            result = read_result()
            if result is None:
                self._home_cross_center = None
                self._home_cross_confidence = 0.0
                return None, None, None
            self._home_cross_center = getattr(result, 'home_cross_center', None)
            self._home_cross_confidence = getattr(
                result, 'home_cross_confidence', 0.0,
            )
            self._start_marker_center = getattr(
                result, 'start_marker_center', None,
            )
            self._gray_marker_center = getattr(
                result, 'gray_marker_center', None,
            )
            self._gray_marker_box = getattr(result, 'gray_marker_box', None)
            self._gray_marker_confidence = getattr(
                result, 'gray_marker_confidence', 0.0,
            )
            self._gray_marker_sequence = getattr(
                result, 'gray_marker_sequence', None,
            )
            return (
                getattr(result, 'frame', None),
                getattr(result, 'green_ratio', None),
                getattr(result, 'digit', None),
            )

        # 兼容旧 Camera/测试桩的抓帧接口。
        ret, frame = self.camera.read()
        if not ret:
            return None, None, None

        # 计算绿色占比
        green_ratio = 0.0
        if hasattr(self.camera, 'detector') and self.camera.detector is not None:
            hsv = self.camera.convert_to_hsv(frame)
            green_ratio = self.camera.detector.calc_green_ratio(hsv)

        # OCR识别
        ocr_result = None
        if hasattr(self.camera, 'digit_reader') and self.camera.digit_reader is not None:
            detector = getattr(self.camera, 'detector', None)
            ocr_result = self.camera.digit_reader.extract_digits(frame, detector=detector)

        return frame, green_ratio, ocr_result

    # ── 异常检测 ──────────────────────────────────────────

    def _check_exceptions(self):
        """检查各种异常条件"""
        alt = self.mcu.read_altitude()

        # 1. 通信超时
        if not self.mcu.is_communication_ok(self.cfg['comm_timeout_ms']):
            logger.warning("MCU communication timeout")
            # 触发紧急降落 (MCU应该有自己的关断逻辑)
            if self.mcu.get_communication_age_ms() > self.cfg['comm_timeout_ms'] * 3:
                self._emergency("Lost communication with MCU")
                return

        # 2. 高度异常。起飞阶段的 0~数厘米是正常爬升过程，绝不能
        # 触发低高度紧急降落；降落相关状态也禁止发送补升命令。
        if alt > 0:  # 有有效高度读数时才检查
            altitude_guard_states = (
                FlightState.FIND_START, FlightState.SPRAY,
                FlightState.NAVIGATE, FlightState.RETURN_HOME,
                FlightState.ALIGN_HOME,
            )
            if self.state in altitude_guard_states:
                if alt < self.cfg['alt_critical_low_cm']:
                    self._emergency(f"Critical low altitude: {alt}cm")
                    return
                if alt < self.cfg['alt_low_warn_cm']:
                    logger.warning(f"Altitude too low: {alt}cm, attempting ascent")
                    self.mcu.send_cmd_ascend(
                        self.cfg['takeoff_height_cm'] - alt,
                        self.cfg['ascent_speed']
                    )
            if alt > self.cfg['alt_max_cm']:
                self._emergency(f"Altitude too high: {alt}cm")
                return

        # 3. 任务超时
        if self.mission_time > self.cfg['max_mission_time_s']:
            self._emergency(f"Mission timeout ({self.mission_time:.0f}s > "
                           f"{self.cfg['max_mission_time_s']}s)")
            return

        # 4. 电池低压。0V 表示尚未收到有效遥测，不能据此触发保护。
        voltage = self.mcu.read_voltage()
        low_threshold = self.cfg.get('low_voltage_threshold', 10.5)
        critical_threshold = self.cfg.get(
            'critical_voltage_threshold', low_threshold - 0.5,
        )
        if voltage > 0 and voltage <= low_threshold:
            self._low_voltage_count += 1
        else:
            self._low_voltage_count = 0

        if self._low_voltage_count >= self.cfg.get(
                'low_voltage_confirm_samples', 3):
            if voltage <= critical_threshold:
                self._emergency(f"Critical battery voltage: {voltage:.2f}V")
                return
            elif self.state not in (
                    FlightState.RETURN_HOME, FlightState.ALIGN_HOME,
                    FlightState.LAND, FlightState.LOCK,
                    FlightState.EMERGENCY, FlightState.DONE):
                logger.warning(
                    "Low battery voltage: %.2fV; returning home", voltage,
                )
                self._transition(FlightState.RETURN_HOME)

    # ── 查询接口 ──────────────────────────────────────────

    def get_state(self) -> FlightState:
        return self.state

    def get_progress(self) -> dict:
        """获取任务进度"""
        progress = {
            'state': self.state.name,
            'visited': self.visited_count,
            'total': self.total_blocks,
            'progress_pct': self.visited_count / self.total_blocks * 100,
            'mission_time': self.mission_time,
            'altitude': self.mcu.read_altitude(),
            'emergency': self.is_emergency,
            'emergency_reason': self._emergency_reason if self.is_emergency else '',
        }
        progress.update(self._navigation_details())
        return progress

    def get_stats(self) -> dict:
        """获取运行统计"""
        stats = {
            'total_state_transitions': self._total_states,
            'mission_time_s': self.mission_time,
            'blocks_completed': self.visited_count,
            'retries': self._retry_count,
        }
        stats.update({
            f'gray_calibration_{key}': value
            for key, value in self._calibration_stats.items()
        })
        return stats
