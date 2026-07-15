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
        self._start_block_confirmed = bool(config.get('dry_run', False))
        self._start_marker_center = None
        self._home_cross_center = None
        self._home_cross_confidence = 0.0
        self._home_cross_confirm_count = 0
        self._last_home_align_command = 0.0

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
            self.mcu.send_cmd_unlock()
            self._state_command_sent = True
        # 等待解锁完成
        if self.state_start_time > self.cfg['unlock_wait_s']:
            self._transition(FlightState.SET_PROGRAM_MODE)

    def _state_set_program_mode(self, frame, green_ratio, ocr_result):
        """切换到程控模式"""
        if not self._state_command_sent:
            logger.info("Setting program control mode...")
            self.mcu.send_cmd_mode(3)  # 程控模式
            self._state_command_sent = True
        if self.state_start_time > self.cfg['mode_switch_wait_s']:
            # 重置光流零点
            self.mcu.send_of_zero_reset()
            self.localizer._init_global_position()
            self._mission_start_time = time.time()
            self._transition(FlightState.TAKEOFF)

    def _state_takeoff(self, frame, green_ratio, ocr_result):
        """起飞至目标高度"""
        takeoff_height = self.cfg['takeoff_height_cm']
        tolerance = self.cfg['takeoff_height_tolerance']

        # 仅第一次进入时发送指令
        if not self._state_command_sent:
            logger.info(f"Taking off to {takeoff_height}cm...")
            self.mcu.send_cmd_takeoff(takeoff_height)
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
        # 计算到区块21的移动指令
        target_pos = get_block_position(21)
        if target_pos is None:
            self._emergency("Cannot find block 21 position")
            return

        distance, direction = self._calc_move_to_target(target_pos[0], target_pos[1])

        position_ready = distance < 10 or self.cfg.get('manual_navigation', False)
        if position_ready:
            start_seen = (
                self._start_marker_center is not None or ocr_result == 21
            )
            if start_seen:
                self._start_confirm_count += 1
                logger.info(
                    "Start observation %d/%d (%s)",
                    self._start_confirm_count,
                    self.cfg.get('start_block_confirm_frames', 3),
                    "A marker" if self._start_marker_center is not None
                    else "digit 21",
                )
            else:
                self._start_confirm_count = 0

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

        if not self.visited[cur_block]:
            if self.laser is not None:
                self.laser.blink(
                    count=self.cfg.get('laser_count', 2),
                    period_ms=self.cfg.get('laser_period', 1500),
                )
            self.visited[cur_block] = True
            logger.info(f"[SPRAY] Block {cur_block} sprayed "
                        f"({self.visited_count}/{self.total_blocks})")

        # 检查是否全部完成
        if self.visited_count >= self.total_blocks:
            logger.info("All blocks sprayed! Returning home.")
            self._transition(FlightState.RETURN_HOME)
        else:
            self._transition(FlightState.NAVIGATE)

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

            if ocr_result == next_block:
                logger.info("[FIELD] OCR确认到达区块 %d", next_block)
                self.localizer.apply_ocr(next_block)
                self._transition(FlightState.SPRAY)
            return

        distance, direction = self._calc_move_to_target(target_pos[0], target_pos[1])

        if distance < 10:
            # 已到达, 直接撒药
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
                    logger.error(f"Failed to reach block {next_block}, marking as visited")
                    self.visited[next_block] = True  # 放弃此块
                    self.localizer.advance_block()
                    self._transition(FlightState.SPRAY)
                else:
                    self.mcu.send_cmd_move(distance, self.cfg['move_speed'], direction)
                    self._state_start_time = time.time()

    def _state_return_home(self, frame, green_ratio, ocr_result):
        """返回起降点"""
        hx, hy = get_home_position()
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

        # 实测标定：机体中心对准十字时，十字应位于画面中心向下25cm
        # 对应的位置。该量是图像目标偏移，不再用相机安装方向命名。
        error_forward = self.cfg.get('home_target_down_offset_cm', 25.0) \
            - (center[1] - cy) * alt / fy
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
            self.mcu.send_cmd_land()
            self._state_command_sent = True

        alt = self.mcu.read_altitude()
        if self._has_flight_status() and alt <= self.cfg['land_alt_threshold_cm']:
            logger.info(f"Land complete: altitude={alt}cm")
            self._transition(FlightState.LOCK)
        elif self.state_start_time > self.cfg['land_timeout_s']:
            self._emergency("Land timeout")

    def _state_lock(self, frame, green_ratio, ocr_result):
        """加锁"""
        if not self._state_command_sent:
            logger.info("Locking motors...")
            self.mcu.send_cmd_lock()
            self._state_command_sent = True
        if self.state_start_time > 1:
            logger.info("=== MISSION COMPLETE ===")
            self._transition(FlightState.DONE)

    def _state_emergency(self, frame, green_ratio, ocr_result):
        """
        紧急状态处理

        执行紧急降落流程
        """
        if not self._state_command_sent:
            logger.critical(f"EMERGENCY: {self._emergency_reason}")
            self.mcu.send_cmd_land()
            self._state_command_sent = True

        alt = self.mcu.read_altitude()
        if self._has_flight_status() and alt <= self.cfg['land_alt_threshold_cm']:
            self.mcu.send_cmd_lock()
            self._transition(FlightState.DONE)
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
        self._total_states += 1
        self._state_history.append((old_state, new_state, time.time()))

    def _emergency(self, reason: str):
        """触发紧急状态"""
        self._emergency_reason = reason
        logger.critical(f"Emergency triggered: {reason}")
        self._transition(FlightState.EMERGENCY)

    # ── 移动计算 ──────────────────────────────────────────

    def _calc_move_to_target(self, target_x: float, target_y: float) -> tuple[int, int]:
        """计算从当前位置到目标位置的距离(cm)和方向(度)"""
        cur_x, cur_y = self.localizer.get_global_position()
        dx = target_x - cur_x
        dy = target_y - cur_y
        distance = int(math.sqrt(dx ** 2 + dy ** 2))
        direction = int(math.degrees(math.atan2(dy, dx)) % 360)
        return distance, direction

    def _has_flight_status(self) -> bool:
        checker = getattr(self.mcu, 'has_flight_status', None)
        return bool(checker()) if callable(checker) else True

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
            return result.frame, result.green_ratio, result.digit

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

        # 2. 高度异常
        if alt > 0:  # 有有效高度读数时才检查
            if alt < self.cfg['alt_critical_low_cm']:
                self._emergency(f"Critical low altitude: {alt}cm")
            elif alt < self.cfg['alt_low_warn_cm']:
                logger.warning(f"Altitude too low: {alt}cm, attempting ascent")
                self.mcu.send_cmd_ascend(
                    self.cfg['takeoff_height_cm'] - alt,
                    self.cfg['ascent_speed']
                )
            elif alt > self.cfg['alt_max_cm']:
                self._emergency(f"Altitude too high: {alt}cm")

        # 3. 任务超时
        if self.mission_time > self.cfg['max_mission_time_s']:
            self._emergency(f"Mission timeout ({self.mission_time:.0f}s > "
                           f"{self.cfg['max_mission_time_s']}s)")

    # ── 查询接口 ──────────────────────────────────────────

    def get_state(self) -> FlightState:
        return self.state

    def get_progress(self) -> dict:
        """获取任务进度"""
        return {
            'state': self.state.name,
            'visited': self.visited_count,
            'total': self.total_blocks,
            'progress_pct': self.visited_count / self.total_blocks * 100,
            'mission_time': self.mission_time,
            'altitude': self.mcu.read_altitude(),
            'emergency': self.is_emergency,
            'emergency_reason': self._emergency_reason if self.is_emergency else '',
        }

    def get_stats(self) -> dict:
        """获取运行统计"""
        return {
            'total_state_transitions': self._total_states,
            'mission_time_s': self.mission_time,
            'blocks_completed': self.visited_count,
            'retries': self._retry_count,
        }
