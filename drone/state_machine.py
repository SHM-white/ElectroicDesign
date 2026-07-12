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

from path_plan import PATH, get_block_position, get_home_position, init_grid

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
            camera: Camera实例
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

        # 28个区块的访问记录 (下标1~28, 22为空不用)
        self.visited = [True] + [False] * 28  # visited[0]=True占位

        # 计时器
        self._state_start_time = time.time()
        self._mission_start_time = 0.0
        self._retry_count = 0
        self._max_retries = 3

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
        return sum(1 for v in self.visited if v)

    @property
    def total_blocks(self) -> int:
        return len(self.localizer.path)  # 27, not 28 (plan document error)

    # ── 主循环 ────────────────────────────────────────────

    def run_iteration(self) -> FlightState:
        """
        每个循环周期调用一次 (20-50Hz)

        Returns:
            当前状态
        """
        # 读取串口数据 (必须最先调用, 否则所有传感器读数为0)
        self.mcu.poll()

        # 获取视觉数据
        frame, green_ratio, ocr_result = self._get_vision_data()

        # 更新光流
        of_dx, of_dy = self.mcu.read_optical_flow()
        if of_dx != 0.0 or of_dy != 0.0:
            self.localizer.update_optical_flow(of_dx, of_dy)

        # 颜色跳变检测 (所有飞行状态都检查)
        if self.state not in (FlightState.IDLE, FlightState.ARM_UNLOCK,
                              FlightState.SET_PROGRAM_MODE, FlightState.TAKEOFF,
                              FlightState.EMERGENCY, FlightState.DONE):
            if self.localizer.check_boundary_crossed(green_ratio):
                self.localizer.advance_block()

        # OCR校准
        if self.localizer.should_do_ocr() and ocr_result is not None:
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
        # 检查启动信号: CH6开关 >1700us
        if self.mcu.read_aux2() > 1700:
            logger.info("Start signal received!")
            self._transition(FlightState.ARM_UNLOCK)

    def _state_arm_unlock(self, frame, green_ratio, ocr_result):
        """解锁电机"""
        logger.info("Sending unlock command...")
        self.mcu.send_cmd_unlock()
        # 等待解锁完成
        if self.state_start_time > self.cfg['unlock_wait_s']:
            self._transition(FlightState.SET_PROGRAM_MODE)

    def _state_set_program_mode(self, frame, green_ratio, ocr_result):
        """切换到程控模式"""
        logger.info("Setting program control mode...")
        self.mcu.send_cmd_mode(3)  # 程控模式
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
        if self._retry_count == 0 and self.state_start_time < 0.5:
            logger.info(f"Taking off to {takeoff_height}cm...")
            self.mcu.send_cmd_takeoff(takeoff_height)

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

    def _state_find_start(self, frame, green_ratio, ocr_result):
        """寻找起始区块 (A标记/区块21)"""
        # 计算到区块21的移动指令
        target_pos = get_block_position(21)
        if target_pos is None:
            self._emergency("Cannot find block 21 position")
            return

        distance, direction = self._calc_move_to_target(target_pos[0], target_pos[1])

        if distance < 10:
            # 已到达, 微调确认
            if ocr_result == 21:
                self.localizer.apply_ocr(21)
                self._transition(FlightState.SPRAY)
        else:
            # 发送移动指令
            if self.state_start_time < 0.5:
                self.mcu.send_cmd_move(distance, self.cfg['move_speed'], direction)

            # 超时仍切换 (依赖颜色跳变 + OCR 确认)
            if self.state_start_time > 15:
                logger.warning("Find start timeout, proceeding anyway")
                self._transition(FlightState.SPRAY)

    def _state_spray(self, frame, green_ratio, ocr_result):
        """撒药 (激光闪烁)"""
        cur_block = self.localizer.get_current_target()

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

        distance, direction = self._calc_move_to_target(target_pos[0], target_pos[1])

        if distance < 10:
            # 已到达, 直接撒药
            self.localizer.advance_block()
            self._transition(FlightState.SPRAY)
        else:
            # 发送移动指令
            if self.state_start_time < 0.5 or self._retry_count > 0:
                self.mcu.send_cmd_move(distance, self.cfg['move_speed'], direction)

            # 超时处理
            if self.state_start_time > self.cfg['block_timeout']:
                logger.warning(f"Navigate timeout to block {next_block}")
                self._retry_count += 1
                if self._retry_count > self._max_retries:
                    logger.error(f"Failed to reach block {next_block}, marking as visited")
                    self.visited[next_block] = True  # 放弃此块
                    self.localizer.advance_block()
                    self._transition(FlightState.SPRAY)

    def _state_return_home(self, frame, green_ratio, ocr_result):
        """返回起降点"""
        hx, hy = get_home_position()
        distance, direction = self._calc_move_to_target(hx, hy)

        if distance < 20:
            logger.info("Arrived at home position")
            self._transition(FlightState.LAND)
        else:
            if self.state_start_time < 0.5:
                self.mcu.send_cmd_move(
                    distance, self.cfg['return_home_speed_cmps'], direction
                )

            # 超时: 仍然降落
            if self.state_start_time > distance / self.cfg['move_speed'] + 10:
                logger.warning("Return home timeout, landing anyway")
                self._transition(FlightState.LAND)

    def _state_land(self, frame, green_ratio, ocr_result):
        """着陆"""
        if self.state_start_time < 0.5:
            logger.info("Landing...")
            self.mcu.send_cmd_land()

        alt = self.mcu.read_altitude()
        if alt < self.cfg['land_alt_threshold_cm'] and alt > 0:
            time.sleep(1)  # 确认稳定
            logger.info(f"Land complete: altitude={alt}cm")
            self._transition(FlightState.LOCK)
        elif self.state_start_time > self.cfg['land_timeout_s']:
            self._emergency("Land timeout")

    def _state_lock(self, frame, green_ratio, ocr_result):
        """加锁"""
        if self.state_start_time < 0.5:
            logger.info("Locking motors...")
            self.mcu.send_cmd_lock()
        if self.state_start_time > 1:
            logger.info("=== MISSION COMPLETE ===")
            self._transition(FlightState.DONE)

    def _state_emergency(self, frame, green_ratio, ocr_result):
        """
        紧急状态处理

        执行紧急降落流程
        """
        if self.state_start_time < 0.5:
            logger.critical(f"EMERGENCY: {self._emergency_reason}")
            self.mcu.send_cmd_land()

        if self.state_start_time > 5:
            self.mcu.send_cmd_lock()
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

    # ── 视觉数据获取 ──────────────────────────────────────

    def _get_vision_data(self):
        """获取视觉处理数据"""
        if self.camera is None:
            return None, 0.0, None

        ret, frame = self.camera.read()
        if not ret:
            return None, 0.0, None

        # 计算绿色占比
        green_ratio = 0.0
        if hasattr(self.camera, 'detector') and self.camera.detector is not None:
            hsv = self.camera.convert_to_hsv(frame)
            green_ratio = self.camera.detector.calc_green_ratio(hsv)

        # OCR识别
        ocr_result = None
        if hasattr(self.camera, 'digit_reader') and self.camera.digit_reader is not None:
            ocr_result = self.camera.digit_reader.extract_digits(frame)

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
