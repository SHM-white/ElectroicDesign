"""
mcu_serial.py — MCU串口通信模块
Section 3: 主控与MCU串口通信协议

通信结构:
  主控 USB-TTL (TX) ←→ MCU UART (RX)
  主控 USB-TTL (RX) ←→ MCU UART (TX)
  波特率: 115200bps, 3.3V电平

帧类型:
  A. 指令转发帧 (主控→MCU→IMU):  0xAA + CMD_LEN + TYPE(0x01) + PAYLOAD + SUM16
  B. 查询帧 (主控→MCU):           0xBB + CMD
  C. 光流位置回传 (MCU→主控):      0xCC + 0x01 + POS_X(4B) + POS_Y(4B) + QUALITY(1B)
  D. 飞行状态回传 (MCU→主控):      0xCC + 0x02 + MODE(1B) + LOCKED(1B) + ALT(4B)
"""

import struct
import math
import time
import threading
import logging
from typing import Optional, Tuple, List, Callable
from collections import deque

try:
    from .lx_protocol import (
        build_pi_frame, build_pi_query_frame, build_heartbeat_query,
        parse_of_position, parse_flight_status, parse_battery_voltage,
        cmd_unlock, cmd_lock, cmd_mode,
        cmd_takeoff, cmd_land, cmd_move,
        cmd_ascend, cmd_descend,
    )
except ImportError:
    from lx_protocol import (
        build_pi_frame, build_pi_query_frame, build_heartbeat_query,
        parse_of_position, parse_flight_status, parse_battery_voltage,
        cmd_unlock, cmd_lock, cmd_mode,
        cmd_takeoff, cmd_land, cmd_move,
        cmd_ascend, cmd_descend,
    )

logger = logging.getLogger('drone.mcu')


# ── 串口抽象层 ────────────────────────────────────────────

class DummySerial:
    """模拟串口 (dry_run / 测试用)"""

    def __init__(self):
        self._buf = bytearray()
        self._seq = 0

    def write(self, data: bytes) -> int:
        # 模拟回显和响应
        self._buf.extend(data)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        if len(self._buf) < size:
            return b''
        result = bytes(self._buf[:size])
        self._buf = self._buf[size:]
        return result

    def readline(self) -> bytes:
        # 模拟返回一行
        return b''

    @property
    def in_waiting(self) -> int:
        return len(self._buf)

    def close(self):
        pass

    def flush(self):
        self._buf.clear()

    def inject_response(self, data: bytes):
        """注入模拟回传数据（测试用）"""
        self._buf.extend(data)


class RealSerial:
    """真实串口包装"""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        import serial
        self._ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=timeout,
        )

    def write(self, data: bytes) -> int:
        return self._ser.write(data)

    def read(self, size: int = 1) -> bytes:
        return self._ser.read(size)

    def readline(self) -> bytes:
        return self._ser.readline()

    @property
    def in_waiting(self) -> int:
        return self._ser.in_waiting

    def close(self):
        self._ser.close()


# ── MCU通信主类 ───────────────────────────────────────────

class MCUSerial:
    """
    MCU串口通信管理

    职责:
    - 发送指令 (解锁/起飞/移动/降落等)
    - 接收光流位置数据
    - 接收飞行状态数据
    - 通信超时检测
    """

    # 串口接收缓冲
    RX_BUF_SIZE = 512

    def __init__(self, dry_run: bool = False,
                 port: str = '/dev/ttyUSB0',
                 baudrate: int = 115200):
        self.dry_run = dry_run
        self.port = port
        self.baudrate = baudrate

        # 串口对象
        self._ser = None

        # 接收缓冲
        self._rx_buf = bytearray()

        # 最新数据缓存
        self._lock = threading.Lock()
        self._of_pos_x = 0.0      # 光流X积分 (cm)
        self._of_pos_y = 0.0      # 光流Y积分 (cm)
        self._of_quality = 0       # 光流质量
        self._of_dx = 0.0          # 增量X (cm)
        self._of_dy = 0.0          # 增量Y (cm)
        self._last_of_x = 0.0      # 上一次光流X
        self._last_of_y = 0.0      # 上一次光流Y
        self._altitude = 0         # 高度 (cm)
        self._mode = 0             # 飞行模式
        self._locked = 0           # V7协议: 0=锁定, 1=解锁
        self._voltage_mv = 0       # 电池电压 (mV)
        self._of_updated = False   # 光流数据是否更新
        self._last_of_update = 0.0  # 上次光流更新时间
        self._flight_status_received = dry_run
        self._last_flight_status_update = time.time() if dry_run else 0.0

        # 通信超时
        self._last_rx_time = time.time()

        # 统计
        self._tx_count = 0
        self._rx_count = 0
        self._comm_errors = 0

        logger.info(f"MCUSerial initialized (dry_run={dry_run}, port={port}, baud={baudrate})")

    # ── 连接管理 ──────────────────────────────────────────

    def connect(self) -> bool:
        """建立串口连接"""
        if self.dry_run:
            self._ser = DummySerial()
            logger.info("Using dummy serial (dry run mode)")
            return True
        try:
            self._ser = RealSerial(self.port, self.baudrate)
            logger.info(f"Connected to {self.port} @ {self.baudrate}bps")
            return True
        except Exception as e:
            logger.error(f"Failed to open serial port: {e}")
            self._ser = DummySerial()
            return False

    def disconnect(self):
        """断开串口连接"""
        if self._ser:
            self._ser.close()
            self._ser = None
        logger.info("Disconnected")

    # ── 数据发送 ──────────────────────────────────────────

    def send_raw(self, data: bytes) -> bool:
        """发送原始字节到MCU"""
        if not self._ser:
            logger.error("Serial not connected")
            return False
        try:
            written = self._ser.write(data)
            self._tx_count += 1
            logger.debug(f"TX ({len(data)}B): {data.hex().upper()}")
            return written == len(data)
        except Exception as e:
            logger.error(f"TX error: {e}")
            self._comm_errors += 1
            return False

    def send_imu_frame(self, frame: bytes) -> bool:
        """
        发送IMU指令 (包装为TYPE=0x01转发帧)

        帧格式: AA [CMD_LEN] 01 [IMU_API_FRAME] [SUM_LO] [SUM_HI]
        """
        return self.send_raw(build_pi_frame(frame, frame_type=0x01))

    def send_query(self, cmd: int) -> bool:
        """
        发送查询命令 (TYPE=0xBB)

        cmd:
            0x01 = 请求光流位置
            0x02 = 请求飞行状态
            0x03 = 重置光流零点
        """
        return self.send_raw(build_pi_query_frame(cmd))

    # ── 高级指令 ──────────────────────────────────────────

    def send_cmd_unlock(self) -> bool:
        ok = self.send_imu_frame(cmd_unlock())
        if ok and self.dry_run:
            with self._lock:
                self._locked = 1
                self._touch_simulated_status()
        return ok

    def send_cmd_lock(self) -> bool:
        ok = self.send_imu_frame(cmd_lock())
        if ok and self.dry_run:
            with self._lock:
                self._locked = 0
                self._touch_simulated_status()
        return ok

    def send_cmd_mode(self, mode: int) -> bool:
        """切换飞行模式 (0=自稳, 1=自稳+定高, 2=定点, 3=程控)"""
        ok = self.send_imu_frame(cmd_mode(mode))
        if ok and self.dry_run:
            with self._lock:
                self._mode = mode
                self._touch_simulated_status()
        return ok

    def send_cmd_takeoff(self, height_cm: int = 150) -> bool:
        ok = self.send_imu_frame(cmd_takeoff(height_cm))
        if ok and self.dry_run:
            with self._lock:
                self._altitude = height_cm or 150
                self._touch_simulated_status()
        return ok

    def send_cmd_land(self) -> bool:
        ok = self.send_imu_frame(cmd_land())
        if ok and self.dry_run:
            with self._lock:
                self._altitude = 0
                self._touch_simulated_status()
        return ok

    def send_cmd_move(self, distance_cm: int, speed_cmps: int, direction_deg: int) -> bool:
        ok = self.send_imu_frame(cmd_move(distance_cm, speed_cmps, direction_deg))
        if ok and self.dry_run:
            angle = math.radians(direction_deg)
            dx = distance_cm * math.cos(angle)
            dy = distance_cm * math.sin(angle)
            with self._lock:
                self._of_dx += dx
                self._of_dy += dy
                self._of_pos_x += dx
                self._of_pos_y += dy
                self._of_quality = 255
                self._of_updated = True
                self._last_of_update = time.time()
                self._last_rx_time = time.time()
        return ok

    def send_cmd_ascend(self, height_cm: int, speed_cmps: int) -> bool:
        ok = self.send_imu_frame(cmd_ascend(height_cm, speed_cmps))
        if ok and self.dry_run:
            with self._lock:
                self._altitude += height_cm
                self._touch_simulated_status()
        return ok

    def send_cmd_descend(self, height_cm: int, speed_cmps: int) -> bool:
        ok = self.send_imu_frame(cmd_descend(height_cm, speed_cmps))
        if ok and self.dry_run:
            with self._lock:
                self._altitude = max(0, self._altitude - height_cm)
                self._touch_simulated_status()
        return ok

    def send_of_zero_reset(self) -> bool:
        """请求MCU重置光流积分零点"""
        ok = self.send_query(0x03)
        if ok and self.dry_run:
            with self._lock:
                self._of_pos_x = 0.0
                self._of_pos_y = 0.0
                self._of_dx = 0.0
                self._of_dy = 0.0
        return ok

    def send_heartbeat(self) -> bool:
        """发送心跳查询 (CMD=0x04)"""
        return self.send_raw(build_heartbeat_query())

    # ── 数据接收与解析 ────────────────────────────────────

    def poll(self) -> bool:
        """
        读取串口数据并解析帧
        应在主循环中高频调用 (20-50Hz)

        Returns:
            True if new data was received
        """
        if not self._ser:
            return False

        try:
            n = self._ser.in_waiting
            if n > 0:
                data = self._ser.read(min(n, self.RX_BUF_SIZE))
                self._rx_buf.extend(data)
                self._rx_count += 1
                self._last_rx_time = time.time()
                self._parse_buffer()
                return True
        except Exception as e:
            logger.warning(f"RX error: {e}")
            self._comm_errors += 1

        return False

    def _parse_buffer(self):
        """
        从接收缓冲区解析帧
        支持的帧头:
          0xCC = 数据回传帧 (光流位置 / 飞行状态)
        """
        while len(self._rx_buf) >= 2:
            if self._rx_buf[0] == 0xCC:
                if len(self._rx_buf) < 3:
                    break

                cmd = self._rx_buf[1]
                if cmd == 0x01 and len(self._rx_buf) >= 11:
                    # 光流位置帧
                    parsed = parse_of_position(bytes(self._rx_buf[:11]))
                    if parsed is not None:
                        with self._lock:
                            prev_x = self._of_pos_x
                            prev_y = self._of_pos_y
                            self._of_pos_x = parsed['pos_x']
                            self._of_pos_y = parsed['pos_y']
                            self._of_quality = parsed['quality']
                            # 计算增量
                            self._of_dx = self._of_pos_x - prev_x
                            self._of_dy = self._of_pos_y - prev_y
                            self._of_updated = True
                            self._last_of_update = time.time()
                    self._rx_buf = self._rx_buf[11:]
                elif cmd == 0x02 and len(self._rx_buf) >= 8:
                    # 飞行状态帧
                    parsed = parse_flight_status(bytes(self._rx_buf[:8]))
                    if parsed is not None:
                        with self._lock:
                            self._mode = parsed['mode']
                            self._locked = parsed['locked']
                            self._altitude = parsed['alt']
                            self._flight_status_received = True
                            self._last_flight_status_update = time.time()
                    self._rx_buf = self._rx_buf[8:]
                elif cmd == 0x03 and len(self._rx_buf) >= 4:
                    # 电池电压帧
                    parsed = parse_battery_voltage(bytes(self._rx_buf[:4]))
                    if parsed is not None:
                        with self._lock:
                            self._voltage_mv = parsed['voltage_mv']
                    self._rx_buf = self._rx_buf[4:]
                else:
                    # 未知命令或数据不足
                    self._rx_buf.pop(0)
            else:
                # 跳过非法字节
                self._rx_buf.pop(0)

        # 防止缓冲区无限增长
        if len(self._rx_buf) > self.RX_BUF_SIZE * 2:
            self._rx_buf = self._rx_buf[-self.RX_BUF_SIZE:]

    # ── 数据读取接口 ──────────────────────────────────────

    def read_optical_flow(self) -> Tuple[float, float]:
        """
        读取光流增量

        Returns:
            (dx_cm, dy_cm) 自上次读取以来的位置增量
        """
        with self._lock:
            dx = self._of_dx
            dy = self._of_dy
            self._of_dx = 0.0
            self._of_dy = 0.0
        return dx, dy

    def read_optical_flow_position(self) -> Tuple[float, float, int]:
        """读取光流绝对位置"""
        with self._lock:
            return self._of_pos_x, self._of_pos_y, self._of_quality

    def read_altitude(self) -> int:
        """读取当前高度(cm)"""
        with self._lock:
            return self._altitude

    def read_mode(self) -> int:
        """读取飞行模式"""
        with self._lock:
            return self._mode

    def read_locked(self) -> int:
        """读取锁定状态"""
        with self._lock:
            return self._locked

    def read_aux2(self) -> int:
        """
        读取AUX2/CH6通道值 (用于启动信号)

        NOTE: 此方法需要MCU额外发送CH6数据或通过飞行状态帧扩展
        当前为占位实现
        """
        if self.dry_run:
            return 1800
        # TODO: 通过扩展飞行状态帧或增加独立查询帧实现
        return 1000  # 默认值(未触发)

    def read_voltage(self) -> float:
        """
        读取电池电压 (V)

        Returns:
            电压值 (V), 由MCU上报的mV值转换而来
        """
        with self._lock:
            return self._voltage_mv / 1000.0

    # ── 通信状态 ──────────────────────────────────────────

    def is_connected(self) -> bool:
        return self._ser is not None

    def _touch_simulated_status(self) -> None:
        self._flight_status_received = True
        self._last_flight_status_update = time.time()
        self._last_rx_time = time.time()

    def has_flight_status(self, max_age_s: float = 2.0) -> bool:
        """是否收到过近期有效的飞行状态数据。"""
        with self._lock:
            return self._flight_status_received and (
                self.dry_run or time.time() - self._last_flight_status_update <= max_age_s
            )

    def is_communication_ok(self, timeout_ms: float = 500.0) -> bool:
        """检查通信是否正常"""
        if self.dry_run:
            return True
        return (time.time() - self._last_rx_time) * 1000 < timeout_ms

    def get_communication_age_ms(self) -> float:
        """距上次通信的时间(ms)"""
        return (time.time() - self._last_rx_time) * 1000

    def get_stats(self) -> dict:
        """获取通信统计"""
        return {
            'tx_count': self._tx_count,
            'rx_count': self._rx_count,
            'comm_errors': self._comm_errors,
            'last_rx_age_ms': self.get_communication_age_ms(),
            'ser_buf_size': len(self._rx_buf),
        }
