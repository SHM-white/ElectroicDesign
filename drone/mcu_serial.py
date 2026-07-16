"""通过USB-TTL直连凌霄IMU串口2，收发匿名通信协议V7原生帧。"""

import struct
import math
import time
import threading
import logging
from typing import Tuple

try:
    from .lx_protocol import (
        parse_lx_frame,
        cmd_unlock, cmd_lock, cmd_mode,
        cmd_takeoff, cmd_land, cmd_move,
        cmd_ascend, cmd_descend, cmd_reset_optical_flow,
    )
except ImportError:
    from lx_protocol import (
        parse_lx_frame,
        cmd_unlock, cmd_lock, cmd_mode,
        cmd_takeoff, cmd_land, cmd_move,
        cmd_ascend, cmd_descend, cmd_reset_optical_flow,
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
    凌霄IMU原生V7串口通信管理。

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
                 baudrate: int = 500000):
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
        self._altitude_sequence = 0  # 高度遥测帧序号（仅新帧递增）
        self._last_altitude_update = time.time() if dry_run else 0.0
        self._mode = 0             # 飞行模式
        self._locked = 0           # V7协议: 0=锁定, 1=解锁
        self._last_lock_status_update = time.time() if dry_run else 0.0
        self._voltage_mv = 0       # 电池电压 (mV)
        self._of_updated = False   # 光流数据是否更新
        self._last_of_update = 0.0  # 上次光流更新时间
        self._flight_status_received = dry_run
        self._last_flight_status_update = time.time() if dry_run else 0.0
        self._aux6 = 1800 if dry_run else 0
        self._rc_channels_received = dry_run

        # 通信超时
        self._last_rx_time = time.time()

        # 统计
        self._tx_count = 0
        self._rx_count = 0
        self._comm_errors = 0
        self._valid_frame_count = 0
        self._checksum_errors = 0
        self._ack_count = 0
        self._last_ack = None

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
        """直接发送一帧匿名V7指令，不添加自定义桥接封装。"""
        return self.send_raw(frame)

    def send_query(self, cmd: int) -> bool:
        """兼容旧调用；V7遥测由IMU主动输出，不需要BB查询。"""
        if cmd == 0x03:
            return self.send_of_zero_reset()
        return self.is_connected()

    # ── 高级指令 ──────────────────────────────────────────

    def send_cmd_unlock(self) -> bool:
        ok = self.send_imu_frame(cmd_unlock())
        if ok and self.dry_run:
            with self._lock:
                self._locked = 1
                self._last_lock_status_update = time.time()
                self._touch_simulated_status()
        return ok

    def send_cmd_lock(self) -> bool:
        ok = self.send_imu_frame(cmd_lock())
        if ok and self.dry_run:
            with self._lock:
                self._locked = 0
                self._last_lock_status_update = time.time()
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
                self._altitude_sequence += 1
                self._last_altitude_update = time.time()
                self._touch_simulated_status()
        return ok

    def send_cmd_land(self) -> bool:
        ok = self.send_imu_frame(cmd_land())
        if ok and self.dry_run:
            with self._lock:
                self._altitude = 0
                self._altitude_sequence += 1
                self._last_altitude_update = time.time()
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
        """发送V7原生光流运动解算复位命令。"""
        ok = self.send_imu_frame(cmd_reset_optical_flow())
        if ok and self.dry_run:
            with self._lock:
                self._of_pos_x = 0.0
                self._of_pos_y = 0.0
                self._of_dx = 0.0
                self._of_dy = 0.0
        return ok

    def send_heartbeat(self) -> bool:
        """原生V7没有项目自定义心跳；保持接口兼容但不发送数据。"""
        return self.is_connected()

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
                self._parse_buffer()
                return True
        except Exception as e:
            logger.warning(f"RX error: {e}")
            self._comm_errors += 1

        return False

    def _parse_buffer(self):
        """
        从接收缓冲区提取 AA ADDR ID LEN DATA SC AC 原生V7帧。
        """
        while len(self._rx_buf) >= 4:
            try:
                start = self._rx_buf.index(0xAA)
            except ValueError:
                self._rx_buf.clear()
                break
            if start:
                del self._rx_buf[:start]
            if len(self._rx_buf) < 4:
                break
            frame_length = self._rx_buf[3] + 6
            if frame_length > 261:
                self._rx_buf.pop(0)
                continue
            if len(self._rx_buf) < frame_length:
                break
            raw = bytes(self._rx_buf[:frame_length])
            parsed = parse_lx_frame(raw)
            if parsed is None:
                self._checksum_errors += 1
                self._rx_buf.pop(0)
                continue
            del self._rx_buf[:frame_length]
            self._valid_frame_count += 1
            self._last_rx_time = time.time()
            self._handle_v7_frame(parsed)

        # 防止缓冲区无限增长
        if len(self._rx_buf) > self.RX_BUF_SIZE * 2:
            self._rx_buf = self._rx_buf[-self.RX_BUF_SIZE:]

    def _handle_v7_frame(self, frame: dict) -> None:
        """更新官方V7遥测缓存。"""
        frame_id = frame['id']
        data = frame['data']
        now = time.time()
        with self._lock:
            if frame_id == 0x00 and len(data) >= 3:
                self._last_ack = tuple(data[:3])
                self._ack_count += 1
            elif frame_id == 0x05 and len(data) >= 9:
                self._altitude = struct.unpack_from('<i', data, 0)[0]
                self._altitude_sequence += 1
                self._last_altitude_update = now
                self._flight_status_received = True
                self._last_flight_status_update = now
            elif frame_id == 0x06 and len(data) >= 2:
                self._mode = data[0]
                self._locked = data[1]
                self._last_lock_status_update = now
                self._flight_status_received = True
                self._last_flight_status_update = now
            elif frame_id == 0x08 and len(data) >= 8:
                pos_x, pos_y = struct.unpack_from('<ii', data, 0)
                self._update_position(pos_x, pos_y, 255, now)
            elif frame_id == 0x0D and len(data) >= 2:
                # 官方字段为VOLTAGE*100，转为mV。
                self._voltage_mv = struct.unpack_from('<H', data, 0)[0] * 10
            elif frame_id == 0x40 and len(data) >= 20:
                # ROL/PIT/THR/YAW/AUX1..AUX6，均为S16小端；AUX6位于第10项。
                self._aux6 = struct.unpack_from('<h', data, 18)[0]
                self._rc_channels_received = True
            elif frame_id == 0x51 and len(data) >= 2 and data[0] == 2 \
                    and len(data) >= 15:
                state = data[1]
                integ_x, integ_y = struct.unpack_from('<hh', data, 10)
                self._update_position(
                    integ_x, integ_y, data[14] if state else 0, now,
                )

    def _update_position(self, pos_x: int, pos_y: int,
                         quality: int, now: float) -> None:
        prev_x, prev_y = self._of_pos_x, self._of_pos_y
        self._of_pos_x = float(pos_x)
        self._of_pos_y = float(pos_y)
        self._of_quality = quality
        self._of_dx += self._of_pos_x - prev_x
        self._of_dy += self._of_pos_y - prev_y
        self._of_updated = True
        self._last_of_update = now

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

    def read_altitude_sample(self) -> Tuple[int, int, float]:
        """返回高度、独立帧序号和样本年龄（秒）。"""
        with self._lock:
            age = (time.time() - self._last_altitude_update
                   if self._last_altitude_update > 0 else float('inf'))
            return self._altitude, self._altitude_sequence, age

    def has_recent_altitude(self, max_age_s: float = 1.0) -> bool:
        """是否收到过近期有效的高度遥测帧。"""
        with self._lock:
            return self._last_altitude_update > 0 and (
                self.dry_run
                or time.time() - self._last_altitude_update <= max_age_s
            )

    def read_mode(self) -> int:
        """读取飞行模式"""
        with self._lock:
            return self._mode

    def read_locked(self) -> int:
        """读取锁定状态"""
        with self._lock:
            return self._locked

    def has_recent_lock_status(self, max_age_s: float = 2.0) -> bool:
        """是否收到过近期模式/锁定状态遥测。"""
        with self._lock:
            return self._last_lock_status_update > 0 and (
                self.dry_run
                or time.time() - self._last_lock_status_update <= max_age_s
            )

    def read_aux6(self) -> int:
        """读取V7 0x40遥控帧中的AUX6。"""
        with self._lock:
            return self._aux6

    def has_rc_channels(self) -> bool:
        """是否已收到至少一帧有效的V7 0x40遥控通道数据。"""
        with self._lock:
            return self._rc_channels_received

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
            'valid_frames': self._valid_frame_count,
            'checksum_errors': self._checksum_errors,
            'ack_count': self._ack_count,
            'last_ack': self._last_ack,
            'last_rx_age_ms': self.get_communication_age_ms(),
            'ser_buf_size': len(self._rx_buf),
        }
