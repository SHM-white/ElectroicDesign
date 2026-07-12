"""
lx_protocol.py — 凌霄IMU API帧构建库
基于匿名通信协议V7 + 源码验证
控制类指令使用 CID=0x10 (非协议文档中的CID=0x00)

Section 4: 凌霄IMU API指令速查
"""

import struct
from typing import Optional


def build_lx_frame(d_addr: int, frame_id: int, data: bytes) -> bytes:
    """
    构建凌霄API帧，自动计算校验和 (SC + AC)

    帧格式:
        AA [D_ADDR] [ID] [LEN] [DATA...] [SC] [AC]

    Args:
        d_addr: 目标地址 (0xFF=广播)
        frame_id: 帧ID (0xE0=命令帧)
        data: 载荷数据

    Returns:
        完整帧字节序列
    """
    buf = bytes([0xAA, d_addr, frame_id])
    buf += bytes([len(data)])  # LEN
    buf += data

    # 计算 SC(和校验) 和 AC(累加校验)
    # 注意: 校验和覆盖整个帧(含帧头 AA)
    sumcheck = 0
    addcheck = 0
    for b in buf:
        sumcheck = (sumcheck + b) & 0xFF
        addcheck = (addcheck + sumcheck) & 0xFF

    buf += bytes([sumcheck, addcheck])
    return buf


def build_pi_frame(data: bytes, frame_type: int = 0x01) -> bytes:
    """
    构建树莓派→MCU 指令转发帧

    帧结构:
        0xAA [CMD_LEN] [TYPE] [PAYLOAD] [SUM_LO] [SUM_HI]

    Args:
        data: 完整的凌霄API帧 (不含外围校验，MCU会加上SC/AC)
        frame_type: 0x01=转发至IMU, 0xBB=控制查询

    Returns:
        完整帧字节序列

    NOTE: 计划中的帧格式有歧义:
    - 3.2.A节: 帧头0xAA, 字段顺序为 AA + CMD_LEN + TYPE + PAYLOAD + SUM
    - 3.2.B节: 帧头0xBB, 字段顺序为 BB + CMD
    两者是不同的帧类型。此处实现TYPE=0x01的转发帧。
    """
    cmd_len = len(data)
    buf = bytes([0xAA, cmd_len, frame_type]) + data

    # 双字节校验和 (所有字节之和 & 0xFFFF)
    checksum = sum(buf) & 0xFFFF
    buf += bytes([checksum & 0xFF, (checksum >> 8) & 0xFF])

    return buf


def build_pi_query_frame(cmd: int) -> bytes:
    """
    构建树莓派→MCU 查询帧 (Section 3.2.B)

    帧结构:
        0xBB [CMD]

    CMD:
        0x01 = 请求光流位置
        0x02 = 请求飞行状态
        0x03 = 重置光流积分零点
    """
    return bytes([0xBB, cmd])


def verify_lx_frame(frame: bytes) -> bool:
    """
    验证凌霄API帧的校验和

    Returns:
        True if checksum is correct
    """
    if len(frame) < 6:
        return False

    # 帧中最后两字节是校验和
    expected_sc = frame[-2]
    expected_ac = frame[-1]

    # 重新计算
    actual_sc = 0
    actual_ac = 0
    for b in frame[:-2]:
        actual_sc = (actual_sc + b) & 0xFF
        actual_ac = (actual_ac + actual_sc) & 0xFF

    return actual_sc == expected_sc and actual_ac == expected_ac


# ── 模式切换 ──────────────────────────────────────────────


def cmd_mode(mode: int) -> bytes:
    """
    切换飞行模式

    AA FF E0 0B  01 01 01 [MODE] 00 00 00 00 00 00 00  SC AC

    mode:
        0 = 自稳
        1 = 自稳+定高
        2 = 定点(GPS/光流)
        3 = 程控模式
    """
    if mode not in (0, 1, 2, 3):
        raise ValueError(f"Invalid mode: {mode}. Must be 0-3.")
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x01, 0x01, 0x01, mode]) + bytes(7))


# ── 解锁/加锁 ─────────────────────────────────────────────


def cmd_unlock() -> bytes:
    """解锁电机"""
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x00, 0x01]) + bytes(8))


def cmd_lock() -> bytes:
    """加锁电机"""
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x00, 0x02]) + bytes(8))


# ── 起飞/降落 ─────────────────────────────────────────────


def cmd_takeoff(height_cm: int = 0) -> bytes:
    """
    一键起飞

    AA FF E0 0B  10 00 05 [H_LO] [H_HI] 00 00 00 00 00 00  SC AC

    height_cm: 目标高度(u16小端, cm), 0=使用默认值(约150cm)
    """
    h = struct.pack('<H', height_cm)
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x00, 0x05]) + h + bytes(6))


def cmd_land() -> bytes:
    """一键降落"""
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x00, 0x06]) + bytes(8))


# ── 水平移动 ──────────────────────────────────────────────


def cmd_move(distance_cm: int, speed_cmps: int, direction_deg: int) -> bytes:
    """
    水平移动 (最关键指令)

    AA FF E0 0B  10 02 03 [D_LO] [D_HI] [S_LO] [S_HI] [A_LO] [A_HI] 00 00  SC AC

    Args:
        distance_cm: 距离(u16小端, cm), 范围 0~10000
        speed_cmps: 速度(u16小端, cm/s), 范围 10~300
        direction_deg: 方向(u16小端, 0~359度), 0=机头方向, 顺时针增加
    """
    if not (0 <= distance_cm <= 10000):
        raise ValueError(f"distance_cm out of range: {distance_cm}")
    if not (10 <= speed_cmps <= 300):
        raise ValueError(f"speed_cmps out of range: {speed_cmps}")
    if not (0 <= direction_deg <= 359):
        raise ValueError(f"direction_deg out of range: {direction_deg}")

    d = struct.pack('<H', distance_cm)
    s = struct.pack('<H', speed_cmps)
    a = struct.pack('<H', direction_deg)
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x02, 0x03]) + d + s + a + bytes(2))


# ── 高度升降 ──────────────────────────────────────────────


def cmd_ascend(height_cm: int, speed_cmps: int) -> bytes:
    """上升"""
    h = struct.pack('<H', height_cm)
    s = struct.pack('<H', speed_cmps)
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x02, 0x01]) + h + s + bytes(4))


def cmd_descend(height_cm: int, speed_cmps: int) -> bytes:
    """下降"""
    h = struct.pack('<H', height_cm)
    s = struct.pack('<H', speed_cmps)
    return build_lx_frame(0xFF, 0xE0,
        bytes([0x10, 0x02, 0x02]) + h + s + bytes(4))


# ── 辅助函数 ──────────────────────────────────────────────


def print_frame(frame: bytes, label: str = "") -> None:
    """以十六进制格式打印帧（调试用）"""
    hex_str = " ".join(f"{b:02X}" for b in frame)
    if label:
        print(f"[{label}] ({len(frame)}B) {hex_str}")
    else:
        print(f"({len(frame)}B) {hex_str}")


def frame_to_hex(frame: bytes) -> str:
    """帧转十六进制字符串"""
    return "".join(f"{b:02X}" for b in frame)


# ── MCU回传帧解析 ─────────────────────────────────────────


def parse_of_position(buf: bytes) -> Optional[dict]:
    """
    解析MCU回传的光流位置帧

    帧格式: 0xCC 0x01 [POS_X 4B s32 cm] [POS_Y 4B s32 cm] [QUALITY 1B]

    Returns:
        {'pos_x': int, 'pos_y': int, 'quality': int} or None
    """
    if len(buf) < 11:
        return None
    if buf[0] != 0xCC or buf[1] != 0x01:
        return None
    pos_x = struct.unpack('<i', buf[2:6])[0]
    pos_y = struct.unpack('<i', buf[6:10])[0]
    quality = buf[10]
    return {'pos_x': pos_x, 'pos_y': pos_y, 'quality': quality}


def parse_flight_status(buf: bytes) -> Optional[dict]:
    """
    解析MCU回传的飞行状态帧

    帧格式: 0xCC 0x02 [MODE 1B] [LOCKED 1B] [ALT 4B s32 cm]

    Returns:
        {'mode': int, 'locked': int, 'alt': int} or None
    """
    if len(buf) < 8:
        return None
    if buf[0] != 0xCC or buf[1] != 0x02:
        return None
    mode = buf[2]
    locked = buf[3]
    alt = struct.unpack('<i', buf[4:8])[0]
    return {'mode': mode, 'locked': locked, 'alt': alt}


def parse_battery_voltage(buf: bytes) -> Optional[dict]:
    """
    解析MCU回传的电池电压帧

    帧格式: 0xCC 0x03 [V_LO V_HI] (u16 小端, mV)

    Returns:
        {'voltage_mv': int} or None
    """
    if len(buf) < 4:
        return None
    if buf[0] != 0xCC or buf[1] != 0x03:
        return None
    voltage_mv = struct.unpack('<H', buf[2:4])[0]
    return {'voltage_mv': voltage_mv}


def build_heartbeat_query() -> bytes:
    """
    构建心跳查询帧 (Pi→MCU)

    帧结构: 0xBB 0x04

    Returns:
        心跳查询帧字节序列
    """
    return bytes([0xBB, 0x04])
