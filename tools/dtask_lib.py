#!/usr/bin/env python3
"""
dtask_lib.py — DTask UDP v1 协议 Python 实现

与 PROTOCOL_V1.md 和 DTaskProtocol.cpp 完全一致的编解码库。
支持 CAR_TELEMETRY、TASK_SELECTION、MISSION_STATUS、HEARTBEAT 消息类型。

用法:
    from dtask_lib import DTaskCodec, MessageType, CarState, MissionPhase
"""

import hashlib
import hmac
import secrets
import socket
import struct
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


# ── 协议常量 ─────────────────────────────────────────────

MAGIC = 0x4454
PROTOCOL_VERSION = 1
HEADER_SIZE = 22
CRC_SIZE = 2
HMAC_SIZE = 8
MAX_PAYLOAD = 64
MAX_PACKET = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE + HMAC_SIZE  # 96

# 固定端点
NUC_IP = "192.168.20.1"
NUC_PORT = 42000
CAR_IP = "192.168.20.2"
CAR_PORT = 42001
HMI_IP = "192.168.20.3"
HMI_PORT = 42002

# Sender ID (与 C++ 一致)
SENDER_ROS = 0x524F5331   # "ROS1"
SENDER_CAR = 0x43415231   # "CAR1"
SENDER_HMI = 0x484D4931   # "HMI1"

STALE_MS = 750


# ── 枚举 ─────────────────────────────────────────────────

class MessageType(IntEnum):
    HEARTBEAT = 1
    CAR_TELEMETRY = 2
    TASK_SELECTION = 3
    MISSION_STATUS = 4
    DIAGNOSTIC = 5


class CarState(IntEnum):
    READY = 0
    RUNNING = 1
    COMPLETE = 2
    SAFE_STOP = 3


class TurnClass(IntEnum):
    STRAIGHT = 0
    SMALL = 1
    LARGE = 2


class RouteEvent(IntEnum):
    NONE = 0
    START = 1
    B = 2
    D = 3
    A = 4
    COMPLETE = 5


class MissionPhase(IntEnum):
    PRESTART = 0
    SELECTION_ACKED = 1
    ARMED_READY = 2
    CAR_RUNNING = 3
    COMPLETE = 4
    FAULT = 5


class QualityFlag(IntEnum):
    LINE_VALID = 1 << 0
    ENCODER_VALID = 1 << 1
    WIFI_CONNECTED = 1 << 2
    SELECTION_COMMITTED = 1 << 3


class MissionStatusFlag(IntEnum):
    DRONE_LINK_OK = 1 << 0
    DRONE_ARMED = 1 << 1
    VISION_VALID = 1 << 2
    ROS_READY = 1 << 3


class FaultFlag(IntEnum):
    NONE = 0
    WIFI_TIMEOUT = 1 << 0
    LINE_LOST = 1 << 1
    ENCODER_DISAGREE = 1 << 2
    PID_OVERRUN = 1 << 3
    BUTTON_STUCK = 1 << 4
    MOTOR = 1 << 5
    STALE_DATA = 1 << 6
    PROTOCOL = 1 << 7
    NO_COMMITTED_SELECTION = 1 << 8
    BROWNOUT = 1 << 9


# ── 数据类 ───────────────────────────────────────────────

@dataclass
class PacketHeader:
    msg_type: MessageType
    sender_id: int
    boot_id: int
    sequence: int
    source_millis: int
    payload: bytes = b""


@dataclass
class CarTelemetry:
    state: CarState = CarState.READY
    turn: TurnClass = TurnClass.STRAIGHT
    event: RouteEvent = RouteEvent.NONE
    event_id: int = 0
    quality_flags: int = 0
    displacement_mm: int = 0
    velocity_mm_s: int = 0
    line_error_milli: int = 0
    fault_flags: int = 0


@dataclass
class TaskSelection:
    selection_id: int = 0
    car_boot_id: int = 0
    task: int = 0


@dataclass
class MissionStatus:
    selection_id: int = 0
    car_boot_id: int = 0
    hmi_boot_id: int = 0
    phase: MissionPhase = MissionPhase.PRESTART
    selected_task: int = 0
    reason_flags: int = 0
    status_flags: int = 0


# ── 载荷格式 ─────────────────────────────────────────────

CAR_TELEMETRY_FMT = struct.Struct("<BBBHHihhH")   # 17 bytes
TASK_SELECTION_FMT = struct.Struct("<IIB")          # 9 bytes
MISSION_STATUS_FMT = struct.Struct("<IIIBBHH")      # 18 bytes
HEADER_STRUCT = struct.Struct("<HBBHIIII")           # 22 bytes


# ── CRC16-CCITT ──────────────────────────────────────────

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


# ── 编码 ─────────────────────────────────────────────────

def encode_packet(msg_type: int, sender_id: int, boot_id: int,
                  sequence: int, source_millis: int,
                  payload: bytes, key: bytes) -> bytes:
    """编码 UDP v1 数据包"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too large: {len(payload)} > {MAX_PAYLOAD}")

    header = HEADER_STRUCT.pack(
        MAGIC, PROTOCOL_VERSION, msg_type, len(payload),
        sender_id, boot_id, sequence, source_millis
    )
    packet = header + payload
    crc = crc16_ccitt(packet)
    packet += struct.pack("<H", crc)
    mac = hmac.new(key, packet, hashlib.sha256).digest()[:HMAC_SIZE]
    packet += mac
    return packet


def encode_car_telemetry(telemetry: CarTelemetry) -> bytes:
    return CAR_TELEMETRY_FMT.pack(
        telemetry.state, telemetry.turn, telemetry.event,
        telemetry.event_id, telemetry.quality_flags,
        telemetry.displacement_mm, telemetry.velocity_mm_s,
        telemetry.line_error_milli, telemetry.fault_flags
    )


def encode_task_selection(selection: TaskSelection) -> bytes:
    return TASK_SELECTION_FMT.pack(
        selection.selection_id, selection.car_boot_id, selection.task
    )


def encode_mission_status(status: MissionStatus) -> bytes:
    return MISSION_STATUS_FMT.pack(
        status.selection_id, status.car_boot_id, status.hmi_boot_id,
        status.phase, status.selected_task,
        status.reason_flags, status.status_flags
    )


# ── 解码 ─────────────────────────────────────────────────

def decode_packet(raw: bytes, key: bytes) -> Optional[PacketHeader]:
    """解码并校验 UDP v1 数据包"""
    if len(raw) < HEADER_SIZE + CRC_SIZE + HMAC_SIZE:
        return None
    if len(raw) > MAX_PACKET:
        return None

    header_and_payload = raw[:len(raw) - CRC_SIZE - HMAC_SIZE]
    crc_bytes = raw[len(raw) - CRC_SIZE - HMAC_SIZE:len(raw) - HMAC_SIZE]
    mac_bytes = raw[len(raw) - HMAC_SIZE:]

    expected_mac = hmac.new(key, raw[:len(raw) - HMAC_SIZE], hashlib.sha256).digest()[:HMAC_SIZE]
    if not hmac.compare_digest(mac_bytes, expected_mac):
        return None

    expected_crc = struct.unpack("<H", crc_bytes)[0]
    if crc16_ccitt(header_and_payload) != expected_crc:
        return None

    magic, ver, msg_type, payload_len, sender_id, boot_id, seq, source_ms = \
        HEADER_STRUCT.unpack(header_and_payload[:HEADER_SIZE])

    if magic != MAGIC or ver != PROTOCOL_VERSION:
        return None
    if payload_len > MAX_PAYLOAD:
        return None
    if len(header_and_payload) != HEADER_SIZE + payload_len:
        return None

    try:
        mt = MessageType(msg_type)
    except ValueError:
        return None

    return PacketHeader(
        msg_type=mt, sender_id=sender_id, boot_id=boot_id,
        sequence=seq, source_millis=source_ms,
        payload=header_and_payload[HEADER_SIZE:]
    )


def decode_car_telemetry(payload: bytes) -> Optional[CarTelemetry]:
    if len(payload) != 17:
        return None
    state, turn, event, event_id, quality, disp_mm, vel_mm, line_err, faults = \
        CAR_TELEMETRY_FMT.unpack(payload)
    return CarTelemetry(
        state=CarState(state), turn=TurnClass(turn), event=RouteEvent(event),
        event_id=event_id, quality_flags=quality,
        displacement_mm=disp_mm, velocity_mm_s=vel_mm,
        line_error_milli=line_err, fault_flags=faults
    )


def decode_task_selection(payload: bytes) -> Optional[TaskSelection]:
    if len(payload) != 9:
        return None
    sel_id, boot_id, task = TASK_SELECTION_FMT.unpack(payload)
    return TaskSelection(selection_id=sel_id, car_boot_id=boot_id, task=task)


def decode_mission_status(payload: bytes) -> Optional[MissionStatus]:
    if len(payload) != 18:
        return None
    sel_id, car_boot, hmi_boot, phase, task, reason, flags = \
        MISSION_STATUS_FMT.unpack(payload)
    return MissionStatus(
        selection_id=sel_id, car_boot_id=car_boot, hmi_boot_id=hmi_boot,
        phase=MissionPhase(phase), selected_task=task,
        reason_flags=reason, status_flags=flags
    )


# ── UDP 收发器 ───────────────────────────────────────────

class DTaskCodec:
    """DTask UDP 编解码器，绑定本地端口收发"""

    def __init__(self, key: bytes, bind_host: str = "0.0.0.0",
                 bind_port: int = NUC_PORT):
        self.key = key
        self.boot_id = secrets.randbits(32) or 1
        self._seq = 0
        self._start_ms = time.monotonic()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        self.sock.bind((bind_host, bind_port))
        self.sock.settimeout(0.1)

    def close(self):
        self.sock.close()

    def _source_ms(self) -> int:
        return int((time.monotonic() - self._start_ms) * 1000) & 0xFFFFFFFF

    def send(self, msg_type: MessageType, payload: bytes,
             target_ip: str, target_port: int):
        pkt = encode_packet(
            msg_type, SENDER_ROS, self.boot_id,
            self._seq, self._source_ms(), payload, self.key
        )
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        self.sock.sendto(pkt, (target_ip, target_port))

    def send_heartbeat(self, target_ip: str, target_port: int):
        self.send(MessageType.HEARTBEAT, b"", target_ip, target_port)

    def send_mission_status(self, status: MissionStatus,
                            target_ip: str = HMI_IP, target_port: int = HMI_PORT):
        self.send(MessageType.MISSION_STATUS,
                  encode_mission_status(status), target_ip, target_port)

    def recv(self) -> Optional[tuple[PacketHeader, tuple[str, int]]]:
        try:
            data, addr = self.sock.recvfrom(MAX_PACKET)
        except (socket.timeout, OSError):
            return None
        hdr = decode_packet(data, self.key)
        if hdr is None:
            return None
        return hdr, addr
