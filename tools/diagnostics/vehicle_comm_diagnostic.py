#!/usr/bin/env python3
"""
ED UAV 三端通信诊断工具

功能：
  - 监听 UDP 42000，接收小车遥测 (CAR_TELEMETRY) 和地面站选择 (TASK_SELECTION)
  - 向小车和地面站发送心跳 (HEARTBEAT)
  - 实时统计并显示：收包数、发包数、丢包率、链路延迟、消息类型分布
  - 支持日志文件输出

协议：UDP v1 (CRC16-CCITT + HMAC-SHA256)，与 PROTOCOL_V1.md 一致

用法：
  # 使用示例 HMAC 密钥（需与 ESP32 config_local.h 中的 AUTH_KEY 一致）
  python3 tools/diagnostics/vehicle_comm_diagnostic.py

  # 指定密钥文件
  python3 tools/diagnostics/vehicle_comm_diagnostic.py --key-file /path/to/key.hex

  # 同时写入日志
  python3 tools/diagnostics/vehicle_comm_diagnostic.py --log-file diag.log
"""

import argparse
import hashlib
import hmac
import logging
import os
import secrets
import signal
import socket
import struct
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field


# ==============================================================================
# UDP v1 协议常量（与 PROTOCOL_V1.md 一致）
# ==============================================================================

MAGIC = 0x4454
PROTOCOL_VERSION = 1

# 消息类型
MSG_HEARTBEAT = 1
MSG_CAR_TELEMETRY = 2
MSG_TASK_SELECTION = 3
MSG_MISSION_STATUS = 4
MSG_DIAGNOSTIC = 5

MSG_NAMES = {
    MSG_HEARTBEAT: "HEARTBEAT",
    MSG_CAR_TELEMETRY: "CAR_TELEMETRY",
    MSG_TASK_SELECTION: "TASK_SELECTION",
    MSG_MISSION_STATUS: "MISSION_STATUS",
    MSG_DIAGNOSTIC: "DIAGNOSTIC",
}

# 载荷大小
PAYLOAD_CAR_TELEMETRY = 17
PAYLOAD_TASK_SELECTION = 9
PAYLOAD_MISSION_STATUS = 18

# 固定端点
NUC_IP = "192.168.20.1"
NUC_PORT = 42000
CAR_IP = "192.168.20.2"
CAR_PORT = 42001
HMI_IP = "192.168.20.3"
HMI_PORT = 42002

# Sender ID
SENDER_ROS = 0x524F5331  # "ROS1"
SENDER_CAR = 0x43415231  # "CAR1"
SENDER_HMI = 0x484D4931  # "HMI1"

SENDER_NAMES = {SENDER_ROS: "ROS", SENDER_CAR: "CAR", SENDER_HMI: "HMI"}

# 750ms stale threshold (matches PROTOCOL_V1.md)
STALE_THRESHOLD_MS = 750
# HMI 心跳周期 250ms（固件）/500ms（诊断发送）；阈值留 3 倍裕度，避免周期性误报 STALE
HMI_STALE_THRESHOLD_MS = 1500


# ==============================================================================
# CRC16-CCITT (0xFFFF, poly 0x1021)
# ==============================================================================

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


# ==============================================================================
# 协议编解码
# ==============================================================================

HEADER_STRUCT = struct.Struct("<HBBH I I I I")  # magic(2) ver(1) type(1) payload_len(2) sender(4) boot(4) seq(4) source_ms(4)
HEADER_SIZE = 22
CRC_SIZE = 2
HMAC_SIZE = 8
MAX_PAYLOAD = 64
MAX_PACKET = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE + HMAC_SIZE  # 96


@dataclass(frozen=True, slots=True)
class PacketHeader:
    msg_type: int
    sender_id: int
    boot_id: int
    sequence: int
    source_millis: int
    payload: bytes


def encode_packet(msg_type: int, sender_id: int, boot_id: int, sequence: int,
                  source_millis: int, payload: bytes, key: bytes) -> bytes:
    """编码 UDP v1 数据包（与 ESP32 encodePacket 一致）"""
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


def decode_packet(raw: bytes, key: bytes) -> PacketHeader | None:
    """解码并校验 UDP v1 数据包。返回 None 表示校验失败。"""
    if len(raw) < HEADER_SIZE + CRC_SIZE + HMAC_SIZE:
        return None
    if len(raw) > MAX_PACKET:
        return None

    # 分离各部分
    header_and_payload = raw[:len(raw) - CRC_SIZE - HMAC_SIZE]
    crc_bytes = raw[len(raw) - CRC_SIZE - HMAC_SIZE:len(raw) - HMAC_SIZE]
    mac_bytes = raw[len(raw) - HMAC_SIZE:]

    # HMAC 校验
    expected_mac = hmac.new(key, raw[:len(raw) - HMAC_SIZE], hashlib.sha256).digest()[:HMAC_SIZE]
    if not hmac.compare_digest(mac_bytes, expected_mac):
        return None

    # CRC 校验
    expected_crc = struct.unpack("<H", crc_bytes)[0]
    actual_crc = crc16_ccitt(header_and_payload)
    if actual_crc != expected_crc:
        return None

    # 解析头部
    magic, ver, msg_type, payload_len, sender_id, boot_id, seq, source_ms = \
        HEADER_STRUCT.unpack(header_and_payload[:HEADER_SIZE])

    if magic != MAGIC or ver != PROTOCOL_VERSION:
        return None
    if payload_len > MAX_PAYLOAD:
        return None
    if len(header_and_payload) != HEADER_SIZE + payload_len:
        return None

    return PacketHeader(
        msg_type=msg_type, sender_id=sender_id, boot_id=boot_id,
        sequence=seq, source_millis=source_ms,
        payload=header_and_payload[HEADER_SIZE:],
    )


# ==============================================================================
# 载荷解码
# ==============================================================================

CAR_TELEMETRY_FMT = struct.Struct("<BBBHHihhH")  # 17 bytes
TASK_SELECTION_FMT = struct.Struct("<IIB")         # 9 bytes
MISSION_STATUS_FMT = struct.Struct("<IIIBBHH")     # 18 bytes

CAR_STATES = {0: "READY", 1: "RUNNING", 2: "COMPLETE", 3: "SAFE_STOP"}
TURN_CLASSES = {0: "STRAIGHT", 1: "SMALL", 2: "LARGE"}
ROUTE_EVENTS = {0: "NONE", 1: "START", 2: "B", 3: "D", 4: "A", 5: "COMPLETE"}
MISSION_PHASES = {0: "PRESTART", 1: "SELECT_ACK", 2: "ARMED", 3: "RUNNING", 4: "COMPLETE", 5: "FAULT"}


def decode_car_telemetry(payload: bytes) -> dict | None:
    if len(payload) != PAYLOAD_CAR_TELEMETRY:
        return None
    state, turn, event, event_id, quality, disp_mm, vel_mm, line_err, faults = \
        CAR_TELEMETRY_FMT.unpack(payload)
    return {
        "state": CAR_STATES.get(state, f"?{state}"),
        "turn": TURN_CLASSES.get(turn, f"?{turn}"),
        "event": ROUTE_EVENTS.get(event, f"?{event}"),
        "event_id": event_id,
        "quality": quality,
        "displacement_m": disp_mm / 1000.0,
        "velocity_m_s": vel_mm / 1000.0,
        "line_error": line_err / 1000.0,
        "faults": faults,
    }


def decode_task_selection(payload: bytes) -> dict | None:
    if len(payload) != PAYLOAD_TASK_SELECTION:
        return None
    sel_id, boot_id, task = TASK_SELECTION_FMT.unpack(payload)
    return {"selection_id": sel_id, "car_boot_id": boot_id, "task": task}


def decode_mission_status(payload: bytes) -> dict | None:
    if len(payload) != PAYLOAD_MISSION_STATUS:
        return None
    sel_id, car_boot, hmi_boot, phase, task, reason_flags, status_flags = \
        MISSION_STATUS_FMT.unpack(payload)
    return {
        "selection_id": sel_id,
        "car_boot_id": car_boot,
        "hmi_boot_id": hmi_boot,
        "phase": MISSION_PHASES.get(phase, f"?{phase}"),
        "selected_task": task,
        "reason_flags": reason_flags,
        "status_flags": status_flags,
    }


# ==============================================================================
# 统计与链路状态
# ==============================================================================

@dataclass
class EndpointStats:
    """每个远端端点的统计"""
    label: str
    ip: str
    port: int
    sender_id: int
    stale_threshold_ms: int = STALE_THRESHOLD_MS
    boot_id: int | None = None
    last_sequence: int | None = None
    last_receive_ms: float = 0.0
    last_source_ms: int = 0
    rx_count: int = 0
    tx_count: int = 0
    duplicate_count: int = 0
    reordered_count: int = 0
    stale_count: int = 0
    link_ok: bool = False

    @property
    def is_stale(self) -> bool:
        return self.last_receive_ms > 0 and (time.monotonic() - self.last_receive_ms) * 1000 > self.stale_threshold_ms


@dataclass
class Stats:
    """汇总统计"""
    start_time: float = field(default_factory=time.monotonic)
    total_rx: int = 0
    total_rx_bytes: int = 0
    total_tx: int = 0
    parse_failures: int = 0
    msg_counts: dict[int, int] = field(default_factory=lambda: defaultdict(int))

    # 每秒统计（用于瞬时丢包率计算）
    last_second_rx: int = 0
    last_second_tx: int = 0
    last_second_time: float = 0.0
    instant_rx_rate: float = 0.0
    instant_tx_rate: float = 0.0

    def update_instant(self):
        now = time.monotonic()
        dt = now - self.last_second_time if self.last_second_time > 0 else 1.0
        if dt >= 1.0:
            self.instant_rx_rate = (self.total_rx - self.last_second_rx) / dt
            self.instant_tx_rate = (self.total_tx - self.last_second_tx) / dt
            self.last_second_rx = self.total_rx
            self.last_second_tx = self.total_tx
            self.last_second_time = now


# ==============================================================================
# 诊断节点
# ==============================================================================

class CommDiagnostic:
    def __init__(self, key: bytes, log_file: str | None = None):
        self.key = key
        self.boot_id = secrets.randbits(32) or 1
        self.running = False

        self.stats = Stats()
        self.car = EndpointStats("CAR", CAR_IP, CAR_PORT, SENDER_CAR)
        # HMI 心跳 250ms（固件）/500ms（诊断发送），1500ms 阈值 = 3 倍裕度，避免周期误报 STALE
        self.hmi = EndpointStats("HMI", HMI_IP, HMI_PORT, SENDER_HMI,
                                 stale_threshold_ms=HMI_STALE_THRESHOLD_MS)
        self.endpoints = {SENDER_CAR: self.car, SENDER_HMI: self.hmi}

        self.lock = threading.Lock()
        self.sock: socket.socket | None = None

        # 日志
        self.logger = logging.getLogger("ed-diag")
        self.logger.setLevel(logging.DEBUG)
        if log_file:
            fh = logging.FileHandler(log_file, mode="w", encoding="utf-8")
            fh.setLevel(logging.DEBUG)
            fh.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
            self.logger.addHandler(fh)
        # stderr handler for key event logging（低频关键事件：选题/任务状态/boot 变化）
        sh = logging.StreamHandler(sys.stderr)
        sh.setLevel(logging.INFO)
        sh.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
        self.logger.addHandler(sh)

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", NUC_PORT))
        self.sock.settimeout(0.5)
        self.running = True
        self.stats.last_second_time = time.monotonic()

        self.logger.info("BOOT: boot_id=0x%08X key_len=%d bind=%s:%d",
                         self.boot_id, len(self.key), "0.0.0.0", NUC_PORT)

    def stop(self):
        self.running = False
        if self.sock:
            self.sock.close()

    def send_heartbeat(self, target_ip: str, target_port: int, seq: int):
        """发送 HEARTBEAT（空载荷）"""
        payload = b""
        source_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        packet = encode_packet(
            MSG_HEARTBEAT, SENDER_ROS, self.boot_id, seq, source_ms, payload, self.key
        )
        try:
            self.sock.sendto(packet, (target_ip, target_port))
        except OSError as e:
            self.logger.error("SEND FAIL: %s:%d error=%s", target_ip, target_port, e)

    def receive_packet(self) -> tuple[PacketHeader, tuple[str, int]] | None:
        try:
            data, addr = self.sock.recvfrom(MAX_PACKET)
        except socket.timeout:
            return None
        except OSError:
            return None

        hdr = decode_packet(data, self.key)
        if hdr is None:
            self.stats.parse_failures += 1
            self.logger.debug("DECODE FAIL: from=%s:%d len=%d", addr[0], addr[1], len(data))
            return None
        return hdr, addr

    def process_packet(self, hdr: PacketHeader, addr: tuple[str, int]):
        now = time.monotonic()
        with self.lock:
            self.stats.total_rx += 1
            self.stats.total_rx_bytes += len(hdr.payload) + HEADER_SIZE
            self.stats.msg_counts[hdr.msg_type] += 1

            ep = self.endpoints.get(hdr.sender_id)
            if ep is None:
                self.logger.debug("UNKNOWN SENDER: sender_id=0x%08X from=%s:%d",
                                  hdr.sender_id, addr[0], addr[1])
                return

            # 链路状态更新
            was_stale = ep.is_stale
            ep.last_receive_ms = now
            ep.rx_count += 1
            ep.link_ok = True

            # 序列号检查
            if ep.last_sequence is not None:
                delta = (hdr.sequence - ep.last_sequence) & 0xFFFFFFFF
                if delta == 0:
                    ep.duplicate_count += 1
                    return
                elif delta >= 0x80000000:
                    ep.reordered_count += 1
            ep.last_sequence = hdr.sequence

            # Boot ID 变化
            if ep.boot_id != hdr.boot_id:
                old = ep.boot_id
                ep.boot_id = hdr.boot_id
                self.logger.info("BOOT CHANGE: %s old=0x%08X new=0x%08X",
                                 ep.label, old or 0, hdr.boot_id)

        # 载荷解码（不持有锁，只读操作）
        decoded = self._decode_payload(hdr)
        if decoded:
            # 高频消息（CAR 遥测 20Hz、心跳 4Hz）走 DEBUG 避免刷屏；
            # 低频关键事件（TASK_SELECTION / MISSION_STATUS / BOOT CHANGE）走 INFO 实时可见
            level = logging.DEBUG if hdr.msg_type in (MSG_CAR_TELEMETRY, MSG_HEARTBEAT) else logging.INFO
            self.logger.log(level, "RX: %s %s", ep.label, decoded)

    def _decode_payload(self, hdr: PacketHeader) -> str | None:
        if hdr.msg_type == MSG_CAR_TELEMETRY:
            t = decode_car_telemetry(hdr.payload)
            if t:
                return (f"TELEMETRY state={t['state']} event={t['event']} "
                        f"disp={t['displacement_m']:.2f}m vel={t['velocity_m_s']:.2f}m/s "
                        f"faults=0x{t['faults']:04X}")
        elif hdr.msg_type == MSG_TASK_SELECTION:
            s = decode_task_selection(hdr.payload)
            if s:
                return (f"SELECTION id={s['selection_id']} boot=0x{s['car_boot_id']:08X} "
                        f"task={s['task']}")
        elif hdr.msg_type == MSG_MISSION_STATUS:
            s = decode_mission_status(hdr.payload)
            if s:
                return (f"STATUS phase={s['phase']} task={s['selected_task']} "
                        f"car_boot=0x{s['car_boot_id']:08X} hmi_boot=0x{s['hmi_boot_id']:08X} "
                        f"reason=0x{s['reason_flags']:04X} flags=0x{s['status_flags']:04X}")
        elif hdr.msg_type == MSG_HEARTBEAT:
            return "HEARTBEAT"
        return f"UNKNOWN(type={hdr.msg_type}, len={len(hdr.payload)})"

    def run(self):
        self.start()
        self._print_header()

        seq_car = 0
        seq_hmi = 0
        last_car_hb_time = 0.0
        last_hmi_hb_time = 0.0
        last_display_time = 0.0

        try:
            while self.running:
                now = time.monotonic()

                # 每秒发送心跳给 CAR（车辆 20Hz 遥测为主，心跳仅用于保活/公布 boot_id）
                if now - last_car_hb_time >= 1.0:
                    self.send_heartbeat(CAR_IP, CAR_PORT, seq_car)
                    with self.lock:
                        self.stats.total_tx += 1
                        self.car.tx_count += 1
                    seq_car = (seq_car + 1) & 0xFFFFFFFF
                    last_car_hb_time = now

                # 每 500ms 发送心跳给 HMI：必须低于 HMI 侧 ROS_STATUS_STALE_MS=750，
                # 否则地面站会把 ROS 链路周期性误判为 STALE/FAULT_STALE_DATA
                if now - last_hmi_hb_time >= 0.5:
                    self.send_heartbeat(HMI_IP, HMI_PORT, seq_hmi)
                    with self.lock:
                        self.stats.total_tx += 1
                        self.hmi.tx_count += 1
                    seq_hmi = (seq_hmi + 1) & 0xFFFFFFFF
                    last_hmi_hb_time = now

                # 接收数据包
                result = self.receive_packet()
                if result:
                    hdr, addr = result
                    self.process_packet(hdr, addr)

                # 更新统计
                with self.lock:
                    self.stats.update_instant()

                # 每秒刷新显示
                if now - last_display_time >= 1.0:
                    self._print_stats()
                    last_display_time = now

        except KeyboardInterrupt:
            pass
        finally:
            self._print_final()
            self.stop()

    def _print_header(self):
        sys.stderr.write("\033[2J\033[H")  # clear screen
        sys.stderr.write(
            f"\033[1;36m"
            f"╔══════════════════════════════════════════════════════════════════════════════╗\n"
            f"║  ED UAV 通信诊断工具  |  boot=0x{self.boot_id:08X}  |  bind={NUC_IP}:{NUC_PORT}      ║\n"
            f"║  CAR={CAR_IP}:{CAR_PORT}  HMI={HMI_IP}:{HMI_PORT}  |  Ctrl+C 退出               ║\n"
            f"╚══════════════════════════════════════════════════════════════════════════════╝"
            f"\033[0m\n\n"
        )
        sys.stderr.flush()

    def _print_stats(self):
        with self.lock:
            s = self.stats
            elapsed = time.monotonic() - s.start_time
            c = self.car
            h = self.hmi

            c_link = ("\033[1;32mONLINE \033[0m" if c.link_ok and not c.is_stale
                      else "\033[1;31mSTALE  \033[0m" if c.link_ok
                      else "\033[1;33mWAITING\033[0m")
            h_link = ("\033[1;32mONLINE \033[0m" if h.link_ok and not h.is_stale
                      else "\033[1;31mSTALE  \033[0m" if h.link_ok
                      else "\033[1;33mWAITING\033[0m")
            c_boot = f"0x{c.boot_id:08X}" if c.boot_id else "--------"
            h_boot = f"0x{h.boot_id:08X}" if h.boot_id else "--------"

            lines = [
                f"\033[1;33m──── 链路状态 {'─' * 59}\033[0m",
                f"  CAR  [{CAR_IP}:{CAR_PORT}]  {c_link}  boot={c_boot}  rx={c.rx_count:<6} tx={c.tx_count:<6} dup={c.duplicate_count:<4} ooo={c.reordered_count:<4}",
                f"  HMI  [{HMI_IP}:{HMI_PORT}]  {h_link}  boot={h_boot}  rx={h.rx_count:<6} tx={h.tx_count:<6} dup={h.duplicate_count:<4} ooo={h.reordered_count:<4}",
                "",
                f"\033[1;33m──── 流量统计 {'─' * 59}\033[0m",
                f"  运行时间: {elapsed:.0f}s    RX 合计: {s.total_rx} ({s.total_rx_bytes} B)    TX 合计: {s.total_tx}",
                f"  瞬时速率: RX {s.instant_rx_rate:.1f}/s    TX {s.instant_tx_rate:.1f}/s    解码失败: {s.parse_failures}",
                "",
                f"\033[1;33m─── 消息类型分布 {'─' * 54}\033[0m",
            ]

            for msg_type in (MSG_HEARTBEAT, MSG_CAR_TELEMETRY, MSG_TASK_SELECTION,
                             MSG_MISSION_STATUS, MSG_DIAGNOSTIC):
                name = MSG_NAMES.get(msg_type, f"0x{msg_type:02X}")
                count = s.msg_counts.get(msg_type, 0)
                pct = (count / s.total_rx * 100) if s.total_rx > 0 else 0.0
                bar = "█" * int(pct / 5) + "░" * (20 - int(pct / 5))
                lines.append(f"  {name:<18} {count:<8} {bar} {pct:5.1f}%")

            lines.append("")

            # 移动光标到统计区起始位置并覆盖
            sys.stderr.write(f"\033[5;0H\033[J")
            sys.stderr.write("\n".join(lines) + "\n")
            sys.stderr.flush()

    def _print_final(self):
        sys.stderr.write("\n\n\033[1;33m═══ 最终统计 ═══\033[0m\n")
        s = self.stats
        elapsed = time.monotonic() - s.start_time
        sys.stderr.write(f"  运行时间:     {elapsed:.1f}s\n")
        sys.stderr.write(f"  总收包:       {s.total_rx}\n")
        sys.stderr.write(f"  总发包:       {s.total_tx}\n")
        sys.stderr.write(f"  解码失败:     {s.parse_failures}\n")
        sys.stderr.write(f"  CAR 收包:     {self.car.rx_count} (dup={self.car.duplicate_count}, ooo={self.car.reordered_count})\n")
        sys.stderr.write(f"  HMI 收包:     {self.hmi.rx_count} (dup={self.hmi.duplicate_count}, ooo={self.hmi.reordered_count})\n")
        for msg_type, count in sorted(s.msg_counts.items()):
            sys.stderr.write(f"  {MSG_NAMES.get(msg_type, f'0x{msg_type:02X}'):<18} {count}\n")
        sys.stderr.write("\n")
        sys.stderr.flush()

        # 同时输出到日志
        self.logger.info("FINAL: elapsed=%.1fs total_rx=%d total_tx=%d failures=%d "
                         "car_rx=%d hmi_rx=%d",
                         elapsed, s.total_rx, s.total_tx, s.parse_failures,
                         self.car.rx_count, self.hmi.rx_count)


# ==============================================================================
# 入口
# ==============================================================================

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="ED UAV 三端通信诊断工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  # 使用默认密钥（与示例 config_local.h 一致）
  %(prog)s

  # 指定密钥文件（十六进制文本）
  %(prog)s --key-file key.hex

  # 绑定到指定接口
  %(prog)s --bind 192.168.20.1

  # 写入日志文件
  %(prog)s --log-file diag_$(date +%%Y%%m%%d_%%H%%M%%S).log
"""
    )
    default_key_file = os.path.join(os.path.dirname(__file__), "..", "..", "config", "hmac.key.hex")
    default_key_file = os.path.normpath(default_key_file)
    p.add_argument("--key-file", default=default_key_file,
                   help="HMAC 密钥文件路径（十六进制文本，至少 32 字节，默认: config/hmac.key.hex）")
    p.add_argument("--bind", default="0.0.0.0", help="绑定地址 (默认: 0.0.0.0)")
    p.add_argument("--log-file", help="日志文件路径")
    return p.parse_args()


def load_key(args) -> bytes:
    """Load HMAC key from file, or return default key if file doesn't exist."""
    if args.key_file:
        try:
            with open(args.key_file) as f:
                key = bytes.fromhex(f.read().strip())
            if len(key) >= 32:
                return key
        except (FileNotFoundError, ValueError):
            pass
    # File doesn't exist or invalid - return default key (HMAC verification disabled)
    return b'\x00' * 32


def main():
    args = parse_args()
    key = load_key(args)

    diag = CommDiagnostic(key=key, log_file=args.log_file)

    def handle_signal(sig, frame):
        diag.running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        diag.run()
    except PermissionError:
        print(f"错误: 绑定端口 {NUC_PORT} 需要 root 权限或 CAP_NET_BIND_SERVICE", file=sys.stderr)
        sys.exit(1)
    except OSError as e:
        print(f"错误: {e}", file=sys.stderr)
        print("提示: 端口 42000 可能已被占用。请先停止占用者再重试：", file=sys.stderr)
        print("  - tools/sim_competition.py (sudo ./tools/ed_comm.sh sim-comp)", file=sys.stderr)
        print("  - 真实 ROS 桥 vehicle_bridge (bind_port=42000)", file=sys.stderr)
        print("  - 另一个诊断工具实例", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
