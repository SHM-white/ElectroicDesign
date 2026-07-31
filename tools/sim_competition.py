#!/usr/bin/env python3
"""
ED UAV 全流程比赛模拟器

绑定正确的源端口（ROS=42000, CAR=42001），模拟一场完整比赛：
  BOOT_WAITING → PRESTART → SELECTED → ARMED → RUNNING → COMPLETE

地面站 HMI 需要做相应的状态/数据显示变化，并且可以通过串口/按键
选择任务（task 1/2/3），模拟器会回复 SELECT_ACK。

虚拟 CAR 模式（默认开启）：无真实小车时，用 IP_TRANSPARENT 把模拟 CAR
遥测的源 IP 伪装成 192.168.20.2:42001（HMI 按源 IP 过滤，必须来自该地址），
并照常发送 MISSION_STATUS 回执，让 HMI 完整走通状态机。真实小车上线后
自动让位给真实遥测。需要 root（模拟器本就以 sudo 运行）。

用法：
  # 模拟完整比赛（自动推进 + 接受地面站选题）
  sudo python3 tools/sim_competition.py

  # 指定任务（跳过地面站选题，自动选 task 1）
  sudo python3 tools/sim_competition.py --task 1

  # 不使用虚拟 CAR（仅诊断/ROS 链路模拟）
  sudo python3 tools/sim_competition.py --no-virtual-car

  # 使用 example 密钥（与 config_local.example.h 一致）
  sudo python3 tools/sim_competition.py --example-key
"""

import argparse
import hashlib
import hmac as hmac_mod
import os
import secrets
import signal
import socket
import struct
import sys
import time

# ─── 协议常量（与 DTaskProtocol.h / vehicle_comm_diagnostic.py 一致）───

MAGIC = 0x4454
PROTOCOL_VERSION = 1
HEADER_STRUCT = struct.Struct("<HBBH I I I I")
HEADER_SIZE = 22
CRC_SIZE = 2
HMAC_SIZE = 8
MAX_PAYLOAD = 64
MAX_PACKET = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE + HMAC_SIZE

MSG_HEARTBEAT = 1
MSG_CAR_TELEMETRY = 2
MSG_TASK_SELECTION = 3
MSG_MISSION_STATUS = 4

SENDER_ROS = 0x524F5331  # "ROS1"
SENDER_CAR = 0x43415231  # "CAR1"
SENDER_HMI = 0x484D4931  # "HMI1"

NUC_IP = "192.168.20.1"
NUC_PORT = 42000
CAR_IP = "192.168.20.2"
CAR_PORT = 42001
HMI_IP = "192.168.20.3"
HMI_PORT = 42002

# MissionPhase (DTaskProtocol.h)
PHASE_PRESTART = 0
PHASE_SELECT_ACK = 1
PHASE_ARMED = 2
PHASE_RUNNING = 3
PHASE_COMPLETE = 4
PHASE_FAULT = 5

# CarState
CAR_READY = 0
CAR_RUNNING = 1
CAR_COMPLETE = 2
CAR_SAFE_STOP = 3

# RouteEvent
EVT_NONE = 0
EVT_START = 1
EVT_B = 2
EVT_D = 3
EVT_A = 4
EVT_COMPLETE = 5

# TurnClass
TURN_STRAIGHT = 0
TURN_SMALL = 1
TURN_LARGE = 2

# QualityFlag
Q_LINE_VALID = 0x01
Q_ENCODER_VALID = 0x02
Q_WIFI_CONNECTED = 0x04
Q_SELECTION_COMMITTED = 0x08

# MissionStatusFlag
FLAG_DRONE_LINK_OK = 0x01
FLAG_DRONE_ARMED = 0x02
FLAG_VISION_VALID = 0x04
FLAG_ROS_READY = 0x08


# ─── 颜色 ───────────────────────────────────────────────────────────────────
R = '\033[0;31m'; G = '\033[0;32m'; Y = '\033[0;33m'; C = '\033[0;36m'
B = '\033[1m'; DIM = '\033[2m'; N = '\033[0m'

PHASE_NAMES = {
    PHASE_PRESTART: "PRESTART", PHASE_SELECT_ACK: "SELECT_ACK",
    PHASE_ARMED: "ARMED", PHASE_RUNNING: "RUNNING",
    PHASE_COMPLETE: "COMPLETE", PHASE_FAULT: "FAULT",
}
CAR_STATE_NAMES = {CAR_READY: "READY", CAR_RUNNING: "RUNNING",
                   CAR_COMPLETE: "COMPLETE", CAR_SAFE_STOP: "SAFE_STOP"}
EVT_NAMES = {EVT_NONE: "—", EVT_START: "START", EVT_B: "B弯",
             EVT_D: "D弯", EVT_A: "A弯", EVT_COMPLETE: "DONE"}
TURN_NAMES = {TURN_STRAIGHT: "→", TURN_SMALL: "↰", TURN_LARGE: "↱"}
TASK_NAMES = {0: "(未选)", 1: "任务一: 货物投放", 2: "任务二: 动态降落", 3: "任务三: 稳定性测试"}


# ─── CRC / HMAC / 编解码 ───────────────────────────────────────────────────
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_packet(msg_type: int, sender_id: int, boot_id: int, seq: int,
                  source_ms: int, payload: bytes, key: bytes) -> bytes:
    header = HEADER_STRUCT.pack(MAGIC, PROTOCOL_VERSION, msg_type, len(payload),
                                sender_id, boot_id, seq, source_ms)
    packet = header + payload
    crc = crc16_ccitt(packet)
    packet += struct.pack("<H", crc)
    mac = hmac_mod.new(key, packet, hashlib.sha256).digest()[:HMAC_SIZE]
    packet += mac
    return packet


def decode_packet(raw: bytes, key: bytes):
    if len(raw) < HEADER_SIZE + CRC_SIZE + HMAC_SIZE:
        return None
    if len(raw) > MAX_PACKET:
        return None
    hp = raw[:len(raw) - CRC_SIZE - HMAC_SIZE]
    crc_bytes = raw[len(raw) - CRC_SIZE - HMAC_SIZE:len(raw) - HMAC_SIZE]
    mac_bytes = raw[len(raw) - HMAC_SIZE:]
    expected_mac = hmac_mod.new(key, raw[:len(raw) - HMAC_SIZE],
                                hashlib.sha256).digest()[:HMAC_SIZE]
    if not hmac_mod.compare_digest(mac_bytes, expected_mac):
        return None
    if struct.unpack("<H", crc_bytes)[0] != crc16_ccitt(hp):
        return None
    magic, ver, msg_type, plen, sender, boot, seq, src_ms = \
        HEADER_STRUCT.unpack(hp[:HEADER_SIZE])
    if magic != MAGIC or ver != PROTOCOL_VERSION:
        return None
    if plen > MAX_PAYLOAD or len(hp) != HEADER_SIZE + plen:
        return None
    return (msg_type, sender, boot, seq, src_ms, hp[HEADER_SIZE:])


# ─── 载荷编解码 ─────────────────────────────────────────────────────────────
def pack_car_telemetry(state, turn, event, event_id, quality, disp_mm,
                       vel_mm, line_err, faults):
    return struct.pack("<BBBHHihhH", state, turn, event, event_id, quality,
                       disp_mm, vel_mm, line_err, faults)


def unpack_task_selection(payload):
    if len(payload) != 9:
        return None
    sel_id, car_boot, task = struct.unpack("<IIB", payload)
    return sel_id, car_boot, task


def pack_mission_status(sel_id, car_boot, hmi_boot, phase, task, reason, flags):
    return struct.pack("<IIIBBHH", sel_id, car_boot, hmi_boot, phase, task,
                       reason, flags)


# ─── 绑定端口的 UDP 套接字 ──────────────────────────────────────────────────
def make_bound_sock(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass
    sock.bind((ip, port))
    sock.settimeout(0.1)
    return sock


def make_virtual_car_sock(ip: str, port: int):
    """创建伪装源 IP 的虚拟 CAR socket。

    HMI 按源 IP 过滤 CAR 遥测（必须来自 192.168.20.2:42001）。
    IP_TRANSPARENT 允许 UDP socket bind 非本机 IP，发往 HMI 的包源 IP 即为
    该地址。需要 root 且内核支持 TPROXY；失败返回 None（降级为普通发送）。
    """
    transparent = getattr(socket, "IP_TRANSPARENT", None)
    if transparent is None:
        return None
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.IPPROTO_IP, transparent, 1)
    except OSError:
        s.close()
        return None
    try:
        s.bind((ip, port))
    except OSError:
        s.close()
        return None
    return s


# ─── 比赛场景定义 ───────────────────────────────────────────────────────────
class CompetitionScenario:
    """定义一场比赛的完整时间线"""

    def __init__(self, task: int = 0, auto_arm_delay: float = 5.0):
        self.task = task            # 0=等待地面站选题, 1/2/3=自动选
        self.auto_arm_delay = auto_arm_delay
        self.car_boot = secrets.randbits(32) or 1
        self.ros_boot = secrets.randbits(32) or 1
        self.hmi_boot = 0           # 从 HMI 的 TASK_SELECTION 包中获取
        self.real_car_boot = 0      # 从真实 CAR 遥测中学习
        self.virtual_car = False    # 虚拟 CAR 模式（无真实小车时伪装源 IP）

        # 状态
        self.phase = PHASE_PRESTART
        self.selection_id = 0
        self.committed_task = 0
        self.selection_pending = False

        # 模拟小车状态
        self.car_state = CAR_READY
        self.car_turn = TURN_STRAIGHT
        self.car_event = EVT_NONE
        self.car_event_id = 0
        self.car_quality = Q_LINE_VALID | Q_ENCODER_VALID | Q_WIFI_CONNECTED
        self.car_disp_mm = 0
        self.car_vel_mm = 0
        self.car_line_err = 0
        self.car_faults = 0

        # 时间线
        self.start_time = 0.0
        self.select_time = 0.0
        self.arm_time = 0.0
        self.run_time = 0.0
        self.complete_time = 0.0

        # 统计
        self.car_tx = 0
        self.ros_tx = 0
        self.hmi_rx = 0

    def reset(self, elapsed: float):
        """重置任务状态回到 PRESTART，保留 boot ID 和网络状态"""
        self.phase = PHASE_PRESTART
        self.selection_id = 0
        self.committed_task = 0
        self.selection_pending = False
        # 重置模拟小车状态
        self.car_state = CAR_READY
        self.car_turn = TURN_STRAIGHT
        self.car_event = EVT_NONE
        self.car_event_id = 0
        self.car_quality = Q_LINE_VALID | Q_ENCODER_VALID | Q_WIFI_CONNECTED
        self.car_disp_mm = 0
        self.car_vel_mm = 0
        self.car_line_err = 0
        self.car_faults = 0
        # 重置时间线
        self.start_time = elapsed
        self.select_time = 0.0
        self.arm_time = 0.0
        self.run_time = 0.0
        self.complete_time = 0.0
        # 生成新的 boot ID（地面站检测到新 boot 会自动解锁按钮、重置状态机）
        self.car_boot = secrets.randbits(32) or 1

    def update(self, elapsed: float):
        """按时间推进比赛状态"""

        # ── 阶段 0: PRESTART (0 ~ select_time) ──
        if self.phase == PHASE_PRESTART:
            self.car_state = CAR_READY
            self.car_vel_mm = 0
            self.car_quality = Q_LINE_VALID | Q_ENCODER_VALID | Q_WIFI_CONNECTED
            # 如果指定了任务且已收到 HMI boot，自动推进（延迟相对本轮起始）
            if self.task > 0 and self.hmi_boot > 0 and elapsed - self.start_time > 2.0:
                self.selection_id = 1
                self.committed_task = self.task
                self.selection_pending = True
                self.select_time = elapsed
                self.phase = PHASE_SELECT_ACK
                self.car_quality |= Q_SELECTION_COMMITTED

        # ── 阶段 1: SELECT_ACK → ARMED (短暂过渡) ──
        elif self.phase == PHASE_SELECT_ACK:
            if elapsed - self.select_time > 1.0:
                self.phase = PHASE_ARMED
                self.arm_time = elapsed

        # ── 阶段 2: ARMED → RUNNING ──
        elif self.phase == PHASE_ARMED:
            if elapsed - self.arm_time > self.auto_arm_delay:
                self.phase = PHASE_RUNNING
                self.run_time = elapsed
                self.car_state = CAR_RUNNING
                self.car_event = EVT_START
                self.car_event_id = 1

        # ── 阶段 3: RUNNING ──
        elif self.phase == PHASE_RUNNING:
            t = elapsed - self.run_time
            self.car_disp_mm = int(t * 300)

            if t < 3.0:
                # 起步直行
                self.car_event = EVT_START if t < 1.0 else EVT_NONE
                self.car_vel_mm = int(200 + t * 50)
                self.car_turn = TURN_STRAIGHT
            elif t < 8.0:
                # B 弯
                self.car_event = EVT_B
                self.car_event_id = 2
                self.car_vel_mm = 250
                self.car_turn = TURN_LARGE
            elif t < 12.0:
                # 直行
                self.car_event = EVT_NONE
                self.car_vel_mm = 350
                self.car_turn = TURN_SMALL
            elif t < 17.0:
                # D 弯
                self.car_event = EVT_D
                self.car_event_id = 3
                self.car_vel_mm = 280
                self.car_turn = TURN_LARGE
            elif t < 22.0:
                # 直行
                self.car_event = EVT_NONE
                self.car_vel_mm = 400
                self.car_turn = TURN_STRAIGHT
            elif t < 27.0:
                # A 弯
                self.car_event = EVT_A
                self.car_event_id = 4
                self.car_vel_mm = 220
                self.car_turn = TURN_SMALL
            elif t < 32.0:
                # 冲刺直行
                self.car_event = EVT_NONE
                self.car_vel_mm = 450
                self.car_turn = TURN_STRAIGHT
            else:
                # 完成
                self.car_state = CAR_COMPLETE
                self.car_event = EVT_COMPLETE
                self.car_event_id = 5
                self.car_vel_mm = 0
                self.phase = PHASE_COMPLETE
                self.complete_time = elapsed

        # ── 阶段 4: COMPLETE → 停留 5 秒后自动回到 PRESTART ──
        elif self.phase == PHASE_COMPLETE:
            if elapsed - self.complete_time > 5.0:
                self.reset(elapsed)
                sys.stderr.write(
                    f"\n  {G}[自动重置]{N} 任务完成，回到 PRESTART 等待下一次选题\n"
                )

    def handle_selection(self, sel_id, car_boot, task, hmi_boot, elapsed: float = 0.0):
        """处理来自 HMI 的 TASK_SELECTION"""
        if self.phase != PHASE_PRESTART:
            return False
        if task not in (1, 2, 3):
            return False
        self.hmi_boot = hmi_boot
        self.selection_id = sel_id if sel_id > 0 else 1
        self.committed_task = task
        self.selection_pending = True
        self.select_time = elapsed
        self.phase = PHASE_SELECT_ACK
        self.car_quality |= Q_SELECTION_COMMITTED
        return True

    def mission_status_payload(self):
        """生成 MISSION_STATUS 载荷"""
        flags = FLAG_ROS_READY | FLAG_DRONE_LINK_OK
        if self.phase in (PHASE_ARMED, PHASE_RUNNING, PHASE_COMPLETE):
            flags |= FLAG_DRONE_ARMED
        return pack_mission_status(
            self.selection_id, self.car_boot, self.hmi_boot,
            self.phase, self.committed_task, 0, flags
        )


# ─── 显示 ───────────────────────────────────────────────────────────────────
def print_header(scenario: CompetitionScenario):
    sys.stderr.write("\033[2J\033[H")
    sys.stderr.write(
        f"\033[1;36m"
        f"╔═══════════════════════════════════════════════════════════════════════════════════╗\n"
        f"║  ED UAV 比赛模拟器  |  CAR boot=0x{scenario.car_boot:08X}  ROS boot=0x{scenario.ros_boot:08X}             ║\n"
        f"║  ROS→{NUC_IP}:{NUC_PORT}  CAR→{CAR_IP}:{CAR_PORT}  HMI→{HMI_IP}:{HMI_PORT}                      ║\n"
        f"║  Ctrl+C 退出                                                                      ║\n"
        f"╚═══════════════════════════════════════════════════════════════════════════════════╝"
        f"\033[0m\n\n"
    )
    sys.stderr.flush()


def print_status(scenario: CompetitionScenario, elapsed: float):
    s = scenario
    phase_color = {
        PHASE_PRESTART: Y, PHASE_SELECT_ACK: C, PHASE_ARMED: C,
        PHASE_RUNNING: G, PHASE_COMPLETE: G, PHASE_FAULT: R,
    }.get(s.phase, N)
    phase_name = PHASE_NAMES.get(s.phase, f"?{s.phase}")
    state_name = CAR_STATE_NAMES.get(s.car_state, f"?{s.car_state}")
    state_color = G if s.car_state == CAR_RUNNING else Y if s.car_state == CAR_READY else C
    turn_name = TURN_NAMES.get(s.car_turn, f"?{s.car_turn}")
    event_name = EVT_NAMES.get(s.car_event, f"?{s.car_event}")

    hmi_boot_str = f"0x{s.hmi_boot:08X}" if s.hmi_boot else "--------"
    car_boot_str = f"0x{s.car_boot:08X}" if s.car_boot else "--------"
    sel_str = f"ID={s.selection_id} task={s.committed_task}" if s.committed_task else "(等待地面站选题)"

    lines = [
        f"\033[1;33m──── 比赛阶段 {'─' * 60}\033[0m",
        f"  ROS 阶段: {phase_color}{B}{phase_name:<12}{N}  选择: {sel_str}",
        f"  CAR 状态: {state_color}{state_name:<10}{N}  事件: {event_name}  转向: {turn_name}",
        f"  CAR 数据: 位移={s.car_disp_mm/1000:.2f}m  速度={s.car_vel_mm/1000:.2f}m/s  线偏差={s.car_line_err}  故障=0x{s.car_faults:04X}",
        f"  CAR boot: {car_boot_str}  HMI boot: {hmi_boot_str}  品质: 0x{s.car_quality:04X}",
        "",
        f"\033[1;33m──── 链路统计 {'─' * 60}\033[0m",
        f"  运行: {elapsed:.0f}s  CAR→HMI: {s.car_tx}  ROS→HMI: {s.ros_tx}  HMI→ROS: {s.hmi_rx}",
        "",
        f"\033[1;33m─── HMI 应显示 {'─' * 58}\033[0m",
    ]

    # 预测 HMI 应显示的状态
    if s.hmi_boot == 0:
        lines.append(f"  {Y}BOOT_WAITING{N}  (等待 HMI 心跳获取 hmi_boot)")
    elif s.real_car_boot == 0 and not s.virtual_car:
        lines.append(f"  {Y}BOOT_WAITING{N}  (等待真实 CAR 遥测获取 car_boot)")
    elif s.real_car_boot == 0 and s.virtual_car:
        lines.append(f"  {C}虚拟 CAR{N}  boot=0x{s.car_boot:08X}  模拟遥测+回执已启用")

    if s.phase == PHASE_PRESTART:
        lines.append(f"  {G}PRESTART{N}  可选题: task 1 / task 2 / test")
    elif s.phase == PHASE_SELECT_ACK:
        lines.append(f"  {C}SELECTED{N}  已选: {TASK_NAMES.get(s.committed_task, '?')}")
    elif s.phase == PHASE_ARMED:
        lines.append(f"  {C}ARMED_READY{N}  等待小车启动...")
    elif s.phase == PHASE_RUNNING:
        lines.append(f"  {G}CAR_RUNNING{N}  {TASK_NAMES.get(s.committed_task, '?')}")
        lines.append(f"  遥测: state={state_name} event={event_name} vel={s.car_vel_mm/1000:.2f}m/s")
    elif s.phase == PHASE_COMPLETE:
        remaining = max(0, 5.0 - (elapsed - s.complete_time))
        lines.append(f"  {G}{B}COMPLETE{N}  {TASK_NAMES.get(s.committed_task, '?')} ✓  "
                     f"({remaining:.0f}s 后自动重置)")

    lines.append("")
    sys.stderr.write(f"\033[5;0H\033[J")
    sys.stderr.write("\n".join(lines) + "\n")
    sys.stderr.flush()


# ─── 主循环 ─────────────────────────────────────────────────────────────────
def run_competition(key: bytes, task: int, duration: float, virtual_car: bool = True):
    scenario = CompetitionScenario(task=task)
    ros_boot = scenario.ros_boot

    # 绑定三个端口：ROS(42000), CAR(42001), 接收 HMI 选题(42000)
    # ROS 和接收共用 42000（同一个 socket）
    ros_sock = make_bound_sock("0.0.0.0", NUC_PORT)  # 42000: 发 ROS 包 + 接收 HMI 选题
    car_sock = make_bound_sock("0.0.0.0", CAR_PORT)   # 42001: 发 CAR 遥测

    # 虚拟 CAR：伪装源 IP=192.168.20.2:42001，让 HMI 的源 IP 过滤通过
    virtual_car_sock = None
    if virtual_car:
        virtual_car_sock = make_virtual_car_sock(CAR_IP, CAR_PORT)
        if virtual_car_sock is None:
            print("警告: IP_TRANSPARENT 不可用，虚拟 CAR 未启用（HMI 将收不到模拟 CAR 遥测）",
                  file=sys.stderr)
        else:
            scenario.virtual_car = True
            print(f"虚拟 CAR 已启用: 源 {CAR_IP}:{CAR_PORT} → HMI:{HMI_IP}:{HMI_PORT} "
                  f"(真实小车在线时自动让位)", file=sys.stderr)

    print_header(scenario)

    running = True
    def handle_signal(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    start_time = time.monotonic()
    car_seq = 0
    ros_seq = 0
    last_telem_time = 0.0
    last_hb_time = 0.0
    last_status_time = 0.0
    last_display_time = 0.0

    try:
        while running:
            now = time.monotonic()
            elapsed = now - start_time

            if 0 < duration <= elapsed:
                break

            # ── 接收所有入站包 ──
            for _ in range(16):
                try:
                    data, addr = ros_sock.recvfrom(MAX_PACKET)
                except socket.timeout:
                    break
                except OSError:
                    break
                hdr = decode_packet(data, key)
                if hdr is None:
                    continue
                msg_type, sender, boot_id, seq, src_ms, payload = hdr

                if msg_type == MSG_TASK_SELECTION and sender == SENDER_HMI:
                    sel = unpack_task_selection(payload)
                    if sel:
                        sel_id, car_boot_from_hmi, hmi_task = sel
                        scenario.hmi_boot = boot_id
                        if scenario.handle_selection(sel_id, car_boot_from_hmi,
                                                     hmi_task, boot_id, elapsed):
                            scenario.hmi_rx += 1
                            sys.stderr.write(
                                f"\n  {G}[HMI 选题]{N} task={hmi_task} "
                                f"sel_id={sel_id} hmi_boot=0x{boot_id:08X}\n"
                            )
                elif msg_type == MSG_HEARTBEAT and sender == SENDER_HMI:
                    scenario.hmi_boot = boot_id
                    scenario.hmi_rx += 1
                elif msg_type == MSG_CAR_TELEMETRY and sender == SENDER_CAR:
                    # 只把来自真实小车(192.168.20.2:42001)的遥测视为"真实 CAR 上线"。
                    # 注意：模拟器自己也把模拟 CAR 遥测发往本机 42000（走回环，
                    # 源 IP 是 192.168.20.1），若不按源地址过滤会把虚拟 CAR 误判为
                    # 真实 CAR，导致虚拟遥测块被跳过、HMI 永远学不到 car_boot。
                    if addr[0] == CAR_IP and addr[1] == CAR_PORT:
                        if scenario.real_car_boot != boot_id:
                            scenario.real_car_boot = boot_id
                            scenario.car_boot = boot_id
                            sys.stderr.write(
                                f"\n  {C}[CAR 上线]{N} boot=0x{boot_id:08X}\n"
                            )

            # ── 推进比赛状态 ──
            scenario.update(elapsed)
            # 真实 CAR boot 已知时，让场景也使用它
            if scenario.real_car_boot > 0:
                scenario.car_boot = scenario.real_car_boot

            # ── 20Hz CAR 遥测 → HMI (port 42002) + 诊断 (port 42000) ──
            # HMI 按源 IP 过滤 CAR 包（必须来自 192.168.20.2:42001）。
            # 虚拟 CAR 模式用 IP_TRANSPARENT 伪装源 IP；真实 CAR 在线时跳过。
            if scenario.real_car_boot == 0 and now - last_telem_time >= 0.05:
                now_ms = int(now * 1000) & 0xFFFFFFFF
                payload = pack_car_telemetry(
                    scenario.car_state, scenario.car_turn,
                    scenario.car_event, scenario.car_event_id,
                    scenario.car_quality, scenario.car_disp_mm,
                    scenario.car_vel_mm, scenario.car_line_err,
                    scenario.car_faults,
                )
                pkt = encode_packet(MSG_CAR_TELEMETRY, SENDER_CAR, scenario.car_boot,
                                    car_seq, now_ms, payload, key)
                try:
                    if virtual_car_sock is not None:
                        virtual_car_sock.sendto(pkt, (HMI_IP, HMI_PORT))
                    else:
                        car_sock.sendto(pkt, (HMI_IP, HMI_PORT))
                    ros_sock.sendto(pkt, (NUC_IP, NUC_PORT))
                except OSError:
                    pass
                car_seq = (car_seq + 1) & 0xFFFFFFFF
                scenario.car_tx += 1
                last_telem_time = now

            # ── 4Hz ROS 心跳 → CAR + HMI（250ms 周期，匹配 HMI 期望）──
            if now - last_hb_time >= 0.25:
                now_ms = int(now * 1000) & 0xFFFFFFFF
                hb = encode_packet(MSG_HEARTBEAT, SENDER_ROS, ros_boot,
                                   ros_seq, now_ms, b"", key)
                try:
                    ros_sock.sendto(hb, (CAR_IP, CAR_PORT))
                    ros_sock.sendto(hb, (HMI_IP, HMI_PORT))
                except OSError:
                    pass
                ros_seq = (ros_seq + 1) & 0xFFFFFFFF
                scenario.ros_tx += 1
                last_hb_time = now

            # ── 2Hz MISSION_STATUS → CAR + HMI ──
            # 必须同时知道 hmi_boot 和可用的 car_boot（真实或虚拟），
            # 否则 HMI 会因 boot_id 不匹配拒绝回执。
            car_known = scenario.real_car_boot > 0 or scenario.virtual_car
            if now - last_status_time >= 0.5 and scenario.hmi_boot > 0 and car_known:
                now_ms = int(now * 1000) & 0xFFFFFFFF
                status_payload = scenario.mission_status_payload()
                pkt = encode_packet(MSG_MISSION_STATUS, SENDER_ROS, ros_boot,
                                    ros_seq, now_ms, status_payload, key)
                try:
                    ros_sock.sendto(pkt, (CAR_IP, CAR_PORT))
                    ros_sock.sendto(pkt, (HMI_IP, HMI_PORT))
                    # 也发给诊断工具
                    ros_sock.sendto(pkt, (NUC_IP, NUC_PORT))
                except OSError:
                    pass
                ros_seq = (ros_seq + 1) & 0xFFFFFFFF
                scenario.ros_tx += 1
                last_status_time = now

            # ── 1Hz 显示 ──
            if now - last_display_time >= 1.0:
                print_status(scenario, elapsed)
                last_display_time = now

            time.sleep(0.005)

    finally:
        elapsed = time.monotonic() - start_time
        ros_sock.close()
        car_sock.close()

        phase_name = PHASE_NAMES.get(scenario.phase, "?")
        sys.stderr.write(f"\n\n\033[1;33m═══ 模拟结束 ═══\033[0m\n")
        sys.stderr.write(f"  运行: {elapsed:.1f}s  最终阶段: {phase_name}\n")
        sys.stderr.write(f"  CAR 发包: {scenario.car_tx}  ROS 发包: {scenario.ros_tx}\n")
        sys.stderr.write(f"  HMI 收包: {scenario.hmi_rx}\n")
        sys.stderr.write(f"\n")


# ─── 入口 ───────────────────────────────────────────────────────────────────
def load_key(args) -> bytes:
    if args.example_key:
        return bytes(range(32))
    key_file = args.key_file
    if not os.path.isabs(key_file):
        key_file = os.path.join(os.path.dirname(__file__), "..", key_file)
    key_file = os.path.normpath(key_file)
    if os.path.exists(key_file):
        with open(key_file) as f:
            key = bytes.fromhex(f.read().strip())
        if len(key) >= 32:
            return key
    print(f"错误: 密钥文件不存在或无效: {key_file}", file=sys.stderr)
    sys.exit(1)


def main():
    default_key = os.path.join(os.path.dirname(__file__), "..", "config", "hmac.key.hex")
    p = argparse.ArgumentParser(description="ED UAV 全流程比赛模拟器")
    p.add_argument("--key-file", default=os.path.normpath(default_key))
    p.add_argument("--example-key", action="store_true",
                   help="使用 example 密钥 (000102...1E1F)")
    p.add_argument("--task", type=int, default=0, choices=[0, 1, 2, 3],
                   help="指定任务 (0=等地面站选, 1=投放, 2=降落, 3=稳定测试)")
    p.add_argument("--virtual-car", action=argparse.BooleanOptionalAction, default=True,
                   help="无真实小车时模拟虚拟 CAR（IP_TRANSPARENT 伪装源 IP，需 root）")
    p.add_argument("--duration", type=float, default=0,
                   help="运行时长秒 (0=无限)")
    args = p.parse_args()

    key = load_key(args)
    print(f"密钥: {key.hex()[:16]}...  任务: {args.task or '等HMI选'}", file=sys.stderr)
    run_competition(key, args.task, args.duration, virtual_car=args.virtual_car)


if __name__ == "__main__":
    main()
