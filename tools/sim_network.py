#!/usr/bin/env python3
"""
ED UAV 三端通信模拟器

无需 ESP32 硬件，模拟 CAR + ROS 向 HMI 和 NUC 诊断工具发送协议数据包，
用于验证：
  - 诊断工具能否正确解码（密钥是否匹配）
  - 地面站 HMI 能否正常显示遥测和任务状态
  - 全链路协议编解码一致性

用法：
  # 模拟 CAR + ROS 同时发包（默认 20Hz 遥测 + 1Hz 心跳/状态）
  sudo python3 tools/sim_network.py

  # 仅模拟 CAR（遥测 + 心跳）
  sudo python3 tools/sim_network.py --mode car

  # 仅模拟 ROS（心跳 + 任务状态）
  sudo python3 tools/sim_network.py --mode ros

  # 指定密钥文件
  sudo python3 tools/sim_network.py --key-file config/hmac.key.hex

  # 自定义运行时长（秒）
  sudo python3 tools/sim_network.py --duration 60

  # 同时启动诊断工具（双终端模式）
  sudo python3 tools/sim_network.py --with-diag
"""

import argparse
import os
import secrets
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

# 把 tools/diagnostics 加入 path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "diagnostics"))

from vehicle_comm_diagnostic import (
    HEADER_STRUCT, HEADER_SIZE, CRC_SIZE, HMAC_SIZE, MAGIC, PROTOCOL_VERSION,
    MAX_PACKET, MAX_PAYLOAD,
    crc16_ccitt,
    MSG_HEARTBEAT, MSG_CAR_TELEMETRY, MSG_TASK_SELECTION, MSG_MISSION_STATUS,
    SENDER_CAR, SENDER_HMI, SENDER_ROS,
    CAR_IP, CAR_PORT, HMI_IP, HMI_PORT, NUC_IP, NUC_PORT,
    CAR_TELEMETRY_FMT, TASK_SELECTION_FMT, MISSION_STATUS_FMT,
)


# ─── 颜色 ───────────────────────────────────────────────────────────────────
R = '\033[0;31m'; G = '\033[0;32m'; Y = '\033[0;33m'; C = '\033[0;36m'
B = '\033[1m'; DIM = '\033[2m'; N = '\033[0m'


# ─── 协议编码（复用诊断工具的 encode_packet）────────────────────────────────
def encode_packet(msg_type: int, sender_id: int, boot_id: int, sequence: int,
                  source_millis: int, payload: bytes, key: bytes) -> bytes:
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
    import hmac, hashlib
    mac = hmac.new(key, packet, hashlib.sha256).digest()[:HMAC_SIZE]
    packet += mac
    return packet


# ─── 载荷编码 ───────────────────────────────────────────────────────────────
def encode_car_telemetry_payload(state: int, turn: int, event: int, event_id: int,
                                  quality: int, disp_mm: int, vel_mm: int,
                                  line_err: int, faults: int) -> bytes:
    """CAR_TELEMETRY 载荷 (17 bytes): BBBHHihhH"""
    return struct.pack("<BBBHHihhH", state, turn, event, event_id, quality,
                       disp_mm, vel_mm, line_err, faults)


def encode_mission_status_payload(selection_id: int, car_boot: int, hmi_boot: int,
                                   phase: int, task: int, reason: int, flags: int) -> bytes:
    """MISSION_STATUS 载荷 (18 bytes): IIIBBHH"""
    return struct.pack("<IIIBBHH", selection_id, car_boot, hmi_boot, phase, task,
                       reason, flags)


# ─── 模拟场景 ───────────────────────────────────────────────────────────────
CAR_STATES = {0: "READY", 1: "RUNNING", 2: "COMPLETE", 3: "SAFE_STOP"}
TURN_CLASSES = {0: "→", 1: "↰", 2: "↱"}
ROUTE_EVENTS = {0: "—", 1: "START", 2: "B", 3: "D", 4: "A", 5: "DONE"}
MISSION_PHASES = {0: "PRESTART", 1: "SELECT_ACK", 2: "ARMED", 3: "RUNNING", 4: "COMPLETE", 5: "FAULT"}


class SimulatedCar:
    """模拟小车：20Hz 遥测 + 1Hz 心跳"""

    def __init__(self, key: bytes, boot_id: int):
        self.key = key
        self.boot_id = boot_id
        self.seq = 0
        self.running = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 模拟状态
        self.state = 0  # READY
        self.turn = 0
        self.event = 0
        self.event_id = 0
        self.quality = 0x03  # LINE_VALID | ENCODER_VALID
        self.disp_mm = 0
        self.vel_mm = 0
        self.line_err = 0
        self.faults = 0
        self.start_time = 0.0

        # 统计
        self.tx_count = 0

    def start(self):
        self.running = True
        self.start_time = time.monotonic()

    def stop(self):
        self.running = False
        self.sock.close()

    def update_state(self, elapsed: float):
        """按时间推进模拟状态"""
        if elapsed < 2.0:
            self.state = 0  # READY
            self.vel_mm = 0
        elif elapsed < 5.0:
            self.state = 1  # RUNNING
            self.event = 1  # START
            self.event_id = 1
            self.vel_mm = 200
        elif elapsed < 15.0:
            self.state = 1
            self.event = 0  # NONE
            self.event_id = 0
            self.vel_mm = 300 + int(elapsed * 10)
            self.turn = 1 if int(elapsed) % 5 < 2 else 0
        elif elapsed < 25.0:
            self.state = 1
            self.event = 2  # B
            self.event_id = 2
            self.vel_mm = 250
            self.turn = 2
        elif elapsed < 35.0:
            self.state = 1
            self.event = 0
            self.vel_mm = 400
            self.turn = 0
        elif elapsed < 40.0:
            self.state = 2  # COMPLETE
            self.event = 5  # COMPLETE
            self.vel_mm = 0
        else:
            self.state = 0  # READY
            self.vel_mm = 0

        self.disp_mm = int(elapsed * 200)  # 模拟位移

    def send_telemetry(self):
        """发送一帧遥测"""
        now_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        payload = encode_car_telemetry_payload(
            self.state, self.turn, self.event, self.event_id,
            self.quality, self.disp_mm, self.vel_mm, self.line_err, self.faults
        )
        packet = encode_packet(MSG_CAR_TELEMETRY, SENDER_CAR, self.boot_id,
                               self.seq, now_ms, payload, self.key)
        # 发给 NUC (诊断工具) 和 HMI (地面站)
        try:
            self.sock.sendto(packet, (NUC_IP, NUC_PORT))
            self.sock.sendto(packet, (HMI_IP, HMI_PORT))
        except OSError:
            pass
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self.tx_count += 1

    def send_heartbeat(self):
        """发送心跳"""
        now_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        packet = encode_packet(MSG_HEARTBEAT, SENDER_CAR, self.boot_id,
                               self.seq, now_ms, b"", self.key)
        try:
            self.sock.sendto(packet, (NUC_IP, NUC_PORT))
            self.sock.sendto(packet, (HMI_IP, HMI_PORT))
        except OSError:
            pass
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self.tx_count += 1


class SimulatedROS:
    """模拟 ROS 节点：1Hz 心跳 + 任务状态"""

    def __init__(self, key: bytes, boot_id: int, car_boot_id: int):
        self.key = key
        self.boot_id = boot_id
        self.car_boot_id = car_boot_id
        self.seq = 0
        self.running = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 任务状态
        self.selection_id = 0
        self.hmi_boot_id = 0xBBBBBBBB
        self.phase = 0  # PRESTART
        self.selected_task = 0
        self.reason_flags = 0
        self.status_flags = 0x0F  # 全部正常

        self.tx_count = 0
        self.start_time = 0.0

    def start(self):
        self.running = True
        self.start_time = time.monotonic()

    def stop(self):
        self.running = False
        self.sock.close()

    def update_state(self, elapsed: float):
        """按时间推进任务状态"""
        if elapsed < 3.0:
            self.phase = 0  # PRESTART
            self.selected_task = 0
        elif elapsed < 6.0:
            self.phase = 1  # SELECT_ACK
            self.selection_id = 1
            self.selected_task = 1
        elif elapsed < 10.0:
            self.phase = 2  # ARMED
        elif elapsed < 40.0:
            self.phase = 3  # RUNNING
        elif elapsed < 45.0:
            self.phase = 4  # COMPLETE
        else:
            self.phase = 0

    def send_heartbeat(self):
        """发送心跳到 CAR 和 HMI"""
        now_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        packet = encode_packet(MSG_HEARTBEAT, SENDER_ROS, self.boot_id,
                               self.seq, now_ms, b"", self.key)
        try:
            self.sock.sendto(packet, (CAR_IP, CAR_PORT))
            self.sock.sendto(packet, (HMI_IP, HMI_PORT))
        except OSError:
            pass
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self.tx_count += 1

    def send_mission_status(self):
        """发送任务状态到 CAR 和 HMI"""
        now_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        payload = encode_mission_status_payload(
            self.selection_id, self.car_boot_id, self.hmi_boot_id,
            self.phase, self.selected_task, self.reason_flags, self.status_flags
        )
        packet = encode_packet(MSG_MISSION_STATUS, SENDER_ROS, self.boot_id,
                               self.seq, now_ms, payload, self.key)
        try:
            self.sock.sendto(packet, (CAR_IP, CAR_PORT))
            self.sock.sendto(packet, (HMI_IP, HMI_PORT))
        except OSError:
            pass
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self.tx_count += 1


# ─── 显示 ───────────────────────────────────────────────────────────────────
def print_header(car_boot: int, ros_boot: int):
    sys.stderr.write("\033[2J\033[H")
    sys.stderr.write(
        f"\033[1;36m"
        f"╔═══════════════════════════════════════════════════════════════════════════════╗\n"
        f"║  ED UAV 网络模拟器  |  CAR boot=0x{car_boot:08X}  ROS boot=0x{ros_boot:08X}            ║\n"
        f"║  CAR→{NUC_IP}:{NUC_PORT}+{HMI_IP}:{HMI_PORT}   ROS→{CAR_IP}:{CAR_PORT}+{HMI_IP}:{HMI_PORT}  ║\n"
        f"║  Ctrl+C 退出                                                                  ║\n"
        f"╚═══════════════════════════════════════════════════════════════════════════════╝"
        f"\033[0m\n\n"
    )
    sys.stderr.flush()


def print_status(car: SimulatedCar, ros: SimulatedROS, elapsed: float):
    state_name = CAR_STATES.get(car.state, f"?{car.state}")
    turn_name = TURN_CLASSES.get(car.turn, f"?{car.turn}")
    event_name = ROUTE_EVENTS.get(car.event, f"?{car.event}")
    phase_name = MISSION_PHASES.get(ros.phase, f"?{ros.phase}")

    # 颜色
    state_color = G if car.state == 1 else Y if car.state == 0 else C
    phase_color = G if ros.phase == 3 else Y if ros.phase == 0 else C

    lines = [
        f"\033[1;33m──── 模拟小车 (CAR) {'─' * 55}\033[0m",
        f"  状态: {state_color}{state_name:<10}{N}  转向: {turn_name}  事件: {event_name}  事件ID: {car.event_id}",
        f"  位移: {car.disp_mm/1000:.2f}m     速度: {car.vel_mm/1000:.2f}m/s   线偏差: {car.line_err}  故障: 0x{car.faults:04X}",
        f"  已发包: {car.tx_count:<8}  遥测 20Hz + 心跳 1Hz",
        "",
        f"\033[1;33m──── 模拟 ROS 节点 {'─' * 55}\033[0m",
        f"  阶段: {phase_color}{phase_name:<12}{N}  选择ID: {ros.selection_id}  任务: {ros.selected_task}",
        f"  状态标志: 0x{ros.status_flags:04X}  原因标志: 0x{ros.reason_flags:04X}",
        f"  已发包: {ros.tx_count:<8}  心跳 1Hz + 状态 1Hz",
        "",
        f"\033[1;33m──── 统计 {'─' * 63}\033[0m",
        f"  运行时间: {elapsed:.0f}s    CAR 发包: {car.tx_count}    ROS 发包: {ros.tx_count}",
        "",
        f"\033[1;33m──── 预期效果 {'─' * 61}\033[0m",
        f"  诊断工具: CAR={G}ONLINE{N}  HMI={G}ONLINE{N}  (密钥匹配时)",
        f"  地面站:   CAR 遥测实时显示  |  任务阶段: {phase_name}",
    ]

    sys.stderr.write(f"\033[5;0H\033[J")
    sys.stderr.write("\n".join(lines) + "\n")
    sys.stderr.flush()


# ─── 主循环 ─────────────────────────────────────────────────────────────────
def run_simulation(key: bytes, mode: str, duration: float):
    car_boot = secrets.randbits(32) or 1
    ros_boot = secrets.randbits(32) or 1

    car = SimulatedCar(key, car_boot) if mode in ("car", "both") else None
    ros = SimulatedROS(key, ros_boot, car_boot) if mode in ("ros", "both") else None

    print_header(car_boot, ros_boot)

    if car:
        car.start()
    if ros:
        ros.start()

    running = True

    def handle_signal(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    last_telem_time = 0.0
    last_hb_time = 0.0
    last_status_time = 0.0
    last_display_time = 0.0
    start_time = time.monotonic()

    try:
        while running:
            now = time.monotonic()
            elapsed = now - start_time

            if duration > 0 and elapsed >= duration:
                break

            # 更新模拟状态
            if car:
                car.update_state(elapsed)
            if ros:
                ros.update_state(elapsed)

            # 20Hz 遥测 (50ms 间隔)
            if car and now - last_telem_time >= 0.05:
                car.send_telemetry()
                last_telem_time = now

            # 4Hz ROS 心跳 → CAR + HMI（250ms 周期，匹配 HMI HEARTBEAT_PERIOD_MS）
            if now - last_hb_time >= 0.25:
                if car:
                    car.send_heartbeat()
                if ros:
                    ros.send_heartbeat()
                last_hb_time = now

            # 2Hz 任务状态
            if ros and now - last_status_time >= 0.5:
                ros.send_mission_status()
                last_status_time = now

            # 1Hz 显示刷新
            if now - last_display_time >= 1.0:
                print_status(car, ros, elapsed)
                last_display_time = now

            # 避免忙等
            time.sleep(0.005)

    finally:
        elapsed = time.monotonic() - start_time
        if car:
            car.stop()
        if ros:
            ros.stop()

        sys.stderr.write(f"\n\n\033[1;33m═══ 模拟结束 ═══\033[0m\n")
        sys.stderr.write(f"  运行时间: {elapsed:.1f}s\n")
        if car:
            sys.stderr.write(f"  CAR 发包: {car.tx_count}\n")
        if ros:
            sys.stderr.write(f"  ROS 发包: {ros.tx_count}\n")
        sys.stderr.write(f"\n")
        sys.stderr.flush()


# ─── 入口 ───────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description="ED UAV 三端通信模拟器",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  # 模拟全部（CAR + ROS）
  sudo %(prog)s

  # 仅模拟 CAR
  sudo %(prog)s --mode car

  # 运行 30 秒
  sudo %(prog)s --duration 30

  # 使用 example 密钥（与 config_local.example.h 一致）
  sudo %(prog)s --key-file /dev/null  # 不推荐，仅测试
"""
    )
    default_key_file = os.path.join(os.path.dirname(__file__), "..", "config", "hmac.key.hex")
    default_key_file = os.path.normpath(default_key_file)
    p.add_argument("--key-file", default=default_key_file,
                   help="HMAC 密钥文件 (默认: config/hmac.key.hex)")
    p.add_argument("--mode", choices=["car", "ros", "both"], default="both",
                   help="模拟模式: car=仅小车, ros=仅ROS, both=全部 (默认: both)")
    p.add_argument("--duration", type=float, default=0,
                   help="运行时长（秒），0=无限 (默认: 0)")
    p.add_argument("--with-diag", action="store_true",
                   help="同时启动诊断工具（需在另一个终端手动启动）")
    return p.parse_args()


def load_key(args) -> bytes:
    if args.key_file and os.path.exists(args.key_file):
        with open(args.key_file) as f:
            key = bytes.fromhex(f.read().strip())
        if len(key) < 32:
            print(f"错误: 密钥不足 32 字节 ({len(key)})", file=sys.stderr)
            sys.exit(1)
        return key
    # fallback: example key
    print(f"警告: 密钥文件不存在 ({args.key_file})，使用 example 密钥", file=sys.stderr)
    return bytes(range(32))


def main():
    args = parse_args()
    key = load_key(args)

    print(f"密钥: {key.hex()[:16]}... ({len(key)} bytes)", file=sys.stderr)
    print(f"模式: {args.mode}  时长: {'无限' if args.duration <= 0 else f'{args.duration}s'}", file=sys.stderr)

    diag_proc = None
    if args.with_diag:
        diag_script = os.path.join(os.path.dirname(__file__), "diagnostics", "vehicle_comm_diagnostic.py")
        if os.path.exists(diag_script):
            print(f"\n启动诊断工具（后台进程）...", file=sys.stderr)
            diag_proc = subprocess.Popen(
                [sys.executable, diag_script, "--key-file", args.key_file],
                stdout=sys.DEVNULL, stderr=sys.DEVNULL
            )
            print(f"  诊断工具 PID: {diag_proc.pid}", file=sys.stderr)
        else:
            print(f"警告: 诊断工具不存在: {diag_script}", file=sys.stderr)

    try:
        run_simulation(key, args.mode, args.duration)
    finally:
        if diag_proc:
            diag_proc.terminate()
            diag_proc.wait(timeout=3)
            print(f"  诊断工具已停止", file=sys.stderr)


if __name__ == "__main__":
    main()
