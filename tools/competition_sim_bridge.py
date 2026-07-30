#!/usr/bin/env python3
"""
competition_sim_bridge.py — 比赛流程模拟桥接器

充当 ROS 节点，连接真实小车和地面站，执行完整比赛流程：
  PRESTART → SELECTION_ACKED → ARMED_READY → CAR_RUNNING → COMPLETE

用法:
    python3 tools/competition_sim_bridge.py [选项]

选项:
    --key-file FILE     HMAC 密钥文件 (默认: 示例密钥)
    --auto-task N       自动选择任务 1 或 2 (跳过地面站手动选择)
    --drone-cmd CMD     无人机任务命令 (可选, 如: python3 -m drone.main --dry-run)
    --no-drone          不启动无人机任务
    --log-file FILE     日志文件路径
    --duration SEC      最大运行时间 (默认: 300秒)
    --verbose           详细输出
"""

import argparse
import logging
import os
import secrets
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

# 添加 tools 目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dtask_lib import (
    DTaskCodec, MessageType, CarState, TurnClass, RouteEvent,
    MissionPhase, QualityFlag, MissionStatusFlag, FaultFlag,
    CarTelemetry, TaskSelection, MissionStatus,
    PacketHeader, decode_car_telemetry, decode_task_selection,
    NUC_IP, NUC_PORT, CAR_IP, CAR_PORT, HMI_IP, HMI_PORT,
    SENDER_CAR, SENDER_HMI, STALE_MS,
)


# ── 颜色输出 ─────────────────────────────────────────────

class C:
    R = '\033[0;31m'; G = '\033[0;32m'; Y = '\033[0;33m'
    C = '\033[0;36m'; B = '\033[1m'; N = '\033[0m'


def ok(msg):   print(f"{C.G}[OK]{C.N}  {msg}")
def warn(msg): print(f"{C.Y}[!!]{C.N}  {msg}")
def fail(msg): print(f"{C.R}[ERR]{C.N} {msg}", file=sys.stderr)
def info(msg): print(f"{C.C}[>>]{C.N}  {msg}")


# ── 端点状态 ─────────────────────────────────────────────

@dataclass
class EndpointState:
    label: str
    sender_id: int
    boot_id: int = 0
    last_seq: Optional[int] = None
    last_rx_time: float = 0.0
    rx_count: int = 0
    online: bool = False

    def update(self, hdr: PacketHeader):
        now = time.monotonic()
        if self.boot_id != hdr.boot_id:
            if self.boot_id != 0:
                warn(f"{self.label} 重启: 0x{self.boot_id:08X} → 0x{hdr.boot_id:08X}")
            self.boot_id = hdr.boot_id
            self.last_seq = None
        self.last_seq = hdr.sequence
        self.last_rx_time = now
        self.rx_count += 1
        self.online = True

    @property
    def is_stale(self) -> bool:
        if self.last_rx_time == 0:
            return True
        return (time.monotonic() - self.last_rx_time) * 1000 > STALE_MS

    @property
    def age_ms(self) -> float:
        if self.last_rx_time == 0:
            return float('inf')
        return (time.monotonic() - self.last_rx_time) * 1000


# ── 任务状态机 ────────────────────────────────────────────

@dataclass
class MissionState:
    phase: MissionPhase = MissionPhase.PRESTART
    selected_task: int = 0
    selection_id: int = 0
    car_state: CarState = CarState.READY
    car_event: RouteEvent = RouteEvent.NONE
    car_event_id: int = 0
    car_displacement_mm: int = 0
    car_velocity_mm_s: int = 0
    car_faults: int = 0
    hmi_selection_pending: bool = False
    hmi_pending_task: int = 0
    hmi_pending_sel_id: int = 0
    mission_start_time: float = 0.0
    car_running_start_time: float = 0.0
    complete_time: float = 0.0
    fault_reason: int = 0
    events_received: list = field(default_factory=list)

    @property
    def is_terminal(self) -> bool:
        return self.phase in (MissionPhase.COMPLETE, MissionPhase.FAULT)

    @property
    def mission_elapsed(self) -> float:
        if self.mission_start_time == 0:
            return 0.0
        return time.time() - self.mission_start_time

    @property
    def car_running_elapsed(self) -> float:
        if self.car_running_start_time == 0:
            return 0.0
        return time.time() - self.car_running_start_time


# ── 模拟桥接器 ────────────────────────────────────────────

class CompetitionSimBridge:
    """比赛流程模拟桥接器"""

    def __init__(self, key: bytes, auto_task: int = 0,
                 drone_cmd: Optional[str] = None,
                 duration: float = 300.0, verbose: bool = False):
        self.key = key
        self.auto_task = auto_task
        self.drone_cmd = drone_cmd
        self.duration = duration
        self.verbose = verbose

        self.codec = DTaskCodec(key, "0.0.0.0", NUC_PORT)
        self.car = EndpointState("CAR", SENDER_CAR)
        self.hmi = EndpointState("HMI", SENDER_HMI)
        self.mission = MissionState()

        self.running = False
        self._drone_proc: Optional[subprocess.Popen] = None

        # 日志
        self.logger = logging.getLogger("sim-bridge")
        self.logger.setLevel(logging.DEBUG if verbose else logging.INFO)

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        print(f"\n{C.Y}[信号] 收到 {sig}，正在停止...{C.N}")
        self.running = False

    def start(self):
        """启动模拟"""
        self.running = True
        self.mission.mission_start_time = time.time()

        self._print_banner()
        self._check_connectivity()

        if self.drone_cmd and self.drone_cmd != "none":
            self._start_drone()

        ok("比赛流程模拟已启动")
        print()

        try:
            self._main_loop()
        finally:
            self._cleanup()

    def _print_banner(self):
        print(f"""
{C.C}╔══════════════════════════════════════════════════════════════════════════════╗
║                    ED UAV 比赛流程模拟器 v1.0                                ║
║                                                                              ║
║  ROS:  {NUC_IP}:{NUC_PORT}  (本机)                                           ║
║  CAR:  {CAR_IP}:{CAR_PORT}  {'(已连接)' if self.car.online else '(等待...)':10s}                         ║
║  HMI:  {HMI_IP}:{HMI_PORT}  {'(已连接)' if self.hmi.online else '(等待...)':10s}                         ║
║                                                                              ║
║  自动任务: {'任务 ' + str(self.auto_task) if self.auto_task else '关闭 (等待地面站选择)':20s}                          ║
║  无人机:   {'已启用' if self.drone_cmd else '未启用':20s}                          ║
╚══════════════════════════════════════════════════════════════════════════════╝{C.N}
""")

    def _check_connectivity(self):
        """检查网络连通性"""
        import subprocess as sp
        info("检查网络连通性...")

        # 检查本机端口
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("0.0.0.0", NUC_PORT))
            s.close()
            ok(f"端口 {NUC_PORT} 可用")
        except OSError:
            warn(f"端口 {NUC_PORT} 已被占用，可能有其他 ROS 实例在运行")

        # Ping 检查
        for label, ip in [("CAR", CAR_IP), ("HMI", HMI_IP)]:
            try:
                result = sp.run(["ping", "-c", "1", "-W", "2", ip],
                              capture_output=True, timeout=3)
                if result.returncode == 0:
                    ok(f"{label} ({ip}) 可达")
                else:
                    warn(f"{label} ({ip}) 不可达 (等待连接...)")
            except Exception:
                warn(f"{label} ({ip}) 检测失败")

    def _start_drone(self):
        """启动无人机任务进程"""
        info(f"启动无人机任务: {self.drone_cmd}")
        try:
            self._drone_proc = subprocess.Popen(
                self.drone_cmd, shell=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True
            )
            ok(f"无人机进程已启动 (PID: {self._drone_proc.pid})")
        except Exception as e:
            warn(f"无人机启动失败: {e}")

    def _cleanup(self):
        """清理资源"""
        if self._drone_proc:
            info("停止无人机进程...")
            self._drone_proc.terminate()
            try:
                self._drone_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._drone_proc.kill()
            ok("无人机进程已停止")

        self.codec.close()
        self._print_summary()

    def _main_loop(self):
        """主循环"""
        last_hb_time = 0.0
        last_status_time = 0.0
        last_display_time = 0.0
        auto_task_sent = False

        while self.running:
            now = time.monotonic()

            # 超时检查
            if time.time() - self.mission.mission_start_time > self.duration:
                warn(f"任务超时 ({self.duration}秒)")
                self._enter_fault(FaultFlag.STALE_DATA, "任务超时")
                break

            # 发送心跳 (1Hz)
            if now - last_hb_time >= 1.0:
                self.codec.send_heartbeat(CAR_IP, CAR_PORT)
                self.codec.send_heartbeat(HMI_IP, HMI_PORT)
                last_hb_time = now

            # 发送 MISSION_STATUS (5Hz)
            if now - last_status_time >= 0.2:
                self._send_mission_status()
                last_status_time = now

            # 自动任务选择
            if (self.auto_task and not auto_task_sent
                    and self.mission.phase == MissionPhase.PRESTART
                    and self.car.online and self.hmi.online):
                info(f"自动选择任务 {self.auto_task}")
                self._handle_task_selection(
                    TaskSelection(
                        selection_id=secrets.randbits(32) or 1,
                        car_boot_id=self.car.boot_id,
                        task=self.auto_task
                    )
                )
                auto_task_sent = True

            # 接收数据包
            result = self.codec.recv()
            if result:
                hdr, addr = result
                self._process_packet(hdr, addr)

            # 状态机推进
            self._advance_state()

            # 显示更新 (2Hz)
            if now - last_display_time >= 0.5:
                self._display_status()
                last_display_time = now

            # 终态检查
            if self.mission.is_terminal:
                break

    def _process_packet(self, hdr: PacketHeader, addr: tuple):
        """处理接收到的数据包"""
        if hdr.sender_id == SENDER_CAR:
            self.car.update(hdr)
            if hdr.msg_type == MessageType.CAR_TELEMETRY:
                self._handle_car_telemetry(hdr)
        elif hdr.sender_id == SENDER_HMI:
            self.hmi.update(hdr)
            if hdr.msg_type == MessageType.TASK_SELECTION:
                self._handle_hmi_selection(hdr)
        elif hdr.msg_type == MessageType.HEARTBEAT:
            pass  # 忽略心跳
        else:
            if self.verbose:
                warn(f"未知发送者: 0x{hdr.sender_id:08X}")

    def _handle_car_telemetry(self, hdr: PacketHeader):
        """处理小车遥测"""
        telemetry = decode_car_telemetry(hdr.payload)
        if telemetry is None:
            return

        old_state = self.mission.car_state
        self.mission.car_state = telemetry.state
        self.mission.car_event = telemetry.event
        self.mission.car_event_id = telemetry.event_id
        self.mission.car_displacement_mm = telemetry.displacement_mm
        self.mission.car_velocity_mm_s = telemetry.velocity_mm_s
        self.mission.car_faults = telemetry.fault_flags

        # 事件处理
        if telemetry.event != RouteEvent.NONE:
            event_name = telemetry.event.name
            if telemetry.event_id not in self.mission.events_received:
                self.mission.events_received.append(telemetry.event_id)
                info(f"小车事件: {event_name} (ID={telemetry.event_id})")

        # 状态变化
        if old_state != telemetry.state:
            info(f"小车状态: {old_state.name} → {telemetry.state.name}")

        # 故障检测
        if telemetry.fault_flags != 0:
            warn(f"小车故障: 0x{telemetry.fault_flags:04X}")
            if self.mission.phase in (MissionPhase.CAR_RUNNING, MissionPhase.ARMED_READY):
                self._enter_fault(telemetry.fault_flags, f"小车故障 0x{telemetry.fault_flags:04X}")

    def _handle_hmi_selection(self, hdr: PacketHeader):
        """处理地面站任务选择"""
        selection = decode_task_selection(hdr.payload)
        if selection is None:
            return

        if selection.task not in (1, 2):
            warn(f"无效任务选择: {selection.task}")
            return

        info(f"地面站选择: 任务 {selection.task} (sel_id={selection.selection_id})")
        self.mission.hmi_selection_pending = True
        self.mission.hmi_pending_task = selection.task
        self.mission.hmi_pending_sel_id = selection.selection_id

    def _handle_task_selection(self, selection: TaskSelection):
        """处理任务选择（来自 HMI 或自动）"""
        self.mission.selected_task = selection.task
        self.mission.selection_id = selection.selection_id
        self.mission.hmi_selection_pending = False
        self.mission.phase = MissionPhase.SELECTION_ACKED
        ok(f"任务已确认: 任务 {selection.task}")

    def _advance_state(self):
        """推进状态机
        
        小车无独立启动按钮，上电即启动。流程跳过等待启动信号：
        SELECTION_ACKED → ARMED_READY → CAR_RUNNING (小车在线即推进)
        """
        phase = self.mission.phase

        if phase == MissionPhase.SELECTION_ACKED:
            # 小车上电即启动，无需等待 READY 状态
            # 只要小车在线就进入 ARMED_READY
            if self.car.online and not self.car.is_stale:
                self.mission.phase = MissionPhase.ARMED_READY
                info("状态: ARMED_READY (小车已在线)")

        elif phase == MissionPhase.ARMED_READY:
            # 小车状态为 RUNNING 或 COMPLETE 时直接进入 CAR_RUNNING
            # （小车上电就跑，可能在我们确认任务前已经在跑了）
            if self.mission.car_state in (CarState.RUNNING, CarState.COMPLETE):
                self.mission.phase = MissionPhase.CAR_RUNNING
                self.mission.car_running_start_time = time.time()
                info("状态: CAR_RUNNING (小车已启动)")
            # 如果小车在线但还是 READY，也直接推进（跳过等待）
            elif self.car.online and not self.car.is_stale:
                if self.mission.car_state == CarState.READY:
                    self.mission.phase = MissionPhase.CAR_RUNNING
                    self.mission.car_running_start_time = time.time()
                    info("状态: CAR_RUNNING (小车就绪，跳过等待启动)")

        elif phase == MissionPhase.CAR_RUNNING:
            # 等待小车完成
            if self.mission.car_state == CarState.COMPLETE:
                self.mission.phase = MissionPhase.COMPLETE
                self.mission.complete_time = time.time()
                ok("状态: COMPLETE (任务完成!)")

            # 检查小车安全停车
            if self.mission.car_state == CarState.SAFE_STOP:
                self._enter_fault(self.mission.car_faults, "小车安全停车")

        # HMI 选择处理
        if (self.mission.hmi_selection_pending
                and self.mission.phase == MissionPhase.PRESTART):
            self._handle_task_selection(
                TaskSelection(
                    selection_id=self.mission.hmi_pending_sel_id,
                    car_boot_id=self.car.boot_id,
                    task=self.mission.hmi_pending_task
                )
            )

    def _send_mission_status(self):
        """发送 MISSION_STATUS 到地面站"""
        status = MissionStatus(
            selection_id=self.mission.selection_id,
            car_boot_id=self.car.boot_id,
            hmi_boot_id=self.hmi.boot_id,
            phase=self.mission.phase,
            selected_task=self.mission.selected_task,
            reason_flags=self.mission.fault_reason,
            status_flags=self._compute_status_flags()
        )
        self.codec.send_mission_status(status)

    def _compute_status_flags(self) -> int:
        """计算状态标志"""
        flags = MissionStatusFlag.ROS_READY
        if self.car.online and not self.car.is_stale:
            flags |= MissionStatusFlag.DRONE_LINK_OK
        # 如果有无人机进程在运行
        if self._drone_proc and self._drone_proc.poll() is None:
            flags |= MissionStatusFlag.DRONE_ARMED
        return flags

    def _enter_fault(self, reason: int, description: str):
        """进入故障状态"""
        self.mission.phase = MissionPhase.FAULT
        self.mission.fault_reason = reason
        fail(f"故障: {description} (reason=0x{reason:04X})")

    def _display_status(self):
        """显示当前状态"""
        m = self.mission
        car_age = f"{self.car.age_ms:.0f}ms" if self.car.last_rx_time > 0 else "N/A"
        hmi_age = f"{self.hmi.age_ms:.0f}ms" if self.hmi.last_rx_time > 0 else "N/A"

        phase_color = {
            MissionPhase.PRESTART: C.Y,
            MissionPhase.SELECTION_ACKED: C.C,
            MissionPhase.ARMED_READY: C.G,
            MissionPhase.CAR_RUNNING: C.G,
            MissionPhase.COMPLETE: C.G,
            MissionPhase.FAULT: C.R,
        }.get(m.phase, C.N)

        print(
            f"\r{C.B}[状态]{C.N} "
            f"阶段={phase_color}{m.phase.name:16s}{C.N} "
            f"任务={m.selected_task or '-'} "
            f"小车={m.car_state.name:10s} "
            f"事件={m.car_event.name:8s} "
            f"位移={m.car_displacement_mm/1000:.2f}m "
            f"速度={m.car_velocity_mm_s/1000:.2f}m/s "
            f"CAR链路={car_age:8s} "
            f"HMI链路={hmi_age:8s} "
            f"耗时={m.mission_elapsed:.0f}s",
            end="", flush=True
        )

        # 终态换行
        if m.is_terminal:
            print()

    def _print_summary(self):
        """打印任务摘要"""
        m = self.mission
        print(f"""
{C.C}╔══════════════════════════════════════════════════════════════════════════════╗
║                           任务摘要                                           ║
╠══════════════════════════════════════════════════════════════════════════════╣
║  最终阶段:    {m.phase.name:20s}                                             ║
║  选择任务:    {str(m.selected_task) or 'N/A':20s}                                             ║
║  小车状态:    {m.car_state.name:20s}                                             ║
║  小车事件:    {', '.join(RouteEvent(e).name for e in m.events_received) or 'N/A':20s}                                             ║
║  总位移:      {m.car_displacement_mm/1000:.2f}m{'':17s}                             ║
║  总耗时:      {m.mission_elapsed:.1f}s{'':17s}                             ║
║  CAR 收包:    {self.car.rx_count:20d}                                             ║
║  HMI 收包:    {self.hmi.rx_count:20d}                                             ║
║  故障原因:    {hex(m.fault_reason) if m.fault_reason else 'N/A':20s}                                             ║
╚══════════════════════════════════════════════════════════════════════════════╝{C.N}
""")

        if m.phase == MissionPhase.COMPLETE:
            ok("✅ 比赛流程模拟成功完成!")
        elif m.phase == MissionPhase.FAULT:
            fail("❌ 比赛流程模拟因故障终止")
        else:
            warn(f"⚠️ 比赛流程模拟在 {m.phase.name} 阶段结束")


# ── 主入口 ────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="ED UAV 比赛流程模拟桥接器",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--key-file", default=None,
                        help="HMAC 密钥文件 (默认: 示例密钥)")
    parser.add_argument("--auto-task", type=int, default=0, choices=[0, 1, 2],
                        help="自动选择任务 (0=等待地面站)")
    parser.add_argument("--drone-cmd", default=None,
                        help="无人机任务命令")
    parser.add_argument("--no-drone", action="store_true",
                        help="不启动无人机")
    parser.add_argument("--log-file", default=None,
                        help="日志文件")
    parser.add_argument("--duration", type=float, default=300.0,
                        help="最大运行时间 (秒)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="详细输出")

    args = parser.parse_args()

    # 加载密钥
    if args.key_file:
        with open(args.key_file) as f:
            key = bytes.fromhex(f.read().strip())
    else:
        key = bytes([
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
            0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
            0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
        ])
        warn("使用示例 HMAC 密钥 (与 ESP32 config_local.h 不匹配时会解码失败)")

    # 日志
    if args.log_file:
        logging.basicConfig(
            filename=args.log_file, level=logging.DEBUG,
            format="%(asctime)s %(message)s"
        )

    # 无人机命令
    drone_cmd = None
    if not args.no_drone:
        drone_cmd = args.drone_cmd

    # 启动
    bridge = CompetitionSimBridge(
        key=key,
        auto_task=args.auto_task,
        drone_cmd=drone_cmd,
        duration=args.duration,
        verbose=args.verbose,
    )
    bridge.start()


if __name__ == "__main__":
    main()
