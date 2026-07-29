#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────
# run_lidar_displacement_monitor.sh
# 订阅里程计话题，实时输出相对于起始位置的位移（持续运行直到 Ctrl+C）
# 用法: ./tools/run_lidar_displacement_monitor.sh [odom_topic]
# ──────────────────────────────────────────────────────────────────────
set -euo pipefail

readonly ros_setup="/opt/ros/humble/setup.bash"
readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly overlay_setup="$repo_root/ros2_ws/install/setup.bash"
ODOM_TOPIC="${1:-${ODOM_TOPIC:-/localization/odom}}"

if [[ ! -f "$ros_setup" ]]; then
    printf '错误: 未找到 ROS 2 Humble: %s\n' "$ros_setup" >&2
    exit 1
fi

set +u
source "$ros_setup"
[[ -f "$overlay_setup" ]] && source "$overlay_setup"
set -u

printf '═══════════════════════════════════════════════════════\n'
printf ' 雷达位移实时监控\n'
printf ' 里程计话题: %s\n' "$ODOM_TOPIC"
printf ' 参考坐标系: 程序启动时的雷达位置\n'
printf '═══════════════════════════════════════════════════════\n'
printf '按 Ctrl+C 停止。\n\n'

python3 - "$ODOM_TOPIC" <<'PYEOF'
"""Continuous displacement monitor: prints displacement from initial pose."""

import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DisplacementMonitor(Node):
    def __init__(self, odom_topic: str):
        super().__init__("displacement_monitor")
        self._first = None  # (x, y, z, yaw, stamp_ns)
        self._count = 0
        self._last_print = 0.0
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if self._first is None:
            self._first = (pos.x, pos.y, pos.z, yaw, stamp_ns)
            print(f"起始位置已记录: x={pos.x:.4f} y={pos.y:.4f} z={pos.z:.4f} yaw={math.degrees(yaw):.1f}°")
            print(f"{'时间(s)':>8s}  {'dx(m)':>8s}  {'dy(m)':>8s}  {'dz(m)':>8s}  {'水平距离(m)':>10s}  {'3D距离(m)':>10s}  {'航向变化(°)':>10s}")
            print("─" * 80)
            self._last_print = time.monotonic()
            return

        now = time.monotonic()
        # 每 0.2 秒打印一次（5 Hz）
        if now - self._last_print < 0.2:
            return
        self._last_print = now

        x0, y0, z0, yaw0, first_stamp_ns = self._first
        dx = pos.x - x0
        dy = pos.y - y0
        dz = pos.z - z0
        dyaw_deg = math.degrees(yaw - yaw0)
        # 规范化到 [-180, 180]
        dyaw_deg = (dyaw_deg + 180) % 360 - 180
        xy = math.sqrt(dx * dx + dy * dy)
        d3 = math.sqrt(dx * dx + dy * dy + dz * dz)
        elapsed_sec = (stamp_ns - first_stamp_ns) * 1e-9

        self._count += 1
        sys.stdout.write(
            f"\r\x1b[K"
            f"  {elapsed_sec:7.1f}s  "
            f"dx={dx:+8.4f}  dy={dy:+8.4f}  dz={dz:+8.4f}  "
            f"水平={xy:8.4f}  3D={d3:8.4f}  "
            f"航向={dyaw_deg:+7.1f}°  "
            f"[样本#{self._count}]"
        )
        sys.stdout.flush()


def main():
    odom_topic = sys.argv[1] if len(sys.argv) > 1 else "/localization/odom"
    rclpy.init(args=[])
    node = DisplacementMonitor(odom_topic)

    def handle_sigint(*_):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n\n已停止。共接收 {node._count} 个里程计样本。")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
PYEOF
