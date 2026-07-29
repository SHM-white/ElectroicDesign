#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────
# start_real_lidar_rviz.sh
# 一键启动 Livox MID-360 雷达驱动 + rviz2 点云可视化
# 用法: ./tools/start_real_lidar_rviz.sh
# ──────────────────────────────────────────────────────────────────────
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly workspace="$repo_root/ros2_ws"
readonly ros_setup="/opt/ros/humble/setup.bash"
declare -a background_pgids=()

cleanup() {
    local status="$?"
    trap - EXIT INT TERM
    local pgid
    for pgid in "${background_pgids[@]}"; do
        kill -INT -- "-$pgid" 2>/dev/null || true
    done
    for pgid in "${background_pgids[@]}"; do
        wait "$pgid" 2>/dev/null || true
    done
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ── 前置检查 ──────────────────────────────────────────────────────────
if [[ ! -f "$ros_setup" ]]; then
    printf '错误: 未找到 ROS 2 Humble: %s\n' "$ros_setup" >&2
    exit 1
fi

# ── 显示环境 ──────────────────────────────────────────────────────────
if [[ -n "${ED_RVIZ_DISPLAY:-}" ]]; then
    export DISPLAY="$ED_RVIZ_DISPLAY"
elif [[ -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY=:0
else
    export DISPLAY="${DISPLAY:-}"
fi

if [[ -z "$DISPLAY" ]]; then
    printf '错误: 未找到图形显示。请设置 ED_RVIZ_DISPLAY，例如 ED_RVIZ_DISPLAY=:0\n' >&2
    exit 1
fi

if [[ "$DISPLAY" == :0 && -f "/run/user/$UID/gdm/Xauthority" ]]; then
    export XAUTHORITY="/run/user/$UID/gdm/Xauthority"
fi

# ── 源码 & 编译 ──────────────────────────────────────────────────────
# Fix: MVS SDK 的 libusb 覆盖了系统版本，导致 PCL 符号查找失败
export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}

set +u
source "$ros_setup"
set -u

printf '编译 livox_ros_driver2 和 ed_uav_lidar ...\n'
colcon --log-base "$workspace/log" build \
    --build-base "$workspace/build" \
    --install-base "$workspace/install" \
    --symlink-install \
    --packages-up-to livox_ros_driver2 ed_uav_lidar

set +u
source "$workspace/install/setup.bash"
set -u

# ── 雷达驱动配置 ──────────────────────────────────────────────────────
# 使用项目内的 MID360 驱动配置（host IP 自动匹配）
readonly driver_config="$workspace/src/ed_uav_lidar/config/mid360_driver.json"

# 如果环境变量指定了 host IP，更新配置中的 host_net_info
if [[ -n "${MID360_HOST_IP:-}" ]]; then
    printf '使用 MID360_HOST_IP=%s 更新驱动配置\n' "$MID360_HOST_IP"
    tmp_config=$(mktemp)
    python3 -c "
import json, sys
with open('$driver_config') as f:
    cfg = json.load(f)
ip = '$MID360_HOST_IP'
for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
    cfg['MID360']['host_net_info'][key] = ip
with open('$tmp_config', 'w') as f:
    json.dump(cfg, f, indent=2)
"
    driver_config_final="$tmp_config"
else
    driver_config_final="$driver_config"
fi

# ── rviz 配置 ─────────────────────────────────────────────────────────
readonly rviz_config="$workspace/src/third_party/livox_ros_driver2/config/display_point_cloud_ROS2.rviz"

printf '═══════════════════════════════════════════════════════\n'
printf ' 启动 Livox MID-360 雷达点云可视化\n'
printf ' 驱动配置: %s\n' "$driver_config_final"
printf ' rviz 配置: %s\n' "$rviz_config"
printf '═══════════════════════════════════════════════════════\n'
printf '按 Ctrl+C 停止。\n\n'

# ── 启动雷达驱动 ──────────────────────────────────────────────────────
setsid ros2 run livox_ros_driver2 livox_ros_driver2_node \
    --ros-args \
    -p xfer_format:=1 \
    -p multi_topic:=0 \
    -p data_src:=0 \
    -p publish_freq:=10.0 \
    -p output_data_type:=0 \
    -p frame_id:=livox_frame \
    -p user_config_path:="$driver_config_final" \
    &
background_pgids+=("$!")

# 等待驱动发布点云话题
printf '等待雷达驱动就绪 (/livox/lidar 话题) ...\n'
for i in $(seq 1 30); do
    if timeout 2s ros2 topic info /livox/lidar >/dev/null 2>&1; then
        printf '雷达驱动已就绪。\n'
        break
    fi
    if [[ $i -eq 30 ]]; then
        printf '警告: 等待超时，继续尝试启动 rviz ...\n' >&2
    fi
    sleep 1
done

# ── 启动 rviz2 ────────────────────────────────────────────────────────
rviz2 -d "$rviz_config" --ros-args -r __node:=lidar_rviz
