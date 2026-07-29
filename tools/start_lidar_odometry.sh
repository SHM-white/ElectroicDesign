#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────
# start_lidar_odometry.sh
# 一键启动: MID-360 雷达驱动 → FAST-LIO → 定位融合 → 位移实时显示
# 用法: ./tools/start_lidar_odometry.sh
#
# 启动链路:
#   1. livox_ros_driver2  (MID-360 点云 + IMU)
#   2. fastlio_mapping    (FAST-LIO 里程计)
#   3. lio_adapter        (坐标变换适配)
#   4. source_supervisor  (定位融合输出 /localization/odom)
#   5. displacement_monitor (终端实时显示位移)
# ──────────────────────────────────────────────────────────────────────
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly workspace="$repo_root/ros2_ws"
readonly ros_setup="/opt/ros/humble/setup.bash"
readonly manifest_path="$workspace/src/ed_uav_lidar/config/fields/mid360_field_manifest.local.json"
readonly state_dir="${XDG_STATE_HOME:-$repo_root/.omo/evidence}/lidar-odometry"
readonly run_dir="$state_dir/$(date -u +%Y%m%dT%H%M%SZ)-$$"

declare -A owned_pid=()
declare -A owned_pgid=()
declare -a owned_labels=()
launcher_pgid="$(ps -o pgid= -p $$ | tr -d ' ')"

# ── 进程管理 ──────────────────────────────────────────────────────────
process_is_alive() {
    local pid="$1"
    kill -0 "$pid" 2>/dev/null || return 1
    local state
    state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ')"
    [[ -n "$state" && "$state" != Z* ]]
}

cleanup() {
    local status=$1
    trap - EXIT INT TERM
    printf '\n正在停止所有进程 ...\n'
    local label pgid pid
    if ((${#owned_labels[@]} > 0)); then
        for label in "${owned_labels[@]}"; do
            pgid="${owned_pgid[$label]:-}"
            [[ -n "$pgid" ]] && kill -TERM -- "-$pgid" 2>/dev/null || true
        done
    fi
    local deadline=$((SECONDS + 5))
    while ((SECONDS < deadline)); do
        local alive=0
        if ((${#owned_labels[@]} > 0)); then
            for label in "${owned_labels[@]}"; do
                pid="${owned_pid[$label]:-}"
                [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null && alive=1
            done
        fi
        ((alive == 0)) && break
        sleep 0.1
    done
    if ((${#owned_labels[@]} > 0)); then
        for label in "${owned_labels[@]}"; do
            pgid="${owned_pgid[$label]:-}"
            [[ -n "$pgid" ]] && kill -KILL -- "-$pgid" 2>/dev/null || true
        done
        for label in "${owned_labels[@]}"; do
            pid="${owned_pid[$label]:-}"
            [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
        done
    fi
    printf '所有进程已停止。\n'
    exit "$status"
}
trap 'cleanup $?' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ── 读取 manifest ─────────────────────────────────────────────────────
manifest_value() {
    python3 - "$manifest_path" "$1" <<'PY'
import json, sys
from pathlib import Path
manifest = Path(sys.argv[1])
key = sys.argv[2]
data = json.loads(manifest.read_text(encoding='utf-8'))
value = Path(data[key]) if key in {'driver_json', 'extrinsics', 'fast_lio_launch'} else data[key]
if key in {'driver_json', 'extrinsics', 'fast_lio_launch'} and not value.is_absolute():
    value = manifest.parent / value
print(value)
PY
}

# ── 前置检查 ──────────────────────────────────────────────────────────
if [[ ! -f "$ros_setup" ]]; then
    printf '错误: 未找到 ROS 2 Humble: %s\n' "$ros_setup" >&2
    exit 1
fi

if [[ ! -f "$manifest_path" ]]; then
    printf '错误: 未找到 MID360 配置: %s\n' "$manifest_path" >&2
    printf '请先创建 mid360_field_manifest.local.json\n' >&2
    exit 1
fi

set +u
source "$ros_setup"
set -u

# Fix: MVS SDK 的 libusb 覆盖了系统版本，导致 PCL 符号查找失败
export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}

# ── 读取配置 ──────────────────────────────────────────────────────────
FIELD_SERIAL_NUMBER="$(manifest_value serial_number)"
FIELD_LIDAR_IP="$(manifest_value lidar_ip)"
FIELD_HOST_IP="$(manifest_value host_ip)"
FIELD_FIRMWARE="$(manifest_value firmware)"
FIELD_DRIVER_JSON="$(manifest_value driver_json)"
FIELD_EXTRINSICS_PATH="$(manifest_value extrinsics)"
FIELD_FAST_LIO_LAUNCH="$(manifest_value fast_lio_launch)"

# 如果环境变量指定了 host IP，覆盖 manifest 中的值
if [[ -n "${MID360_HOST_IP:-}" ]]; then
    FIELD_HOST_IP="$MID360_HOST_IP"
fi

# ── 编译 ──────────────────────────────────────────────────────────────
printf '编译 ROS2 包 ...\n'
colcon --log-base "$workspace/log" build \
    --build-base "$workspace/build" \
    --install-base "$workspace/install" \
    --symlink-install \
    --packages-up-to livox_ros_driver2 fast_lio ed_uav_lidar ed_uav_localization ed_uav_description

set +u
source "$workspace/install/setup.bash"
set -u

mkdir -p "$run_dir"
printf '日志目录: %s\n' "$run_dir"

# ── 启动函数 ──────────────────────────────────────────────────────────
spawn() {
    local label="$1"
    local cmd="$2"
    local log="$run_dir/$label.log"
    : > "$log"
    if [[ "$label" == monitor ]]; then
        setsid /bin/bash -lc "$cmd" > >(tee -a "$log") 2> >(tee -a "$log" >&2) &
    else
        setsid /bin/bash -lc "$cmd" >>"$log" 2>&1 &
    fi
    local pid=$!
    local pgid
    pgid="$(ps -o pgid= -p "$pid" | tr -d ' ')"
    owned_pid[$label]="$pid"
    owned_pgid[$label]="$pgid"
    owned_labels+=("$label")
    printf '  [%s] pid=%s cmd=%s\n' "$label" "$pid" "$cmd"
}

wait_for_topic() {
    local topic="$1" timeout_sec="${2:-15}"
    printf '等待话题 %s ...' "$topic"
    for i in $(seq 1 "$timeout_sec"); do
        if timeout 2s ros2 topic info "$topic" >/dev/null 2>&1; then
            printf ' 就绪。\n'
            return 0
        fi
        printf '.'
        sleep 1
    done
    printf ' 超时！\n' >&2
    return 1
}

# ══════════════════════════════════════════════════════════════════════
# 启动管线
# ══════════════════════════════════════════════════════════════════════
printf '\n═══════════════════════════════════════════════════════\n'
printf ' 启动 MID-360 里程计管线\n'
printf ' 雷达: %s @ %s (host: %s)\n' "$FIELD_SERIAL_NUMBER" "$FIELD_LIDAR_IP" "$FIELD_HOST_IP"
printf '═══════════════════════════════════════════════════════\n\n'

# Step 1: 雷达驱动
printf '[1/5] 启动 Livox MID-360 驱动 ...\n'
spawn lidar "export MID360_HOST_IP='$FIELD_HOST_IP'; ros2 launch ed_uav_lidar lidar.launch.py lidar_enabled:=true transport:=mid360 serial_number:='$FIELD_SERIAL_NUMBER' sensor_ip:='$FIELD_LIDAR_IP' firmware_version:='$FIELD_FIRMWARE' driver_config_path:='$FIELD_DRIVER_JSON' time_authority:=host"

wait_for_topic /livox/lidar 20

# Step 2: FAST-LIO
printf '[2/5] 启动 FAST-LIO ...\n'
spawn fast_lio "ros2 launch $FIELD_FAST_LIO_LAUNCH"

wait_for_topic /fast_lio/odometry 20

# Step 3: LIO 适配器
printf '[3/5] 启动 LIO 适配器 ...\n'
spawn lio_adapter "ros2 run ed_uav_localization lio_adapter --ros-args -p calibration_file:='$FIELD_EXTRINSICS_PATH'"

# Step 4: 定位融合
printf '[4/5] 启动定位融合 ...\n'
spawn supervisor "ros2 run ed_uav_localization source_supervisor"

wait_for_topic /localization/odom 10

# Step 5: 位移监控
printf '[5/5] 启动位移实时监控 ...\n\n'
spawn monitor "bash $repo_root/tools/run_lidar_displacement_monitor.sh /localization/odom"

printf '\n═══════════════════════════════════════════════════════\n'
printf ' 所有组件已启动。在下方查看实时位移输出。\n'
printf ' 按 Ctrl+C 停止所有组件。\n'
printf '═══════════════════════════════════════════════════════\n\n'

# ── 主循环: 等待 monitor 结束或任一上游进程崩溃 ──────────────────────
monitor_pid="${owned_pid[monitor]}"
upstream_labels=(lidar fast_lio lio_adapter supervisor)
child_status=0
upstream_failed=""

while process_is_alive "$monitor_pid"; do
    for label in "${upstream_labels[@]}"; do
        if ! process_is_alive "${owned_pid[$label]}"; then
            upstream_failed="$label"
            child_status=1
            break 2
        fi
    done
    sleep 0.1
done

if [[ -n "$upstream_failed" ]]; then
    printf '\n错误: 上游进程 %s 已退出！\n' "$upstream_failed" >&2
    printf '查看日志: %s/%s.log\n' "$run_dir" "$upstream_failed" >&2
fi

exit "$child_status"
