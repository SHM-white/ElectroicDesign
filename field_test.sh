#!/usr/bin/env bash
# ==============================================================================
# 场地测试模式 — 视觉跟随 + 可选雷达定位
#
# 功能:
#   1. 双相机（窄角+广角）并排显示 + AprilTag 检测
#   2. Kalman 滤波 + 模拟视觉伺服
#   3. 自动检测雷达并启动定位链路（FAST-LIO → lio_adapter → source_supervisor）
#   4. 里程计数据展示（自动回退: /localization/odom → /localization/lio/odom → /fcu/optical_flow/odom）
#
# 操作:
#   R     设置当前位置为原点
#   S     保存截图
#   Q/ESC 退出
# ==============================================================================

set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export GDK_SCALE="${GDK_SCALE:-1}"
export GTK_CSD="${GTK_CSD:-0}"
export GDK_BACKEND="${GDK_BACKEND:-x11}"

# ─── ROS2 workspace ───────────────────────────────────────────────────────
_ros2_ws_setup="${REPO_ROOT}/ros2_ws/install/setup.bash"
if [[ -f "$_ros2_ws_setup" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$_ros2_ws_setup"
    set -u
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

# ─── 默认参数 ───────────────────────────────────────────────────────────────
CAMERA_PLAN="${REPO_ROOT}/calibration_data/camera_runtime_plan.local.json"
TAG_SIZE_M="0.15"
TAG_FAMILY="tag36h11"
TARGET_TAG_ID="-1"
MAX_DISPLAY_WIDTH="1280"
CAMERA_YAW_OFFSET="-1.5708"
WIDE_CAMERA_YAW_OFFSET="1.5708"
USE_DIRECT_CAPTURE="true"
CAMERA_DEVICE="/dev/video2"
WIDE_CAMERA_DEVICE="/dev/video0"
ODOM_TOPIC=""
LIDAR_IP="192.168.1.3"
SKIP_LIDAR="false"

# ─── 参数解析 ───────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --camera-plan)        CAMERA_PLAN="$2"; shift 2 ;;
        --tag-size)           TAG_SIZE_M="$2"; shift 2 ;;
        --tag-family)         TAG_FAMILY="$2"; shift 2 ;;
        --tag-id)             TARGET_TAG_ID="$2"; shift 2 ;;
        --max-width)          MAX_DISPLAY_WIDTH="$2"; shift 2 ;;
        --camera-yaw)         CAMERA_YAW_OFFSET="$2"; shift 2 ;;
        --wide-camera-yaw)    WIDE_CAMERA_YAW_OFFSET="$2"; shift 2 ;;
        --direct-capture)     USE_DIRECT_CAPTURE="true"; shift ;;
        --camera-device)      CAMERA_DEVICE="$2"; shift 2 ;;
        --wide-camera-device) WIDE_CAMERA_DEVICE="$2"; shift 2 ;;
        --odom-topic)         ODOM_TOPIC="$2"; shift 2 ;;
        --lidar-ip)           LIDAR_IP="$2"; shift 2 ;;
        --skip-lidar)         SKIP_LIDAR="true"; shift ;;
        -h|--help)
            cat <<'EOF'
场地测试模式 — 视觉跟随 + 可选雷达定位

用法: ./field_test.sh [选项]

相机选项:
  --camera-plan FILE         相机运行计划 JSON
  --tag-size FLOAT           AprilTag 边长（默认 0.15m）
  --tag-family NAME          AprilTag 系列（默认 tag36h11）
  --tag-id INT               目标 tag ID（默认 -1 = 任意）
  --max-width INT            显示窗口宽度（默认 1280）
  --camera-yaw FLOAT         窄角偏航角（默认 -π/2）
  --wide-camera-yaw FLOAT    广角偏航角（默认 +π/2）
  --direct-capture           直接 OpenCV 读摄像头
  --camera-device PATH       窄角设备（默认 /dev/video2）
  --wide-camera-device PATH  广角设备（默认 /dev/video0）

定位选项:
  --odom-topic TOPIC         里程计 topic（默认自动检测）
  --lidar-ip IP              雷达 IP（默认 192.168.1.3，不可达则跳过定位链路）
  --skip-lidar               强制跳过雷达定位链路

操作:
  R     设置位移原点
  S     保存截图
  Q/ESC 退出
EOF
            exit 0
            ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# ─── 前置检查 ───────────────────────────────────────────────────────────────
if [[ ! -f "$CAMERA_PLAN" ]]; then
    echo "错误: 相机计划不存在: $CAMERA_PLAN"
    echo "运行 ./tools/calibration/run_camera_calibration.sh"
    exit 1
fi

# ─── 读取 MID360 manifest ────────────────────────────────────────────────
MANIFEST_PATH="${REPO_ROOT}/ros2_ws/src/ed_uav_lidar/config/fields/mid360_field_manifest.local.json"

manifest_value() {
    python3 - "$MANIFEST_PATH" "$1" <<'PY'
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

# ─── 检测雷达 ──────────────────────────────────────────────────────────────
LIDAR_AVAILABLE="false"
LIDAR_PIDS=()

if [[ "$SKIP_LIDAR" == "true" ]]; then
    echo "  雷达: 已跳过 (--skip-lidar)"
elif [[ ! -f "$MANIFEST_PATH" ]]; then
    echo "  雷达: manifest 不存在 ($MANIFEST_PATH)，跳过定位链路"
else
    echo -n "  检测雷达 ${LIDAR_IP}... "
    if ping -c 1 -W 1 "$LIDAR_IP" &>/dev/null; then
        echo "可达"
        LIDAR_AVAILABLE="true"
    else
        echo "不可达，跳过定位链路"
    fi
fi

# ─── 启动定位链路（复用 start_lidar_odometry.sh 已验证的命令） ────────────
_cleanup() {
    for pid in "${LIDAR_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
}
trap _cleanup EXIT

if [[ "$LIDAR_AVAILABLE" == "true" ]]; then
    # 与 tools/start_lidar_odometry.sh 一致的参数（从 manifest 读取）
    FIELD_SERIAL_NUMBER="$(manifest_value serial_number)"
    FIELD_LIDAR_IP="$(manifest_value lidar_ip)"
    FIELD_HOST_IP="$(manifest_value host_ip)"
    FIELD_FIRMWARE="$(manifest_value firmware)"
    FIELD_DRIVER_JSON="$(manifest_value driver_json)"
    FIELD_EXTRINSICS_PATH="$(manifest_value extrinsics)"
    FIELD_FAST_LIO_LAUNCH="$(manifest_value fast_lio_launch)"

    # 覆盖 lidar ip（如果命令行指定了不同值）
    if [[ -n "$LIDAR_IP" && "$LIDAR_IP" != "192.168.1.3" ]]; then
        FIELD_LIDAR_IP="$LIDAR_IP"
    fi
    if [[ -n "${MID360_HOST_IP:-}" ]]; then
        FIELD_HOST_IP="$MID360_HOST_IP"
    fi

    # MVS SDK libusb 覆盖系统版本，导致 PCL 符号查找失败
    export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}

    echo "  启动定位链路 (雷达 ${FIELD_LIDAR_IP} @ host ${FIELD_HOST_IP})..."

    # 1. 雷达驱动（必须带 MID360_HOST_IP + firmware + driver_config）
    setsid /bin/bash -lc "export MID360_HOST_IP='$FIELD_HOST_IP'; ros2 launch ed_uav_lidar lidar.launch.py lidar_enabled:=true transport:=mid360 serial_number:='$FIELD_SERIAL_NUMBER' sensor_ip:='$FIELD_LIDAR_IP' firmware_version:='$FIELD_FIRMWARE' driver_config_path:='$FIELD_DRIVER_JSON' time_authority:=host" > /tmp/field_test_lidar.log 2>&1 &
    LIDAR_PIDS+=($!)

    # 2. FAST-LIO（必须用 manifest 中的绝对路径）
    sleep 3
    setsid /bin/bash -lc "ros2 launch '$FIELD_FAST_LIO_LAUNCH'" > /tmp/field_test_fastlio.log 2>&1 &
    LIDAR_PIDS+=($!)

    # 3. LIO 适配器（必须用 field_extrinsics.yaml）
    sleep 2
    setsid /bin/bash -lc "ros2 run ed_uav_localization lio_adapter --ros-args -p calibration_file:='$FIELD_EXTRINSICS_PATH'" > /tmp/field_test_lio_adapter.log 2>&1 &
    LIDAR_PIDS+=($!)

    # 4. 定位融合
    setsid /bin/bash -lc "ros2 run ed_uav_localization source_supervisor" > /tmp/field_test_supervisor.log 2>&1 &
    LIDAR_PIDS+=($!)

    echo "  定位链路已启动，等待 /localization/odom 就绪..."
    for i in $(seq 1 20); do
        if timeout 2s ros2 topic info /localization/odom >/dev/null 2>&1; then
            echo "  /localization/odom 就绪"
            break
        fi
        sleep 1
    done
fi

# ─── 构建 field_test 启动参数 ──────────────────────────────────────────────
LAUNCH_ARGS=(
    "camera_plan:=$CAMERA_PLAN"
    "tag_size_m:=$TAG_SIZE_M"
    "tag_family:=$TAG_FAMILY"
    "target_tag_id:=$TARGET_TAG_ID"
    "max_display_width:=$MAX_DISPLAY_WIDTH"
    "camera_yaw_offset_rad:=$CAMERA_YAW_OFFSET"
    "wide_camera_yaw_offset_rad:=$WIDE_CAMERA_YAW_OFFSET"
    "use_direct_capture:=$USE_DIRECT_CAPTURE"
    "camera_device:=$CAMERA_DEVICE"
    "wide_camera_device:=$WIDE_CAMERA_DEVICE"
)
if [[ -n "$ODOM_TOPIC" ]]; then
    LAUNCH_ARGS+=("odom_topic:=$ODOM_TOPIC")
fi

# ─── 执行 ──────────────────────────────────────────────────────────────────
echo ""
echo "══════════════════════════════════════════════════════════"
echo "  场地测试 — 视觉跟随 + 可选定位"
echo "══════════════════════════════════════════════════════════"
echo "  tag: ${TAG_FAMILY} ${TAG_SIZE_M}m id=${TARGET_TAG_ID}"
echo "  narrow: ${CAMERA_DEVICE}  yaw=${CAMERA_YAW_OFFSET}"
echo "  wide:   ${WIDE_CAMERA_DEVICE}  yaw=${WIDE_CAMERA_YAW_OFFSET}"
echo "  定位:   $([ "$LIDAR_AVAILABLE" == "true" ] && echo "FAST-LIO 运行中" || echo "无")"
echo "══════════════════════════════════════════════════════════"
echo ""

# 不用 exec：exec 会替换 shell，导致上面 trap 的雷达进程清理失效
ros2 launch ed_uav_mission field_test.launch.py "${LAUNCH_ARGS[@]}"
status=$?
echo "场地测试已退出，清理雷达进程..."
_cleanup
exit "$status"
