#!/usr/bin/env bash
# ==============================================================================
# Task3 一键启动脚本
#
# 用法:
#   ./task3.sh                    # 启动（需要 SROS2 keystore）
#   ./task3.sh --dry-run          # 验证配置，不启动硬件
#   ./task3.sh --fcu /dev/ttyUSB1 # 指定飞控串口
#
# 环境变量:
#   ROS_SECURITY_KEYSTORE  SROS2 keystore 路径（启动时必须）
#   FCU_SERIAL_PORT        飞控串口路径（默认 /dev/ttyUSB0）
# ==============================================================================

set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ─── 自动 source ROS2 workspace ────────────────────────────────────────────
_ros2_ws_setup="${REPO_ROOT}/ros2_ws/install/setup.bash"
if [[ -f "$_ros2_ws_setup" ]]; then
    set +u  # colcon setup.bash 使用未声明变量
    # shellcheck disable=SC1090
    source "$_ros2_ws_setup"
    set -u
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

# ─── 默认路径（使用仓库内已有配置）─────────────────────────────────────────
MISSION_CONFIG="${REPO_ROOT}/ros2_ws/src/ed_uav_mission/config/missions/simulation_stability_test.yaml"
FIELD_PROFILE="${REPO_ROOT}/ros2_ws/src/ed_uav_localization/config/fields/simulation_arena.yaml"
CALIBRATION="${REPO_ROOT}/ros2_ws/src/ed_uav_description/config/example_uncalibrated.yaml"
CAMERA_PLAN="${REPO_ROOT}/calibration_data/camera_runtime_plan.local.json"
HMAC_KEY="${REPO_ROOT}/config/hmac.key.hex"
MID360_DRIVER="${REPO_ROOT}/ros2_ws/src/ed_uav_lidar/config/fields/mid360_field_manifest.local.json"
FAST_LIO="${REPO_ROOT}/ros2_ws/src/ed_uav_lidar/config/fields/fast_lio.launch.py"
FCU_SERIAL="${FCU_SERIAL_PORT:-/dev/ttyUSB0}"
TASK3_IDENTITY="task3-stability-2026"
ENABLE_DISPLAY=""

# ─── 参数解析 ───────────────────────────────────────────────────────────────
DRY_RUN=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)        DRY_RUN="--dry-run"; shift ;;
        --fcu)            FCU_SERIAL="$2"; shift 2 ;;
        --mission)        MISSION_CONFIG="$2"; shift 2 ;;
        --profile)        FIELD_PROFILE="$2"; shift 2 ;;
        --calibration)    CALIBRATION="$2"; shift 2 ;;
        --camera-plan)    CAMERA_PLAN="$2"; shift 2 ;;
        --keystore)       ROS_SECURITY_KEYSTORE="$2"; shift 2 ;;
        --enable-display) ENABLE_DISPLAY="--enable-display"; shift ;;
        --no-display)     ENABLE_DISPLAY=""; shift ;;
        -h|--help)
            cat <<'EOF'
Task3 一键启动脚本

用法: ./task3.sh [选项]

选项:
  --dry-run           验证配置，不启动硬件
  --fcu PATH          飞控串口（默认 /dev/ttyUSB0）
  --mission PATH      任务配置 YAML
  --profile PATH      场地配置 YAML
  --calibration PATH  标定文件 JSON
  --camera-plan PATH  相机计划 JSON（首次需运行标定生成）
  --keystore PATH     SROS2 keystore 目录
  --enable-display    强制启用任务画面输出
  --no-display        禁用任务画面输出
  -h, --help          显示帮助

首次使用:
  1. 运行 ./tools/calibration/run_camera_calibration.sh 生成相机计划
  2. 确保 ROS_SECURITY_KEYSTORE 环境变量指向 keystore 目录
  3. 运行 ./task3.sh --dry-run 验证配置
  4. 运行 ./task3.sh 启动
EOF
            exit 0
            ;;
        *) echo "未知参数: $1（运行 ./task3.sh --help 查看帮助）"; exit 1 ;;
    esac
done

# ─── 前置检查 ───────────────────────────────────────────────────────────────
# Auto-detect display: if --enable-display/--no-display not explicitly set,
# check if a graphical display is available and enable automatically.
if [[ -z "$ENABLE_DISPLAY" ]]; then
    if [[ -n "${DISPLAY:-}" ]] || [[ -n "${WAYLAND_DISPLAY:-}" ]]; then
        ENABLE_DISPLAY="--enable-display"
        echo "检测到显示设备，已自动启用任务画面输出"
    else
        echo "未检测到显示设备，画面输出将以日志形式记录"
    fi
fi
if [[ ! -f "$CAMERA_PLAN" ]]; then
    echo "错误: 相机计划文件不存在: $CAMERA_PLAN"
    echo ""
    echo "首次使用需要先运行相机标定:"
    echo "  ./tools/calibration/run_camera_calibration.sh"
    echo ""
    echo "或手动指定相机计划:"
    echo "  ./task3.sh --camera-plan /path/to/your/camera_plan.json"
    exit 1
fi

# ─── 执行 ───────────────────────────────────────────────────────────────────
exec "${REPO_ROOT}/tools/run_task3_flight_test.sh" \
    $DRY_RUN \
    --mission-config "$MISSION_CONFIG" \
    --field-profile "$FIELD_PROFILE" \
    --calibration "$CALIBRATION" \
    --camera-runtime-plan "$CAMERA_PLAN" \
    --fcu-serial "$FCU_SERIAL" \
    --hmac-key-file "$HMAC_KEY" \
    --mid360-driver-config "$MID360_DRIVER" \
    --fast-lio-launch "$FAST_LIO" \
    --task3-identity "$TASK3_IDENTITY" \
    $ENABLE_DISPLAY
