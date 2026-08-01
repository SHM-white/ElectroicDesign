#!/usr/bin/env bash
# =============================================================================
# Task3 一键启动 — 比赛控制全链路 (重写版)
#
# 完整控制流程 (全部复用已验证硬件模块, 无模拟链路):
#   地面站 HMI (TASK_SELECTION 选择 task, 含 mode 字节 1=实飞/2=模拟飞)
#     → vehicle_bridge (UDP 42000/42001/42002, HMAC 认证)
#     → mission_executor (task3-stability 配置)
#     → flight_command → fcu_bridge → 凌霄飞控 V7 (串口)
#   视觉   : 双相机 PnP AprilTag 跟踪 (target_observation)
#   里程计 : MID360 FAST-LIO 高精里程计 (lio_adapter → source_supervisor)
#   载荷   : 电磁铁/激光 (H7 GPIO bridge, /dev/ttyUSB1)
#   可视化 : mission_display 双路相机 + 任务 HUD (复用 field_test 显示方案)
#
# 实飞门控 (语义完全按 drone/ 工具):
#   启动开关: AUX6 (第10通道, V7 0x40 帧第10项) > 1700us
#     (drone/monitor_aux6.py 阈值 1700; drone/state_machine.py 注释明确
#      "AUX1~AUX5 不可用", 启动只用 AUX6)
#   程控门控: AUX1 (第5通道, V7 0x40 帧第5项 channels_us[4]) 1400~1600us
#     (fcu_bridge realtime_policy, 模式2 位置控制)
#   硬锁    : AUX1 (第5通道) >= 1800us (fcu_bridge session, 紧急停止接管串口)
#   --immediate-start 跳过以上等待 (仅调试)
#
# 用法:
#   ./task3.sh                                  # 实飞 (默认, 等待小车信号 + AUX 门控)
#   ./task3.sh --mode sim                       # 模拟飞 (no_car_sim, 飞机不动)
#   ./task3.sh --mode real --immediate-start    # 实飞调试: 地面站选择即启动
#   ./task3.sh --dry-run                        # 非飞控全链路自检
#   ./task3.sh --fcu /dev/ttyUSB1               # 指定飞控串口
#
# 环境变量:
#   ROS_SECURITY_KEYSTORE  SROS2 keystore 路径 (默认 ${REPO_ROOT}/keystore)
#   FCU_SERIAL_PORT        飞控串口路径 (默认 /dev/ttyUSB0)
# =============================================================================

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
MISSION_CONFIG="${REPO_ROOT}/ros2_ws/src/ed_uav_mission/config/missions/d_arena_stability_test.yaml"
FIELD_PROFILE="${REPO_ROOT}/ros2_ws/src/ed_uav_localization/config/fields/d_arena_2026.yaml"
CALIBRATION="${REPO_ROOT}/calibration_data/field_calibrated_v1.yaml"
CAMERA_PLAN="${REPO_ROOT}/calibration_data/camera_runtime_plan.local.json"
HMAC_KEY="${REPO_ROOT}/config/hmac.key.hex"
MID360_DRIVER="${REPO_ROOT}/ros2_ws/src/ed_uav_lidar/config/fields/mid360_field_manifest.local.json"
FAST_LIO="${REPO_ROOT}/ros2_ws/src/ed_uav_lidar/config/fields/fast_lio.launch.py"
FCU_SERIAL="${FCU_SERIAL_PORT:-/dev/ttyUSB0}"
H7_SERIAL="${H7_SERIAL_PORT:-/dev/ttyUSB1}"
LIDAR_IP="192.168.1.3"
TASK3_IDENTITY="task3-stability-2026"
ROS_SECURITY_KEYSTORE="${ROS_SECURITY_KEYSTORE:-${REPO_ROOT}/keystore}"
ENABLE_DISPLAY=""
IMMEDIATE_START="false"

# ─── 参数解析 ───────────────────────────────────────────────────────────────
DRY_RUN=""
MODE="real"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)         DRY_RUN="--dry-run"; shift ;;
        --mode)            MODE="$2"; shift 2 ;;
        --immediate-start) IMMEDIATE_START="true"; shift ;;
        --wait-car)        IMMEDIATE_START="false"; shift ;;
        --fcu)             FCU_SERIAL="$2"; shift 2 ;;
        --h7)              H7_SERIAL="$2"; shift 2 ;;
        --lidar-ip)        LIDAR_IP="$2"; shift 2 ;;
        --mission)         MISSION_CONFIG="$2"; shift 2 ;;
        --profile)         FIELD_PROFILE="$2"; shift 2 ;;
        --calibration)     CALIBRATION="$2"; shift 2 ;;
        --camera-plan)     CAMERA_PLAN="$2"; shift 2 ;;
        --keystore)        ROS_SECURITY_KEYSTORE="$2"; shift 2 ;;
        --enable-display)  ENABLE_DISPLAY="--enable-display"; shift ;;
        --no-display)      ENABLE_DISPLAY=""; shift ;;
        -h|--help)
            cat <<'EOF'
Task3 一键启动 — 比赛控制全链路

用法: ./task3.sh [选项]

模式:
  --mode real           实飞: fcu_bridge + CALIBRATED + SROS2 Enforce + AUX5 门控 (默认)
  --mode sim            模拟飞: no_car_sim 应答飞行指令 + simulation_only, 飞机不会动
                         (等价 run_no_car_mode.sh, 地面站选择即启动)

启动门控:
  --wait-car            等待小车 START + AUX6(第10通道 >1700us) 启动 (默认, 实飞安全)
  --immediate-start     跳过小车/门控等待, 地面站选择即启动 (仅调试)

选项:
  --dry-run           非飞控全链路自检: 地面站/雷达里程计/相机/视觉跟踪/电磁铁/显示, 跳过飞控桥
  --fcu PATH          飞控串口 (默认 /dev/ttyUSB0)
  --h7 PATH           H7 GPIO (电磁铁/激光) 串口 (默认 /dev/ttyUSB1)
  --lidar-ip IP       MID360 雷达 IP (默认 192.168.1.3, dry-run 不可达则跳过定位链路)
  --mission PATH      任务配置 YAML
  --profile PATH      场地配置 YAML
  --calibration PATH  标定文件 YAML (实飞必须 CALIBRATED)
  --camera-plan PATH  相机计划 JSON (首次需运行标定生成)
  --keystore PATH     SROS2 keystore 目录 (默认 ./keystore)
  --enable-display    强制启用任务画面输出
  --no-display        禁用任务画面输出
  -h, --help          显示帮助

实飞门控 (语义完全按 drone/ 工具):
  启动开关: AUX6 (第10通道) >1700us    (drone/monitor_aux6.py, AUX1~AUX5 不可用)
  程控门控: AUX1 (第5通道) 1400~1600us (fcu_bridge 模式2 位置控制)
  硬锁    : AUX1 (第5通道) >=1800us    (fcu_bridge 紧急停止)

地面站控制: HMI 的 TASK_SELECTION 帧含 mode 字节 (1=实飞, 2=模拟飞),
与当前启动模式不匹配时 bridge 会拒绝并回执原因。
EOF
            exit 0
            ;;
        *) echo "未知参数: $1（运行 ./task3.sh --help 查看帮助）"; exit 1 ;;
    esac
done

case "$MODE" in
    real|sim) ;;
    *) echo "错误: --mode 仅支持 real 或 sim"; exit 1 ;;
esac

# ─── 前置检查 ───────────────────────────────────────────────────────────────
if [[ -z "$ENABLE_DISPLAY" ]]; then
    if [[ -n "${DISPLAY:-}" ]] || [[ -n "${WAYLAND_DISPLAY:-}" ]]; then
        ENABLE_DISPLAY="--enable-display"
        echo "检测到显示设备，已自动启用任务画面输出"
    else
        echo "未检测到显示设备，画面输出将以日志形式记录"
    fi
fi

check_file() {
    local name="$1" path="$2"
    [[ -f "$path" ]] || { echo "错误: $name 不存在: $path"; exit 1; }
}
check_dir() {
    local name="$1" path="$2"
    [[ -d "$path" ]] || { echo "错误: $name 不存在: $path"; exit 1; }
}

check_file "任务配置"      "$MISSION_CONFIG"
check_file "场地配置"      "$FIELD_PROFILE"
check_file "标定文件"      "$CALIBRATION"
check_file "HMAC 密钥"     "$HMAC_KEY"
check_file "MID360 manifest" "$MID360_DRIVER"
check_file "FAST-LIO launch" "$FAST_LIO"
check_dir  "SROS2 keystore" "$ROS_SECURITY_KEYSTORE"

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

if [[ "$MODE" == "real" ]]; then
    # 实飞强制要求标定状态 CALIBRATED (与 runner 校验一致, 提前报错)
    if ! grep -Eq '^calibration_status:[[:space:]]*["'"'"']?CALIBRATED["'"'"']?([[:space:]]|$)' "$CALIBRATION" 2>/dev/null; then
        echo "错误: 实飞模式要求标定文件状态为 CALIBRATED: $CALIBRATION"
        exit 1
    fi
    if ! [[ -e "$FCU_SERIAL" ]]; then
        echo "警告: 飞控串口不存在: $FCU_SERIAL (将启动但 fcu_bridge 会持续重试)"
    fi
fi

# keystore 需要被子进程 (ros2 launch) 看到
export ROS_SECURITY_KEYSTORE

# ─── 执行 ───────────────────────────────────────────────────────────────────
if [[ "$MODE" == "sim" ]]; then
    # 模拟飞: no_car_sim 应答飞行指令, executor simulation_only, 飞机不会动
    echo "模拟飞模式: no_car_sim + simulation_only (地面站选择即启动)"
    exec "${REPO_ROOT}/tools/run_no_car_mode.sh" \
        --mission "$MISSION_CONFIG" \
        --profile "$FIELD_PROFILE" \
        --calibration "$CALIBRATION" \
        --hmac-key "$HMAC_KEY"
fi

echo "实飞模式: fcu_bridge + CALIBRATED + SROS2 Enforce"
if [[ "$IMMEDIATE_START" == "true" ]]; then
    echo "  门控: --immediate-start (跳过小车 START / AUX 等待, 地面站选择即启动)"
else
    echo "  门控: 等待小车 START + AUX6(第10通道 >1700us) 启动, AUX1(第5通道 1400~1600us) 程控解锁"
fi

exec "${REPO_ROOT}/tools/run_task3_flight_test.sh" \
    $DRY_RUN \
    --mission-config "$MISSION_CONFIG" \
    --field-profile "$FIELD_PROFILE" \
    --calibration "$CALIBRATION" \
    --camera-runtime-plan "$CAMERA_PLAN" \
    --fcu-serial "$FCU_SERIAL" \
    --h7-serial-port "$H7_SERIAL" \
    --lidar-ip "$LIDAR_IP" \
    --hmac-key-file "$HMAC_KEY" \
    --mid360-driver-config "$MID360_DRIVER" \
    --fast-lio-launch "$FAST_LIO" \
    --task3-identity "$TASK3_IDENTITY" \
    --immediate-start "$IMMEDIATE_START" \
    $ENABLE_DISPLAY
