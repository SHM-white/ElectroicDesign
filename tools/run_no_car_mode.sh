#!/usr/bin/env bash
# ==============================================================================
# 无小车模式 (No-Car Mode) 一键启动
#
# 流程: 地面站发送 TASK 指令 → vehicle_bridge 直接派发任务 → mission_executor
#       以 simulation_only 运行, 模拟调用各飞行模块 (no_car_sim 应答所有
#       FlightCommand 并发布模拟飞控/定位状态)。
#
# 用法:
#   ./tools/run_no_car_mode.sh              # 前台运行 (Ctrl+C 清理)
#   ./tools/run_no_car_mode.sh --daemon     # 后台守护运行 (guardian 托管 bridge)
#
# 环境变量:
#   ROS_SECURITY_KEYSTORE   (可选) SROS2 keystore; 无小车模式默认不启用安全层
#   ED_LOG_DIR              日志目录 (默认 /var/log/ed-uav, 不可写则 /tmp/ed-uav-guardian)
#   ED_BIND_HOST            bridge 绑定地址 (默认 0.0.0.0)
#   ED_BIND_PORT / ED_HMI_PEER_PORT  端口覆盖 (默认 42000/42002, 端口冲突时可改)
#   ED_CAR_PEER_HOST / ED_HMI_PEER_HOST  对端地址覆盖 (默认 192.168.20.2/.3, 本地测试可设 127.0.0.1)
#   ED_ROS_DOMAIN_ID        ROS_DOMAIN_ID (默认继承环境; 隔离测试可设独立 domain)
# ==============================================================================

set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ─── 自动 source ROS2 workspace ────────────────────────────────────────────
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

# ─── 默认配置 (与 task3.sh / install_boot.sh 一致) ─────────────────────────
MISSION_CONFIG="${REPO_ROOT}/ros2_ws/src/ed_uav_mission/config/missions/d_arena_stability_test.yaml"
FIELD_PROFILE="${REPO_ROOT}/ros2_ws/src/ed_uav_localization/config/fields/d_arena_2026.yaml"
# simulation_only 要求 SYNTHETIC 标定; 真实 CALIBRATED 标定会被 preflight 拒绝
CALIBRATION="${REPO_ROOT}/ros2_ws/src/ed_uav_description/config/synthetic_calibrated.yaml"
HMAC_KEY="${REPO_ROOT}/config/hmac.key.hex"
PAYLOAD_CONFIG="${REPO_ROOT}/ros2_ws/src/ed_uav_mission/config/payload_adapter.yaml"
TASK3_IDENTITY="task3-stability-2026"
TASK3_MISSION_PROFILE_ID="task3-stability"
TASK3_DEPLOYMENT_PRESET_ID="field-2026"
TASK3_TARGET_REVISION="d2026-apriltag-v1"
FIELD_PROFILE_ID="$(grep -m1 '^profile_id:' "$FIELD_PROFILE" | awk '{print $2}')"
NUC_IP="192.168.20.1"
CAR_IP="${ED_CAR_PEER_HOST:-192.168.20.2}"
HMI_IP="${ED_HMI_PEER_HOST:-192.168.20.3}"
BIND_HOST="${ED_BIND_HOST:-0.0.0.0}"
BIND_PORT="${ED_BIND_PORT:-42000}"
HMI_PORT="${ED_HMI_PEER_PORT:-42002}"

DAEMON=""
LOG_DIR="${ED_LOG_DIR:-/var/log/ed-uav}"
mkdir -p "$LOG_DIR" 2>/dev/null || LOG_DIR="/tmp/ed-uav-guardian"

# ─── 参数解析 ───────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --daemon)        DAEMON="1"; shift ;;
        --mission)       MISSION_CONFIG="$2"; shift 2 ;;
        --profile)       FIELD_PROFILE="$2"; shift 2 ;;
        --calibration)   CALIBRATION="$2"; shift 2 ;;
        --hmac-key)      HMAC_KEY="$2"; shift 2 ;;
        --payload-config) PAYLOAD_CONFIG="$2"; shift 2 ;;
        --log-dir)       LOG_DIR="$2"; shift 2 ;;
        -h|--help)
            cat <<'EOF'
无小车模式一键启动

用法: ./tools/run_no_car_mode.sh [选项]

选项:
  --daemon            后台守护运行 (guardian 托管 vehicle_bridge)
  --mission PATH      任务配置 YAML (默认 d_arena_stability_test.yaml)
  --profile PATH      场地配置 YAML (默认 d_arena_2026.yaml)
  --calibration PATH  标定文件 (默认 synthetic_calibrated.yaml, simulation_only 必需 SYNTHETIC)
  --hmac-key PATH     HMAC 密钥文件 (默认 config/hmac.key.hex)
  --payload-config    载荷配置 YAML
  --log-dir DIR       日志目录
  -h, --help          显示帮助

流程: 地面站 TASK 指令 → bridge(no_car_mode) 直接派发 → executor(simulation_only)
      → no_car_sim 模拟应答飞行模块调用
EOF
            exit 0
            ;;
        *) echo "未知参数: $1 (运行 $0 --help 查看帮助)"; exit 1 ;;
    esac
done

# ─── 前置检查 ───────────────────────────────────────────────────────────────
for required in "$MISSION_CONFIG" "$FIELD_PROFILE" "$CALIBRATION" "$HMAC_KEY" "$PAYLOAD_CONFIG"; do
    [[ -f "$required" ]] || { echo "错误: 文件不存在: $required"; exit 1; }
done

mkdir -p "$LOG_DIR" 2>/dev/null || { echo "警告: 无法创建日志目录 $LOG_DIR, 改用 /tmp/ed-uav-guardian"; LOG_DIR="/tmp/ed-uav-guardian"; mkdir -p "$LOG_DIR"; }

# ─── 启动命令定义 ───────────────────────────────────────────────────────────
# 子进程可能是非登录 shell (guardian/systemd), 必须显式 source 两层环境
ROS_DOMAIN_LINE=""
if [[ -n "${ED_ROS_DOMAIN_ID:-}" ]]; then
    ROS_DOMAIN_LINE="export ROS_DOMAIN_ID=${ED_ROS_DOMAIN_ID}; "
fi
ROS_SETUP_LINE="${ROS_DOMAIN_LINE}source /opt/ros/humble/setup.bash; source ${_ros2_ws_setup:-/opt/ros/humble/setup.bash}"

NO_CAR_SIM_CMD="ros2 run ed_uav_bringup no_car_sim --ros-args -p state_rate_hz:=10.0"

EXECUTOR_CMD="ros2 run ed_uav_mission mission_executor \
  --ros-args \
  -p profile_path:=${FIELD_PROFILE} \
  -p mission_config_path:=${MISSION_CONFIG} \
  -p calibration_file:=${CALIBRATION} \
  -p simulation_only:=true \
  -p payload_config_path:=${PAYLOAD_CONFIG} \
  -p task3_mission_profile_id:=${TASK3_MISSION_PROFILE_ID} \
  -p task3_deployment_preset_id:=${TASK3_DEPLOYMENT_PRESET_ID} \
  -p task3_target_revision:=${TASK3_TARGET_REVISION} \
  -r /vehicle/telemetry:=/d_task/vehicle/telemetry \
  -r /mission/status:=/d_task/mission_status \
  -r /mission/select_d_task:=/d_task/pre_arm/select_mission"

BRIDGE_CMD="ros2 run ed_uav_vehicle_bridge vehicle_bridge \
  --ros-args \
  -p bind_host:=${BIND_HOST} -p bind_port:=${BIND_PORT} \
  -p car_peer_host:=${CAR_IP} -p car_peer_port:=42001 \
  -p hmi_peer_host:=${HMI_IP} -p hmi_peer_port:=${HMI_PORT} \
  -p car_sender_id:=1128419121 -p hmi_sender_id:=1213024561 -p bridge_sender_id:=1381122353 \
  -p hmac_key_file:=${HMAC_KEY} \
  -p mission_timeout_seconds:=90.0 -p telemetry_stale_seconds:=0.75 \
  -p task3_flight_test_mode:=true \
  -p no_car_mode:=true \
  -p task3_mission_id:=${TASK3_IDENTITY} \
  -p task3_field_profile_id:=${FIELD_PROFILE_ID} \
  -p task3_mission_profile_id:=${TASK3_MISSION_PROFILE_ID} \
  -p task3_deployment_preset_id:=${TASK3_DEPLOYMENT_PRESET_ID} \
  -p task3_target_revision:=${TASK3_TARGET_REVISION} \
  -p task3_timeout_seconds:=120.0"

# ─── 启动 ───────────────────────────────────────────────────────────────────
echo "══════════════════════════════════════════════════════════"
echo "  无小车模式 (No-Car Mode)"
echo "  mission:  ${MISSION_CONFIG}"
echo "  profile:  ${FIELD_PROFILE} (id=${FIELD_PROFILE_ID})"
echo "  sim:      no_car_sim + mission_executor(simulation_only)"
echo "  bridge:   no_car_mode=true 地面站 TASK 直接开始任务"
echo "  日志:     ${LOG_DIR}"
echo "══════════════════════════════════════════════════════════"

if [[ -n "$DAEMON" ]]; then
    mkdir -p "$LOG_DIR"
    setsid bash -lc "$ROS_SETUP_LINE && exec $NO_CAR_SIM_CMD" >> "$LOG_DIR/no_car_sim.log" 2>&1 &
    echo "no_car_sim  pid=$! 日志=$LOG_DIR/no_car_sim.log"
    sleep 1
    setsid bash -lc "$ROS_SETUP_LINE && exec $EXECUTOR_CMD" >> "$LOG_DIR/mission_executor.log" 2>&1 &
    echo "executor    pid=$! 日志=$LOG_DIR/mission_executor.log"
    sleep 2
ED_GUARDIAN_WATCH_NAME="vehicle_bridge" \
ED_GUARDIAN_START_CMD="$ROS_SETUP_LINE && $BRIDGE_CMD" \
    setsid bash "$REPO_ROOT/tools/ed_guardian.sh" >> "$LOG_DIR/guardian.log" 2>&1 &
    echo "guardian    pid=$! 日志=$LOG_DIR/guardian.log"
    echo "完成: 地面站向 ${HMI_IP}:42002 发送 TASK 指令即可开始任务"
    exit 0
fi

# ─── 前台模式 ───────────────────────────────────────────────────────────────
GUARDIAN_PID=""
WATCH_PID_FILE="$LOG_DIR/vehicle_bridge.pid"

_cleanup() {
    echo ""
    echo "清理无小车模式进程..."
    [[ -n "$GUARDIAN_PID" ]] && kill "$GUARDIAN_PID" 2>/dev/null || true
    [[ -f "$WATCH_PID_FILE" ]] && kill "$(cat "$WATCH_PID_FILE")" 2>/dev/null || true
    pkill -f "ed_uav_bringup no_car_sim" 2>/dev/null || true
    pkill -f "ed_uav_mission mission_executor" 2>/dev/null || true
}
trap _cleanup EXIT INT TERM

bash -lc "$ROS_SETUP_LINE && exec $NO_CAR_SIM_CMD" >> "$LOG_DIR/no_car_sim.log" 2>&1 &
echo "no_car_sim  pid=$! 日志=$LOG_DIR/no_car_sim.log"
sleep 1

bash -lc "$ROS_SETUP_LINE && exec $EXECUTOR_CMD" >> "$LOG_DIR/mission_executor.log" 2>&1 &
echo "executor    pid=$! 日志=$LOG_DIR/mission_executor.log"
sleep 2

ED_GUARDIAN_LOG_DIR="$LOG_DIR" \
ED_GUARDIAN_WATCH_NAME="vehicle_bridge" \
ED_GUARDIAN_START_CMD="$ROS_SETUP_LINE && $BRIDGE_CMD" \
    bash "$REPO_ROOT/tools/ed_guardian.sh" &
GUARDIAN_PID=$!
echo "guardian    pid=$GUARDIAN_PID (托管 vehicle_bridge, 崩溃自动拉起)"
echo ""
echo "地面站向 ${HMI_IP}:42002 发送 TASK 指令即可开始任务 (Ctrl+C 退出)"
echo "实时日志: tail -f $LOG_DIR/vehicle_bridge.log $LOG_DIR/mission_executor.log"
wait "$GUARDIAN_PID"
