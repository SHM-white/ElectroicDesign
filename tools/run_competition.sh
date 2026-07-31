#!/usr/bin/env bash
# ==============================================================================
# ED UAV 竞赛全链路一键启动
#
# 组合链路 (完整闭环):
#   地面站确认 ──UDP──▶ vehicle_bridge ──▶ mission_executor (D-task 行为树)
#   mission_executor ──▶ fcu_bridge (飞控串口) / localization / perception
#   h7_gpio_bridge (电磁铁/激光) ◀── /h7_gpio/command
#   结束降落 → 恢复空闲 → 等待下一次地面站确认
#
# 用法:
#   ./tools/run_competition.sh                    # 默认启动完整链路
#   ./tools/run_competition.sh --sim              # + 比赛模拟器 (模拟 CAR/HMI 发包)
#   ./tools/run_competition.sh --simulation       # 仿真模式 (simulation_only=true)
#   ./tools/run_competition.sh --build            # 先构建再启动
#   ./tools/run_competition.sh --flight           # 启用飞控指令 (需 SROS2 keystore)
#   ./tools/run_competition.sh --no-h7            # 不启动 H7 GPIO 桥
#   ./tools/run_competition.sh --no-fcu           # 不启动飞控桥 (调试链路用)
#   ./tools/run_competition.sh --no-hotspot       # 不启动热点 (已有网络时)
#   ./tools/run_competition.sh --mission PATH     # 指定任务配置文件
#   ./tools/run_competition.sh --profile PATH     # 指定场地配置文件
#   ./tools/run_competition.sh --calibration PATH # 指定标定文件
#   ./tools/run_competition.sh --image-topic NAME # perception 图像 topic
#   ./tools/run_competition.sh --help
# ==============================================================================

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
REPO_ROOT="$(pwd)"

# ─── 颜色 ───────────────────────────────────────────────────────────────────
R='\033[0;31m'; G='\033[0;32m'; Y='\033[0;33m'; C='\033[0;36m'; B='\033[1m'; N='\033[0m'
ok()   { echo -e "${G}[OK]${N}  $*"; }
warn() { echo -e "${Y}[!!]${N}  $*"; }
fail() { echo -e "${R}[ERR]${N} $*" >&2; }
die()  { fail "$*"; exit 1; }

# ─── 默认参数 ───────────────────────────────────────────────────────────────
DO_BUILD=0
SIMULATION_ONLY=false
WITH_H7=1
WITH_FCU=1
WITH_HOTSPOT=1
WITH_SIM=0
ENABLE_FLIGHT_COMMANDS=false
MISSION_CONFIG=""
PROFILE_PATH=""
CALIBRATION_FILE=""
IMAGE_TOPIC=""
CAMERA_INFO_TOPIC=""

# ─── 参数解析 ───────────────────────────────────────────────────────────────
usage() {
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --simulation) SIMULATION_ONLY=true; shift ;;
        --sim)        WITH_SIM=1; shift ;;
        --flight)     ENABLE_FLIGHT_COMMANDS=true; shift ;;
        --build)      DO_BUILD=1; shift ;;
        --no-h7)      WITH_H7=0; shift ;;
        --no-fcu)     WITH_FCU=0; shift ;;
        --no-hotspot) WITH_HOTSPOT=0; shift ;;
        --mission)    MISSION_CONFIG="$2"; shift 2 ;;
        --profile)    PROFILE_PATH="$2"; shift 2 ;;
        --calibration) CALIBRATION_FILE="$2"; shift 2 ;;
        --image-topic) IMAGE_TOPIC="$2"; shift 2 ;;
        --camera-info-topic) CAMERA_INFO_TOPIC="$2"; shift 2 ;;
        -h|--help)    usage ;;
        *) die "未知参数: $1 (运行 $0 --help 查看帮助)" ;;
    esac
done

# ─── 前置检查 ───────────────────────────────────────────────────────────────
check_ros() {
    if [[ -z "${ROS_DISTRO:-}" ]]; then
        source /opt/ros/humble/setup.bash 2>/dev/null \
            || die "ROS 2 Humble 未找到"
    fi
    source "$REPO_ROOT/ros2_ws/install/setup.bash" 2>/dev/null \
        || warn "install/setup.bash 未找到, 请先运行 --build 或 colcon build"
}

check_hardware() {
    if [[ "$WITH_FCU" -eq 1 && ! -e /dev/ttyUSB0 ]]; then
        warn "/dev/ttyUSB0 未找到, 飞控桥可能无法连接"
    fi
    if [[ "$WITH_H7" -eq 1 && ! -e /dev/ttyUSB1 ]]; then
        warn "/dev/ttyUSB1 未找到, H7 GPIO 桥可能无法连接"
    fi
}

# ─── 构建 ───────────────────────────────────────────────────────────────────
do_build() {
    ok "构建 ROS 2 工作空间 ..."
    source /opt/ros/humble/setup.bash
    (cd "$REPO_ROOT/ros2_ws" && colcon build --symlink-install \
        --packages-select ed_uav_interfaces ed_uav_fcu_bridge \
        ed_uav_vehicle_bridge ed_uav_mission ed_uav_bringup ed_uav_localization \
        2>&1 | tail -20)
    source "$REPO_ROOT/ros2_ws/install/setup.bash"
    ok "构建完成"
}

# ─── 热点 ───────────────────────────────────────────────────────────────────
start_hotspot() {
    [[ "$WITH_HOTSPOT" -eq 1 ]] || { ok "跳过热点 (--no-hotspot)"; return; }
    if command -v nmcli >/dev/null && nmcli -t -f NAME connection show --active 2>/dev/null | grep -q "^ed-hotspot:"; then
        ok "热点已在运行"
        return
    fi
    warn "热点未运行。启动:"
    warn "  sudo ./tools/ed_comm.sh setup && sudo ./tools/ed_comm.sh"
    warn "或手动创建热点 (SSID: ED-UAV, 子网 192.168.20.0/24)"
}

# ─── 组合启动参数 ───────────────────────────────────────────────────────────
build_launch_args() {
    local args=()
    [[ -n "$MISSION_CONFIG" ]] && args+=("mission_config_path:=$MISSION_CONFIG")
    [[ -n "$PROFILE_PATH" ]] && args+=("profile_path:=$PROFILE_PATH")
    [[ -n "$CALIBRATION_FILE" ]] && args+=("calibration_file:=$CALIBRATION_FILE")
    [[ -n "$IMAGE_TOPIC" ]] && args+=("image_topic:=$IMAGE_TOPIC")
    [[ -n "$CAMERA_INFO_TOPIC" ]] && args+=("camera_info_topic:=$CAMERA_INFO_TOPIC")
    args+=("simulation_only:=$SIMULATION_ONLY")
    args+=("enable_flight_commands:=$ENABLE_FLIGHT_COMMANDS")
    printf '%s ' "${args[@]}"
}

# ─── 主流程 ─────────────────────────────────────────────────────────────────
main() {
    check_ros
    check_hardware
    [[ "$DO_BUILD" -eq 1 ]] && do_build

    echo ""
    echo -e "${C}══════════════════════════════════════════════════════════${N}"
    echo -e "${C}  ED UAV 竞赛全链路启动${N}"
    echo -e "${C}══════════════════════════════════════════════════════════${N}"
    echo -e "  simulation_only : ${B}$SIMULATION_ONLY${N}"
    echo -e "  vehicle_bridge  : ${G}启用${N} (UDP 192.168.20.1:42000)"
    echo -e "  mission_executor: ${G}启用${N} (D-task 行为树)"
    echo -e "  localization    : ${G}启用${N} (source_supervisor + field_anchor)"
    echo -e "  perception      : ${G}启用${N} (target_observation)"
    [[ "$WITH_FCU" -eq 1 ]] && echo -e "  fcu_bridge      : ${G}启用${N} (/dev/ttyUSB0 @ 500000, flight=$ENABLE_FLIGHT_COMMANDS)" \
                            || echo -e "  fcu_bridge      : ${Y}跳过 (--no-fcu)${N}"
    [[ "$WITH_H7" -eq 1 ]] && echo -e "  h7_gpio_bridge  : ${G}启用${N} (/dev/ttyUSB1 @ 115200)" \
                            || echo -e "  h7_gpio_bridge  : ${Y}跳过 (--no-h7)${N}"
    [[ "$WITH_SIM" -eq 1 ]] && echo -e "  sim_competition : ${G}启用${N} (模拟 CAR/HMI)" \
                             || echo -e "  sim_competition : ${Y}关闭 (--sim 启用)${N}"
    echo -e "${C}══════════════════════════════════════════════════════════${N}"
    echo ""

    start_hotspot

    # ── 比赛模拟器 (模拟 CAR/HMI 发包, 20Hz 遥测 + 选题) ──
    SIM_PID=""
    if [[ "$WITH_SIM" -eq 1 ]]; then
        ok "启动比赛模拟器 (模拟 CAR/HMI) ..."
        python3 "$REPO_ROOT/tools/sim_competition.py" \
            --key-file "$REPO_ROOT/config/hmac.key.hex" \
            --task 0 --duration 0 &
        SIM_PID=$!
        sleep 1
    fi
    cleanup() {
        if [[ -n "$SIM_PID" ]]; then
            kill "$SIM_PID" 2>/dev/null || true
            wait "$SIM_PID" 2>/dev/null || true
            ok "比赛模拟器已停止"
        fi
    }
    trap cleanup EXIT INT TERM

    local launch_args
    launch_args="$(build_launch_args)"

    ok "启动完整链路 (Ctrl+C 停止) ..."
    echo "  参数: $launch_args"

    # 由 launch 文件内部编排全部节点
    # shellcheck disable=SC2086
    ros2 launch ed_uav_bringup full_competition.launch.py $launch_args
}

main "$@"
