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
#   ./tools/run_competition.sh                    # 默认启动完整链路 (热点+串口自动检测+双相机+雷达+电磁铁)
#   ./tools/run_competition.sh --sim              # + 比赛模拟器 (模拟 CAR/HMI 发包)
#   ./tools/run_competition.sh --simulation       # 仿真模式 (simulation_only=true)
#   ./tools/run_competition.sh --dry-run          # 非飞控全链路自检 (跳过飞控桥)
#   ./tools/run_competition.sh --immediate-start  # 跳过小车/AUX 门控, 地面站选择即启动
#   ./tools/run_competition.sh --build            # 先构建再启动
#   ./tools/run_competition.sh --flight           # 启用飞控指令 (需 SROS2 keystore)
#   ./tools/run_competition.sh --enable-display   # 可视化窗口 (默认自动检测)
#   ./tools/run_competition.sh --no-h7            # 不启动 H7 GPIO 桥
#   ./tools/run_competition.sh --no-fcu           # 不启动飞控桥 (= --dry-run)
#   ./tools/run_competition.sh --no-hotspot       # 不启动热点 (已有网络时)
#   ./tools/run_competition.sh --hotspot-only     # 仅管理热点后退出
#   ./tools/run_competition.sh --mission PATH     # 指定任务配置文件
#   ./tools/run_competition.sh --profile PATH     # 指定场地配置文件
#   ./tools/run_competition.sh --calibration PATH # 指定标定文件
#   ./tools/run_competition.sh --camera-plan PATH # 相机计划 JSON
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
DRY_RUN=""
IMMEDIATE_START="false"
ENABLE_DISPLAY=""
ENABLE_DISPLAY_EXPLICIT=""
NUC_IP="192.168.20.1"
HOTSPOT_CON_NAME="ed-hotspot"
ED_COMM="${REPO_ROOT}/tools/ed_comm.sh"
MISSION_CONFIG=""
PROFILE_PATH=""
CALIBRATION_FILE=""
CAMERA_PLAN=""
IMAGE_TOPIC=""
CAMERA_INFO_TOPIC=""
LIDAR_IP="192.168.1.3"
FCU_SERIAL="${FCU_SERIAL_PORT:-/dev/ttyUSB0}"
H7_SERIAL="${H7_SERIAL_PORT:-/dev/ttyUSB1}"
ROS_SECURITY_KEYSTORE="${ROS_SECURITY_KEYSTORE:-${REPO_ROOT}/keystore}"

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
        --dry-run)    DRY_RUN="dry_run:=true"; WITH_FCU=0; shift ;;
        --immediate-start) IMMEDIATE_START="true"; shift ;;
        --enable-display)  ENABLE_DISPLAY="enable_display:=true"; ENABLE_DISPLAY_EXPLICIT=1; shift ;;
        --no-display)  ENABLE_DISPLAY=""; ENABLE_DISPLAY_EXPLICIT=1; shift ;;
        --no-h7)      WITH_H7=0; shift ;;
        --no-fcu)     WITH_FCU=0; DRY_RUN="dry_run:=true"; shift ;;
        --no-hotspot) WITH_HOTSPOT=0; shift ;;
        --hotspot-only) WITH_HOTSPOT=2; shift ;;
        --mission)    MISSION_CONFIG="$2"; shift 2 ;;
        --profile)    PROFILE_PATH="$2"; shift 2 ;;
        --calibration) CALIBRATION_FILE="$2"; shift 2 ;;
        --camera-plan) CAMERA_PLAN="$2"; shift 2 ;;
        --lidar-ip)   LIDAR_IP="$2"; shift 2 ;;
        --fcu)        FCU_SERIAL="$2"; shift 2 ;;
        --h7)         H7_SERIAL="$2"; shift 2 ;;
        --image-topic) IMAGE_TOPIC="$2"; shift 2 ;;
        --camera-info-topic) CAMERA_INFO_TOPIC="$2"; shift 2 ;;
        -h|--help)    usage ;;
        *) die "未知参数: $1 (运行 $0 --help 查看帮助)" ;;
    esac
done

# ─── 前置检查 ───────────────────────────────────────────────────────────────
check_ros() {
    if [[ -z "${ROS_DISTRO:-}" ]]; then
        set +u
        source /opt/ros/humble/setup.bash 2>/dev/null \
            || die "ROS 2 Humble 未找到"
        set -u
    fi
    set +u
    source "$REPO_ROOT/ros2_ws/install/setup.bash" 2>/dev/null \
        || warn "install/setup.bash 未找到, 请先运行 --build 或 colcon build"
    set -u
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

# ─── 热点管理 (复用 tools/ed_comm.sh; 开机自启已禁用, 由本脚本按需管理) ──
hotspot_is_active() {
    nmcli -t -f NAME,TYPE connection show --active 2>/dev/null | grep -q "^${HOTSPOT_CON_NAME}:"
}
hotspot_ensure() {
    if hotspot_is_active; then
        ok "热点已在运行 (${HOTSPOT_CON_NAME})"
        return 0
    fi
    warn "热点未运行, 用 sudo 启动 ..."
    if [[ -x "$ED_COMM" ]]; then
        if sudo -n true 2>/dev/null; then
            sudo "$ED_COMM" setup
        else
            sudo "$ED_COMM" setup || die "热点启动失败: 请手动 sudo $ED_COMM setup"
        fi
    else
        die "热点工具缺失: $ED_COMM"
    fi
    hotspot_is_active && ok "热点已就绪" || warn "热点未确认运行"
}

# ─── 组合启动参数 ───────────────────────────────────────────────────────────
build_launch_args() {
    local args=()
    [[ -n "$MISSION_CONFIG" ]] && args+=("mission_config_path:=$MISSION_CONFIG")
    [[ -n "$PROFILE_PATH" ]] && args+=("profile_path:=$PROFILE_PATH")
    [[ -n "$CALIBRATION_FILE" ]] && args+=("calibration_file:=$CALIBRATION_FILE")
    [[ -n "$CAMERA_PLAN" ]] && args+=("camera_runtime_plan:=$CAMERA_PLAN")
    [[ -n "$IMAGE_TOPIC" ]] && args+=("image_topic:=$IMAGE_TOPIC")
    [[ -n "$CAMERA_INFO_TOPIC" ]] && args+=("camera_info_topic:=$CAMERA_INFO_TOPIC")
    args+=("simulation_only:=$SIMULATION_ONLY")
    args+=("enable_flight_commands:=$ENABLE_FLIGHT_COMMANDS")
    args+=("task3_immediate_start:=$IMMEDIATE_START")
    args+=("lidar_ip:=$LIDAR_IP")
    args+=("fcu_serial_port:=$FCU_SERIAL")
    args+=("h7_serial_port:=$H7_SERIAL")
    args+=("ros_security_keystore:=$ROS_SECURITY_KEYSTORE")
    # dry-run: 绑定 0.0.0.0 (无小车网卡时避免 Cannot assign requested address)
    if [[ -n "$DRY_RUN" ]]; then
        args+=("vehicle_bind_host:=0.0.0.0")
        args+=("ros_security_enable:=false")
        args+=("$DRY_RUN")
    else
        args+=("vehicle_bind_host:=$NUC_IP")
    fi
    [[ -n "$ENABLE_DISPLAY" ]] && args+=("$ENABLE_DISPLAY")
    printf '%s ' "${args[@]}"
}

# ─── 主流程 ─────────────────────────────────────────────────────────────────
# ─── 主流程 ─────────────────────────────────────────────────────────────────
main() {
    check_ros
    check_hardware
    [[ "$DO_BUILD" -eq 1 ]] && do_build

    # ── 热点专用命令 (不启动 ROS 链路) ──
    if [[ "$WITH_HOTSPOT" -eq 2 ]]; then
        ok "仅管理热点 (--hotspot-only)"
        hotspot_ensure
        ok "热点就绪, 退出"
        exit 0
    fi

    # 显示自动检测 (未显式指定时)
    if [[ -z "$ENABLE_DISPLAY_EXPLICIT" && -z "$ENABLE_DISPLAY" ]]; then
        if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
            ENABLE_DISPLAY="enable_display:=true"
            ok "检测到显示设备, 已自动启用可视化窗口"
        fi
    fi

    # 确保热点子网 192.168.20.x 可用 (CAR/HMI 通信)
    # dry-run 自动跳过热点 (无小车/地面站不需要)
    # if [[ -n "$DRY_RUN" && "$WITH_HOTSPOT" -eq 1 ]]; then
    #     ok "dry-run 模式: 跳过热点检查 (无小车/地面站)"
    # elif [[ "$WITH_HOTSPOT" -eq 1 ]]; then
        hotspot_ensure
    # fi

    echo ""
    echo -e "${C}══════════════════════════════════════════════════════════${N}"
    echo -e "${C}  ED UAV 竞赛全链路启动${N}"
    echo -e "${C}══════════════════════════════════════════════════════════${N}"
    echo -e "  simulation_only : ${B}$SIMULATION_ONLY${N}"
    echo -e "  vehicle_bridge  : ${G}启用${N} (UDP $NUC_IP:42000)"
    echo -e "  mission_executor: ${G}启用${N} (D-task 行为树)"
    echo -e "  localization    : ${G}启用${N} (source_supervisor + field_anchor)"
    echo -e "  perception      : ${G}启用${N} (双相机 AprilTag PnP)"
    echo -e "  MID360 里程计   : ${G}启用${N} (FAST-LIO → /localization/odom)"
    [[ "$WITH_FCU" -eq 1 ]] && echo -e "  fcu_bridge      : ${G}启用${N} ($FCU_SERIAL @ 500000, flight=$ENABLE_FLIGHT_COMMANDS)" \
                            || echo -e "  fcu_bridge      : ${Y}跳过 (--no-fcu)${N}"
    [[ "$WITH_H7" -eq 1 ]] && echo -e "  h7_gpio_bridge  : ${G}启用${N} ($H7_SERIAL @ 115200)" \
                            || echo -e "  h7_gpio_bridge  : ${Y}跳过 (--no-h7)${N}"
    [[ -n "$ENABLE_DISPLAY" ]] && echo -e "  可视化          : ${G}启用${N}" \
                                || echo -e "  可视化          : ${Y}禁用${N}"
    [[ "$WITH_SIM" -eq 1 ]] && echo -e "  sim_competition : ${G}启用${N} (模拟 CAR/HMI)" \
                             || echo -e "  sim_competition : ${Y}关闭 (--sim 启用)${N}"
    echo -e "${C}══════════════════════════════════════════════════════════${N}"
    echo ""

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

    local launch_args
    launch_args="$(build_launch_args)"

    ok "启动完整链路 (Ctrl+C 停止) ..."
    echo "  参数: $launch_args"

    # shellcheck disable=SC2086
    ros2 launch ed_uav_bringup full_competition.launch.py $launch_args &
    LAUNCH_PID=$!

    # Ctrl+C/TERM: 转发信号给 ros2 launch 并等待其完整回收子进程,
    # 否则 vehicle_bridge 等子节点残留占住 42000 端口
    _SHUTDOWN_REQUESTED=0
    _forward_and_wait() {
        local sig="$1"
        _SHUTDOWN_REQUESTED=1
        trap - INT TERM
        echo ""
        warn "收到 $sig, 正在关闭 ROS 链路 ..."
        kill -"$sig" "$LAUNCH_PID" 2>/dev/null || true
        local waited=0
        while kill -0 "$LAUNCH_PID" 2>/dev/null && [[ "$waited" -lt 15 ]]; do
            sleep 1
            waited=$((waited + 1))
        done
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            warn "launch 未及时退出, 强制清理子进程"
            pkill -9 -P "$LAUNCH_PID" 2>/dev/null || true
            kill -9 "$LAUNCH_PID" 2>/dev/null || true
        fi
        [[ -n "${SIM_PID:-}" ]] && kill "$SIM_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
        ok "ROS 链路已关闭"
        exit 0
    }
    trap '_forward_and_wait INT' INT
    trap '_forward_and_wait TERM' TERM

    # 用轮询代替 wait: wait 在子 shell (后台 & 调用) 中不被 SIGINT 中断,
    # 导致 trap 永远不触发, 所有子进程残留. 轮询 sleep 1 可被 trap 打断.
    EXIT_CODE=0
    while kill -0 "$LAUNCH_PID" 2>/dev/null; do
        sleep 1
    done
    wait "$LAUNCH_PID" 2>/dev/null && EXIT_CODE=0 || EXIT_CODE=$?
    [[ -n "${SIM_PID:-}" ]] && kill "$SIM_PID" 2>/dev/null || true
    exit "$EXIT_CODE"
}

main "$@"
