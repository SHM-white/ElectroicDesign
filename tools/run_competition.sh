#!/usr/bin/env bash
# Unified entry point for either the real D-task chain or the hardware-free Gazebo loop.
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly workspace="$repo_root/ros2_ws"

# ── 模式开关：0=容器（默认），1=本机 ROS2 ──
FORCE_NATIVE=${FORCE_NATIVE:-0}
# ── 强制使用容器，即使本机有 ROS2 ──
HUMBLE_FORCE_CONTAINER=${HUMBLE_FORCE_CONTAINER:-0}

mode="real"
do_build=0
with_hotspot=1
hotspot_only=0
with_fcu=1
with_h7=1
display="auto"
auto_start=true
simulation_task=1
mission_config=""
profile_path=""
calibration_file=""
camera_plan="$repo_root/calibration_data/camera_runtime_plan.local.json"
hmac_key="$repo_root/config/hmac.key.hex"
lidar_manifest="$repo_root/ros2_ws/src/ed_uav_lidar/config/fields/mid360_field_manifest.local.json"
lidar_ip="192.168.1.3"
fcu_serial="${FCU_SERIAL_PORT:-/dev/ttyUSB0}"
h7_serial="${H7_SERIAL_PORT:-/dev/ttyUSB1}"
vehicle_bind_host="192.168.20.1"
network_host=0

usage() {
    cat <<'EOF'
用法: tools/run_competition.sh [选项]

  --simulation, --sim, --dry-run  运行 Gazebo D 场地纯仿真闭环
  --real                         运行真实 HMI/小车/NUC/飞控链路（默认）
  --build                        启动前构建完整依赖包
  --task 1|2                     仿真自动选择投放/动平台降落任务（默认 1）
  --manual-start                 仿真不自动选题、解锁和发车
  --enable-display               打开 Gazebo GUI 与 RViz/实机 HUD
  --no-display                   无 GUI 运行
  --mission PATH                 覆盖任务 YAML
  --profile PATH                 覆盖场地 YAML
  --calibration PATH             覆盖实机标定 YAML
  --hmac-key PATH                实机 UDP HMAC 密钥（至少 32 字节十六进制）
  --lidar-manifest PATH          实机 MID-360 本地清单
  --no-hotspot | --hotspot-only  禁用热点或仅启动热点
  --no-fcu | --no-h7             明确跳过对应实机桥
  --fcu PATH | --h7 PATH         指定串口
  --network-host                 Docker 使用 host 网络（便于 rqt 调试）
  --force-container              强制使用 Docker 容器（即使本机有 ROS2）

环境变量:
  FORCE_NATIVE=1                 强制使用本机 ROS2（需已安装 Humble）
  HUMBLE_FORCE_CONTAINER=1       强制使用 Docker 容器
  HUMBLE_NETWORK=host            Docker 使用 host 网络
EOF
}

die() { printf '[ERR] %s\n' "$*" >&2; exit 1; }
note() { printf '[ED] %s\n' "$*"; }

require_value() {
    [[ $# -ge 2 && -n "$2" ]] || die "$1 需要一个值"
}

while (($#)); do
    case "$1" in
        --simulation|--sim|--dry-run) mode="simulation"; with_hotspot=0; shift ;;
        --real) mode="real"; shift ;;
        --build) do_build=1; shift ;;
        --task) require_value "$@"; simulation_task="$2"; shift 2 ;;
        --manual-start) auto_start=false; shift ;;
        --enable-display) display="true"; shift ;;
        --no-display) display="false"; shift ;;
        --mission) require_value "$@"; mission_config="$2"; shift 2 ;;
        --profile) require_value "$@"; profile_path="$2"; shift 2 ;;
        --calibration) require_value "$@"; calibration_file="$2"; shift 2 ;;
        --camera-plan) require_value "$@"; camera_plan="$2"; shift 2 ;;
        --hmac-key) require_value "$@"; hmac_key="$2"; shift 2 ;;
        --lidar-manifest) require_value "$@"; lidar_manifest="$2"; shift 2 ;;
        --lidar-ip) require_value "$@"; lidar_ip="$2"; shift 2 ;;
        --fcu) require_value "$@"; fcu_serial="$2"; shift 2 ;;
        --h7) require_value "$@"; h7_serial="$2"; shift 2 ;;
        --vehicle-bind-host) require_value "$@"; vehicle_bind_host="$2"; shift 2 ;;
        --no-fcu) with_fcu=0; shift ;;
        --no-h7) with_h7=0; shift ;;
        --no-hotspot) with_hotspot=0; shift ;;
        --hotspot-only) hotspot_only=1; shift ;;
        --network-host) network_host=1; shift ;;
        --force-container) HUMBLE_FORCE_CONTAINER=1; shift ;;
        --flight|--immediate-start) note "$1 已不再需要：普通指令无额外软件安全锁"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "未知参数: $1" ;;
    esac
done

[[ "$simulation_task" == 1 || "$simulation_task" == 2 ]] \
    || die "--task 仅接受 1（投放）或 2（动平台降落）"

native_humble=0
if ((FORCE_NATIVE)) && [[ -r /opt/ros/humble/setup.bash ]] && [[ "$HUMBLE_FORCE_CONTAINER" != 1 ]]; then
    native_humble=1
fi

if [[ "$display" == auto ]]; then
    if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
        display=true
    else
        display=false
    fi
fi

hotspot_ensure() {
    local connection="ed-hotspot"
    command -v nmcli >/dev/null 2>&1 || die "未安装 nmcli；请用 --no-hotspot 或先配置网络"
    if nmcli -t -f NAME,TYPE connection show --active 2>/dev/null | grep -q "^${connection}:"; then
        note "热点 ${connection} 已运行"
        return
    fi
    [[ -x "$repo_root/tools/ed_comm.sh" ]] || die "缺少 tools/ed_comm.sh"
    note "启动热点 ${connection}"
    sudo "$repo_root/tools/ed_comm.sh" setup
}

if ((hotspot_only)); then
    hotspot_ensure
    exit 0
fi

if ((do_build)); then
    note "构建完整 ROS/Gazebo 工作空间"
    HUMBLE_FORCE_CONTAINER="$HUMBLE_FORCE_CONTAINER" bash "$repo_root/tools/build_sim_packages.sh"
fi

runtime_path() {
    local requested="$1"
    if ((native_humble)); then
        printf '%s\n' "$requested"
    elif [[ "$requested" == "$repo_root"/* ]]; then
        printf '/workspace/%s\n' "${requested#"$repo_root"/}"
    else
        die "容器模式只能访问仓库内路径: $requested"
    fi
}

run_ros() {
    local gui="$1"
    shift
    local -a command=("$@")
    if ((native_humble)); then
        exec bash -lc '
            set -euo pipefail
            set +u
            source /opt/ros/humble/setup.bash
            set -u
            ws="$1"
            shift
            [[ -r "$ws/install/setup.bash" ]] || { echo "缺少 install/setup.bash，请加 --build" >&2; exit 2; }
            set +u
            source "$ws/install/setup.bash"
            set -u
            exec "$@"
        ' bash "$workspace" "${command[@]}"
    fi
    local network="${HUMBLE_NETWORK:-}"
    if ((network_host)); then
        network="host"
    fi
    HUMBLE_GUI="$gui" HUMBLE_INTERACTIVE=1 HUMBLE_NETWORK="$network" HUMBLE_FORCE_CONTAINER="$HUMBLE_FORCE_CONTAINER" exec bash "$repo_root/tools/run_humble.sh" bash -lc '
        set -eo pipefail
        set +u
        source /opt/ros/humble/setup.bash
        set -u
        ws="$1"
        shift
        [[ -r "$ws/install/setup.bash" ]] || { echo "缺少 install/setup.bash，请加 --build" >&2; exit 2; }
        set +u
        source "$ws/install/setup.bash"
        set -u
        exec "$@"
    ' bash /workspace/ros2_ws "${command[@]}"
}

if [[ "$mode" == simulation ]]; then
    sim_mission="${mission_config:-$repo_root/ros2_ws/src/ed_uav_mission/config/missions/d_arena_competition.yaml}"
    sim_profile="${profile_path:-$repo_root/ros2_ws/src/ed_uav_localization/config/fields/d_arena_2026.yaml}"
    [[ -f "$sim_mission" ]] || die "任务文件不存在: $sim_mission"
    [[ -f "$sim_profile" ]] || die "场地文件不存在: $sim_profile"
    note "启动 D 题纯仿真：地面真值定位、靶车与 AprilTag"
    run_ros "$([[ "$display" == true ]] && echo 1 || echo 0)" \
        ros2 launch ed_uav_gazebo sim.launch.py \
        "gui:=$display" "use_rviz:=$display" "auto_start:=$auto_start" \
        "simulation_task:=$simulation_task" \
        "localization_mode:=ground_truth" \
        "mission_config:=$(runtime_path "$sim_mission")" \
        "profile_path:=$(runtime_path "$sim_profile")"
fi

((native_humble)) || die "实机模式要求本机安装 ROS 2 Humble；容器网络/串口不会冒充实机链路"
[[ -f "$lidar_manifest" ]] || die "缺少 MID-360 本地清单；用 --lidar-manifest 指定"
[[ -f "$camera_plan" ]] || die "相机运行计划不存在: $camera_plan"
real_mission="${mission_config:-$repo_root/ros2_ws/src/ed_uav_mission/config/missions/d_arena_competition.yaml}"
real_profile="${profile_path:-$repo_root/ros2_ws/src/ed_uav_localization/config/fields/d_arena_2026.yaml}"
real_calibration="${calibration_file:-$repo_root/calibration_data/field_calibrated_v1.yaml}"
[[ -f "$real_mission" && -f "$real_profile" && -f "$real_calibration" ]] \
    || die "实机任务、场地或标定文件缺失"
((with_hotspot)) && hotspot_ensure
note "启动真实链路：HMI↔NUC↔小车、FAST-LIO、任务执行与 AUX1 一键紧急锁浆"
run_ros 0 ros2 launch ed_uav_bringup full_competition.launch.py \
    "mission_config_path:=$real_mission" "profile_path:=$real_profile" \
    "calibration_file:=$real_calibration" "camera_runtime_plan:=$camera_plan" \
    "hmac_key_file:=$hmac_key" "mid360_driver_config_path:=$lidar_manifest" \
    "lidar_ip:=$lidar_ip" "vehicle_bind_host:=$vehicle_bind_host" \
    "fcu_serial_port:=$fcu_serial" "h7_serial_port:=$h7_serial" \
    "enable_fcu:=$([[ $with_fcu -eq 1 ]] && echo true || echo false)" \
    "enable_h7:=$([[ $with_h7 -eq 1 ]] && echo true || echo false)" \
    "enable_display:=$display"
