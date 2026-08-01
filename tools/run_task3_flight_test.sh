#!/usr/bin/env bash
# ==============================================================================
# Task3 Stability Flight Test Runner
#
# One-command bringup for Task3 stability test with AUX5 gating and hard lock.
# Requires: calibrated field profile, camera plan, SROS2 keystore, FCU serial.
#
# Usage:
#   ./tools/run_task3_flight_test.sh --dry-run [args]   # Validate and print
#   ./tools/run_task3_flight_test.sh [args]              # Launch
# ==============================================================================

set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ─── Colors ──────────────────────────────────────────────────────────────────
R='\033[0;31m'; G='\033[0;32m'; Y='\033[0;33m'; C='\033[0;36m'; B='\033[1m'; N='\033[0m'
ok()   { echo -e "${G}[OK]${N}  $*"; }
warn() { echo -e "${Y}[!!]${N}  $*"; }
fail() { echo -e "${R}[ERR]${N} $*" >&2; }
die()  { fail "$*"; exit 64; }

# ─── Defaults ────────────────────────────────────────────────────────────────
DRY_RUN=0
MISSION_CONFIG=""
FIELD_PROFILE=""
CALIBRATION_FILE=""
CAMERA_PLAN=""
FCU_SERIAL=""
HMAC_KEY_FILE=""
MID360_DRIVER_CONFIG=""
FAST_LIO_LAUNCH=""
TASK3_IDENTITY=""
ENABLE_DISPLAY="false"
ROS_SECURITY_KEYSTORE="${ROS_SECURITY_KEYSTORE:-}"

# ─── Argument parsing ───────────────────────────────────────────────────────
usage() {
    cat <<'EOF'
Usage: ./tools/run_task3_flight_test.sh [OPTIONS]

Task3 stability flight-test launcher with AUX5 gating and hard-lock emergency.

Required:
  --mission-config PATH        Task3 mission YAML
  --field-profile PATH         CALIBRATED field profile YAML
  --calibration PATH           CALIBRATED sensor calibration JSON
  --camera-runtime-plan PATH   Camera runtime plan JSON
  --fcu-serial PATH            FCU serial device path
  --hmac-key-file PATH         HMAC key hex file
  --mid360-driver-config PATH  MID-360 driver JSON config
  --fast-lio-launch PATH       FAST-LIO launch file path
  --task3-identity STR         Task3 mission identity

Environment:
  ROS_SECURITY_KEYSTORE        SROS2 keystore directory (required)

Flags:
  --dry-run                    Validate inputs, then launch all non-FCU modules
                               (ground station / lidar odometry / cameras /
                               visual tracking / electromagnet / display)
  --enable-display             Enable mission display window (auto-detects headless)
  -h, --help                   Show this help
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)              DRY_RUN=1; shift ;;
        --mission-config)       MISSION_CONFIG="$2"; shift 2 ;;
        --field-profile)        FIELD_PROFILE="$2"; shift 2 ;;
        --calibration)          CALIBRATION_FILE="$2"; shift 2 ;;
        --camera-runtime-plan)  CAMERA_PLAN="$2"; shift 2 ;;
        --fcu-serial)           FCU_SERIAL="$2"; shift 2 ;;
        --hmac-key-file)        HMAC_KEY_FILE="$2"; shift 2 ;;
        --mid360-driver-config) MID360_DRIVER_CONFIG="$2"; shift 2 ;;
        --fast-lio-launch)      FAST_LIO_LAUNCH="$2"; shift 2 ;;
        --task3-identity)       TASK3_IDENTITY="$2"; shift 2 ;;
        --enable-display)       ENABLE_DISPLAY="true"; shift ;;
        -h|--help)              usage ;;
        *)                      die "Unknown argument: $1" ;;
    esac
done

# ─── Validation ──────────────────────────────────────────────────────────────
validate_required() {
    local name="$1" value="$2"
    [[ -n "$value" ]] || die "Missing required argument: $name"
    [[ -r "$value" ]] || die "Unreadable path for $name: $value"
}

validate_required "--mission-config" "$MISSION_CONFIG"
validate_required "--field-profile" "$FIELD_PROFILE"
validate_required "--calibration" "$CALIBRATION_FILE"
validate_required "--camera-runtime-plan" "$CAMERA_PLAN"
validate_required "--fcu-serial" "$FCU_SERIAL"
validate_required "--hmac-key-file" "$HMAC_KEY_FILE"
validate_required "--mid360-driver-config" "$MID360_DRIVER_CONFIG"
validate_required "--fast-lio-launch" "$FAST_LIO_LAUNCH"
[[ -n "$TASK3_IDENTITY" ]] || die "Missing required argument: --task3-identity"

# Check calibration is CALIBRATED (not UNCALIBRATED or SYNTHETIC)
# YAML 值无引号: calibration_status: CALIBRATED
if ! grep -Eq '^calibration_status:[[:space:]]*["'"'"']?CALIBRATED["'"'"']?([[:space:]]|$)' "$CALIBRATION_FILE" 2>/dev/null; then
    die "Calibration file must contain CALIBRATED status: $CALIBRATION_FILE"
fi

# Check field profile is not synthetic/blocked
if grep -Eq '^classification:[[:space:]]*["'"'"']?synthetic_simulation' "$FIELD_PROFILE" 2>/dev/null; then
    die "Field profile must not be synthetic simulation: $FIELD_PROFILE"
fi
if grep -Eq '^activation:[[:space:]]*["'"'"']?blocked' "$FIELD_PROFILE" 2>/dev/null; then
    die "Field profile must not be blocked: $FIELD_PROFILE"
fi

# Check camera plan has no PLACEHOLDER controller IDs
if grep -qi 'PLACEHOLDER' "$CAMERA_PLAN" 2>/dev/null; then
    die "Camera plan must not contain PLACEHOLDER controller IDs: $CAMERA_PLAN"
fi

# Check SROS2 keystore
[[ -n "$ROS_SECURITY_KEYSTORE" ]] || die "ROS_SECURITY_KEYSTORE environment variable is required"
[[ -d "$ROS_SECURITY_KEYSTORE" ]] || die "ROS_SECURITY_KEYSTORE directory does not exist: $ROS_SECURITY_KEYSTORE"

# ─── Build launch command ───────────────────────────────────────────────────
LAUNCH_CMD=(
    ros2 launch ed_uav_bringup task3_flight_test.launch.py
    "mission_config_path:=$MISSION_CONFIG"
    "field_profile_path:=$FIELD_PROFILE"
    "calibration_file:=$CALIBRATION_FILE"
    "camera_runtime_plan:=$CAMERA_PLAN"
    "fcu_serial_port:=$FCU_SERIAL"
    "hmac_key_file:=$HMAC_KEY_FILE"
    "mid360_driver_config_path:=$MID360_DRIVER_CONFIG"
    "fast_lio_launch_path:=$FAST_LIO_LAUNCH"
    "task3_identity:=$TASK3_IDENTITY"
    "ros_security_enable:=true"
    "ros_security_strategy:=Enforce"
    "ros_security_keystore:=$ROS_SECURITY_KEYSTORE"
    "enable_flight_commands:=true"
    "enable_realtime_control:=true"
    "enable_programmable_commands:=false"
    "enable_display:=$ENABLE_DISPLAY"
)

# ─── Dry-run mode ───────────────────────────────────────────────────────────
if [[ "$DRY_RUN" -eq 1 ]]; then
    # 非飞控全链路自检: 启动地面站/雷达里程计/相机/视觉跟踪/电磁铁/显示,
    # 跳过飞控桥并强制关闭飞行指令
    LAUNCH_CMD+=(
        "dry_run:=true"
        "enable_flight_commands:=false"
        "enable_realtime_control:=false"
    )
    ok "Task3 配置校验通过, 以 dry-run 模式启动非飞控全链路自检"
    ok "跳过: 飞控桥 (ed_uav_fcu_bridge); 保留: 地面站/雷达/相机/视觉/电磁铁/显示"
    printf '%s\n' "${LAUNCH_CMD[*]}"
fi

# ─── Launch ─────────────────────────────────────────────────────────────────
echo ""
echo -e "${C}══════════════════════════════════════════════════════════${N}"
echo -e "${C}  Task3 Stability Flight Test${N}"
echo -e "${C}══════════════════════════════════════════════════════════${N}"
echo -e "  mission      : ${B}$MISSION_CONFIG${N}"
echo -e "  field profile: ${B}$FIELD_PROFILE${N}"
echo -e "  calibration  : ${B}$CALIBRATION_FILE${N}"
echo -e "  FCU serial   : ${B}$FCU_SERIAL${N}"
echo -e "  task3 identity: ${B}$TASK3_IDENTITY${N}"
echo -e "${C}══════════════════════════════════════════════════════════${N}"
echo ""

# Process management
LAUNCH_PID=""
EXIT_CODE=0

handle_signal() {
    local sig=$1
    trap - "$sig"
    if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -TERM -- "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
    case "$sig" in
        INT)  exit 130 ;;
        TERM) exit 143 ;;
    esac
}

trap 'handle_signal INT' INT
trap 'handle_signal TERM' TERM

ok "Launching Task3 flight test..."
# shellcheck disable=SC2086
"${LAUNCH_CMD[@]}" &
LAUNCH_PID=$!
wait "$LAUNCH_PID" && EXIT_CODE=0 || EXIT_CODE=$?
exit "$EXIT_CODE"
