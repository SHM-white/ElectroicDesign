#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly workspace="$repo_root/ros2_ws"
readonly ros_setup="/opt/ros/humble/setup.bash"
readonly duration_seconds="${ED_RVIZ_DURATION_SECONDS:-6000}"
readonly rate_hz="${ED_RVIZ_RATE_HZ:-10}"
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

if [[ ! -f "$ros_setup" ]]; then
    printf 'ROS 2 Humble not found: %s\n' "$ros_setup" >&2
    exit 1
fi

if [[ -n "${ED_RVIZ_DISPLAY:-}" ]]; then
    export DISPLAY="$ED_RVIZ_DISPLAY"
elif [[ -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY=:0
else
    export DISPLAY="${DISPLAY:-}"
fi

if [[ -z "$DISPLAY" ]]; then
    printf 'No graphical display found. Set ED_RVIZ_DISPLAY, for example ED_RVIZ_DISPLAY=:0.\n' >&2
    exit 1
fi

if [[ "$DISPLAY" == :0 && -f "/run/user/$UID/gdm/Xauthority" ]]; then
    export XAUTHORITY="/run/user/$UID/gdm/Xauthority"
fi

if command -v xdpyinfo >/dev/null 2>&1 && ! xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    printf 'Cannot connect to display %s. Check XAUTHORITY or set ED_RVIZ_DISPLAY.\n' "$DISPLAY" >&2
    exit 1
fi

set +u
source "$ros_setup"
set -u

if ! python3 -c 'from typing_extensions import assert_never' >/dev/null 2>&1; then
    python3 -m pip install --user 'typing_extensions>=4.4,<5'
fi

printf 'Building ROS 2 visualization packages...\n'
colcon --log-base "$workspace/log" build \
    --base-paths "$workspace/src" \
    --build-base "$workspace/build" \
    --install-base "$workspace/install" \
    --symlink-install \
    --packages-up-to ed_uav_bringup ed_uav_verification

set +u
source "$workspace/install/setup.bash"
set -u

readonly description_share="$workspace/install/ed_uav_description/share/ed_uav_description"
readonly bringup_share="$workspace/install/ed_uav_bringup/share/ed_uav_bringup"
readonly calibration_file="$description_share/config/synthetic_calibrated.yaml"
readonly rviz_config="$bringup_share/rviz/offline_integration.rviz"

printf 'Starting synthetic lidar point cloud on /lidar/points (display=%s).\n' "$DISPLAY"
printf 'Press Ctrl+C in this terminal to stop RViz and the publisher.\n'

setsid ros2 launch ed_uav_bringup bringup.launch.py \
    profile:=offline \
    calibration_file:="$calibration_file" \
    camera_narrow_serial:=SYNTHETIC-NARROW-001 \
    camera_wide_serial:=SYNTHETIC-WIDE-001 \
    lidar_serial:=SYNTHETIC-LIDAR-001 \
    use_sim_time:=false \
    &
background_pgids+=("$!")

setsid bash -c '
    while true; do
        ros2 run ed_uav_verification ed-uav-verify-ros \
            --seed 7 \
            --duration-seconds "$1" \
            --rate-hz "$2"
    done
' bash "$duration_seconds" "$rate_hz" &
background_pgids+=("$!")

rviz2 -d "$rviz_config" --ros-args -r __node:=lidar_rviz
