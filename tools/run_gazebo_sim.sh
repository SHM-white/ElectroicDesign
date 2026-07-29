#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$-gazebo-sim"
readonly evidence_relative=".omo/evidence/gazebo/$run_id"
readonly evidence_dir="$repo_root/$evidence_relative"

mkdir -p "$evidence_dir"

record_failure() {
    local exit_code="$?"
    if ((exit_code != 0)); then
        printf 'GAZEBO_SIM_FAILED exit_code=%s\n' "$exit_code" >"$evidence_dir/FAILED"
    fi
}
trap record_failure EXIT
trap 'exit 0' INT

{
    printf 'command=./tools/run_gazebo_sim.sh\n'
    printf 'mode=interactive\n'
    printf 'HUMBLE_GUI=1\n'
    printf 'HUMBLE_INTERACTIVE=1\n'
    printf 'HUMBLE_TIMEOUT_SECONDS=%s\n' "${HUMBLE_TIMEOUT_SECONDS:-0}"
} >"$evidence_dir/command.txt"
{
    printf 'DISPLAY=%s\n' "${DISPLAY:-}"
    printf 'WAYLAND_DISPLAY=%s\n' "${WAYLAND_DISPLAY:-}"
    printf 'XDG_RUNTIME_DIR=%s\n' "${XDG_RUNTIME_DIR:-}"
    printf 'HUMBLE_GUI=1\nHUMBLE_INTERACTIVE=1\nHUMBLE_TIMEOUT_SECONDS=%s\n' "${HUMBLE_TIMEOUT_SECONDS:-0}"
} | sort >"$evidence_dir/host-environment.txt"

set +e
{
    HUMBLE_GUI=1 \
    HUMBLE_INTERACTIVE=1 \
    HUMBLE_TIMEOUT_SECONDS="${HUMBLE_TIMEOUT_SECONDS:-0}" \
    bash "$repo_root/tools/run_humble.sh" bash -s -- "/workspace/$evidence_relative" 2>&1 <<'CONTAINER_SCRIPT'
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
set -euo pipefail

evidence_dir="$1"
mkdir -p "$evidence_dir"

export GZ_SIM_RESOURCE_PATH="/workspace/ros2_ws/src/ed_uav_gazebo/models:/workspace/ros2_ws/src/ed_uav_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"

printf 'GZ_SIM_RESOURCE_PATH=%s\nIGN_GAZEBO_RESOURCE_PATH=%s\n' \
    "$GZ_SIM_RESOURCE_PATH" "$IGN_GAZEBO_RESOURCE_PATH" >"$evidence_dir/environment.txt"
printf '%s\n' \
    'colcon build --base-paths /workspace/ros2_ws/src --build-base "$evidence_dir/build" --install-base "$evidence_dir/install" --symlink-install' \
    'ros2 launch ed_uav_gazebo gazebo_simulation.launch.py gui:=true use_rviz:=true' \
    >"$evidence_dir/commands.txt"

colcon --log-base "$evidence_dir/colcon-log" build \
    --base-paths /workspace/ros2_ws/src \
    --build-base "$evidence_dir/build" \
    --install-base "$evidence_dir/install" \
    --symlink-install 2>&1 | tee "$evidence_dir/build.log"
set +u
source "$evidence_dir/install/setup.bash"
set -u

launch_pid=""
cleanup() {
    local exit_code="$?"
    trap - EXIT INT TERM
    if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
        kill -INT -- "-$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
    if ((exit_code == 0)); then
        printf 'GAZEBO_SIM_SUCCESS\n' | tee "$evidence_dir/SUCCESS"
    else
        printf 'GAZEBO_SIM_FAILED exit_code=%s\n' "$exit_code" | tee "$evidence_dir/FAILED"
    fi
    exit "$exit_code"
}
trap cleanup EXIT
trap 'exit 0' INT
trap 'exit 143' TERM

setsid bash -c \
    'set -o pipefail; ros2 launch ed_uav_gazebo gazebo_simulation.launch.py gui:=true use_rviz:=true 2>&1 | tee "$1"' \
    bash "$evidence_dir/gazebo.log" &
launch_pid="$!"
wait "$launch_pid"
CONTAINER_SCRIPT
} | tee "$evidence_dir/runner.log"
runner_status="${PIPESTATUS[0]}"
set -e

if ((runner_status == 130)); then
    exit 0
fi
exit "$runner_status"
