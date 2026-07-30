#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$-stability-test"
readonly evidence_relative=".omo/evidence/stability/$run_id"
readonly evidence_dir="$repo_root/$evidence_relative"

mkdir -p "$evidence_dir"

record_failure() {
    local exit_code="$?"
    if ((exit_code != 0)); then
        printf 'STABILITY_TEST_FAILED exit_code=%s\n' "$exit_code" >"$evidence_dir/FAILED"
    fi
}
trap record_failure EXIT
trap 'exit 0' INT

{
    printf 'command=./tools/run_stability_test_sim.sh\n'
    printf 'mode=interactive\n'
    printf 'HUMBLE_GUI=1\n'
    printf 'HUMBLE_INTERACTIVE=1\n'
    printf 'HUMBLE_TIMEOUT_SECONDS=%s\n' "${HUMBLE_TIMEOUT_SECONDS:-0}"
} >"$evidence_dir/command.txt"

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

# Find the stability test mission config
MISSION_CONFIG="/workspace/ros2_ws/src/ed_uav_mission/config/missions/simulation_stability_test.yaml"
if [[ ! -f "$MISSION_CONFIG" ]]; then
    echo "ERROR: stability test mission config not found at $MISSION_CONFIG"
    exit 1
fi

printf 'mission_config=%s\n' "$MISSION_CONFIG" >>"$evidence_dir/environment.txt"

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
        printf 'STABILITY_TEST_SUCCESS\n' | tee "$evidence_dir/SUCCESS"
    else
        printf 'STABILITY_TEST_FAILED exit_code=%s\n' "$exit_code" | tee "$evidence_dir/FAILED"
    fi
    exit "$exit_code"
}
trap cleanup EXIT
trap 'exit 0' INT
trap 'exit 143' TERM

echo "=== Launching stability test simulation ==="
echo "Mission config: $MISSION_CONFIG"
echo ""

setsid bash -c \
    'set -o pipefail; ros2 launch ed_uav_gazebo sim.launch.py gui:=true use_rviz:=true mission_config:="$1" 2>&1 | tee "$2"' \
    bash "$MISSION_CONFIG" "$evidence_dir/sim.log" &
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
