#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$-sim"
readonly evidence_relative=".omo/evidence/offline-integration/scripts/$run_id"
readonly evidence_dir="$repo_root/$evidence_relative"
mkdir -p "$evidence_dir"

record_failure() {
    local exit_code="$1"
    if ((exit_code != 0)); then
        printf 'SIM_OFFLINE_RED exit_code=%s\n' "$exit_code" >"$evidence_dir/FAILED"
    fi
}
trap 'record_failure "$?"' EXIT

bash "$repo_root/tools/run_humble.sh" bash -lc '
source /opt/ros/humble/setup.bash
set -euo pipefail
evidence_dir="$1"

cleanup_jobs() {
    local pid
    for pid in $(jobs -pr); do
        kill "$pid" 2>/dev/null || true
    done
}
trap cleanup_jobs EXIT
trap "exit 130" INT
trap "exit 143" TERM

timeout --foreground 300s colcon --log-base "$evidence_dir/colcon-log" build \
    --base-paths /workspace/ros2_ws/src \
    --build-base "$evidence_dir/build" \
    --install-base "$evidence_dir/install" \
    --symlink-install \
    2>&1 | tee "$evidence_dir/build.log"
set +u
source "$evidence_dir/install/setup.bash"
set -u

timeout --foreground --signal=INT --kill-after=5s 30s \
    ros2 launch ed_uav_bringup offline_integration.launch.py \
    use_sim_time:=false use_rviz:=false \
    duration_seconds:=3 rate_hz:=20 seed:=7 \
    2>&1 | tee "$evidence_dir/simulation.log"

grep -Fq "ROS SCENARIO: GREEN virtual replay completed" "$evidence_dir/simulation.log"
if grep -Fq "process has died" "$evidence_dir/simulation.log"; then
    printf "simulation reported a process failure\n" >&2
    exit 1
fi
if grep -Fq "offline_integration_rviz" "$evidence_dir/simulation.log"; then
    printf "simulation unexpectedly started RViz\n" >&2
    exit 1
fi

printf "SIM_OFFLINE_GREEN\n" | tee "$evidence_dir/SUCCESS"
' bash "/workspace/$evidence_relative" 2>&1 | tee "$evidence_dir/runner.log"
