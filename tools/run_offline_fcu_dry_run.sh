#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$-fcu"
readonly evidence_relative=".omo/evidence/offline-integration/scripts/$run_id"
readonly evidence_dir="$repo_root/$evidence_relative"
mkdir -p "$evidence_dir"

record_failure() {
    local exit_code="$1"
    if ((exit_code != 0)); then
        printf 'FCU_DRY_RUN_RED exit_code=%s\n' "$exit_code" >"$evidence_dir/FAILED"
    fi
}
trap 'record_failure "$?"' EXIT

bash "$repo_root/tools/run_humble.sh" bash -lc '
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
set -euo pipefail
evidence_dir="$1"
pty_device="$evidence_dir/fcu-pty"

cleanup_fcu() {
    local pid
    for pid in $(jobs -pr); do
        kill "$pid" 2>/dev/null || true
    done
    rm -f "$pty_device"
}
trap cleanup_fcu EXIT
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
    ros2 launch ed_uav_bringup fcu_dry_run.launch.py \
    pty_device:="$pty_device" duration_seconds:=3 rate_hz:=20 \
    seed:=7 use_sim_time:=false \
    2>&1 | tee "$evidence_dir/fcu.log"

grep -Fq "FAKE FCU READY: $pty_device" "$evidence_dir/fcu.log"
grep -Fq "ed_uav_fcu_bridge" "$evidence_dir/fcu.log"
if grep -Eq \
    "KeyboardInterrupt|Traceback|process has died|exit code -[0-9]+|exit code 1|rcl_shutdown already called" \
    "$evidence_dir/fcu.log"; then
    printf "FCU dry-run reported an unexpected bridge failure\n" >&2
    exit 1
fi
if [[ -e "$pty_device" || -L "$pty_device" ]]; then
    printf "FCU dry-run left its PTY path behind\n" >&2
    exit 1
fi

printf "FCU_DRY_RUN_GREEN\n" | tee "$evidence_dir/SUCCESS"
' bash "/workspace/$evidence_relative" 2>&1 | tee "$evidence_dir/runner.log"
