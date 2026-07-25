#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$-replay"
readonly evidence_relative=".omo/evidence/offline-integration/scripts/$run_id"
readonly evidence_dir="$repo_root/$evidence_relative"
mkdir -p "$evidence_dir"

record_failure() {
    local exit_code="$1"
    if ((exit_code != 0)); then
        printf 'FULL_REPLAY_RED exit_code=%s\n' "$exit_code" >"$evidence_dir/FAILED"
    fi
}
trap 'record_failure "$?"' EXIT

bash "$repo_root/tools/run_humble.sh" bash -lc '
source /opt/ros/humble/setup.bash
set -euo pipefail
evidence_dir="$1"
bag_dir="$evidence_dir/event-only-bag"
event_json="$evidence_dir/events.json"

cleanup_jobs() {
    local pid
    for pid in $(jobs -pr); do
        kill "$pid" 2>/dev/null || true
    done
}
trap cleanup_jobs EXIT
trap "exit 130" INT
trap "exit 143" TERM

timeout --foreground 300s colcon --log-base "$evidence_dir/colcon-build-log" build \
    --base-paths /workspace/ros2_ws/src \
    --build-base "$evidence_dir/build" \
    --install-base "$evidence_dir/install" \
    --symlink-install \
    2>&1 | tee "$evidence_dir/build.log"
set +u
source "$evidence_dir/install/setup.bash"
set -u

timeout --foreground 180s colcon --log-base "$evidence_dir/colcon-test-log" test \
    --build-base "$evidence_dir/build" \
    --install-base "$evidence_dir/install" \
    --packages-select ed_uav_verification ed_uav_bringup \
    --event-handlers console_direct+ \
    2>&1 | tee "$evidence_dir/test.log"
timeout --foreground 30s colcon test-result \
    --test-result-base "$evidence_dir/build" --all --verbose \
    2>&1 | tee "$evidence_dir/test-result.log"

timeout --foreground 30s python3 -m ed_uav_verification.cli \
    --seed 37 --duration-seconds 1 --rate-hz 20 \
    --event-json "$event_json" --rosbag-dir "$bag_dir" \
    2>&1 | tee "$evidence_dir/create-bag.log"
grep -Fq "SCENARIO: GREEN" "$evidence_dir/create-bag.log"

timeout --foreground 30s ros2 bag info "$bag_dir" \
    2>&1 | tee "$evidence_dir/bag-info.log"
grep -Fq "Topic: /verification/events" "$evidence_dir/bag-info.log"
topic_count="$(grep -c "^Topic information:" "$evidence_dir/bag-info.log")"
if [[ "$topic_count" -ne 1 ]]; then
    printf "expected one event-only rosbag topic, found %s\n" "$topic_count" >&2
    exit 1
fi

set +e
timeout --foreground --signal=INT --kill-after=5s 12s \
    ros2 launch ed_uav_bringup offline_replay.launch.py \
    bag_path:="$bag_dir" bag_rate:=1.0 use_sim_time:=false \
    2>&1 | tee "$evidence_dir/replay.log"
replay_status="${PIPESTATUS[0]}"
set -e
if [[ "$replay_status" -ne 0 && "$replay_status" -ne 124 ]]; then
    printf "offline replay launch failed with exit code %s\n" "$replay_status" >&2
    exit "$replay_status"
fi
grep -Eq "rosbag_replay.*process has finished cleanly" "$evidence_dir/replay.log"
if grep -Fq "process has died" "$evidence_dir/replay.log"; then
    printf "offline replay reported a process failure\n" >&2
    exit 1
fi

printf "FULL_REPLAY_GREEN\n" | tee "$evidence_dir/SUCCESS"
' bash "/workspace/$evidence_relative" 2>&1 | tee "$evidence_dir/runner.log"
