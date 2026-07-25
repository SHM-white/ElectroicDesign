#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$-static"
readonly evidence_relative=".omo/evidence/offline-integration/scripts/$run_id"
readonly evidence_dir="$repo_root/$evidence_relative"
mkdir -p "$evidence_dir"

record_failure() {
    local exit_code="$1"
    if ((exit_code != 0)); then
        printf 'STATIC_OFFLINE_RED exit_code=%s\n' "$exit_code" >"$evidence_dir/FAILED"
    fi
}
trap 'record_failure "$?"' EXIT

timeout --foreground 120s bash "$repo_root/tools/test_run_humble.sh" \
    2>&1 | tee "$evidence_dir/test-run-humble.log"

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

PYTHONPATH=/workspace/ros2_ws/src/ed_uav_verification \
    timeout --foreground 120s python3 -m pytest -q \
    tools/test_offline_scripts_contract.py \
    ros2_ws/src/ed_uav_verification/test/test_offline_integration_contract.py \
    ros2_ws/src/ed_uav_verification/test/test_fake_fcu_contract.py::test_fake_fcu_creates_requested_pty_emits_fresh_telemetry_and_releases_it \
    2>&1 | tee "$evidence_dir/focused-pytest.log"

timeout --foreground 30s python3 \
    ros2_ws/src/ed_uav_bringup/tools/verify_launch_surface.py \
    --launch ros2_ws/src/ed_uav_bringup/launch/bringup.launch.py \
    2>&1 | tee "$evidence_dir/launch-surface.log"

timeout --foreground 30s python3 \
    ros2_ws/src/ed_uav_bringup/tools/verify_launch_profiles.py \
    --launch ros2_ws/src/ed_uav_bringup/launch/offline_replay.launch.py \
    --profile offline_replay \
    2>&1 | tee "$evidence_dir/replay-profile.log"

timeout --foreground 30s python3 \
    ros2_ws/src/ed_uav_interfaces/tools/check_contract.py \
    ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json \
    2>&1 | tee "$evidence_dir/interface-contract.log"

timeout --foreground 30s python3 tools/parity_check.py \
    2>&1 | tee "$evidence_dir/parity.log"

printf "STATIC_OFFLINE_GREEN\n" | tee "$evidence_dir/SUCCESS"
' bash "/workspace/$evidence_relative" 2>&1 | tee "$evidence_dir/runner.log"
