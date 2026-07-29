#!/usr/bin/env bash
if ! source /opt/ros/humble/setup.bash; then
    printf 'ROS 2 Humble setup is unavailable at /opt/ros/humble/setup.bash\n' >&2
    exit 1
fi
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$"
readonly evidence_dir="$repo_root/.omo/evidence/lidar-odometry/$run_id"
readonly overlay_setup="$repo_root/ros2_ws/install/setup.bash"
ODOM_TOPIC="${ODOM_TOPIC:-/localization/odom}"
readonly expected_topic_type="nav_msgs/msg/Odometry"
readonly preflight_timeout_sec=10
failure_reason=""

mkdir -p "$evidence_dir"

record_result() {
    local exit_status="$?"
    trap - EXIT
    {
        printf 'exit_status=%s\n' "$exit_status"
        if [[ -f "$evidence_dir/result.json" ]]; then
            printf 'result_json=%s\n' "$evidence_dir/result.json"
        else
            printf 'result_json=unavailable\n'
        fi
        if [[ -n "$failure_reason" ]]; then
            printf 'reason=%s\n' "$failure_reason"
        fi
    } >>"$evidence_dir/result.txt"
    if [[ -f "$evidence_dir/result.json" ]]; then
        printf 'LIDAR_ODOMETRY_RESULT=%s\n' "$evidence_dir/result.json"
    fi
    printf 'LIDAR_ODOMETRY_EVIDENCE=%s\n' "$evidence_dir"
    exit "$exit_status"
}

preflight_failure() {
    local reason="$1"
    printf 'status=failed\nreason=%s\n' "$reason" >>"$evidence_dir/preflight.txt"
    printf 'Odometry preflight failed for %s. Start the provisioned Livox + ROS 2 FAST-LIO + localization chain; this runner does not start it.\n' "$ODOM_TOPIC" >&2
    exit 1
}

trap record_result EXIT
trap ':' INT
trap 'exit 143' TERM

for argument in "$@"; do
    case "$argument" in
        --odom-topic | --odom-topic=*)
            printf 'Do not pass --odom-topic to this runner; use ODOM_TOPIC so preflight and the demo use the same topic.\n' >&2
            exit 1
            ;;
    esac
done

{
    printf 'command=./tools/run_lidar_odometry_accuracy_demo.sh\n'
    printf 'odom_topic=%s\n' "$ODOM_TOPIC"
    printf 'arguments='
    printf '%q ' "$@"
    printf '\n'
} >"$evidence_dir/command.txt"

if [[ "${ED_ODOMETRY_DEMO_SKIP_BUILD:-0}" == "1" ]]; then
    printf 'build=skipped ED_ODOMETRY_DEMO_SKIP_BUILD=1\n' >"$evidence_dir/build.txt"
else
    (
        cd "$repo_root/ros2_ws"
        colcon build --symlink-install --packages-up-to ed_uav_localization
    ) 2>&1 | tee "$evidence_dir/build.log"
fi

if [[ ! -f "$overlay_setup" ]]; then
    failure_reason="missing_install_overlay"
    if [[ "${ED_ODOMETRY_DEMO_SKIP_BUILD:-0}" == "1" ]]; then
        printf 'Localization install overlay is missing at %s; rerun without ED_ODOMETRY_DEMO_SKIP_BUILD=1 to build ed_uav_localization.\n' "$overlay_setup" >&2
    else
        printf 'Localization install overlay is missing at %s after the build.\n' "$overlay_setup" >&2
    fi
    exit 1
fi
set +u
source "$overlay_setup"
set -u

printf 'topic=%s\nexpected_type=%s\n' "$ODOM_TOPIC" "$expected_topic_type" >"$evidence_dir/preflight.txt"
if ! ros2 topic type "$ODOM_TOPIC" >"$evidence_dir/topic-type.txt" 2>&1; then
    preflight_failure "topic_type_query_failed"
fi
topic_type="$(tr -d '\r\n' <"$evidence_dir/topic-type.txt")"
if [[ "$topic_type" != "$expected_topic_type" ]]; then
    preflight_failure "wrong_topic_type:$topic_type"
fi

if ! ros2 topic info "$ODOM_TOPIC" >"$evidence_dir/topic-info.txt" 2>&1; then
    preflight_failure "topic_info_query_failed"
fi
if ! grep -Eq '^Publisher count: [1-9][0-9]*$' "$evidence_dir/topic-info.txt"; then
    preflight_failure "no_publishers"
fi

if ! timeout "${preflight_timeout_sec}s" ros2 topic echo --once "$ODOM_TOPIC" >"$evidence_dir/odom-message.yaml" 2>&1; then
    preflight_failure "no_message_within_${preflight_timeout_sec}s"
fi
if [[ ! -s "$evidence_dir/odom-message.yaml" ]]; then
    preflight_failure "empty_message"
fi
printf 'status=passed\ntopic_type=%s\npublisher_observed=1\nmessage_observed=1\n' "$topic_type" >>"$evidence_dir/preflight.txt"

demo_command=(ros2 run ed_uav_localization odometry_accuracy_demo --odom-topic "$ODOM_TOPIC")
if (($# == 0)); then
    demo_command+=(--mode stationary --duration-sec 60 --min-samples 100)
else
    demo_command+=("$@")
fi
{
    printf 'demo_command='
    printf '%q ' "${demo_command[@]}"
    printf '\n'
} >>"$evidence_dir/command.txt"

set +e
"${demo_command[@]}" 2>&1 | (
    trap '' INT
    tee "$evidence_dir/demo.log"
)
pipeline_statuses=("${PIPESTATUS[@]}")
set -e
readonly demo_status="${pipeline_statuses[0]}"
readonly tee_status="${pipeline_statuses[1]}"
if ((tee_status != 0)); then
    pipeline_status="$tee_status"
else
    pipeline_status="$demo_status"
fi

readonly result_count="$(awk '/^ODOMETRY_ACCURACY_RESULT=/{count += 1} END {print count + 0}' "$evidence_dir/demo.log")"
if ((result_count == 1)); then
    readonly candidate_result="$evidence_dir/result.json.candidate"
    awk '/^ODOMETRY_ACCURACY_RESULT=/{sub(/^ODOMETRY_ACCURACY_RESULT=/, ""); print}' "$evidence_dir/demo.log" >"$candidate_result"
    if python3 - "$candidate_result" <<'PY'
import json
import sys

required_fields = {
    "schema_version",
    "status",
    "trial",
    "interpretation",
    "input_topic",
    "sample_count",
    "rejected_count",
    "metrics",
}
try:
    with open(sys.argv[1], encoding="utf-8") as result_file:
        result = json.load(result_file)
except (OSError, json.JSONDecodeError):
    raise SystemExit(1)
if not isinstance(result, dict) or not required_fields <= result.keys():
    raise SystemExit(1)
PY
    then
        mv "$candidate_result" "$evidence_dir/result.json"
    else
        failure_reason="invalid_result_json"
        printf 'The demo emitted an invalid ODOMETRY_ACCURACY_RESULT JSON object.\n' >&2
        if ((pipeline_status == 0)); then
            pipeline_status=1
        fi
    fi
else
    failure_reason="missing_single_result"
    printf 'The demo did not emit exactly one ODOMETRY_ACCURACY_RESULT line.\n' >&2
    if ((pipeline_status == 0)); then
        pipeline_status=1
    fi
fi
printf 'demo_status=%s\ntee_status=%s\npipeline_status=%s\nresult_count=%s\n' \
    "$demo_status" "$tee_status" "$pipeline_status" "$result_count" >"$evidence_dir/result.txt"

exit "$pipeline_status"
