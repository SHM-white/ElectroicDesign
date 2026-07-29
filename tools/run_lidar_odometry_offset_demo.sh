#!/usr/bin/env bash
if ! source /opt/ros/humble/setup.bash; then
    printf 'ROS 2 Humble setup is unavailable at /opt/ros/humble/setup.bash\n' >&2
    exit 1
fi
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly overlay_setup="$repo_root/ros2_ws/install/setup.bash"
readonly expected_topic_type="nav_msgs/msg/Odometry"
readonly preflight_timeout_sec=10
cli_odom_topic=""
cli_topic_supplied=0
remaining_arguments=()

preflight_failure() {
    local reason="$1"
    printf 'Lidar odometry preflight failed for %s: %s. Start the provisioned Livox + ROS 2 FAST-LIO + localization chain; this runner does not start it.\n' \
        "$resolved_topic" "$reason" >&2
    exit 1
}

while (($# > 0)); do
    case "$1" in
        --odom-topic)
            if (($# < 2)); then
                printf 'Missing value for --odom-topic\n' >&2
                exit 2
            fi
            cli_odom_topic="$2"
            cli_topic_supplied=1
            shift 2
            ;;
        --odom-topic=*)
            cli_odom_topic="${1#--odom-topic=}"
            cli_topic_supplied=1
            shift
            ;;
        *)
            remaining_arguments+=("$1")
            shift
            ;;
    esac
done

if ((cli_topic_supplied)); then
    resolved_topic="$cli_odom_topic"
elif [[ -v ODOM_TOPIC ]]; then
    resolved_topic="$ODOM_TOPIC"
else
    resolved_topic="/localization/odom"
fi

if [[ -z "$resolved_topic" ]]; then
    printf 'Resolved odometry topic must not be empty\n' >&2
    exit 2
fi

if [[ "${ED_ODOMETRY_OFFSET_SKIP_BUILD:-0}" != "1" ]]; then
    (
        cd "$repo_root/ros2_ws"
        colcon build --symlink-install --packages-up-to ed_uav_localization
    )
fi

if [[ ! -f "$overlay_setup" ]]; then
    printf 'Localization install overlay is missing at %s; build ed_uav_localization or unset ED_ODOMETRY_OFFSET_SKIP_BUILD.\n' \
        "$overlay_setup" >&2
    exit 1
fi
set +u
source "$overlay_setup"
set -u

if ! topic_type="$(ros2 topic type "$resolved_topic")"; then
    preflight_failure "topic type query failed"
fi
topic_type="${topic_type//$'\r'/}"
topic_type="${topic_type//$'\n'/}"
if [[ "$topic_type" != "$expected_topic_type" ]]; then
    preflight_failure "wrong type $topic_type (expected $expected_topic_type)"
fi

if ! topic_info="$(ros2 topic info "$resolved_topic")"; then
    preflight_failure "topic info query failed"
fi
if ! grep -Eq '^Publisher count: [1-9][0-9]*$' <<<"$topic_info"; then
    preflight_failure "publisher count is zero"
fi

if ! timeout --foreground "${preflight_timeout_sec}s" ros2 topic echo --once "$resolved_topic" >/dev/null; then
    preflight_failure "no message within ${preflight_timeout_sec}s"
fi

exec ros2 run ed_uav_localization lidar_odometry_offset_demo --odom-topic "$resolved_topic" "${remaining_arguments[@]}"
