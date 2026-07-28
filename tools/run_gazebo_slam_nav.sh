#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$-gazebo-slam-nav"
readonly evidence_relative=".omo/evidence/gazebo/$run_id"
readonly evidence_dir="$repo_root/$evidence_relative"

mkdir -p "$evidence_dir"

humblerunner_pid=""
cleanup_outer() {
    local exit_code="$?"
    trap - EXIT INT TERM
    if [[ -n "$humblerunner_pid" ]] && kill -0 "$humblerunner_pid" 2>/dev/null; then
        kill -TERM -- "-$humblerunner_pid" 2>/dev/null || true
        wait "$humblerunner_pid" 2>/dev/null || true
    fi
    if ((exit_code == 130)) && [[ -e "$evidence_dir/SUCCESS" ]]; then
        exit 0
    fi
    if ((exit_code != 0)) && [[ ! -e "$evidence_dir/FAILED" ]] && [[ ! -e "$evidence_dir/SUCCESS" ]]; then
        printf 'GAZEBO_SLAM_NAV_FAILED exit_code=%s\n' "$exit_code" >"$evidence_dir/FAILED"
    fi
    exit "$exit_code"
}
trap cleanup_outer EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

{
    printf 'command=./tools/run_gazebo_slam_nav.sh\n'
    printf 'mode=interactive-fast-lio-planner-only\n'
    printf 'HUMBLE_GUI=1\nHUMBLE_INTERACTIVE=1\nHUMBLE_TIMEOUT_SECONDS=0\n'
    printf 'ROS_DOMAIN_ID=42\nROS_LOCALHOST_ONLY=1\n'
} >"$evidence_dir/command.txt"
{
    printf 'DISPLAY=%s\n' "${DISPLAY:-}"
    printf 'WAYLAND_DISPLAY=%s\n' "${WAYLAND_DISPLAY:-}"
    printf 'XDG_RUNTIME_DIR=%s\n' "${XDG_RUNTIME_DIR:-}"
    printf 'HUMBLE_GUI=1\nHUMBLE_INTERACTIVE=1\nHUMBLE_TIMEOUT_SECONDS=0\n'
} | sort >"$evidence_dir/host-environment.txt"

set +e
HUMBLE_GUI=1 \
HUMBLE_INTERACTIVE=1 \
HUMBLE_TIMEOUT_SECONDS=0 \
setsid bash -c \
    'set -o pipefail; bash "$1" bash -s -- "$3" "$4" "$5" "$6" 2>&1 | tee "$2"' \
    bash \
    "$repo_root/tools/run_humble.sh" \
    "$evidence_dir/runner.log" \
    "$repo_root" \
    "$evidence_dir" \
    "/workspace" \
    "/workspace/$evidence_relative" <<'CONTAINER_SCRIPT' &
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
set -euo pipefail

host_workspace_root="$1"
host_evidence_dir="$2"
container_workspace_root="$3"
container_evidence_dir="$4"
if [[ -d "$host_workspace_root/ros2_ws" && -f "$host_workspace_root/tools/run_humble.sh" ]]; then
    workspace_root="$host_workspace_root"
    evidence_dir="$host_evidence_dir"
elif [[ -d "$container_workspace_root/ros2_ws" && -f "$container_workspace_root/tools/run_humble.sh" ]]; then
    workspace_root="$container_workspace_root"
    evidence_dir="$container_evidence_dir"
else
    printf 'no valid workspace path was provided\n' >&2
    exit 1
fi
third_party_dir="$evidence_dir/third_party"
build_base="$evidence_dir/build"
install_base="$evidence_dir/install"
log_base="$evidence_dir/log"
fast_lio_simulation_patch="$workspace_root/tools/patches/fast_lio_simulation.patch"
livox_sdk2_dir="$third_party_dir/livox_sdk2"
livox_sdk2_build="$evidence_dir/livox-sdk2-build"
livox_sdk2_install="$evidence_dir/livox-sdk2-install"
mkdir -p "$third_party_dir" "$build_base" "$install_base" "$log_base"

export GZ_SIM_RESOURCE_PATH="$workspace_root/ros2_ws/src/ed_uav_gazebo/models:$workspace_root/ros2_ws/src/ed_uav_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"
printf 'GZ_SIM_RESOURCE_PATH=%s\nIGN_GAZEBO_RESOURCE_PATH=%s\n' \
    "$GZ_SIM_RESOURCE_PATH" "$IGN_GAZEBO_RESOURCE_PATH" >"$evidence_dir/environment.txt"

python3 - "$workspace_root/ros2_ws/dependencies.repos" "$evidence_dir/fast_lio.repos" <<'PY'
import json
import sys

source_path, output_path = sys.argv[1:]
with open(source_path, encoding="utf-8") as source_file:
    repositories = json.load(source_file)["repositories"]
selected_names = ("livox_sdk2", "livox_ros_driver2", "fast_lio_ros2")
selected = {name: repositories[name] for name in selected_names}
with open(output_path, "w", encoding="utf-8") as output_file:
    json.dump({"repositories": selected}, output_file, indent=2)
    output_file.write("\n")
PY

printf '%s\n' \
    'vcs import "$third_party_dir" < "$evidence_dir/fast_lio.repos"' \
    'git -C "$third_party_dir/fast_lio_ros2" submodule update --init --recursive' \
    'patch --batch --forward --fuzz=0 -d "$third_party_dir/fast_lio_ros2" -p1 < "$fast_lio_simulation_patch"' \
    'cmake -S "$livox_sdk2_dir" -B "$livox_sdk2_build"' \
    'cmake --build "$livox_sdk2_build"' \
    'cmake --install "$livox_sdk2_build" --prefix "$livox_sdk2_install"' \
    'colcon build --base-paths "$workspace_root/ros2_ws/src" "$third_party_dir" --build-base "$build_base" --install-base "$install_base" --log-base "$log_base" --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DLIVOX_LIDAR_SDK_LIBRARY=$livox_sdk2_install/lib/liblivox_lidar_sdk_shared.so -DLIVOX_LIDAR_SDK_INCLUDE_DIR=$livox_sdk2_install/include' \
    'ros2 launch ed_uav_gazebo gazebo_simulation.launch.py gui:=true use_rviz:=true localization_mode:=fast_lio' \
    >"$evidence_dir/commands.txt"

vcs import "$third_party_dir" <"$evidence_dir/fast_lio.repos" 2>&1 | tee "$evidence_dir/vcs-import.log"
git -C "$third_party_dir/fast_lio_ros2" submodule update --init --recursive \
    2>&1 | tee "$evidence_dir/fast-lio-submodule.log"
test -f "$fast_lio_simulation_patch"
patch --batch --forward --fuzz=0 -d "$third_party_dir/fast_lio_ros2" -p1 \
    <"$fast_lio_simulation_patch" \
    2>&1 | tee "$evidence_dir/fast-lio-simulation-patch.log"

cmake -S "$livox_sdk2_dir" -B "$livox_sdk2_build" \
    2>&1 | tee "$evidence_dir/livox-sdk2-configure.log"
cmake --build "$livox_sdk2_build" \
    2>&1 | tee "$evidence_dir/livox-sdk2-build.log"
cmake --install "$livox_sdk2_build" --prefix "$livox_sdk2_install" \
    2>&1 | tee "$evidence_dir/livox-sdk2-install.log"
test -f "$livox_sdk2_install/lib/liblivox_lidar_sdk_shared.so"
test -d "$livox_sdk2_install/include"
export LD_LIBRARY_PATH="$livox_sdk2_install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

livox_dir="$third_party_dir/livox_ros_driver2"
cp "$livox_dir/package_ROS2.xml" "$livox_dir/package.xml"
mkdir -p "$livox_dir/launch"
cp -a "$livox_dir/launch_ROS2/." "$livox_dir/launch/"
printf 'livox_package=%s\nlivox_launch=%s\n' \
    "$livox_dir/package.xml" "$livox_dir/launch" >"$evidence_dir/livox-ros2-preparation.txt"

colcon --log-base "$log_base" build \
    --base-paths "$workspace_root/ros2_ws/src" "$third_party_dir" \
    --build-base "$build_base" \
    --install-base "$install_base" \
    --symlink-install \
    --cmake-args \
        -DROS_EDITION=ROS2 \
        -DDISTRO_ROS=humble \
        "-DLIVOX_LIDAR_SDK_LIBRARY=$livox_sdk2_install/lib/liblivox_lidar_sdk_shared.so" \
        "-DLIVOX_LIDAR_SDK_INCLUDE_DIR=$livox_sdk2_install/include" \
    2>&1 | tee "$evidence_dir/build.log"
set +u
source "$install_base/setup.bash"
set -u

launch_pid=""
mission_completed=0
cleanup() {
    local exit_code="$?"
    trap - EXIT INT TERM
    if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
        kill -INT -- "-$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
    if ((mission_completed == 1)) && ((exit_code == 0 || exit_code == 130)); then
        printf 'GAZEBO_SLAM_NAV_SUCCESS\n' | tee "$evidence_dir/SUCCESS"
        exit 0
    fi
    printf 'GAZEBO_SLAM_NAV_FAILED exit_code=%s\n' "$exit_code" | tee "$evidence_dir/FAILED"
    exit "$exit_code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid bash -c \
    'set -o pipefail; ros2 launch ed_uav_gazebo gazebo_simulation.launch.py gui:=true use_rviz:=true localization_mode:=fast_lio 2>&1 | tee "$1"' \
    bash "$evidence_dir/gazebo.log" &
launch_pid="$!"

required_topics=(
    /clock
    /lidar/points
    /lidar/imu
    /localization/lio/odom
    /localization/lio/cloud_registered
    /localization/lio/map
    /localization/lio/path
    /localization/odom
    /map
)
required_actions=(
    /compute_path_to_pose
    /fcu/flight_command
    /mission/execute
)
graph_ready=0
for _ in {1..60}; do
    ros2 topic list >"$evidence_dir/ros2-topics.log" 2>&1 || true
    ros2 action list >"$evidence_dir/ros2-actions.log" 2>&1 || true
    graph_ready=1
    for topic_name in "${required_topics[@]}"; do
        grep -Fxq "$topic_name" "$evidence_dir/ros2-topics.log" || graph_ready=0
    done
    for action_name in "${required_actions[@]}"; do
        grep -Fxq "$action_name" "$evidence_dir/ros2-actions.log" || graph_ready=0
    done
    ((graph_ready == 1)) && break
    sleep 1
done
((graph_ready == 1)) || { printf 'required FAST-LIO/Nav2 graph did not become ready\n' >&2; exit 1; }

localization_ready=0
for _ in {1..60}; do
    timeout 5s ros2 topic echo --once /localization/status >"$evidence_dir/localization-status.log" 2>&1 || true
    if grep -q 'state: 1' "$evidence_dir/localization-status.log" \
        && grep -q 'map_to_odom_valid: true' "$evidence_dir/localization-status.log"; then
        localization_ready=1
        break
    fi
    sleep 1
done
((localization_ready == 1)) || {
    printf 'LocalizationStatus.STATE_ACTIVE and map_to_odom_valid were not observed\n' >&2
    exit 1
}

timeout 20s ros2 action send_goal --feedback /fcu/flight_command \
    ed_uav_interfaces/action/FlightCommand \
    '{command: 1, timeout_sec: 5.0, correlation_id: gazebo-slam-nav-arm}' \
    >"$evidence_dir/arm.log" 2>&1
grep -q 'status: SUCCEEDED' "$evidence_dir/arm.log"

timeout 120s ros2 action send_goal --feedback /mission/execute \
    ed_uav_interfaces/action/ExecuteMission \
    '{mission_id: simulation-competition, field_profile_id: simulation-arena, timeout_sec: 90.0}' \
    >"$evidence_dir/mission.log" 2>&1
grep -q 'status: SUCCEEDED' "$evidence_dir/mission.log"

disarmed=0
for _ in {1..20}; do
    timeout 5s ros2 topic echo --once /fcu/state >"$evidence_dir/fcu-final-state.log" 2>&1 || true
    if grep -q 'motors_armed: false' "$evidence_dir/fcu-final-state.log"; then
        disarmed=1
        break
    fi
    sleep 1
done
((disarmed == 1)) || { printf 'FcuState motors_armed false was not observed\n' >&2; exit 1; }

mission_completed=1
printf 'GAZEBO_SLAM_NAV_MISSION_SUCCEEDED\n' | tee "$evidence_dir/mission-success.txt"
wait "$launch_pid"
CONTAINER_SCRIPT
humblerunner_pid="$!"
wait "$humblerunner_pid"
runner_status="$?"
humblerunner_pid=""
set -e

if ((runner_status == 130)) && [[ -e "$evidence_dir/SUCCESS" ]]; then
    exit 0
fi
exit "$runner_status"
