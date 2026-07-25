#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$-gazebo-smoke"
readonly evidence_relative=".omo/evidence/gazebo/$run_id"
readonly evidence_dir="$repo_root/$evidence_relative"

mkdir -p "$evidence_dir"

record_failure() {
    local exit_code="$?"
    if ((exit_code != 0)) && [[ ! -e "$evidence_dir/FAILED" ]]; then
        printf 'GAZEBO_SMOKE_FAILED exit_code=%s\n' "$exit_code" >"$evidence_dir/FAILED"
    fi
}
trap record_failure EXIT

{
    printf 'command=./tools/run_gazebo_smoke.sh\n'
    printf 'mode=bounded-headless\n'
    printf 'HUMBLE_GUI=unset\n'
    printf 'HUMBLE_INTERACTIVE=0\n'
    printf 'HUMBLE_TIMEOUT_SECONDS=%s\n' "${HUMBLE_TIMEOUT_SECONDS:-120}"
} >"$evidence_dir/command.txt"
{
    printf 'HUMBLE_GUI=unset\nHUMBLE_INTERACTIVE=0\nHUMBLE_TIMEOUT_SECONDS=%s\n' "${HUMBLE_TIMEOUT_SECONDS:-120}"
    printf 'GZ_SIM_RESOURCE_PATH=%s\n' "${GZ_SIM_RESOURCE_PATH:-}"
    printf 'IGN_GAZEBO_RESOURCE_PATH=%s\n' "${IGN_GAZEBO_RESOURCE_PATH:-}"
} | sort >"$evidence_dir/host-environment.txt"

{
    HUMBLE_INTERACTIVE=0 \
    HUMBLE_TIMEOUT_SECONDS="${HUMBLE_TIMEOUT_SECONDS:-120}" \
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
    'ros2 launch ed_uav_gazebo gazebo_simulation.launch.py gui:=false use_rviz:=false' \
    'ros2 topic list; ros2 topic echo --once /clock; ros2 topic pub --once /simulation/enable std_msgs/msg/Bool; ros2 topic pub --once /simulation/cmd_vel geometry_msgs/msg/Twist; ros2 topic echo --once /simulation/ground_truth/odom' \
    >"$evidence_dir/commands.txt"

colcon --log-base "$evidence_dir/colcon-log" build \
    --base-paths /workspace/ros2_ws/src \
    --build-base "$evidence_dir/build" \
    --install-base "$evidence_dir/install" \
    --symlink-install 2>&1 | tee "$evidence_dir/build.log"
set +u
source "$evidence_dir/install/setup.bash"
set -u

sim_pid=""
cleanup() {
    local exit_code="$?"
    trap - EXIT INT TERM
    if [[ -n "$sim_pid" ]] && kill -0 "$sim_pid" 2>/dev/null; then
        kill -INT -- "-$sim_pid" 2>/dev/null || true
        wait "$sim_pid" 2>/dev/null || true
    fi
    if ((exit_code == 0)); then
        printf 'GAZEBO_SMOKE_SUCCESS\n' | tee "$evidence_dir/SUCCESS"
    else
        printf 'GAZEBO_SMOKE_FAILED exit_code=%s\n' "$exit_code" | tee "$evidence_dir/FAILED"
    fi
    exit "$exit_code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid bash -c \
    'set -o pipefail; ros2 launch ed_uav_gazebo gazebo_simulation.launch.py gui:=false use_rviz:=false 2>&1 | tee "$1"' \
    bash "$evidence_dir/gazebo.log" &
sim_pid="$!"

ready=0
for _ in {1..30}; do
    if ros2 topic list 2>>"$evidence_dir/ros2-topic.log" | grep -Fxq /clock; then
        ready=1
        break
    fi
    sleep 1
done
((ready == 1)) || { printf 'Gazebo did not publish /clock\n' >&2; exit 1; }

ros2 topic echo --once /clock >"$evidence_dir/clock.log" 2>&1
ros2 topic pub --once /simulation/enable std_msgs/msg/Bool '{data: true}' \
    >"$evidence_dir/movement-enable.log" 2>&1
ros2 topic pub --once /simulation/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >"$evidence_dir/movement-command.log" 2>&1
ros2 topic echo --once /simulation/ground_truth/odom >"$evidence_dir/odom.log" 2>&1
test -s "$evidence_dir/odom.log"

for _ in {1..30}; do
    if ros2 topic echo --once /localization/status 2>/dev/null | grep -q 'state: 1'; then
        break
    fi
    sleep 1
done
ros2 topic echo --once /localization/status >"$evidence_dir/localization-status.log" 2>&1
grep -q 'state: 1' "$evidence_dir/localization-status.log"

ros2 action send_goal --feedback /fcu/flight_command \
    ed_uav_interfaces/action/FlightCommand \
    '{command: 1, timeout_sec: 5.0, correlation_id: smoke-arm}' \
    >"$evidence_dir/arm.log" 2>&1
grep -q 'status: SUCCEEDED' "$evidence_dir/arm.log"

ros2 action send_goal --feedback /mission/execute \
    ed_uav_interfaces/action/ExecuteMission \
    '{mission_id: simulation-patrol, field_profile_id: simulation-arena, timeout_sec: 90.0}' \
    >"$evidence_dir/mission.log" 2>&1
ros2 topic echo --once /simulation/ground_truth/odom \
    >"$evidence_dir/mission-final-odom.log" 2>&1
grep -q 'status: SUCCEEDED' "$evidence_dir/mission.log"

sim_session="$sim_pid"
kill -INT -- "-$sim_pid" 2>/dev/null || true
if wait "$sim_pid"; then
    :
else
    sim_status="$?"
    [[ "$sim_status" == 0 || "$sim_status" == 130 || "$sim_status" == 143 ]]
fi
sim_pid=""

if pgrep -s "$sim_session" >/dev/null 2>&1; then
    printf 'Gazebo process group still has children\n' >&2
    exit 1
fi
if pgrep -af '(^|/)(gz|ros2|rviz2|parameter_bridge)( |$)' >/dev/null 2>&1; then
    printf 'ROS/Gazebo child remains after SIGINT\n' >&2
    exit 1
fi

printf 'GAZEBO_SMOKE_SUCCESS\n' | tee "$evidence_dir/SUCCESS"
CONTAINER_SCRIPT
} | tee "$evidence_dir/runner.log"
