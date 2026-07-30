#!/usr/bin/env bash
# Run stability test mission with fake FCU (no Gazebo needed)
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=========================================="
echo "  Stability Test (Fake FCU Mode)"
echo "=========================================="
echo ""
echo "This runs the mission executor with a simulated FCU"
echo "without requiring Gazebo. Useful for testing mission logic."
echo ""
echo "Press Ctrl+C to stop"
echo ""

bash "$repo_root/tools/run_humble.sh" bash -c '
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

echo "=== Setting up fake FCU environment ==="
echo ""

# Create a temporary directory for the fake FCU
FAKE_FCU_DIR=$(mktemp -d)
trap "rm -rf $FAKE_FCU_DIR" EXIT

# Start fake FCU in background
echo "Starting fake FCU..."
ros2 run ed_uav_fcu_bridge fake_fcu --ros-args \
    -p pty_device:="$FAKE_FCU_DIR/fcu-pty" &
FAKE_FCU_PID=$!
sleep 2

# Check if fake FCU started
if ! kill -0 $FAKE_FCU_PID 2>/dev/null; then
    echo "ERROR: Failed to start fake FCU"
    exit 1
fi

echo "Fake FCU started: $FAKE_FCU_DIR/fcu-pty"
echo ""

# Start mission executor
echo "Starting mission executor with stability test config..."
echo ""
ros2 launch ed_uav_mission mission_executor.launch.py \
    use_sim_time:=true \
    profile_path:="/workspace/ros2_ws/src/ed_uav_localization/config/fields/simulation_arena.yaml" \
    mission_config_path:="/workspace/ros2_ws/src/ed_uav_mission/config/missions/simulation_stability_test.yaml" \
    calibration_file:="/workspace/ros2_ws/src/ed_uav_description/config/synthetic_calibrated.yaml" \
    simulation_only:=true &
MISSION_PID=$!

sleep 3

echo ""
echo "=== Mission executor started ==="
echo ""
echo "To send a mission goal, run in another terminal:"
echo ""
echo "  ros2 action send_goal /mission/execute ed_uav_interfaces/action/ExecuteMission \\"
echo "    \"mission_id: simulation-stability-test, field_profile_id: simulation-arena, timeout_sec: 120\" \\"
echo "    --feedback"
echo ""
echo "Waiting for Ctrl+C..."
echo ""

# Wait for interrupt
wait $MISSION_PID || true
'
