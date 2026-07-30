#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=========================================="
echo "  Stability Test Simulation Launcher"
echo "=========================================="
echo ""

# Check if install directory exists
if [[ ! -d "$repo_root/ros2_ws/install" ]]; then
    echo "ERROR: No install directory found. Please build first with:"
    echo "  ./tools/run_humble.sh bash -lc 'cd /workspace && colcon build --symlink-install'"
    exit 1
fi

echo "Using existing build from: $repo_root/ros2_ws/install"
echo ""

# Find the stability test mission config
MISSION_CONFIG="$repo_root/ros2_ws/src/ed_uav_mission/config/missions/simulation_stability_test.yaml"
if [[ ! -f "$MISSION_CONFIG" ]]; then
    echo "ERROR: stability test mission config not found at $MISSION_CONFIG"
    exit 1
fi

echo "Mission config: $MISSION_CONFIG"
echo ""
echo "Launching simulation..."
echo ""

# Launch simulation
HUMBLE_GUI=1 \
HUMBLE_INTERACTIVE=1 \
HUMBLE_TIMEOUT_SECONDS="${HUMBLE_TIMEOUT_SECONDS:-0}" \
bash "$repo_root/tools/run_humble.sh" bash -c '
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

export GZ_SIM_RESOURCE_PATH="/workspace/ros2_ws/src/ed_uav_gazebo/models:/workspace/ros2_ws/src/ed_uav_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH"

echo "=== Starting Gazebo simulation ==="
echo "Mission config: /workspace/ros2_ws/src/ed_uav_mission/config/missions/simulation_stability_test.yaml"
echo ""

ros2 launch ed_uav_gazebo sim.launch.py \
    gui:=true \
    use_rviz:=true \
    mission_config:="/workspace/ros2_ws/src/ed_uav_mission/config/missions/simulation_stability_test.yaml"
'
