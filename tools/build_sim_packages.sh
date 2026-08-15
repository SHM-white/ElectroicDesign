#!/usr/bin/env bash
# Build the complete real/simulation dependency closure.
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=========================================="
echo "  Building Simulation Packages"
echo "=========================================="
echo ""

# Build only the necessary packages
bash "$repo_root/tools/run_humble.sh" bash -c '
source /opt/ros/humble/setup.bash
set -eo pipefail
cd /workspace/ros2_ws

echo "=== Step 1: Build livox_sdk2 ==="
colcon build --packages-select livox_sdk2 --symlink-install --event-handlers console_direct+

echo ""
echo "=== Step 2: Activate workspace livox_sdk2 ==="
source install/setup.bash
export LD_LIBRARY_PATH="/workspace/ros2_ws/install/livox_sdk2/lib:${LD_LIBRARY_PATH:-}"

echo ""
echo "=== Step 3: Build livox_ros_driver2 ==="
colcon build --packages-select livox_ros_driver2 --symlink-install \
    --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
    --event-handlers console_direct+

echo ""
echo "=== Step 4: Build fast_lio ==="
source install/setup.bash
colcon build --packages-select fast_lio --symlink-install --event-handlers console_direct+

echo ""
echo "=== Step 5: Build remaining packages ==="
colcon build \
    --packages-select \
        ed_uav_interfaces \
        ed_uav_verification \
        ed_uav_description \
        ed_uav_camera \
        ed_uav_localization \
        ed_uav_mission \
        ed_uav_fcu_bridge \
        ed_uav_vehicle_bridge \
        ed_uav_gazebo \
        ed_uav_bringup \
        ed_uav_navigation \
        ed_uav_perception \
        ed_uav_lidar \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+

echo ""
echo "=== Build complete ==="
'

echo ""
echo "Now you can run: ./tools/run_competition.sh --simulation"
