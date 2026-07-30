#!/usr/bin/env bash
# Build only the packages needed for stability test simulation
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=========================================="
echo "  Building Simulation Packages"
echo "=========================================="
echo ""

# Build only the necessary packages
bash "$repo_root/tools/run_humble.sh" bash -c '
source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws

echo "=== Step 1: Build livox_sdk2 ==="
colcon build --packages-select livox_sdk2 --symlink-install 2>&1 | tail -5

echo ""
echo "=== Step 2: Install livox_sdk2 library to /usr/local/lib ==="
source install/setup.bash
cp install/livox_sdk2/lib/liblivox_lidar_sdk_shared.so /usr/local/lib/
ldconfig

echo ""
echo "=== Step 3: Build livox_ros_driver2 ==="
colcon build --packages-select livox_ros_driver2 --symlink-install 2>&1 | tail -10

echo ""
echo "=== Step 4: Build fast_lio ==="
colcon build --packages-select fast_lio --symlink-install 2>&1 | tail -10

echo ""
echo "=== Step 5: Build remaining packages ==="
colcon build \
    --packages-select \
        ed_uav_interfaces \
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
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -15

echo ""
echo "=== Build complete ==="
'

echo ""
echo "Now you can run: ./tools/run_stability_test_sim.sh"
