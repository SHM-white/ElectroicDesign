#!/bin/bash
. /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/workspace/ros2_ws/install/livox_sdk2/lib:${LD_LIBRARY_PATH:-}
cd /workspace/ros2_ws
. install/setup.bash
timeout 35 ros2 launch ed_uav_gazebo sim.launch.py gui:=false use_rviz:=false auto_start:=true 2>&1 | grep -E 'LIO-RAW|FUSER|ODOM-GT|No Effective|No point' | head -30
