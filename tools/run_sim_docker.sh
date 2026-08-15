#!/usr/bin/env bash
# 在 WSL 中使用 Windows Docker 运行仿真
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WIN_REPO_ROOT=$(wslpath -w "$REPO_ROOT")

echo "=========================================="
echo "  使用 Windows Docker 运行仿真"
echo "=========================================="
echo ""
echo "项目路径: $REPO_ROOT"
echo "Windows路径: $WIN_REPO_ROOT"
echo ""

# 使用 Windows Docker 运行
docker.exe run --rm \
    -v "${WIN_REPO_ROOT}:/workspace" \
    -w /workspace \
    ed-humble-toolchain:jammy-humble \
    bash -c "
        source /opt/ros/humble/setup.bash
        export LD_LIBRARY_PATH=/workspace/ros2_ws/install/livox_sdk2/lib:\${LD_LIBRARY_PATH:-}
        cd /workspace/ros2_ws
        source install/setup.bash
        
        echo '启动仿真...'
        ros2 launch ed_uav_gazebo sim.launch.py \
            gui:=false \
            use_rviz:=false \
            auto_start:=true
    "
