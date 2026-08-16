#!/bin/bash
# 快捷脚本：在运行中的 Docker 容器内执行 ROS2 命令
CONTAINER=$(docker ps --format "{{.Names}}" | grep "ed-humble-run" | head -1)

if [ -z "$CONTAINER" ]; then
    echo "错误：没有找到运行中的 ed-humble 容器"
    exit 1
fi

echo "容器: $CONTAINER"
echo "执行: $@"
echo "---"

docker exec "$CONTAINER" /bin/bash -c "
    source /opt/ros/humble/setup.bash 2>/dev/null
    source /workspace/ros2_ws/install/setup.bash 2>/dev/null
    $@
"
