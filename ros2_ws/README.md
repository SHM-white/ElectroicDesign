# ROS 2 工作空间

`ros2_ws/src` 包含 ED UAV ROS 2 软件包。请通过 Humble 运行器构建和测试工作空间，
不要在 Ubuntu 24.04 开发主机上安装 Humble。

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon list --base-paths ros2_ws/src && ros2 doctor --report'
```

`colcon list --base-paths ros2_ws/src` 报告软件包工作空间。针对 CI 的构建和测试门禁也请使用
同一个运行器：

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && rosdep install --from-paths ros2_ws/src --ignore-src -r -y && colcon build --symlink-install && colcon test --event-handlers console_direct+ && colcon test-result --all --verbose'
```

镜像平台为 `linux/amd64`，基于 `docker/Dockerfile.humble` 中声明的按摘要固定的
`ros:humble-ros-base-jammy` manifest。`tools/run_humble.sh` 仅在 Ubuntu 22.04 上使用原生
`/opt/ros/humble`，其他主机使用 Docker 或 Podman。镜像会在
`/usr/local/share/ed-humble-toolchain-versions.txt` 中记录已安装的软件包和 Python 工具版本。

对于交互式、仅仿真的 Gazebo FAST-LIO 和仅规划器的 Nav2 竞赛任务，请运行
`./tools/run_gazebo_slam_nav.sh`。它会为每次运行保存隔离的第三方源代码和构建证据；请参阅
[`docs/testing/GAZEBO_SLAM_NAV.md`](../docs/testing/GAZEBO_SLAM_NAV.md) 了解
其中的前置条件和限制。

对于外部提供的真实 Livox/FAST-LIO/定位链，请在仓库根目录运行
`./tools/run_lidar_odometry_accuracy_demo.sh`。它会预检查 `/localization/odom`，采集有界的里程计试验，
且不会启动硬件、FAST-LIO、FCU、任务、动作或 Gazebo。请参阅
[`docs/localization/REAL_LIDAR_ODOMETRY_DEMO.md`](../docs/localization/REAL_LIDAR_ODOMETRY_DEMO.md)
了解其中的一键式静止、环行和直线工作流。
