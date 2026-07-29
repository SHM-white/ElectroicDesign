# Gazebo FAST-LIO 仅规划器任务

在 WSLg 会话中运行集成的、仅用于仿真的 FAST-LIO 与 Nav2 仅规划器竞赛任务，并确保 `DISPLAY`、`WAYLAND_DISPLAY`、`XDG_RUNTIME_DIR` 和 `/mnt/wslg` 可用：

```bash
./tools/run_gazebo_slam_nav.sh
```

该命令在 GUI 和交互模式下使用 `tools/run_humble.sh`。首次执行时，它只导入 `ros2_ws/dependencies.repos` 中为 `livox_sdk2`、`livox_ros_driver2` 和 `fast_lio_ros2` 固定的修订版本，包括 FAST-LIO 的 `ikd-Tree` 子模块。它会在本次运行的证据目录下配置、构建和安装 Livox SDK2，记录每条 SDK 日志，并将该私有库目录和头文件目录传给驱动构建。运行器会在证据目录中的 FAST-LIO 检出副本上应用 `tools/patches/fast_lio_simulation.patch` 后再构建。该补丁使用两秒、200 个样本的 IMU 初始化窗口，并初始化上一帧激光扫描结束时间戳；补丁必须零 fuzz 应用，若固定源码不再匹配则失败。源码导入、构建、安装、colcon 日志、启动日志、话题/动作就绪日志和动作结果均保存在 `.omo/evidence/gazebo/<run-id>/` 下；它不会将第三方源码写入 `ros2_ws/src`，也不会全局安装 SDK2。

运行器构建 Livox 和 FAST-LIO 的 ROS 2 版本，使用 `localization_mode:=fast_lio` 启动 `ed_uav_gazebo gazebo_simulation.launch.py`、Gazebo GUI 和 RViz，并等待 LIO、地图、规划器、FCU 和任务接口就绪。它要求定位状态为 ACTIVE 且存在有效的 `map -> odom` 变换，然后才通过 `/fcu/flight_command` 解锁，向 `/mission/execute` 发送 `simulation-competition`。它会记录成功的动作结果，并确认模拟器 FCU 在本次运行成功前已解除武装。

任务成功后，Gazebo 和 RViz 会保持打开以供检查。按 `Ctrl-C` 关闭会话，这是成功任务后的正常完成方式，并会停止启动进程组。

## 限制

这不是硬件、HIL、固件、传感器标定或飞行安全测试。仿真的通用 `PointCloud2` 流不具备真实 Livox Mid-360 的逐点时间保真度。Nav2 仅作为规划器，规划固定高度的 XY 路径；它不会启动 Nav2 控制器或 `bt_navigator`，也不会发布 `cmd_vel`。任务使用模拟器的 `FlightCommand` 动作而非硬件驱动，因此不能证明硬件飞行能力。
