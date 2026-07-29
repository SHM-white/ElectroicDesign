# ED UAV Gazebo Fortress 仿真器

`ed_uav_gazebo` 启动一个无硬件的 Gazebo Fortress 场景，其中包含本地四旋翼模型、原生
`MulticopterVelocityControl`、四个原生 `MulticopterMotorModel` 系统、相机、GPU 激光雷达、IMU、
向下射线传感器、真实值里程计，以及 ED 软件栈使用的 ROS bridge 契约。

支持的一键路径会将固定版本的 FAST-LIO、Livox 驱动和 Livox SDK2 源代码导入隔离的证据目录，
构建覆盖层，并启动 Gazebo 和 RViz：

```bash
./tools/run_gazebo_slam_nav.sh
```

在这些依赖和工作空间已经构建并 source 后，也可以直接启动该启动文件：

```bash
ros2 launch ed_uav_gazebo gazebo_simulation.launch.py use_sim_time:=true use_rviz:=true
```

仿真器 FCU 动作服务器是 `/fcu/flight_command` 的唯一所有者。它使用真实的
`/simulation/ground_truth/odom`，发布仿真器 FCU 状态和诊断信息，但集成启动会禁用其 TF 输出。
`ed_uav_localization.source_supervisor` 是动态 `odom -> base_link` 的唯一发布者，
而 `field_anchor` 负责 `map -> odom`。静态传感器变换仍由 `robot_state_publisher` 负责。
GPU 激光雷达发布 `PointCloudPacked`，bridge 将其转换为标准的 `/lidar/points` `PointCloud2`；
默认仿真模式将该流和 `/lidar/imu` 提供给 FAST-LIO。

此软件包用于验证 ROS/任务/传感器集成和 Gazebo 原生飞行器动力学。
它不验证 V7 固件行为、HIL 时序、串行硬件或飞行安全。
