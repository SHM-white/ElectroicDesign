# ED UAV Gazebo Fortress 仿真器

`ed_uav_gazebo` 提供 D 题整套无硬件闭环。默认场景按仓库中的《陆空协同无人机系统（D题）》图 1
建立：作业区 4 m × 5 m，H 点中心为 `(1.125, 1.125)`，小车路线为
`A(1.5, 2.0) → B(1.5, 3.5) → C(3.0, 3.5) → D(3.0, 2.0) → A`，两端半圆半径
0.75 m。场地四周有 2.5 m 围墙，并增加了不对称的室内雷达反射几何，避免完全对称场景造成的
平面定位退化。

小车是 Gazebo 中的动态模型。它按 0.15 m/s 走完一圈，并根据实际模型里程计发布
`/vehicle/telemetry`。车顶保留题图中的 50 cm 外圈、30 cm 内圈和十字，同时在正中心覆盖一张
15 cm × 15 cm 的 `tag36h11` ID 0 AprilTag；检测端只接受同一 ID。

四旋翼使用 Gazebo Fortress 原生速度控制和四电机模型。两个相机与测距仪朝下；水平激光雷达只有
一个垂直采样层，不再伪装成多线雷达。默认定位链路为：

```text
/lidar/points_raw -> pointcloud_normalizer -> FAST-LIO -> lio_adapter
                    -> planar_odom_fuser -> /localization/lio/odom
                    -> source_supervisor -> /localization/odom
```

`planar_odom_fuser` 只从 FAST-LIO 取 X/Y/偏航，从 Gazebo 真值取不可观的 Z 和垂直速度；不会用真值
替换平面位姿。输出限制异常 Z 跳变并去掉横滚/俯仰，因此适合验证本题所需的连续里程计。
本项目不保存地图、不做跨进程重定位；仿真重启后里程计重新从当前起点初始化。

构建并启动：

```bash
./tools/build_sim_packages.sh
./tools/run_competition.sh --simulation --task 1 --no-display
```

将 `--task 1` 改为 `--task 2` 可运行动态平台降落流程；去掉 `--no-display` 会在可用的图形环境中
启动 Gazebo 和 RViz。兼容入口仍可直接启动：

```bash
ros2 launch ed_uav_gazebo sim.launch.py gui:=false use_rviz:=false
```

该路径仅验证 ROS 图、任务编排、Gazebo 动力学、传感器和连续定位，不验证 V7 固件、HIL 时序、
串口硬件或实飞安全。
