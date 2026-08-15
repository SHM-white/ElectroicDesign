# Gazebo Fortress D 题仿真

这条路径仅用于仿真，用来验证 D 题 ROS 闭环；它不是实飞、V7 固件、HIL 或串口硬件测试。

## 场地与目标

场景按仓库根目录《陆空协同无人机系统（D题）》中的尺寸建立：

- 作业区为 4 m × 5 m，H 点、A/B/C/D 点和 0.75 m 半径胶囊路线使用题图坐标；
- 四周 2.5 m 围墙模拟室内安全网边界，并布置非对称反射板供二维雷达约束 X/Y/偏航；
- 小车按 0.15 m/s 运行一圈，遥测来自 Gazebo 实际模型位置；
- 靶面中心以 15 cm × 15 cm 的 `tag36h11` ID 0 AprilTag 覆盖原十字中心。

## 定位边界

激光雷达是严格的单层二维扫描。FAST-LIO 负责连续的 X/Y/偏航里程计；
`planar_odom_fuser` 仅用仿真高度补齐二维扫描无法观测的 Z，并限制异常高度跳变。验证时应观察：

- `/lidar/points`：规范化后的单层 `PointCloud2`；
- `/fast_lio/odometry`：FAST-LIO 原始结果；
- `/localization/lio/odom`：补齐 Z 后的平面 LIO；
- `/localization/odom`：任务实际使用的连续里程计；
- `/localization/status`：应进入 `STATE_ACTIVE` 且 `map_to_odom_valid=true`。

项目只要求连续里程计，因此不保存 PCD 地图，也不实现跨次启动的全局重定位。Gazebo 或定位节点重启后，
当前里程计会重新初始化；同一次运行内不得出现 Z 漂移或不连续跳变。

## 运行

首次构建：

```bash
./tools/build_sim_packages.sh
```

无头自动闭环：

```bash
./tools/run_competition.sh --simulation --task 1 --no-display
```

图形模式与任务 2：

```bash
./tools/run_competition.sh --simulation --task 2 --enable-display
```

旧入口 `./tools/run_gazebo_sim.sh` 和 `./tools/run_gazebo_smoke.sh` 仍保留；前者为交互式运行，后者为
有界冒烟验证。仿真路径不会打开真实串口，也不会启动 HMI/小车 UDP 实机端口。
