# Livox Mid-360 实机里程计演示

本流程只采集实机相对里程计表现，不控制飞行器。仓库当前没有真实世界的基准真值，
所以 `status: passed` 只表示采集按配置完成，不表示精度合格。报告中的协方差、静止
漂移、回环残差和尺度误差都不能单独证明绝对精度。

## 一条命令运行

先在另一终端启动并确认现场已完成部署的 Livox、ROS 2 FAST-LIO 和定位输出链。
本运行器不启动 Livox、FAST-LIO、FCU、任务、动作或 Gazebo；它只构建
`ed_uav_localization`、验证里程计并采集结果。从仓库根目录执行：

```bash
./tools/run_lidar_odometry.sh
```

无参数默认执行 60 秒静止试验，要求至少 100 个样本。回环试验可显式传递演示参数；话题统一由
`ODOM_TOPIC` 指定，不能传递 `--odom-topic`，以确保预检与采集使用同一输入：

```bash
ODOM_TOPIC=/localization/odom \
  ./tools/run_lidar_odometry.sh --mode loop --duration-sec 60 --min-samples 100
```

直线试验必须提供人工实测的水平距离，不能从 odom 反算：

```bash
read -r -p "Physically measured level distance in meters: " KNOWN_DISTANCE_M
ODOM_TOPIC=/localization/odom \
  ./tools/run_lidar_odometry.sh \
  --mode straight_line --duration-sec 60 --min-samples 100 \
  --known-distance-m "$KNOWN_DISTANCE_M"
```

每次运行会在 `.omo/evidence/lidar-odometry/<UTC>-<pid>/` 保存命令、预检和演示输出。仅当演示
发出一条有效 JSON 对象结果且包含必需报告字段时，运行器才会写入 `result.json` 并打印该路径。若
`/localization/odom` 缺失、类型错误、没有发布者或在限定时间内没有消息，先修复现场链路；运行器
会在开始采集前退出且不会报告指标。

已构建且 `ros2_ws/install/setup.bash` 存在的工作区可跳过重复构建：

```bash
ED_ODOMETRY_DEMO_SKIP_BUILD=1 ./tools/run_lidar_odometry.sh
```

若 overlay 工作空间不存在，去掉 `ED_ODOMETRY_DEMO_SKIP_BUILD=1` 重新运行以构建 `ed_uav_localization`。

## 1. 安全边界

开始前必须同时满足：

1. 螺旋桨已拆除，电机供电已物理断开。给传感器上电时，电机仍必须物理无法得电。
2. 机体放在稳定、水平、通风的地面或工作台上，Mid-360 刚性固定，线缆有应力释放。
3. 不启动 `ed_uav_fcu_bridge`、任务节点或任何动作客户端。本流程不需要 FCU。
4. Mid-360 外壳温度保持在文档限制内。发现松动、异常发热、异味或网络线脱落时立即断电。
5. 已实测 `base_link` 到 `lidar_link` 外参，并使用真实设备标定文件。不要使用合成或
   占位外参。

## 2. 构建

在装有 ROS 2 Humble 的硬件主机上执行。Ubuntu 24.04 开发机的容器运行器不等同于已配置
硬件网络的目标机。

```bash
cd /home/xtyf/ed/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to ed_uav_lidar ed_uav_localization
source install/setup.bash
ros2 pkg executables ed_uav_lidar
ros2 pkg executables ed_uav_localization | grep odometry_accuracy_demo
ros2 pkg executables livox_ros_driver2
```

`livox_ros_driver2` 和现场使用的 FAST-LIO ROS 2 实现必须已单独安装、固定版本并完成实机配置。
本仓库不提供可直接用于实机的 FAST-LIO 启动命令。

## 3. Mid-360 启动骨架

仓库内的 `config/lidar.yaml` 和 `config/mid360_driver.json` 是占位配置，禁止实机使用。
准备另一份现场 JSON，其中设备 IP、主机网卡 IP、端口和其他 Livox 字段均已按该台设备验证。
启动参数中的序列号、IP、固件版本必须来自设备标签、设备工具或现场记录，不能猜测。

```bash
source /opt/ros/humble/setup.bash
source /home/xtyf/ed/ros2_ws/install/setup.bash
read -r -p "Field-verified Mid-360 serial: " MID360_SERIAL
read -r -p "Field-verified Mid-360 sensor IP: " MID360_SENSOR_IP
read -r -p "Field-verified Mid-360 firmware: " MID360_FIRMWARE
read -r -p "Absolute path to field-verified driver JSON: " MID360_DRIVER_JSON
test -f "$MID360_DRIVER_JSON"
ros2 launch ed_uav_lidar lidar.launch.py \
  lidar_enabled:=true \
  transport:=mid360 \
  serial_number:="$MID360_SERIAL" \
  sensor_ip:="$MID360_SENSOR_IP" \
  firmware_version:="$MID360_FIRMWARE" \
  driver_config_path:="$MID360_DRIVER_JSON" \
  time_authority:=host
```

`time_authority:=host` 只会报告 `HOST_TIME_UNVERIFIED`，不证明时间同步。若现场已独立配置并测量
PTP，可按已验证方案改为 `ptp`，但 `PTP_CONFIGURED_UNVERIFIED` 本身也不是测量证据。

该启动文件提供 `/livox/lidar` 和 `/livox/imu`，不会启动 FAST-LIO。继续前，现场 FAST-LIO、
实测外参适配和定位输出链必须已经完成部署，最终输出必须是规范话题
`/localization/odom`。不要为了本演示启动 FCU bridge、任务或动作。

## 4. 预检

在另一个已加载同一工作空间环境的终端执行：

```bash
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/imu
ros2 topic info -v /localization/odom
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /localization/odom
ros2 topic echo /livox/lidar --once
ros2 topic echo /livox/imu --once
ros2 topic echo /localization/odom --once
```

每个 `hz` 观察至少 10 秒后按 `Ctrl-C`。确认两路 Livox 数据持续更新，
`/localization/odom` 类型为 `nav_msgs/msg/Odometry`，发布者、订阅者和 QoS 与现场链路相符。
单次里程计应有非空且稳定的 `header.frame_id`、严格递增时间戳和有限位姿。任何话题缺失、停更、
坐标系改变或时间戳回退都先修复，不能开始演示。
若在隔离话题上手工发布合成里程计，消息 YAML 必须显式包含 `header: {frame_id: odom}`。
不要使用 `header: auto`，ROS 2 Humble 会让 `frame_id` 保持为空，并按 `empty_frame` 拒绝该消息。

## 5. 三项演示

所有试验均由操作员手持或推移已断开电机供电的机体。运行器收到第一条里程计后开始按消息时间计时，
并保留演示的退出码和单个 JSON 结果。

### 5.1 静止 60 秒

机体保持水平并完全不动。规范时长是 30 到 60 秒，本命令取 60 秒。

```bash
./tools/run_lidar_odometry.sh
```

### 5.2 操作员声明的 return-to-mark 回环

在水平地面标出起点和机体朝向。启动命令后沿现场安全路径移动，最后由操作员确认机体回到同一标记
和朝向，并在 60 秒结束前停稳。该标记是操作员声明的回点，不是测量系统的真实基准。

```bash
ODOM_TOPIC=/localization/odom \
  ./tools/run_lidar_odometry.sh --mode loop --duration-sec 60 --min-samples 100
```

### 5.3 水平直线实测距离

用卷尺在水平地面测量起点到终点的直线距离，标记两个端点。不要用 odom 反算该距离。启动后沿直线
平移至终点，在 60 秒结束前停稳。提示框输入以米为单位的正数。

```bash
read -r -p "Physically measured level distance in meters: " KNOWN_DISTANCE_M
ODOM_TOPIC=/localization/odom \
  ./tools/run_lidar_odometry.sh \
  --mode straight_line --duration-sec 60 --min-samples 100 \
  --known-distance-m "$KNOWN_DISTANCE_M"
```

演示进程会输出一行 `ODOMETRY_ACCURACY_RESULT=<JSON>`。运行器只在观察到单条此前缀、且其为包含必需
字段的有效 JSON 对象时写入 `result.json`；非零退出时仍保留该结构化结果和原始演示输出。若运行
返回非零，先读 `status`，不要把失败文件当作测量结果。

## 6. JSON 与指标

`schema_version` 是报告版本，`status` 是完成状态，`trial` 是模式，`interpretation` 是解释边界。
`input_topic` 应为 `/localization/odom`，`frame_id` 和首尾时间戳来自消息，`duration_sec` 是两者之差。
`sample_count` 是接受数，`rejected_count` 是拒绝数，失败时 `metrics` 为 `null`。

| 模式 | 指标 | 含义 |
|---|---|---|
| `stationary` | `end_xy_drift_m`, `end_3d_drift_m`, `max_xy_excursion_m`, `path_length_m`, `yaw_delta_rad`, `xy_drift_rate_m_per_s` | 相对第一帧的静止表现和累计轨迹长度 |
| `loop` | `xy_residual_m`, `three_dimensional_residual_m`, `path_length_m` | 首尾相对闭合差和累计轨迹长度 |
| `straight_line` | `measured_xy_endpoint_displacement_m`, `measured_3d_endpoint_displacement_m`, `scale_factor_xy`, `scale_error_percent` | odom 首尾位移与人工实测水平距离的比值 |

这些值受放置误差、人工回点误差、路径水平度、外参、时间同步和环境几何影响。它们描述相对里程计行为，
不是协方差评估，也不是静止漂移、回环残差、尺度误差或系统绝对精度的独立证明。

## 7. 稳定状态与失败码

| `status` | 含义 |
|---|---|
| `passed` | 按配置收满时长和最少样本，仅表示采集完成 |
| `NO_SAMPLE_TIMEOUT` | 启动后 10 秒内没有首条 odom |
| `STALE_ODOMETRY` | 首条后连续 0.5 秒没有新 odom |
| `INSUFFICIENT_SAMPLES` | 到达时长时少于 `--min-samples` |
| `INVALID_CONFIGURATION` | CLI 参数格式、选项或通用配置无效 |
| `INTERRUPTED` | 运行中收到 `Ctrl-C`；输出结果后受控地以非零退出码退出 |
| `empty_frame` | odom `header.frame_id` 为空 |
| `nonfinite_pose` | 位置或 yaw 含非有限值 |
| `frame_changed` | 采集中 `header.frame_id` 改变 |
| `non_increasing_stamp` | 时间戳未严格递增 |
| `insufficient_samples` | 处理引擎收到少于两条样本 |
| `unexpected_known_distance` | 非直线模式传入 `--known-distance-m` |
| `missing_known_distance` | 直线模式缺少 `--known-distance-m` |
| `invalid_known_distance` | 实测距离不是有限正数 |

权威依据：[`LIDAR_MOUNT_AND_EXTRINSICS.md`](../calibration/LIDAR_MOUNT_AND_EXTRINSICS.md)、[`ed_uav_lidar/README.md`](../../ros2_ws/src/ed_uav_lidar/README.md)、[`odometry_accuracy_demo.py`](../../ros2_ws/src/ed_uav_localization/ed_uav_localization/odometry_accuracy_demo.py)、[`odometry_accuracy_report.py`](../../ros2_ws/src/ed_uav_localization/ed_uav_localization/odometry_accuracy_report.py)、[`odometry_accuracy.py`](../../ros2_ws/src/ed_uav_localization/ed_uav_localization/odometry_accuracy.py)。
