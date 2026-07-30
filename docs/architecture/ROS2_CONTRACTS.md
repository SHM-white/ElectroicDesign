# ROS 2 契约冻结

状态：已针对 2026-07-22 的代码/离线里程碑冻结。本文件和
`ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json` 是 ROS 图名称、坐标系、时序、QoS 和所有权的事实来源。

## 基线映射

以下内容描述定义任何 ROS 接口之前的当前进程内边界。这是对旧行为的刻画，不表示旧坐标已经满足 REP-103。

| 旧值 | 类型和单位 | 当前约定 | ROS 边界规则 |
| --- | --- | --- | --- |
| `VisionResult.green_ratio` | `float`，比值 `[0, 1]` | 图像结果标量 | 只有同时带有采集时间和相机坐标系来源信息时才能发布。 |
| `home_cross_center`、`start_marker_center`、`gray_marker_center` | 像素 `(u, v)` | 相机图像像素 | 绝不能视为世界位姿；应使用 `sensor_msgs/Image`、标准检测结果或部分观测。 |
| `gray_marker_box` | 像素 `(u, v, width, height)` | 相机图像像素 | 标准检测框保留此表示方式。 |
| 视觉置信度字段 | `float`，比值 `[0, 1]` | 检测器得分 | 不是位置协方差。 |
| `MCUSerial._of_pos_x`、`_of_pos_y`、`_of_dx`、`_of_dy` | `float`，cm | V7 相对于起飞点的位置；旧约定为 X 向前、Y 向右 | 仅 `ed_uav_fcu_bridge` 将其转换为米和 ROS ENU。`0x08` 是连续位置源。 |
| `MCUSerial._altitude` | 有符号整数，cm | V7 高度 | 仅 `ed_uav_fcu_bridge` 将其转换为米。 |
| `MCUSerial._voltage_mv` | 整数，mV | 电池电压值 | 以伏为单位发布标准 `sensor_msgs/BatteryState`。 |
| `MCUSerial._mode`、`_locked`、`_aux6` | 整数/位/脉冲 us | V7 模式；`locked=1` 表示已解锁 | 规范化为带类型的 FCU 状态，并保留源序列号和采集时间。 |
| `cmd_move(distance_cm, speed_cmps, direction_deg)` | cm、cm/s、度 | 相对于机体：0 为机头前方，顺时针为正 | 仅 `ed_uav_fcu_bridge` 将获批准的 SI/ENU 命令转换为 V7。 |

旧版 `DroneStateMachine` 是当前的执行器仲裁器。ROS 将其所有权替换为唯一的动作服务端所有者 `ed_uav_fcu_bridge`；任务客户端和安全客户端从不打开 FCU 端点。V7 `0x41` 仅作为 bridge 内部的 MOVE/HOVER 实时后端，不新增图接口、不进入 ACK 控制器，并同时受源码回退宏和默认关闭的运行时硬件门禁约束。

## 图契约

经检查的清单列出了完整的获批准图。本次冻结不批准任何其他话题、服务、动作、TF 边或硬件所有者。

| 话题 | 类型 | 所有者 | 坐标系 | QoS 和新鲜度 |
| --- | --- | --- | --- | --- |
| `/fcu/state` | `FcuState` | FCU bridge 节点 | `base_link` | `state_reliable`, 0.50 s |
| `/fcu/battery` | `BatteryState` | FCU bridge 节点 | `base_link` | `state_reliable`, 1.00 s |
| `/fcu/optical_flow/odom` | `Odometry` | FCU bridge 节点 | `odom` | `state_reliable`, 0.20 s |
| `/fcu/diagnostics` | `DiagnosticArray` | FCU bridge 节点 | `base_link` | `state_reliable`, 0.50 s |
| `/rangefinder/range` | `Range` | FCU bridge 节点 | `rangefinder_link` | sensor best-effort, 0.20 s |
| `/camera/narrow/image_raw`, `/camera/narrow/camera_info` | `Image`, `CameraInfo` | 窄相机节点 | narrow optical | sensor best-effort, 0.20 s; latched reliable |
| `/camera/wide/image_raw`, `/camera/wide/camera_info` | `Image`, `CameraInfo` | 宽相机节点 | wide optical | sensor best-effort, 0.20 s; latched reliable |
| `/lidar/points`, `/lidar/imu` | `PointCloud2`, `Imu` | LiDAR 节点 | `lidar_link` | sensor best-effort, 0.15 s |
| `/localization/lio/odom` | `Odometry` | LIO 适配器 | `odom` | `state_reliable`, 0.15 s |
| `/localization/boundary_observation` | `BoundaryObservation` | 边界感知节点 | wide optical | `state_reliable`, 0.20 s |
| `/localization/status`, `/localization/odom` | `LocalizationStatus`, `Odometry` | 定位监督器、EKF | `map`, `odom` | `state_reliable`, 0.20/0.15 s |
| `/perception/narrow/detections` | `Detection2DArray` | 窄相机感知节点 | narrow optical | `state_reliable`, 0.20 s |
| `/diagnostics` | `DiagnosticArray` | bringup 聚合器 | `base_link` | `state_reliable`, 1.00 s |
| `/d_task/vehicle/telemetry` | `VehicleTelemetry` | 地面车辆 bridge | `vehicle_start` | `state_reliable`, 0.50 s |
| `/d_task/target_observation` | `TargetObservation` | 目标感知节点 | message `frame_id` | sensor best-effort, 0.20 s |
| `/d_task/mission_status` | `MissionStatus` | 任务节点 | `map` ENU | `state_reliable`, 1.00 s |
| `/d_task/payload_contact_state` | `PayloadContactState` | 载荷 bridge | `base_link` | `state_reliable`, 0.20 s |

`/localization/start_map_session` 仅由地图归档拥有，并使用 `StartMapSession`；不包含已保存地图加载或重新定位。
`/fcu/flight_command` 仅由 FCU bridge 拥有，并使用 `FlightCommand`。`/mission/execute` 仅由任务软件包拥有，并使用 `ExecuteMission`。
`/d_task/pre_arm/select_mission` 也仅由任务软件包拥有，只接受解锁前的 `SelectDTaskMission` 请求。

### 仅仿真图

以下条目仅获准用于仅仿真的 Gazebo FAST-LIO 和仅规划器图：`/fast_lio/odometry`、`/fast_lio/cloud_registered`、
`/fast_lio/laser_map`、`/fast_lio/path` 和 `/fast_lio/tf`；规范适配器输出
`/localization/lio/cloud_registered`、`/localization/lio/map` 和
`/localization/lio/path`；Nav2 的 `/map` 话题；以及
`/compute_path_to_pose` 动作。FAST-LIO 原始话题仍是仿真图的私有内容。`/fast_lio/tf` 是私有的 `TFMessage` 话题，不是全局 TF 边。适配器将 LIO 的 `camera_init` 世界坐标系重命名为 `odom`，用于规范可视化输出，但不发布 TF。

所有名称均为绝对名称，并位于固定的 `/fcu`、`/rangefinder`、
`/camera/narrow`、`/camera/wide`、`/lidar`、`/localization`、`/perception`、
`/mission`、`/d_task` 和 `/diagnostics` 命名空间。`/tf` 和 `/tf_static` 只承载下文指定的权威来源，不是备用数据接口。

所有物理量均使用 SI，所有世界坐标和机体坐标均按 REP-103 使用 ENU。图像值在其光学坐标系中仍为像素。
消息头和采集时间戳使用设备或数据源的 ROS 时间。新鲜度始终使用本地 steady clock 测量，绝不能用可能回退的 ROS 时间相减。

QoS 配置固定如下：`sensor_data_best_effort` 表示 keep-last 5、best-effort、volatile；
`state_reliable` 表示 keep-last 10、reliable、volatile；`latched_reliable` 表示 keep-last 1、reliable、transient-local；
`command_reliable` 表示 keep-last 10、reliable、volatile。确切的配置名称、采集时钟和新鲜度期限是清单中的必填字段。

`map -> odom` 只有一个发布者，即 `ed_uav_localization.field_anchor`。
`odom -> base_link` 只有一个发布者，即 `ed_uav_localization.source_supervisor`。
`ed_uav_description.robot_state_publisher` 发布从静态 `base_link` 到
`fcu_link`、`lidar_link`、`camera_narrow_optical_frame`、
`camera_wide_optical_frame` 和 `rangefinder_link` 的边。任何其他组件都不得发布这些边。

清单规定了生命周期激活顺序：bridge 和传感器先获取硬件独占所有权；定位等待新鲜且符合条件的输入；
任务等待有效标定/配置、已激活定位、新鲜的启动事件和 FCU 控制权。定位完全丢失时，后续的安全所有者先命令悬停，再控制降落；绝不会在空中自动加锁电机。

## 接口规则

`FcuState` 携带来源、状态、采集时间、序列号、规范化的 FCU 模式/解锁状态以及 SI 遥测数据。
`LocalizationStatus` 携带来源、状态和有长度上限的原因文本。`BoundaryObservation` 包含
`observable_dof_mask`；其位姿中未设置的 DOF 没有定义。`FlightCommand` 和 `ExecuteMission`
使用有长度上限的关联、身份和原因字段。`StartMapSession` 使用有长度上限的 ID 和路径。
自定义接口不得使用无长度上限的字符串或序列。

每个 D-task 自定义消息和解锁前请求都携带契约版本 1。`VehicleTelemetry` 携带启动时间戳/事件、心跳、采集时间、源序列号、CRC-16、SI 单位的位移或轮速、转弯类别、弧度单位且逆时针为正的航向、弧度/秒单位的有符号偏航角速度，以及有序的 START/B/D/A/COMPLETE 状态。航向以车辆 x 轴前方相对于 `vehicle_start` 消息坐标系表示。`TargetObservation` 指定获批准的 `d2026-circle-cross-v1` 几何和相机采集坐标系；对每幅处理过的图像发布有效/拒绝状态、候选数量、重投影 RMS、质量、协方差和有长度上限的拒绝原因。`MissionStatus` 和 `PayloadContactState` 发布有长度上限的操作员及接触/载荷状态，并且各自只有一个发布者。消费者使用本地 steady clock 测量新鲜度；ROS 采集时间戳只提供来源信息，绝不用于计算年龄。Vehicle 和 ESP32 源序列使用 uint32 序列号算法：模差 `1..2^31-1` 表示前进，零表示重复，模差 `2^31..2^32-1` 表示过期。因此 `UINT32_MAX -> 0` 是有效回绕，而 `8 -> 7` 会被拒绝。

严格的 D-task 模式和不含凭据的示例位于
`ed_uav_interfaces/contracts/d_task`。真实 Mid-360 序列号、传感器/主机 IP、固件和地面站对端值只允许出现在被 git 忽略的 `deployment_preset.local.yaml` 中；现场加载会拒绝占位值和 RFC 5737 文档地址，不会用默认值替代。

`.msg` 和 `.action` 源文件中的枚举值已冻结：FCU 来源为 V7 或
仿真器；FCU 模式为 stabilize（0）、altitude hold（1）、position hold（2）或 program（3）。定位源为 none、LIO、visual boundary 或 fused，状态为 uninitialized、active、degraded 或 lost。边界掩码位为 X=1、Y=2、Z=4、roll=8、pitch=16、yaw=32。飞行动作为 arm、disarm、mode、takeoff、move、hover 和 land，结果为 succeeded/rejected/timeout/FCU-error。任务结果为 succeeded、rejected、aborted 或 timeout。没有任何枚举直接映射到线协议帧号；bridge 可在内部把 MOVE/HOVER 选择为 V7 `0x41` 实时流，源码宏关闭时仍使用原 `0xE0` 高级命令。

使用以下命令运行独立检查：

```bash
./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_contract.py \
  ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json

./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_d_task_config.py \
  mission ros2_ws/src/ed_uav_interfaces/contracts/d_task/examples/mission_profile.example.yaml
```
