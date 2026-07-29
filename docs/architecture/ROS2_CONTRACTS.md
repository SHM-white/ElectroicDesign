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
| `MCUSerial._of_pos_x`, `_of_pos_y`, `_of_dx`, `_of_dy` | `float`, cm | V7 position relative to takeoff; legacy X forward, Y right | `ed_uav_fcu_bridge` alone converts to meters and ROS ENU. `0x08` is the continuous source. |
| `MCUSerial._altitude` | signed integer, cm | V7 altitude | `ed_uav_fcu_bridge` alone converts to meters. |
| `MCUSerial._voltage_mv` | integer, mV | battery electrical value | Publish standard `sensor_msgs/BatteryState` in volts. |
| `MCUSerial._mode`, `_locked`, `_aux6` | integer/bit/pulse us | V7 mode; `locked=1` means unlocked | Normalize to typed FCU state; preserve source sequence and acquisition time. |
| `cmd_move(distance_cm, speed_cmps, direction_deg)` | cm, cm/s, degrees | body-relative: 0 is nose-forward, clockwise positive | Only `ed_uav_fcu_bridge` converts an approved SI/ENU command to V7. |

Legacy `DroneStateMachine` is the current actuator arbiter. ROS replaces that
ownership with exactly one action-server owner, `ed_uav_fcu_bridge`; mission and
safety clients never open the FCU endpoint. V7 `0x41` is excluded.

## 图契约

经检查的清单列出了完整的获批准图。本次冻结不批准任何其他话题、服务、动作、TF 边或硬件所有者。

| 话题 | 类型 | 所有者 | 坐标系 | QoS 和新鲜度 |
| --- | --- | --- | --- | --- |
| `/fcu/state` | `FcuState` | FCU bridge | `base_link` | `state_reliable`, 0.50 s |
| `/fcu/battery` | `BatteryState` | FCU bridge | `base_link` | `state_reliable`, 1.00 s |
| `/fcu/optical_flow/odom` | `Odometry` | FCU bridge | `odom` | `state_reliable`, 0.20 s |
| `/fcu/diagnostics` | `DiagnosticArray` | FCU bridge | `base_link` | `state_reliable`, 0.50 s |
| `/rangefinder/range` | `Range` | FCU bridge | `rangefinder_link` | sensor best-effort, 0.20 s |
| `/camera/narrow/image_raw`, `/camera/narrow/camera_info` | `Image`, `CameraInfo` | narrow camera | narrow optical | sensor best-effort, 0.20 s; latched reliable |
| `/camera/wide/image_raw`, `/camera/wide/camera_info` | `Image`, `CameraInfo` | wide camera | wide optical | sensor best-effort, 0.20 s; latched reliable |
| `/lidar/points`, `/lidar/imu` | `PointCloud2`, `Imu` | lidar | `lidar_link` | sensor best-effort, 0.15 s |
| `/localization/lio/odom` | `Odometry` | LIO adapter | `odom` | `state_reliable`, 0.15 s |
| `/localization/boundary_observation` | `BoundaryObservation` | boundary perception | wide optical | `state_reliable`, 0.20 s |
| `/localization/status`, `/localization/odom` | `LocalizationStatus`, `Odometry` | localization supervisor, EKF | `map`, `odom` | `state_reliable`, 0.20/0.15 s |
| `/perception/narrow/detections` | `Detection2DArray` | narrow perception | narrow optical | `state_reliable`, 0.20 s |
| `/diagnostics` | `DiagnosticArray` | bringup aggregator | `base_link` | `state_reliable`, 1.00 s |
| `/d_task/vehicle/telemetry` | `VehicleTelemetry` | ground-vehicle bridge | `vehicle_start` | `state_reliable`, 0.50 s |
| `/d_task/target_observation` | `TargetObservation` | target perception | message `frame_id` | sensor best-effort, 0.20 s |
| `/d_task/mission_status` | `MissionStatus` | mission | `map` ENU | `state_reliable`, 1.00 s |
| `/d_task/payload_contact_state` | `PayloadContactState` | payload bridge | `base_link` | `state_reliable`, 0.20 s |

`/localization/start_map_session` is owned only by the map archive and uses
`StartMapSession`; saved-map loading/relocalization is excluded. `/fcu/flight_command`
is owned only by the FCU bridge and uses `FlightCommand`. `/mission/execute` is
owned only by the mission package and uses `ExecuteMission`.
`/d_task/pre_arm/select_mission` is also owned only by the mission package and
accepts `SelectDTaskMission` requests only before arming.

### 仅仿真图

The following entries are approved only for the simulation-only Gazebo FAST-LIO
and planner-only graph: `/fast_lio/odometry`, `/fast_lio/cloud_registered`,
`/fast_lio/laser_map`, `/fast_lio/path`, and `/fast_lio/tf`; canonical adapter
outputs `/localization/lio/cloud_registered`, `/localization/lio/map`, and
`/localization/lio/path`; the Nav2 `/map` topic; and the
`/compute_path_to_pose` action. FAST-LIO raw topics remain private to the
simulation graph. `/fast_lio/tf` is a private `TFMessage` topic, not a global
TF edge. The adapter relabels the LIO `camera_init` world frame to `odom` for
the canonical visualization outputs without publishing TF.

所有名称均为绝对名称，并位于固定的 `/fcu`、`/rangefinder`、
`/camera/narrow`, `/camera/wide`, `/lidar`, `/localization`, `/perception`,
`/mission`, `/d_task`, and `/diagnostics` namespaces. `/tf` and `/tf_static` carry only
the authorities named below; they are not alternative data interfaces.

所有物理量均使用 SI，所有世界坐标和机体坐标均按 REP-103 使用 ENU。图像值在其光学坐标系中仍为像素。
消息头和采集时间戳使用设备或数据源的 ROS 时间。新鲜度始终使用本地 steady clock 测量，绝不能用可能回退的 ROS 时间相减。

QoS 配置固定如下：`sensor_data_best_effort` 表示 keep-last 5、
best-effort, volatile; `state_reliable` means keep-last 10, reliable, volatile;
`latched_reliable` means keep-last 1, reliable, transient-local; and
`command_reliable` means keep-last 10, reliable, volatile. The exact profile
name, acquisition clock, and freshness deadline are mandatory manifest fields.

`map -> odom` has exactly one publisher, `ed_uav_localization.field_anchor`.
`odom -> base_link` has exactly one publisher,
`ed_uav_localization.source_supervisor`.
`ed_uav_description.robot_state_publisher` publishes the static `base_link`
edges to `fcu_link`, `lidar_link`, `camera_narrow_optical_frame`,
`camera_wide_optical_frame`, and `rangefinder_link`. No other component may
publish those edges.

清单规定了生命周期激活顺序：bridge 和传感器先获取
exclusive hardware ownership first; localization waits for fresh eligible input;
mission waits for valid calibration/profile, active localization, a fresh start
event, and FCU control authority. On total localization loss the later safety
owner commands hover then controlled land; it never automatically locks motors
in air.

## 接口规则

`FcuState` 携带来源、状态、采集时间、序列号、规范化的 FCU 模式/解锁状态以及 SI 遥测数据。
`LocalizationStatus` 携带来源、状态和有长度上限的原因文本。`BoundaryObservation` 包含
`observable_dof_mask`；其位姿中未设置的 DOF 没有定义。`FlightCommand` 和 `ExecuteMission`
使用有长度上限的关联、身份和原因字段。`StartMapSession` 使用有长度上限的 ID 和路径。
自定义接口不得使用无长度上限的字符串或序列。

Every D-task custom message and pre-arm request carries contract version 1.
`VehicleTelemetry` carries the start stamp/event, heartbeat, acquisition time,
source sequence, CRC-16, displacement or wheel speed in SI, turn class,
CCW-positive heading in radians, signed yaw rate in radians/second, and the
ordered START/B/D/A/COMPLETE state. Heading is vehicle x-forward relative to
the message `vehicle_start` frame. `TargetObservation` names the approved
`d2026-circle-cross-v1` geometry and its camera acquisition frame; it publishes
valid/rejected status, candidate count, reprojection RMS, quality, covariance,
and a bounded rejection reason for every processed image. `MissionStatus` and
`PayloadContactState` expose bounded operator and contact/payload states with a
single publisher owner. Consumers measure freshness on a local steady clock;
ROS acquisition stamps provide provenance and are never used as an age clock.
Vehicle and ESP32 source sequences use uint32 serial-number arithmetic: modulo
deltas `1..2^31-1` advance, zero is duplicate, and deltas `2^31..2^32-1` are
stale. Therefore `UINT32_MAX -> 0` is a valid wrap while `8 -> 7` is rejected.

严格的 D-task 模式和不含凭据的示例位于
`ed_uav_interfaces/contracts/d_task`. Real Mid-360 serial, sensor/host IP,
firmware, and ground-station peer values are permitted only in the gitignored
`deployment_preset.local.yaml`; field loading rejects placeholders and RFC 5737
documentation addresses rather than substituting defaults.

`.msg` 和 `.action` 源文件中的枚举值已冻结：FCU 来源为 V7 或
simulator; FCU mode is stabilize (0), altitude hold (1), position hold (2), or
program (3). Localization source is none, LIO, visual boundary, or fused and
state is uninitialized, active, degraded, or lost. Boundary mask bits are X=1,
Y=2, Z=4, roll=8, pitch=16, and yaw=32. Flight commands are arm, disarm, mode,
takeoff, move, hover, and land, with succeeded/rejected/timeout/FCU-error
results. Mission results are succeeded, rejected, aborted, or timeout. No enum
maps to V7 `0x41`.

使用以下命令运行独立检查：

```bash
./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_contract.py \
  ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json

./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_d_task_config.py \
  mission ros2_ws/src/ed_uav_interfaces/contracts/d_task/examples/mission_profile.example.yaml
```
