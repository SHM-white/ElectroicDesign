# ROS 2 系统架构

> 源码冻结日期：2026-07-23。下文每项说明都可追溯到 `ros2_ws/src/`
> 中的文件，或追溯到以下契约清单：
> `ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json`.

---

## 1. 系统概览

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ed_uav_bringup                              │
│  bringup.launch.py                                                  │
│  ┌──────────────┐  ┌───────────────────┐  ┌──────────────────────┐  │
│  │calibration   │→ │robot_state_       │→ │hardware_owners       │  │
│  │gate          │  │publisher          │  │(cameras, lidar, FCU) │  │
│  └──────────────┘  └───────────────────┘  └──────────────────────┘  │
│                           │                         │                │
│                           ▼                         ▼                │
│                    /tf_static (URDF)        sensor streams           │
└─────────────────────────────────────────────────────────────────────┘
         │                    │                      │
         ▼                    ▼                      ▼
┌─────────────┐  ┌───────────────────┐  ┌──────────────────────────┐
│ ed_uav_     │  │ ed_uav_camera     │  │ ed_uav_lidar             │
│ description │  │ dual V4L2 nodes   │  │ Mid-360 / generic        │
│ URDF+xacro  │  │ /camera/narrow/*  │  │ /lidar/points, /lidar/imu│
│             │  │ /camera/wide/*    │  │ /livox/lidar (CustomMsg) │
└─────────────┘  └───────────────────┘  └──────────────────────────┘
                          │                      │
                          ▼                      ▼
              ┌───────────────────┐  ┌──────────────────────────┐
              │ ed_uav_perception │  │ ed_uav_localization      │
              │ detector_node     │  │ source_supervisor (LIO/  │
              │ boundary_         │  │   visual switching)      │
              │   extractor       │  │ lio_health_monitor       │
              │ visual_odometry   │  │ field_anchor (map→odom)  │
              └───────────────────┘  └──────────────────────────┘
                          │                      │
                          ▼                      ▼
              ┌───────────────────┐  ┌──────────────────────────┐
              │ ed_uav_fcu_bridge │  │ ed_uav_mission           │
              │ Exclusive V7 FCU  │  │ executor (ExecuteMission)│
              │ /fcu/state        │  │ safety_supervisor        │
              │ /fcu/flight_cmd   │  │ (hover→land on loss)     │
              └───────────────────┘  └──────────────────────────┘
```

### 平台

| 组件 | 值 |
|---|---|
| 目标操作系统 | Ubuntu 22.04 (Jammy) |
| ROS 2 发行版 | Humble Hawksbill |
| 构建系统 | colcon (ament_python / ament_cmake) |
| 容器 | `ros:humble-ros-base-jammy`（按摘要固定，linux/amd64） |
| 主机 CPU | Intel i5（最低要求） |
| 运行器 | `tools/run_humble.sh`，Jammy 原生运行，其他环境使用 Docker/Podman |

---

## 2. 软件包清单（10 个软件包）

| # | 软件包 | 构建方式 | 说明 |
|---|---------|-------|-------------|
| 1 | `ed_uav_interfaces` | ament_cmake | 冻结的 `.msg`、`.srv`、`.action` 定义和契约清单 |
| 2 | `ed_uav_description` | ament_python | URDF/xacro 模型、标定验证、静态 TF 渲染 |
| 3 | `ed_uav_camera` | ament_python | 双 V4L2 UVC 传输、运行计划预检、camera_info 门控 |
| 4 | `ed_uav_lidar` | ament_python | 可选的 Mid-360 或通用 PointCloud2 传输 |
| 5 | `ed_uav_fcu_bridge` | ament_cmake | 独占的凌霄 V7 FCU bridge，包括串口、动作服务端和遥测 |
| 6 | `ed_uav_localization` | ament_python | 源监督器、LIO 健康监测、场地锚点（map→odom TF） |
| 7 | `ed_uav_perception` | ament_python | 检测节点、边界提取器、视觉里程计、校正器 |
| 8 | `ed_uav_mission` | ament_python | ExecuteMission 动作服务端、安全监督器（悬停→降落） |
| 9 | `ed_uav_bringup` | ament_python | 启动编排、标定门、诊断聚合 |
| 10 | `ed_uav_verification` | ament_python | 确定性的离线伪实现、回放工具、故障注入 |

---

## 3. 数据流，话题

所有名称均为绝对名称。QoS 配置已在契约清单中冻结。

### 3.1 FCU bridge → 系统

| 话题 | 类型 | QoS | 新鲜度 |
|---|---|---|---|
| `/fcu/state` | `FcuState` | state_reliable (keep-last 10) | 0.50 s |
| `/fcu/battery` | `BatteryState` | state_reliable | 1.00 s |
| `/fcu/optical_flow/odom` | `Odometry` | state_reliable | 0.20 s |
| `/fcu/diagnostics` | `DiagnosticArray` | state_reliable | 0.50 s |
| `/rangefinder/range` | `Range` | sensor_data_best_effort (keep-last 5) | 0.20 s |

### 3.2 相机 → 系统

 | 话题 | 类型 | QoS | 新鲜度 |
|---|---|---|---|
| `/camera/narrow/image_raw` | `Image` | sensor_data_best_effort | 0.20 s |
| `/camera/narrow/camera_info` | `CameraInfo` | latched_reliable (keep-last 1) | — |
| `/camera/wide/image_raw` | `Image` | sensor_data_best_effort | 0.20 s |
| `/camera/wide/camera_info` | `CameraInfo` | latched_reliable | — |

### 3.3 LiDAR → 系统

| 话题 | 类型 | QoS | 新鲜度 |
|---|---|---|---|
| `/lidar/points` | `PointCloud2` | sensor_data_best_effort | 0.15 s |
| `/lidar/imu` | `Imu` | sensor_data_best_effort | 0.15 s |
| `/livox/lidar` | `CustomMsg` | sensor_data_best_effort | —（FAST-LIO 直接使用） |

### 3.4 定位 → 系统

| 话题 | 类型 | 所有者 | QoS |
|---|---|---|---|
| `/localization/lio/odom` | `Odometry` | LIO 适配器（外部 FAST-LIO） | state_reliable, 0.15 s |
| `/localization/boundary_observation` | `BoundaryObservation` | 边界感知 | state_reliable, 0.20 s |
| `/localization/status` | `LocalizationStatus` | source_supervisor | state_reliable, 0.20 s |
| `/localization/odom` | `Odometry` | source_supervisor（→ EKF，后续实现） | state_reliable, 0.15 s |
| `/localization/lio/health` | `DiagnosticArray` | lio_health_monitor | state_reliable |

### 3.5 感知

| 话题 | 类型 | 所有者 |
|---|---|---|
| `/perception/narrow/detections` | `Detection2DArray` | detector_node |

### 3.6 聚合诊断

| 话题 | 类型 | 所有者 |
|---|---|---|
| `/diagnostics` | `DiagnosticArray` | bringup 聚合器 |

---

## 4. 数据流，服务

| 服务 | 类型 | 所有者 | 状态 |
|---|---|---|---|
| `/localization/start_map_session` | `StartMapSession` | map_archive | **已声明，未实现** |

---

## 5. 数据流，动作

| 动作 | 类型 | 服务端 | 客户端 |
|---|---|---|---|
| `/fcu/flight_command` | `FlightCommand` | `ed_uav_fcu_bridge` | `ed_uav_mission` |
| `/mission/execute` | `ExecuteMission` | `ed_uav_mission` | （外部） |

### FlightCommand 命令

`ARM`, `DISARM`, `SET_MODE`, `TAKEOFF`, `MOVE`, `HOVER`, `LAND`

结果：`SUCCEEDED`、`REJECTED`、`TIMEOUT`、`FCU_ERROR`

### ExecuteMission

目标：`mission_id`、`field_profile_id`、`timeout_sec`
结果：`SUCCEEDED`、`REJECTED`、`ABORTED`、`TIMEOUT`

---

## 6. TF 树

### 6.1 静态变换（通过 `robot_state_publisher` 发布 URDF）

`ed_uav_description.robot_state_publisher` 根据标定 YAML 发布：

```
base_link
├── fcu_link                    (xyz_m, rpy_rad from calibration)
├── lidar_link
├── camera_narrow_optical_frame
├── camera_wide_optical_frame
└── rangefinder_link
```

**约束**：禁止将 `map → odom` 和 `odom → base_link` 作为静态关节。
它们必须是动态变换，见下文。

### 6.2 动态变换

| 边 | 发布者 | 机制 |
|---|---|---|
| `map → odom` | `ed_uav_localization.field_anchor` | `/tf_static` (StaticTransformBroadcaster) |
| `odom → base_link` | `ed_uav_localization.source_supervisor` | `/tf` (dynamic) |

### 6.3 坐标约定

按照 REP-103，所有世界坐标和机体坐标均使用 **SI/ENU**。图像值在其光学坐标系中仍为像素。
消息头时间戳使用设备或数据源的 ROS 时间。

---

## 7. 生命周期状态

当前不存在 `LifecycleNode` 子类。生命周期是**概念性的**，由 bringup 启动顺序和契约清单强制执行。

### 7.1 激活顺序

定义于 `bringup.launch.py` 第 18 行：

```
calibration_gate → robot_state_publisher → hardware_owners → localization
```

1. **标定门**：`validate_for_profile()` 检查串口绑定、分辨率、新鲜度和标定状态。competition 配置要求状态为 `CALIBRATED`。
2. **机器人状态发布器**：根据已验证的标定渲染 URDF，并发布静态 TF。
3. **硬件所有者**：相机、LiDAR 和 FCU bridge 获取设备的独占所有权。
4. **定位**：等待新鲜且符合条件的传感器源后再激活。

### 7.2 各节点就绪条件（来自契约清单）

| 节点 | 前置条件 |
|---|---|
| `ed_uav_fcu_bridge` | 串口端点独占锁（TIOCEXCL + flock） |
| `ed_uav_camera.narrow` | 标定和设备身份验证 |
| `ed_uav_camera.wide` | 标定和设备身份验证 |
| `ed_uav_lidar` | 时间和设备验证（串口/IP/固件不能是占位值） |
| `ed_uav_localization` | 新鲜且符合条件的传感器源 |
| `ed_uav_mission` | FCU 就绪、标定有效、定位已激活、启动事件有效且拥有控制权 |

### 7.3 定位源状态

定义于 `LocalizationStatus.msg`：

| 状态 | 值 | 含义 |
|---|---|---|
| `UNINITIALIZED` | 0 | 从未有源处于活动状态 |
| `ACTIVE` | 1 | 主源健康 |
| `DEGRADED` | 2 | 主源过期但可恢复 |
| `LOST` | 3 | 没有可用源，已触发悬停→降落 |

来源：`NONE`、`LIO`、`VISUAL_BOUNDARY`、`FUSED`

### 7.4 任务状态

定义于 `ed_uav_mission/state_machine.py`：

```
IDLE → ARMED → TAKEOFF → EXECUTING → RETURNING → LANDING → COMPLETE
                                  ↘ ABORTED (from any active state)
```

### 7.5 安全监督器状态

定义于 `ed_uav_mission/safety_supervisor.py`：

```
ACTIVE → LOCALIZATION_LOST_HOVERING → LOCALIZATION_LOST_LANDING → CRITICAL
              ↘ (recovered) → ACTIVE
```

---

## 8. 节点清单

| # | 节点 | 软件包 | 作用 |
|---|---|---|---|
| 1 | `ed_uav_fcu_bridge` | ed_uav_fcu_bridge | V7 串口 bridge、动作服务端、遥测发布器 |
| 2 | `mission_executor` | ed_uav_mission | ExecuteMission 服务端、FlightCommand 客户端 |
| 3 | `source_supervisor` | ed_uav_localization | LIO/视觉源切换、`/localization/odom` 发布器 |
| 4 | `field_anchor` | ed_uav_localization | map→odom TF 广播器 |
| 5 | `lio_health_monitor` | ed_uav_localization | FAST-LIO 健康诊断 |
| 6 | `detector_node` | ed_uav_perception | 窄相机 YOLO/边界检测 |
| 7 | `generic_lidar_monitor` | ed_uav_lidar | 通用 PointCloud2 中继 |
| 8 | `mid360_monitoring_adapter` | ed_uav_lidar | Livox CustomMsg → PointCloud2 适配器 |
| 9 | `fake_image_device` | ed_uav_camera | 合成测试图像发布器 |
| 10 | `robot_state_publisher` | （外部） | URDF 静态 TF 发布器 |
| 11 | `ed_uav_verify_ros` | ed_uav_verification | 确定性的虚拟时间发布器 |

---

## 9. 接口定义

### 9.1 消息（`ed_uav_interfaces/msg/`）

| 消息 | 关键字段 |
|---|---|
| `FcuState` | source (V7/SIMULATOR), mode (STABILIZE/ALT_HOLD/POS_HOLD/PROGRAM), motors_armed, optical_flow_position_m, altitude_m, battery_voltage_v |
| `LocalizationStatus` | source (NONE/LIO/VISUAL/FUSED), state (UNINIT/ACTIVE/DEGRADED/LOST), age_sec, reason |
| `BoundaryObservation` | observable_dof_mask (X=1,Y=2,Z=4,R=8,P=16,Y=32), constraint_count, confidence, pose |

### 9.2 动作（`ed_uav_interfaces/action/`）

| 动作 | 目标 | 结果 |
|---|---|---|
| `FlightCommand` | command, target_pose, target_velocity, timeout_sec, correlation_id | result_code, completed_stamp, reason |
| `ExecuteMission` | mission_id, field_profile_id, timeout_sec | result_code, reason |

### 9.3 服务（`ed_uav_interfaces/srv/`）

| 服务 | 请求 | 响应 |
|---|---|---|
| `StartMapSession` | session_id, archive_root, record_pointcloud | accepted, reason, staging_uri |

---

## 10. 契约验证

运行独立的契约检查器：

```bash
./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_contract.py \
  ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json
```

该检查器会根据冻结清单验证所有获批准的话题、服务、动作、TF 边、QoS 配置、新鲜度期限、生命周期顺序和枚举值。
