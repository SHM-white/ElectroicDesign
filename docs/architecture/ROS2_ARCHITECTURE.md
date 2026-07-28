# ROS 2 System Architecture

> Source-frozen: 2026-07-23. Every claim below traces to a file in `ros2_ws/src/`
> or the contract manifest at
> `ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json`.

---

## 1. System Overview

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

### Platform

| Component | Value |
|---|---|
| Target OS | Ubuntu 22.04 (Jammy) |
| ROS 2 distro | Humble Hawksbill |
| Build system | colcon (ament_python / ament_cmake) |
| Container | `ros:humble-ros-base-jammy` (digest-pinned, linux/amd64) |
| Host CPU | Intel i5 (minimum) |
| Runner | `tools/run_humble.sh` — native on Jammy, Docker/Podman elsewhere |

---

## 2. Package Inventory (10 packages)

| # | Package | Build | Description |
|---|---------|-------|-------------|
| 1 | `ed_uav_interfaces` | ament_cmake | Frozen `.msg`, `.srv`, `.action` definitions + contract manifest |
| 2 | `ed_uav_description` | ament_python | URDF/xacro model, calibration validation, static TF rendering |
| 3 | `ed_uav_camera` | ament_python | Dual V4L2 UVC transport, runtime plan preflight, camera_info gate |
| 4 | `ed_uav_lidar` | ament_python | Optional Mid-360 / generic PointCloud2 transport |
| 5 | `ed_uav_fcu_bridge` | ament_cmake | Exclusive Lingxiao V7 FCU bridge (serial, action server, telemetry) |
| 6 | `ed_uav_localization` | ament_python | Source supervisor, LIO health, field anchor (map→odom TF) |
| 7 | `ed_uav_perception` | ament_python | Detector node, boundary extractor, visual odometry, rectifier |
| 8 | `ed_uav_mission` | ament_python | ExecuteMission action server, safety supervisor (hover→land) |
| 9 | `ed_uav_bringup` | ament_python | Launch orchestration, calibration gate, diagnostics aggregation |
| 10 | `ed_uav_verification` | ament_python | Deterministic offline fakes, replay harness, fault injection |

---

## 3. Data Flow — Topics

All names are absolute. QoS profiles are frozen in the contract manifest.

### 3.1 FCU Bridge → System

| Topic | Type | QoS | Freshness |
|---|---|---|---|
| `/fcu/state` | `FcuState` | state_reliable (keep-last 10) | 0.50 s |
| `/fcu/battery` | `BatteryState` | state_reliable | 1.00 s |
| `/fcu/optical_flow/odom` | `Odometry` | state_reliable | 0.20 s |
| `/fcu/diagnostics` | `DiagnosticArray` | state_reliable | 0.50 s |
| `/rangefinder/range` | `Range` | sensor_data_best_effort (keep-last 5) | 0.20 s |

### 3.2 Cameras → System

| Topic | Type | QoS | Freshness |
|---|---|---|---|
| `/camera/narrow/image_raw` | `Image` | sensor_data_best_effort | 0.20 s |
| `/camera/narrow/camera_info` | `CameraInfo` | latched_reliable (keep-last 1) | — |
| `/camera/wide/image_raw` | `Image` | sensor_data_best_effort | 0.20 s |
| `/camera/wide/camera_info` | `CameraInfo` | latched_reliable | — |

### 3.3 LiDAR → System

| Topic | Type | QoS | Freshness |
|---|---|---|---|
| `/lidar/points` | `PointCloud2` | sensor_data_best_effort | 0.15 s |
| `/lidar/imu` | `Imu` | sensor_data_best_effort | 0.15 s |
| `/livox/lidar` | `CustomMsg` | sensor_data_best_effort | — (FAST-LIO direct) |

### 3.4 Localization → System

| Topic | Type | Owner | QoS |
|---|---|---|---|
| `/localization/lio/odom` | `Odometry` | LIO adapter (external FAST-LIO) | state_reliable, 0.15 s |
| `/localization/boundary_observation` | `BoundaryObservation` | boundary perception | state_reliable, 0.20 s |
| `/localization/status` | `LocalizationStatus` | source_supervisor | state_reliable, 0.20 s |
| `/localization/odom` | `Odometry` | source_supervisor (→ EKF, future) | state_reliable, 0.15 s |
| `/localization/lio/health` | `DiagnosticArray` | lio_health_monitor | state_reliable |

### 3.5 Perception

| Topic | Type | Owner |
|---|---|---|
| `/perception/narrow/detections` | `Detection2DArray` | detector_node |

### 3.6 Aggregated Diagnostics

| Topic | Type | Owner |
|---|---|---|
| `/diagnostics` | `DiagnosticArray` | bringup aggregator |

---

## 4. Data Flow — Services

| Service | Type | Owner | Status |
|---|---|---|---|
| `/localization/start_map_session` | `StartMapSession` | map_archive | **Declared, not implemented** |

---

## 5. Data Flow — Actions

| Action | Type | Server | Client |
|---|---|---|---|
| `/fcu/flight_command` | `FlightCommand` | `ed_uav_fcu_bridge` | `ed_uav_mission` |
| `/mission/execute` | `ExecuteMission` | `ed_uav_mission` | (external) |

### FlightCommand commands

`ARM`, `DISARM`, `SET_MODE`, `TAKEOFF`, `MOVE`, `HOVER`, `LAND`

Results: `SUCCEEDED`, `REJECTED`, `TIMEOUT`, `FCU_ERROR`

### ExecuteMission

Goal: `mission_id`, `field_profile_id`, `timeout_sec`
Results: `SUCCEEDED`, `REJECTED`, `ABORTED`, `TIMEOUT`

---

## 6. TF Tree

### 6.1 Static Transforms (URDF via `robot_state_publisher`)

Published by `ed_uav_description.robot_state_publisher` from calibration YAML:

```
base_link
├── fcu_link                    (xyz_m, rpy_rad from calibration)
├── lidar_link
├── camera_narrow_optical_frame
├── camera_wide_optical_frame
└── rangefinder_link
```

**Constraint**: `map → odom` and `odom → base_link` are **forbidden** as static
joints. They must be dynamic (see below).

### 6.2 Dynamic Transforms

| Edge | Publisher | Mechanism |
|---|---|---|
| `map → odom` | `ed_uav_localization.field_anchor` | `/tf_static` (StaticTransformBroadcaster) |
| `odom → base_link` | `ed_uav_localization.source_supervisor` | `/tf` (dynamic) |

### 6.3 Coordinate Convention

All world/body coordinates are **SI/ENU** under REP-103. Image values remain
pixels in their optical frames. Header timestamps use ROS time from the
device/source.

---

## 7. Lifecycle States

No `LifecycleNode` subclass exists. Lifecycle is **conceptual**, enforced by the
bringup launch ordering and the contract manifest.

### 7.1 Activation Order

Defined in `bringup.launch.py` line 18:

```
calibration_gate → robot_state_publisher → hardware_owners → localization
```

1. **Calibration gate**: `validate_for_profile()` checks serial bindings,
   resolution, freshness, and calibration status. Competition profile requires
   `CALIBRATED`.
2. **Robot state publisher**: Renders URDF from validated calibration, publishes
   static TF.
3. **Hardware owners**: Cameras, lidar, FCU bridge acquire exclusive device
   ownership.
4. **Localization**: Waits for fresh eligible sensor sources before activating.

### 7.2 Per-Node Readiness (from contract manifest)

| Node | Prerequisite |
|---|---|
| `ed_uav_fcu_bridge` | Exclusive serial endpoint lock (TIOCEXCL + flock) |
| `ed_uav_camera.narrow` | Calibration + device identity validation |
| `ed_uav_camera.wide` | Calibration + device identity validation |
| `ed_uav_lidar` | Time + device validation (non-placeholder serial/IP/firmware) |
| `ed_uav_localization` | Fresh eligible sensor sources |
| `ed_uav_mission` | FCU ready + valid calibration + active localization + start event + control authority |

### 7.3 Localization Source States

Defined in `LocalizationStatus.msg`:

| State | Value | Meaning |
|---|---|---|
| `UNINITIALIZED` | 0 | No source has ever been active |
| `ACTIVE` | 1 | Primary source is healthy |
| `DEGRADED` | 2 | Primary source stale but recoverable |
| `LOST` | 3 | No usable source; hover→land triggered |

Sources: `NONE`, `LIO`, `VISUAL_BOUNDARY`, `FUSED`

### 7.4 Mission States

Defined in `ed_uav_mission/state_machine.py`:

```
IDLE → ARMED → TAKEOFF → EXECUTING → RETURNING → LANDING → COMPLETE
                                  ↘ ABORTED (from any active state)
```

### 7.5 Safety Supervisor States

Defined in `ed_uav_mission/safety_supervisor.py`:

```
ACTIVE → LOCALIZATION_LOST_HOVERING → LOCALIZATION_LOST_LANDING → CRITICAL
              ↘ (recovered) → ACTIVE
```

---

## 8. Node Inventory

| # | Node | Package | Role |
|---|---|---|---|
| 1 | `ed_uav_fcu_bridge` | ed_uav_fcu_bridge | V7 serial bridge, action server, telemetry publisher |
| 2 | `mission_executor` | ed_uav_mission | ExecuteMission server, FlightCommand client |
| 3 | `source_supervisor` | ed_uav_localization | LIO/visual source switching, /localization/odom publisher |
| 4 | `field_anchor` | ed_uav_localization | map→odom TF broadcaster |
| 5 | `lio_health_monitor` | ed_uav_localization | FAST-LIO health diagnostics |
| 6 | `detector_node` | ed_uav_perception | Narrow-camera YOLO/boundary detection |
| 7 | `generic_lidar_monitor` | ed_uav_lidar | Generic PointCloud2 relay |
| 8 | `mid360_monitoring_adapter` | ed_uav_lidar | Livox CustomMsg → PointCloud2 adapter |
| 9 | `fake_image_device` | ed_uav_camera | Synthetic test image publisher |
| 10 | `robot_state_publisher` | (external) | URDF static TF publisher |
| 11 | `ed_uav_verify_ros` | ed_uav_verification | Deterministic virtual-time publisher |

---

## 9. Interface Definitions

### 9.1 Messages (`ed_uav_interfaces/msg/`)

| Message | Key Fields |
|---|---|
| `FcuState` | source (V7/SIMULATOR), mode (STABILIZE/ALT_HOLD/POS_HOLD/PROGRAM), motors_armed, optical_flow_position_m, altitude_m, battery_voltage_v |
| `LocalizationStatus` | source (NONE/LIO/VISUAL/FUSED), state (UNINIT/ACTIVE/DEGRADED/LOST), age_sec, reason |
| `BoundaryObservation` | observable_dof_mask (X=1,Y=2,Z=4,R=8,P=16,Y=32), constraint_count, confidence, pose |

### 9.2 Actions (`ed_uav_interfaces/action/`)

| Action | Goal | Result |
|---|---|---|
| `FlightCommand` | command, target_pose, target_velocity, timeout_sec, correlation_id | result_code, completed_stamp, reason |
| `ExecuteMission` | mission_id, field_profile_id, timeout_sec | result_code, reason |

### 9.3 Services (`ed_uav_interfaces/srv/`)

| Service | Request | Response |
|---|---|---|
| `StartMapSession` | session_id, archive_root, record_pointcloud | accepted, reason, staging_uri |

---

## 10. Contract Verification

Run the standalone contract checker:

```bash
./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_contract.py \
  ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json
```

This validates all approved topics, services, actions, TF edges, QoS profiles,
freshness deadlines, lifecycle ordering, and enum values against the frozen
manifest.
