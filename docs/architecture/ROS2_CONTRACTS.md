# ROS 2 Contract Freeze

Status: frozen for the code/offline milestone on 2026-07-22. This document and
`ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json` are the
source of truth for ROS graph names, frames, timing, QoS, and ownership.

## Baseline Mapping

The following maps the current in-process boundary before any ROS interface is
defined. It is a characterization of legacy behavior, not a claim that legacy
coordinates already satisfy REP-103.

| Legacy value | Type and unit | Current convention | ROS boundary rule |
| --- | --- | --- | --- |
| `VisionResult.green_ratio` | `float`, ratio `[0, 1]` | image-result scalar | Publish only with acquisition time and camera frame provenance. |
| `home_cross_center`, `start_marker_center`, `gray_marker_center` | pixel `(u, v)` | camera image pixels | Never treat as a world pose; use `sensor_msgs/Image`/standard detections or a partial observation. |
| `gray_marker_box` | pixel `(u, v, width, height)` | camera image pixels | Standard detection bounding boxes retain this representation. |
| vision confidence fields | `float`, ratio `[0, 1]` | detector score | Not a position covariance. |
| `MCUSerial._of_pos_x`, `_of_pos_y`, `_of_dx`, `_of_dy` | `float`, cm | V7 position relative to takeoff; legacy X forward, Y right | `ed_uav_fcu_bridge` alone converts to meters and ROS ENU. `0x08` is the continuous source. |
| `MCUSerial._altitude` | signed integer, cm | V7 altitude | `ed_uav_fcu_bridge` alone converts to meters. |
| `MCUSerial._voltage_mv` | integer, mV | battery electrical value | Publish standard `sensor_msgs/BatteryState` in volts. |
| `MCUSerial._mode`, `_locked`, `_aux6` | integer/bit/pulse us | V7 mode; `locked=1` means unlocked | Normalize to typed FCU state; preserve source sequence and acquisition time. |
| `cmd_move(distance_cm, speed_cmps, direction_deg)` | cm, cm/s, degrees | body-relative: 0 is nose-forward, clockwise positive | Only `ed_uav_fcu_bridge` converts an approved SI/ENU command to V7. |

Legacy `DroneStateMachine` is the current actuator arbiter. ROS replaces that
ownership with exactly one action-server owner, `ed_uav_fcu_bridge`; mission and
safety clients never open the FCU endpoint. V7 `0x41` is excluded.

## Graph Contract

The checked manifest lists the complete approved graph. No other topic, service,
action, TF edge, or hardware owner is approved by this freeze.

| Topic | Type | Owner | Frame | QoS and freshness |
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

### Simulation-Only Graph

The following entries are approved only for the simulation-only Gazebo FAST-LIO
and planner-only graph: `/fast_lio/odometry`, `/fast_lio/cloud_registered`,
`/fast_lio/laser_map`, `/fast_lio/path`, and `/fast_lio/tf`; canonical adapter
outputs `/localization/lio/cloud_registered`, `/localization/lio/map`, and
`/localization/lio/path`; the Nav2 `/map` topic; and the
`/compute_path_to_pose` action. FAST-LIO raw topics remain private to the
simulation graph. `/fast_lio/tf` is a private `TFMessage` topic, not a global
TF edge. The adapter relabels the LIO `camera_init` world frame to `odom` for
the canonical visualization outputs without publishing TF.

All names are absolute and occupy the fixed `/fcu`, `/rangefinder`,
`/camera/narrow`, `/camera/wide`, `/lidar`, `/localization`, `/perception`,
`/mission`, `/d_task`, and `/diagnostics` namespaces. `/tf` and `/tf_static` carry only
the authorities named below; they are not alternative data interfaces.

All physical quantities are SI and all world/body coordinates are ENU under
REP-103. Image values remain pixels in their optical frames. Header/acquisition
timestamps use ROS time from the device/source. Freshness is always measured
with a local steady clock, never by subtracting potentially regressed ROS time.

QoS profiles are fixed: `sensor_data_best_effort` means keep-last 5,
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

Lifecycle activation is ordered by the manifest: bridges and sensors acquire
exclusive hardware ownership first; localization waits for fresh eligible input;
mission waits for valid calibration/profile, active localization, a fresh start
event, and FCU control authority. On total localization loss the later safety
owner commands hover then controlled land; it never automatically locks motors
in air.

## Interface Rules

`FcuState` carries source, state, acquisition time, sequence, normalized FCU
mode/arming, and SI telemetry. `LocalizationStatus` carries source, state, and
bounded reason text. `BoundaryObservation` has `observable_dof_mask`; unset
DOFs in its pose are unspecified. `FlightCommand` and `ExecuteMission` use
bounded correlation/identity/reason fields. `StartMapSession` uses bounded IDs
and paths. Custom interfaces must not use unbounded strings or sequences.

Every D-task custom message and pre-arm request carries contract version 1.
`VehicleTelemetry` carries the start stamp/event, heartbeat, acquisition time,
source sequence, CRC-16, displacement or wheel speed in SI, turn class, and the
ordered START/B/D/A/COMPLETE state. `TargetObservation` names the approved
`d2026-circle-cross-v1` geometry and its acquisition frame. `MissionStatus` and
`PayloadContactState` expose bounded operator and contact/payload states with a
single publisher owner. Consumers measure freshness on a local steady clock;
ROS acquisition stamps provide provenance and are never used as an age clock.
Vehicle and ESP32 source sequences use uint32 serial-number arithmetic: modulo
deltas `1..2^31-1` advance, zero is duplicate, and deltas `2^31..2^32-1` are
stale. Therefore `UINT32_MAX -> 0` is a valid wrap while `8 -> 7` is rejected.

Strict D-task schemas and credential-free examples live under
`ed_uav_interfaces/contracts/d_task`. Real Mid-360 serial, sensor/host IP,
firmware, and ground-station peer values are permitted only in the gitignored
`deployment_preset.local.yaml`; field loading rejects placeholders and RFC 5737
documentation addresses rather than substituting defaults.

Enum values are frozen in the `.msg` and `.action` sources: FCU source is V7 or
simulator; FCU mode is stabilize (0), altitude hold (1), position hold (2), or
program (3). Localization source is none, LIO, visual boundary, or fused and
state is uninitialized, active, degraded, or lost. Boundary mask bits are X=1,
Y=2, Z=4, roll=8, pitch=16, and yaw=32. Flight commands are arm, disarm, mode,
takeoff, move, hover, and land, with succeeded/rejected/timeout/FCU-error
results. Mission results are succeeded, rejected, aborted, or timeout. No enum
maps to V7 `0x41`.

Run the standalone surface with:

```bash
./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_contract.py \
  ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json

./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_d_task_config.py \
  mission ros2_ws/src/ed_uav_interfaces/contracts/d_task/examples/mission_profile.example.yaml
```
