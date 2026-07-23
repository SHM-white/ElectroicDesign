# Lidar Mount and Extrinsics

> **Status**: Contract defined, hardware pending (Task 24)
> **Owner**: `ed_uav_lidar`, `ed_uav_description`
> **Sensor**: Livox Mid-360

---

## 1. Overview

This document specifies the physical mounting, coordinate frames, time
synchronization, and extrinsic calibration requirements for the Livox Mid-360
lidar on the ED UAV platform.

### Current State

| Component | File | Status |
|---|---|---|
| Lidar config | `ed_uav_lidar/config/lidar.yaml` | **Defined (disabled by default)** |
| Mid-360 driver JSON | `ed_uav_lidar/config/mid360_driver.json` | **Placeholder** |
| URDF/xacro | `ed_uav_description/urdf/ed_uav.urdf.xacro` | **Implemented** |
| Calibration parser | `ed_uav_description/calibration.py` | **Implemented** |
| TF ownership | `ed_uav_description/test/test_static_tf_ownership.py` | **Implemented** |
| Health monitoring | `ed_uav_lidar/health.py` | **Implemented** |
| Mid-360 adapter | `ed_uav_lidar/mid360_adapter.py` | **Implemented** |
| Launch plan | `ed_uav_lidar/launch_plan.py` | **Implemented** |
| Physical mount | — | **Not implemented (hardware pending)** |
| PTP configuration | — | **Not implemented** |
| Vibration testing | — | **Not implemented** |

---

## 2. Mid-360 Mounting Requirements

### 2.1 Physical Mount

| Requirement | Specification | Rationale |
|---|---|---|
| Mount surface | Rigid upper plate, metal heat spreader | Thermal management |
| Plate thickness | ≥3 mm | Structural rigidity |
| Exposed area | ≥10,000 mm² | Heat dissipation |
| Airflow clearance | ≥10 mm around sensor | Convective cooling |
| Orientation | Bottom-surface mount (laser pointing down) | FOV optimization |
| Payload | No payload attached to sensor body | Vibration isolation |
| Vibration | Rigid mount preferred | Signal quality |

### 2.2 Mounting Location

The Mid-360 should be mounted:
- At the center of mass (CoM) or as close as possible
- Above the propeller plane to minimize occlusion
- Away from motors and ESCs to reduce EMI
- With clear line of sight below for ground detection

### 2.3 Heat Management

| Parameter | Value | Source |
|---|---|---|
| Average power | 6.5 W | Mid-360 datasheet |
| Cold self-heating peak | 14 W | Mid-360 datasheet |
| Shell temperature limit | ≤70°C | Mid-360 datasheet |
| Thermal throttling | None (sensor continues at reduced accuracy) | Mid-360 datasheet |

The heat spreader must dissipate 14W peak without exceeding 70°C shell
temperature. Monitor with onboard temperature sensor during extended runs.

From `ed_uav_lidar/config/lidar.yaml`:

```yaml
# Health thresholds (to be configured for hardware)
# temperature_warn_c: 60
# temperature_error_c: 70
```

---

## 3. FOV and Occlusion Analysis

### 3.1 Mid-360 FOV

The Mid-360 has a **360° × 59°** FOV:
- Horizontal: 360° (full rotation)
- Vertical: 59° (29.5° above and below horizontal)

### 3.2 Occlusion Requirements

Occlusion analysis must verify:

| Requirement | Threshold | Verification |
|---|---|---|
| Horizontal sector blockage | No sector >15° blocked | FOV analysis |
| Prop guard intrusion | None during hover | Static measurement |
| Landing gear occlusion | No persistent occlusion | Dynamic analysis |
| Camera body intrusion | Outside lidar FOV | CAD analysis |

### 3.3 Occlusion Sources

| Source | Risk | Mitigation |
|---|---|---|
| Prop guards | High (near propellers) | Design clearance |
| Landing gear | Medium (below CoM) | Retractable or slim design |
| Camera bodies | Low (side-mounted) | Mount outside FOV |
| Wiring | Low | Route away from FOV |
| Frame arms | Medium (depends on design) | Minimize arm thickness |

### 3.4 FOV Verification Procedure

1. Mount sensor on airframe
2. Place airframe on level surface
3. Capture 360° point cloud
4. Analyze horizontal sectors for gaps
5. Verify no sector >15° is blocked by contiguous structure
6. Document occlusion map

---

## 4. Coordinate Frames

### 4.1 TF Tree Structure

From `ros2_contract_manifest.json`:

```
map → odom → base_link → lidar_link
                     ├── fcu_link
                     ├── camera_narrow_optical_frame
                     ├── camera_wide_optical_frame
                     └── rangefinder_link
```

### 4.2 Static Frames

All `base_link → *` transforms are static and published by
`robot_state_publisher` from the calibration YAML.

From `ros2_contract_manifest.json`:

```json
{
  "static_frames": [
    {"parent": "base_link", "child": "fcu_link", "publisher": "ed_uav_description.robot_state_publisher"},
    {"parent": "base_link", "child": "lidar_link", "publisher": "ed_uav_description.robot_state_publisher"},
    {"parent": "base_link", "child": "camera_narrow_optical_frame", "publisher": "ed_uav_description.robot_state_publisher"},
    {"parent": "base_link", "child": "camera_wide_optical_frame", "publisher": "ed_uav_description.robot_state_publisher"},
    {"parent": "base_link", "child": "rangefinder_link", "publisher": "ed_uav_description.robot_state_publisher"}
  ]
}
```

### 4.3 Dynamic Frames

| Edge | Publisher | Topic |
|---|---|---|
| `map → odom` | `ed_uav_localization.field_anchor` | `/tf_static` |
| `odom → base_link` | `ed_uav_localization.ekf` | `/tf` |

### 4.4 URDF Definition

From `ed_uav_description/urdf/ed_uav.urdf.xacro`:

```xml
<link name="lidar_link"/>
<joint name="base_to_lidar" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="$(arg lidar_xyz)" rpy="$(arg lidar_rpy)"/>
</joint>
```

The `lidar_xyz` and `lidar_rpy` arguments are populated from the calibration
YAML during launch.

---

## 5. lidar_link Transform

### 5.1 Calibration YAML Format

From `ed_uav_description/config/synthetic_calibrated.yaml`:

```yaml
schema_version: 1
calibration_id: SYNTHETIC-STATIC-MODEL
calibration_status: SYNTHETIC
calibration_hash: e1ec326500451dc318cc55568cbc4f4f1247fe24fd9fb619577c36455310b37c
sensor_serials:
  camera_narrow: SYNTHETIC-NARROW-001
  camera_wide: SYNTHETIC-WIDE-001
  lidar: SYNTHETIC-LIDAR-001
transforms:
  fcu_link: {xyz_m: [0.0, 0.0, 0.0], rpy_rad: [0.0, 0.0, 0.0]}
  lidar_link: {xyz_m: [0.12, 0.0, 0.08], rpy_rad: [0.0, 0.0, 0.0]}
  camera_narrow_optical_frame: {xyz_m: [0.08, 0.04, -0.02], rpy_rad: [0.0, 0.0, 0.0]}
  camera_wide_optical_frame: {xyz_m: [0.08, -0.04, -0.02], rpy_rad: [0.0, 0.0, 0.0]}
  rangefinder_link: {xyz_m: [0.0, 0.0, -0.06], rpy_rad: [0.0, 0.0, 0.0]}
```

### 5.2 Transform Fields

| Field | Type | Description |
|---|---|---|
| `xyz_m` | [x, y, z] | Position offset from `base_link` in meters |
| `rpy_rad` | [roll, pitch, yaw] | Orientation offset in radians |

### 5.3 Standard Mount Values

For a standard bottom-surface mount with laser pointing down:

| Frame | xyz_m | rpy_rad | Notes |
|---|---|---|---|
| `lidar_link` | [0.0, 0.0, 0.15] | [0.0, 0.0, 0.0] | 15cm above CoM, identity rotation |
| `fcu_link` | [0.0, 0.0, 0.0] | [0.0, 0.0, 0.0] | At CoM |
| `rangefinder_link` | [0.0, 0.0, -0.06] | [0.0, 0.0, 0.0] | 6cm below CoM |

### 5.4 Calibration Procedure

1. Mount sensor rigidly on airframe
2. Measure physical offset from `base_link` origin to sensor center
3. Measure physical rotation (should be identity for standard mount)
4. Update calibration YAML with measured values
5. Set `calibration_status: CALIBRATED`
6. Run `check_urdf` to verify TF tree
7. Capture static TF and compare with expected values

From `ed_uav_description/calibration.py`:

```python
SENSOR_NAMES: Final = ("camera_narrow", "camera_wide", "lidar")
FRAME_NAMES: Final = (
    "fcu_link",
    "lidar_link",
    "camera_narrow_optical_frame",
    "camera_wide_optical_frame",
    "rangefinder_link",
)
```

---

## 6. PTP/Time Synchronization

### 6.1 Time Authority Modes

From `ed_uav_lidar/config.py`:

```python
class TimeAuthority(str, Enum):
    HOST = "host"
    PTP = "ptp"
```

| Mode | Status Code | Description |
|---|---|---|
| `host` | `HOST_TIME_UNVERIFIED` | Sensor uses host system time |
| `ptp` | `PTP_CONFIGURED_UNVERIFIED` | Sensor clock synchronized to PTP master |

**Note**: Neither mode claims measured synchronization. Both report UNVERIFIED
status until hardware validation (Task 24).

### 6.2 PTP Configuration

The Mid-360 supports PTPv2 (IEEE 1588) for time synchronization.

From `ed_uav_lidar/config/lidar.yaml`:

```yaml
/lidar_transport:
  ros__parameters:
    time_authority: host  # or ptp
```

### 6.3 PTP Setup Requirements

To enable PTP:

1. **Network**: Mid-360 connected via Ethernet to host
2. **PTP master**: Host runs ptp4l as master
3. **PHC sync**: phc2sys synchronizes host clock to PTP hardware clock
4. **Verification**: PTP offset ≤1 ms

```bash
# Example PTP setup (to be verified with hardware)
sudo ptp4l -i eth0 -m -S &
sudo phc2sys -s eth0 -c CLOCK_REALTIME -w &
```

### 6.4 Time Verification

From `ed_uav_lidar/health.py`:

```python
def evaluate_health(
    state: HealthState, now_steady_ns: int, deadline_ns: int
) -> HealthReport:
    """Evaluate transport liveness without subtracting ROS acquisition timestamps."""
    if not state.driver_alive:
        return HealthReport(code="LIDAR_DRIVER_DEAD", active=False)
    if now_steady_ns - state.last_driver_steady_ns > deadline_ns:
        return HealthReport(code="LIDAR_DRIVER_TIMEOUT", active=False)
    if now_steady_ns - state.last_point_steady_ns > deadline_ns:
        return HealthReport(code="LIDAR_POINT_STALE", active=False)
    if now_steady_ns - state.last_imu_steady_ns > deadline_ns:
        return HealthReport(code="LIDAR_IMU_STALE", active=False)
    return HealthReport(code="LIDAR_ACTIVE", active=True)
```

Health states:
- `LIDAR_ACTIVE` — All timestamps fresh
- `LIDAR_POINT_STALE` — Point cloud timestamp stale
- `LIDAR_IMU_STALE` — IMU timestamp stale
- `LIDAR_DRIVER_TIMEOUT` — Driver heartbeat stale
- `LIDAR_DRIVER_DEAD` — Driver process exited

---

## 7. Driver Configuration

### 7.1 Lidar Config

From `ed_uav_lidar/config/lidar.yaml`:

```yaml
/lidar_transport:
  ros__parameters:
    lidar_enabled: false
    transport: disabled
    serial_number: UNSET
    sensor_ip: 0.0.0.0
    firmware_version: UNSET
    time_authority: host
    monitoring_topic: /lidar/points
    imu_topic: /lidar/imu
    generic_input_topic: /lidar/input/points
    fastlio_custom_topic: /livox/lidar
```

### 7.2 Mid-360 Driver JSON

From `ed_uav_lidar/config/mid360_driver.json`:

```json
{
  "lidar_summary_info": {"lidar_type": 8},
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "0.0.0.0",
      "cmd_data_port": 56101,
      "push_msg_ip": "0.0.0.0",
      "push_msg_port": 56201,
      "point_data_ip": "0.0.0.0",
      "point_data_port": 56301,
      "imu_data_ip": "0.0.0.0",
      "imu_data_port": 56401,
      "log_data_ip": "",
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "0.0.0.0",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0, "y": 0, "z": 0}
    }
  ]
}
```

### 7.3 Field Configuration Gate

From `ed_uav_lidar/config.py`:

```python
def _field_check(config: LidarConfig) -> FieldCheck:
    if not config.requires_livox:
        return FieldCheck(ready=True, missing=())
    missing = tuple(
        field
        for field, value in (
            ("serial_number", config.serial_number),
            ("sensor_ip", config.sensor_ip),
            ("firmware_version", config.firmware_version),
        )
        if value in {"UNSET", "0.0.0.0"}
    )
    if config.driver_config_path == "config/mid360_driver.json" or config.driver_config_path.endswith(
        PLACEHOLDER_DRIVER_CONFIG_SUFFIX
    ):
        missing = (*missing, "driver_config_path")
    return FieldCheck(ready=not missing, missing=missing)
```

The system refuses to start the Livox driver until:
- `serial_number` is not `UNSET`
- `sensor_ip` is not `0.0.0.0`
- `firmware_version` is not `UNSET`
- `driver_config_path` is not the built-in placeholder

---

## 8. Launch Flow

### 8.1 Launch Plan

From `ed_uav_lidar/launch_plan.py`:

```python
def build_launch_plan(config: LidarConfig) -> LaunchPlan:
    """Return the exact optional driver and adapter process plan for one mode."""
    if not config.enabled:
        return LaunchPlan(code="LIDAR_DISABLED", nodes=(), fastlio_custom_topic="")
    if not config.field_check.ready:
        return LaunchPlan(
            code=config.field_check.code,
            nodes=(),
            fastlio_custom_topic=config.fastlio_custom_topic,
        )
    match config.transport:
        case Transport.MID360:
            return LaunchPlan(
                code=config.time_status,
                nodes=(
                    NodeSpec(
                        package="livox_ros_driver2",
                        executable="livox_ros_driver2_node",
                        parameters=(
                            ("xfer_format", 1),
                            ("multi_topic", 0),
                            ("data_src", 0),
                            ("publish_freq", 10.0),
                            ("output_data_type", 0),
                            ("frame_id", "lidar_link"),
                            ("user_config_path", config.driver_config_path),
                        ),
                    ),
                    NodeSpec(
                        package="ed_uav_lidar",
                        executable="mid360_adapter",
                        parameters=(
                            ("custom_topic", config.fastlio_custom_topic),
                            ("monitoring_topic", config.monitoring_topic),
                            ("imu_topic", config.imu_topic),
                        ),
                    ),
                ),
                fastlio_custom_topic=config.fastlio_custom_topic,
            )
```

### 8.2 Data Flow

```
Livox Mid-360 (hardware)
  → livox_ros_driver2 (xfer_format=1, publishes CustomMsg on /livox/lidar)
     → [direct to FAST-LIO for LIO]
  → mid360_adapter (subscribes /livox/lidar, publishes /lidar/points as PointCloud2)
     → [monitoring/verification]
  → mid360_adapter (relays /livox/imu to /lidar/imu)
     → [IMU health monitoring]
```

### 8.3 Topic Ownership

From `ros2_contract_manifest.json`:

```json
{
  "name": "/lidar/points",
  "type": "sensor_msgs/msg/PointCloud2",
  "owner": "ed_uav_lidar",
  "qos": "sensor_data_best_effort",
  "units": "SI: m",
  "frame": "lidar_link",
  "clock": "lidar_acquisition_ros_time",
  "freshness": "0.15 s"
}
```

---

## 9. Point Cloud Normalization

### 9.1 Mid-360 CustomMsg

From `ed_uav_lidar/contracts.py`:

```python
MONITORING_FIELDS = ("x", "y", "z", "intensity", "offset_time")

class LivoxPoint(Protocol):
    offset_time: int
    x: float
    y: float
    z: float
    reflectivity: int
    tag: int
    line: int
```

### 9.2 Normalization

The `mid360_adapter` converts Livox CustomMsg to standard PointCloud2:

```python
fields = (
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="offset_time", offset=16, datatype=PointField.UINT32, count=1),
)
```

### 9.3 Timing Validation

From `ed_uav_lidar/contracts.py`:

```python
def validate_offset_times(offset_times_ns: Sequence[int]) -> tuple[int, ...]:
    """Return ordered raw offsets or raise a deterministic timing contract error."""
    offsets = tuple(offset_times_ns)
    if not offsets:
        raise MissingPointTiming(point_count=0)
    previous = offsets[0]
    for current in offsets[1:]:
        if current < previous:
            raise PointTimeRegression(previous_offset_ns=previous, current_offset_ns=current)
        previous = current
    return offsets
```

Error conditions:
- `MissingPointTiming` — Per-point offset_time is required
- `PointTimeRegression` — offset_time regression detected
- `PacketShapeError` — Declared point count mismatch

---

## 10. Vibration Testing

### 10.1 Pre-Flight Verification

Before first flight, verify:

| Test | Duration | Criteria |
|---|---|---|
| Static test | 60 seconds | Point cloud density stable (±5%), IMU noise within specs |
| Motor test | 60 seconds | Point cloud density stable, IMU noise increase <2× static |
| Flight test | 30 seconds | LIO converges within 5s, no divergence, drift ≤5 cm |

### 10.2 Static Test

1. Place airframe on level surface
2. Motors off
3. Capture 60 seconds of lidar/IMU data
4. Analyze:
   - Point cloud density (points per second)
   - IMU noise (accelerometer/gyroscope variance)
   - Spurious points from vibration

### 10.3 Motor Test

1. Place airframe on level surface
2. Motors at hover throttle (50%)
3. Capture 60 seconds of lidar/IMU data
4. Analyze:
   - Point cloud density stability
   - Vibration-induced artifacts
   - IMU noise increase (should be <2× static baseline)

### 10.4 Flight Test

1. Hover at 1 meter for 30 seconds
2. Analyze:
   - LIO convergence time (should be <5 seconds)
   - LIO divergence (should be none)
   - Position drift (should be ≤5 cm over 30 seconds)

---

## 11. Acceptance Criteria

From Task 24 (hardware gate):

| Criterion | Threshold | Verification |
|---|---|---|
| Runtime duration | 30-minute run | Timer |
| Timestamp regression | 0 | `test_timestamp_regression.py` |
| Driver restart | 0 | Process monitor |
| Dropped samples | <0.1% | Health monitor |
| PTP offset | ≤1 ms (when PTP claimed) | PTP status |
| Shell temperature | ≤70°C | Onboard sensor |
| Thermal throttling | None | Temperature log |
| Horizontal occlusion | No sector >15° blocked | FOV analysis |
| LIO gap | No >0.20s gap | `lio_health.py` |
| Static drift | ≤5 cm over 60s | Position log |

---

## 12. References

- Livox Mid-360 official user manual and specs
- `ed_uav_lidar/config/lidar.yaml` — Lidar ROS parameters
- `ed_uav_lidar/config/mid360_driver.json` — Livox driver configuration
- `ed_uav_lidar/ed_uav_lidar/config.py` — Typed config parser
- `ed_uav_lidar/ed_uav_lidar/launch_plan.py` — Launch planning
- `ed_uav_lidar/ed_uav_lidar/mid360_adapter.py` — CustomMsg to PointCloud2
- `ed_uav_lidar/ed_uav_lidar/health.py` — Health monitoring
- `ed_uav_lidar/ed_uav_lidar/contracts.py` — Normalization contracts
- `ed_uav_description/urdf/ed_uav.urdf.xacro` — URDF definition
- `ed_uav_description/ed_uav_description/calibration.py` — Calibration parser
- `ed_uav_description/config/synthetic_calibrated.yaml` — Example calibration
- `ed_uav_interfaces/contracts/ros2_contract_manifest.json` — Frozen contract
- `ed_uav_localization/ed_uav_localization/lio_health.py` — LIO health
