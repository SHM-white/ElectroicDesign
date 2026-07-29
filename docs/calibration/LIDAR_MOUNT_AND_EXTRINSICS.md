# 激光雷达安装与外参

> **状态**：契约已定义，硬件待完成（Task 24）
> **所有者**：`ed_uav_lidar`、`ed_uav_description`
> **传感器**：Livox Mid-360

---

## 1. 概述

本文规定 ED UAV 平台 Livox Mid-360 激光雷达的物理安装、坐标系、时间同步和外参标定要求。

### 当前状态

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

## 2. Mid-360 安装要求

### 2.1 物理安装

| Requirement | Specification | Rationale |
|---|---|---|
| Mount surface | Rigid upper plate, metal heat spreader | Thermal management |
| Plate thickness | ≥3 mm | Structural rigidity |
| Exposed area | ≥10,000 mm² | Heat dissipation |
| Airflow clearance | ≥10 mm around sensor | Convective cooling |
| Orientation | Bottom-surface mount (laser pointing down) | FOV optimization |
| Payload | No payload attached to sensor body | Vibration isolation |
| Vibration | Rigid mount preferred | Signal quality |

### 2.2 安装位置

Mid-360 应安装：
- At the center of mass (CoM) or as close as possible
- Above the propeller plane to minimize occlusion
- Away from motors and ESCs to reduce EMI
- With clear line of sight below for ground detection

### 2.3 热管理

| Parameter | Value | Source |
|---|---|---|
| Average power | 6.5 W | Mid-360 datasheet |
| Cold self-heating peak | 14 W | Mid-360 datasheet |
| Shell temperature limit | ≤70°C | Mid-360 datasheet |
| Thermal throttling | None (sensor continues at reduced accuracy) | Mid-360 datasheet |

散热板必须在峰值 14W 时仍能散热，且外壳温度不得超过 70°C。长时间运行时使用板载
温度传感器监测。

From `ed_uav_lidar/config/lidar.yaml`:

```yaml
# Health thresholds (to be configured for hardware)
# temperature_warn_c: 60
# temperature_error_c: 70
```

---

## 3. FOV 和遮挡分析

### 3.1 Mid-360 FOV

Mid-360 的 FOV 为 **360° × 59°**：
- Horizontal: 360° (full rotation)
- Vertical: 59° (29.5° above and below horizontal)

### 3.2 遮挡要求

遮挡分析必须验证：

| Requirement | Threshold | Verification |
|---|---|---|
| Horizontal sector blockage | No sector >15° blocked | FOV analysis |
| Prop guard intrusion | None during hover | Static measurement |
| Landing gear occlusion | No persistent occlusion | Dynamic analysis |
| Camera body intrusion | Outside lidar FOV | CAD analysis |

### 3.3 遮挡来源

| Source | Risk | Mitigation |
|---|---|---|
| Prop guards | High (near propellers) | Design clearance |
| Landing gear | Medium (below CoM) | Retractable or slim design |
| Camera bodies | Low (side-mounted) | Mount outside FOV |
| Wiring | Low | Route away from FOV |
| Frame arms | Medium (depends on design) | Minimize arm thickness |

### 3.4 FOV 验证流程

1. Mount sensor on airframe
2. Place airframe on level surface
3. Capture 360° point cloud
4. Analyze horizontal sectors for gaps
5. Verify no sector >15° is blocked by contiguous structure
6. Document occlusion map

---

## 4. 坐标系

### 4.1 TF 树结构

From `ros2_contract_manifest.json`:

```
map → odom → base_link → lidar_link
                     ├── fcu_link
                     ├── camera_narrow_optical_frame
                     ├── camera_wide_optical_frame
                     └── rangefinder_link
```

### 4.2 静态坐标系

所有 `base_link → *` 变换都是静态变换，由 `robot_state_publisher` 根据标定 YAML 发布。

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

### 4.3 动态坐标系

| Edge | Publisher | Topic |
|---|---|---|
| `map → odom` | `ed_uav_localization.field_anchor` | `/tf_static` |
| `odom → base_link` | `ed_uav_localization.source_supervisor` | `/tf` |

### 4.4 URDF 定义

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

## 5. lidar_link 变换

### 5.1 标定 YAML 格式

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

### 5.2 变换字段

| Field | Type | Description |
|---|---|---|
| `xyz_m` | [x, y, z] | Position offset from `base_link` in meters |
| `rpy_rad` | [roll, pitch, yaw] | Orientation offset in radians |

### 5.3 标准安装值

For a standard bottom-surface mount with laser pointing down:

| Frame | xyz_m | rpy_rad | Notes |
|---|---|---|---|
| `lidar_link` | [0.0, 0.0, 0.15] | [0.0, 0.0, 0.0] | 15cm above CoM, identity rotation |
| `fcu_link` | [0.0, 0.0, 0.0] | [0.0, 0.0, 0.0] | At CoM |
| `rangefinder_link` | [0.0, 0.0, -0.06] | [0.0, 0.0, 0.0] | 6cm below CoM |

### 5.4 标定流程

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

## 6. PTP/时间同步

### 6.1 时间权威模式

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

### 6.2 PTP 配置

The Mid-360 supports PTPv2 (IEEE 1588) for time synchronization.

From `ed_uav_lidar/config/lidar.yaml`:

```yaml
/lidar_transport:
  ros__parameters:
    time_authority: host  # or ptp
```

### 6.3 PTP 设置要求

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

### 6.4 时间验证

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

## 7. 驱动配置

### 7.1 激光雷达配置

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

### 7.2 Mid-360 驱动 JSON

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

### 7.3 现场配置门控

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

在以下条件满足前，系统拒绝启动 Livox 驱动：
- `serial_number` is not `UNSET`
- `sensor_ip` is not `0.0.0.0`
- `firmware_version` is not `UNSET`
- `driver_config_path` is not the built-in placeholder

---

## 8. 启动流程

### 8.1 启动计划

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

### 8.2 数据流

```
Livox Mid-360 (hardware)
  → livox_ros_driver2 (xfer_format=1, publishes CustomMsg on /livox/lidar)
     → [direct to FAST-LIO for LIO]
  → mid360_adapter (subscribes /livox/lidar, publishes /lidar/points as PointCloud2)
     → [monitoring/verification]
  → mid360_adapter (relays /livox/imu to /lidar/imu)
     → [IMU health monitoring]
```

### 8.3 主题所有权

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

## 9. 点云规范化

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

### 9.2 规范化

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

### 9.3 时间验证

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

## 10. 振动测试

### 10.1 飞行前验证

首次飞行前必须验证：

| Test | Duration | Criteria |
|---|---|---|
| Static test | 60 seconds | Point cloud density stable (±5%), IMU noise within specs |
| Motor test | 60 seconds | Point cloud density stable, IMU noise increase <2× static |
| Flight test | 30 seconds | LIO converges within 5s, no divergence, drift ≤5 cm |

### 10.2 静态测试

1. Place airframe on level surface
2. Motors off
3. Capture 60 seconds of lidar/IMU data
4. Analyze:
   - Point cloud density (points per second)
   - IMU noise (accelerometer/gyroscope variance)
   - Spurious points from vibration

### 10.3 电机测试

1. Place airframe on level surface
2. Motors at hover throttle (50%)
3. Capture 60 seconds of lidar/IMU data
4. Analyze:
   - Point cloud density stability
   - Vibration-induced artifacts
   - IMU noise increase (should be <2× static baseline)

### 10.4 飞行测试

1. Hover at 1 meter for 30 seconds
2. Analyze:
   - LIO convergence time (should be <5 seconds)
   - LIO divergence (should be none)
   - Position drift (should be ≤5 cm over 30 seconds)

---

## 11. 验收标准

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

## 12. 参考资料

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
