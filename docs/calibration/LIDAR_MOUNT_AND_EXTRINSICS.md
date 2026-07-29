# 激光雷达安装与外参

> **状态**：契约已定义，硬件待完成（Task 24）
> **所有者**：`ed_uav_lidar`、`ed_uav_description`
> **传感器**：Livox Mid-360

---

## 1. 概述

本文规定 ED UAV 平台 Livox Mid-360 激光雷达的物理安装、坐标系、时间同步和外参标定要求。

### 当前状态

| 组件 | 文件 | 状态 |
|---|---|---|
| 激光雷达配置 | `ed_uav_lidar/config/lidar.yaml` | **已定义（默认禁用）** |
| Mid-360 驱动 JSON | `ed_uav_lidar/config/mid360_driver.json` | **占位文件** |
| URDF/xacro | `ed_uav_description/urdf/ed_uav.urdf.xacro` | **已实现** |
| 标定解析器 | `ed_uav_description/calibration.py` | **已实现** |
| TF 所有权 | `ed_uav_description/test/test_static_tf_ownership.py` | **已实现** |
| 健康监测 | `ed_uav_lidar/health.py` | **已实现** |
| Mid-360 适配器 | `ed_uav_lidar/mid360_adapter.py` | **已实现** |
| 启动计划 | `ed_uav_lidar/launch_plan.py` | **已实现** |
| 物理安装 | — | **尚未实现（硬件待完成）** |
| PTP 配置 | — | **尚未实现** |
| 振动测试 | — | **尚未实现** |

---

## 2. Mid-360 安装要求

### 2.1 物理安装

| 要求 | 规格 | 原因 |
|---|---|---|
| 安装表面 | 刚性上板、金属散热板 | 热管理 |
| 板厚 | ≥3 mm | 结构刚度 |
| 暴露面积 | ≥10,000 mm² | 散热 |
| 气流间隙 | 传感器周围 ≥10 mm | 对流冷却 |
| 朝向 | 底面安装（激光向下） | FOV 优化 |
| 载荷 | 传感器本体不得连接载荷 | 隔离振动 |
| 振动 | 优先使用刚性安装 | 信号质量 |

### 2.2 安装位置

Mid-360 应安装：
 - 位于质心（CoM）或尽可能靠近质心
 - 位于螺旋桨平面上方，以尽量减少遮挡
 - 远离电机和 ESC，以降低 EMI
 - 下方视线清晰，用于地面检测

### 2.3 热管理

| 参数 | 值 | 来源 |
|---|---|---|
| 平均功耗 | 6.5 W | Mid-360 数据表 |
| 冷启动自发热峰值 | 14 W | Mid-360 数据表 |
| 外壳温度上限 | ≤70°C | Mid-360 数据表 |
| 热降频 | 无（传感器继续运行但精度降低） | Mid-360 数据表 |

散热板必须在峰值 14W 时仍能散热，且外壳温度不得超过 70°C。长时间运行时使用板载
温度传感器监测。

来源：`ed_uav_lidar/config/lidar.yaml`：

```yaml
# 健康阈值（待根据硬件配置）
# temperature_warn_c: 60
# temperature_error_c: 70
```

---

## 3. FOV 和遮挡分析

### 3.1 Mid-360 FOV

Mid-360 的 FOV 为 **360° × 59°**：
 - 水平：360°（完整旋转）
 - 垂直：59°（水平线上下各 29.5°）

### 3.2 遮挡要求

遮挡分析必须验证：

| 要求 | 阈值 | 验证 |
|---|---|---|
| 水平扇区遮挡 | 不得有 >15° 的扇区被遮挡 | FOV 分析 |
| 螺旋桨保护圈侵入 | 悬停期间不得有侵入 | 静态测量 |
| 起落架遮挡 | 不得有持续遮挡 | 动态分析 |
| 相机机身侵入 | 位于激光雷达 FOV 外 | CAD 分析 |

### 3.3 遮挡来源

| 来源 | 风险 | 缓解措施 |
|---|---|---|
| 螺旋桨保护圈 | 高（靠近螺旋桨） | 设计间隙 |
| 起落架 | 中（位于质心下方） | 可收起或采用纤细设计 |
| 相机机身 | 低（侧装） | 安装在 FOV 外 |
| 线缆 | 低 | 避开 FOV 布线 |
| 机架臂 | 中（取决于设计） | 尽量减小臂厚度 |

### 3.4 FOV 验证流程

1. 将传感器安装到机架
2. 将机架放在水平表面
3. 采集 360° 点云
4. 分析水平扇区中的间隙
5. 验证没有扇区 >15° 被连续结构遮挡
6. 记录遮挡图

---

## 4. 坐标系

### 4.1 TF 树结构

来源：`ros2_contract_manifest.json`：

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

| 边 | 发布者 | 主题 |
|---|---|---|
| `map → odom` | `ed_uav_localization.field_anchor` | `/tf_static` |
| `odom → base_link` | `ed_uav_localization.source_supervisor` | `/tf` |

### 4.4 URDF 定义

来源：`ed_uav_description/urdf/ed_uav.urdf.xacro`：

```xml
<link name="lidar_link"/>
<joint name="base_to_lidar" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="$(arg lidar_xyz)" rpy="$(arg lidar_rpy)"/>
</joint>
```

启动时，`lidar_xyz` 和 `lidar_rpy` 参数从标定 YAML 填充。

---

## 5. lidar_link 变换

### 5.1 标定 YAML 格式

来源：`ed_uav_description/config/synthetic_calibrated.yaml`：

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

| 字段 | 类型 | 描述 |
|---|---|---|
| `xyz_m` | [x, y, z] | 相对于 `base_link` 的位置偏移，单位为米 |
| `rpy_rad` | [roll, pitch, yaw] | 姿态偏移，单位为弧度 |

### 5.3 标准安装值

对于激光向下的标准底面安装：

| 坐标系 | xyz_m | rpy_rad | 说明 |
|---|---|---|---|
| `lidar_link` | [0.0, 0.0, 0.15] | [0.0, 0.0, 0.0] | 质心上方 15cm，单位旋转 |
| `fcu_link` | [0.0, 0.0, 0.0] | [0.0, 0.0, 0.0] | 位于质心 |
| `rangefinder_link` | [0.0, 0.0, -0.06] | [0.0, 0.0, 0.0] | 质心下方 6cm |

### 5.4 标定流程

1. 将传感器刚性安装到机架
2. 测量 `base_link` 原点到传感器中心的物理偏移
3. 测量物理旋转（标准安装应为单位旋转）
4. 用实测值更新标定 YAML
5. 设置 `calibration_status: CALIBRATED`
6. 运行 `check_urdf` 验证 TF 树
7. 采集静态 TF 并与预期值比较

来源：`ed_uav_description/calibration.py`：

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

来源：`ed_uav_lidar/config.py`：

```python
class TimeAuthority(str, Enum):
    HOST = "host"
    PTP = "ptp"
```

| 模式 | 状态码 | 描述 |
|---|---|---|
| `host` | `HOST_TIME_UNVERIFIED` | 传感器使用主机系统时间 |
| `ptp` | `PTP_CONFIGURED_UNVERIFIED` | 传感器时钟与 PTP 主时钟同步 |

**注意**：两种模式都不声称已经完成实测同步。在硬件验证（Task 24）前，两者都报告
UNVERIFIED 状态。

### 6.2 PTP 配置

Mid-360 支持使用 PTPv2（IEEE 1588）进行时间同步。

From `ed_uav_lidar/config/lidar.yaml`:

```yaml
/lidar_transport:
  ros__parameters:
    time_authority: host  # or ptp
```

### 6.3 PTP 设置要求

启用 PTP：

1. **网络**：Mid-360 通过以太网连接到主机
2. **PTP 主时钟**：主机运行 ptp4l 作为主时钟
3. **PHC 同步**：phc2sys 将主机时钟同步到 PTP 硬件时钟
4. **验证**：PTP 偏移 ≤1 ms

```bash
# PTP 设置示例（需使用硬件验证）
sudo ptp4l -i eth0 -m -S &
sudo phc2sys -s eth0 -c CLOCK_REALTIME -w &
```

### 6.4 时间验证

来源：`ed_uav_lidar/health.py`：

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

健康状态：
- `LIDAR_ACTIVE` — 所有时间戳均为新鲜
- `LIDAR_POINT_STALE` — 点云时间戳过期
- `LIDAR_IMU_STALE` — IMU 时间戳过期
- `LIDAR_DRIVER_TIMEOUT` — 驱动心跳过期
- `LIDAR_DRIVER_DEAD` — 驱动进程已退出

---

## 7. 驱动配置

### 7.1 激光雷达配置

来源：`ed_uav_lidar/config/lidar.yaml`：

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

来源：`ed_uav_lidar/config/mid360_driver.json`：

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

来源：`ed_uav_lidar/config.py`：

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
- `serial_number` 不是 `UNSET`
- `sensor_ip` 不是 `0.0.0.0`
- `firmware_version` 不是 `UNSET`
- `driver_config_path` 不是内置占位配置

---

## 8. 启动流程

### 8.1 启动计划

来源：`ed_uav_lidar/launch_plan.py`：

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

来源：`ros2_contract_manifest.json`：

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

来源：`ed_uav_lidar/contracts.py`：

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

`mid360_adapter` 将 Livox CustomMsg 转换为标准 PointCloud2：

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

错误条件：
- `MissingPointTiming` — 每个点都必须有 offset_time
- `PointTimeRegression` — 检测到 offset_time 回退
- `PacketShapeError` — 声明的点数不匹配

---

## 10. 振动测试

### 10.1 飞行前验证

首次飞行前必须验证：

| 测试 | 时长 | 标准 |
|---|---|---|
| 静态测试 | 60 seconds | 点云密度稳定（±5%），IMU 噪声符合规格 |
| 电机测试 | 60 seconds | 点云密度稳定，IMU 噪声增幅 < 静态值的 2× |
| 飞行测试 | 30 seconds | LIO 在 5s 内收敛，无发散，漂移 ≤5 cm |

### 10.2 静态测试

1. 将机架放在水平表面
2. 关闭电机
3. 采集 60 seconds 的 lidar/IMU 数据
4. 分析：
   - 点云密度（每秒点数）
   - IMU 噪声（加速度计/陀螺仪方差）
   - 振动产生的异常点

### 10.3 电机测试

1. 将机架放在水平表面
2. 将电机设为悬停油门（50%）
3. 采集 60 seconds 的 lidar/IMU 数据
4. 分析：
   - 点云密度稳定性
   - 振动引起的伪影
   - IMU 噪声增幅（应 < 静态基线的 2×）

### 10.4 飞行测试

1. 在 1 meter 高度悬停 30 seconds
2. 分析：
   - LIO 收敛时间（应 <5 seconds）
   - LIO 发散（应无发散）
   - 位置漂移（30 seconds 内应 ≤5 cm）

---

## 11. 验收标准

来源：Task 24（硬件门控）：

| 标准 | 阈值 | 验证 |
|---|---|---|
| 运行时长 | 30-minute run | 计时器 |
| 时间戳回退 | 0 | `test_timestamp_regression.py` |
| 驱动重启 | 0 | 进程监视器 |
| 丢失样本 | <0.1% | 健康监测器 |
| PTP 偏移 | ≤1 ms（声称使用 PTP 时） | PTP 状态 |
| 外壳温度 | ≤70°C | 板载传感器 |
| 热降频 | 无 | 温度日志 |
| 水平遮挡 | 不得有扇区 >15° 被遮挡 | FOV 分析 |
| LIO 间隙 | 不得有 >0.20s 的间隙 | `lio_health.py` |
| 静态漂移 | 60s 内 ≤5 cm | 位置日志 |

---

## 12. 参考资料

- Livox Mid-360 官方用户手册和规格
- `ed_uav_lidar/config/lidar.yaml` — 激光雷达 ROS 参数
- `ed_uav_lidar/config/mid360_driver.json` — Livox 驱动配置
- `ed_uav_lidar/ed_uav_lidar/config.py` — 类型化配置解析器
- `ed_uav_lidar/ed_uav_lidar/launch_plan.py` — 启动规划
- `ed_uav_lidar/ed_uav_lidar/mid360_adapter.py` — CustomMsg 到 PointCloud2
- `ed_uav_lidar/ed_uav_lidar/health.py` — 健康监测
- `ed_uav_lidar/ed_uav_lidar/contracts.py` — 规范化契约
- `ed_uav_description/urdf/ed_uav.urdf.xacro` — URDF 定义
- `ed_uav_description/ed_uav_description/calibration.py` — 标定解析器
- `ed_uav_description/config/synthetic_calibrated.yaml` — 标定示例
- `ed_uav_interfaces/contracts/ros2_contract_manifest.json` — 冻结契约
- `ed_uav_localization/ed_uav_localization/lio_health.py` — LIO 健康状态
