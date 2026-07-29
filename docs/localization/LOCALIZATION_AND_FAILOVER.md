# 定位与失效切换操作手册

> 来源：`ros2_ws/src/ed_uav_localization/`（source_supervisor.py、lio_health.py、
> field_anchor.py）、`ros2_ws/src/ed_uav_mission/`（safety_supervisor.py）、
> `ros2_ws/src/ed_uav_interfaces/msg/LocalizationStatus.msg`。

---

## 1. 架构概览

```
┌─────────────────┐     ┌─────────────────┐
│  FAST-LIO       │     │ Boundary        │
│  (external)     │     │ Localizer       │
│  /localization/ │     │ /localization/  │
│  lio/odom       │     │ boundary_obs.   │
└────────┬────────┘     └────────┬────────┘
         │                       │
         ▼                       ▼
┌──────────────────────────────────────────┐
│         SourceSupervisor                 │
│  • evaluate_source_state() per source    │
│  • decide_source_switch() state machine  │
│  • poses_aligned() no-jump gate         │
│  Publishes:                              │
│    /localization/odom  (Odometry)        │
│    /localization/status (LocalizationStatus) │
└────────────────────┬─────────────────────┘
                     │
                     ▼
┌──────────────────────────────────┐
│       FieldAnchor                │
│  map → odom TF broadcaster       │
│  Waits for first odom message    │
│  Publishes: /tf_static           │
└──────────────────────────────────┘
                     │
                     ▼
┌──────────────────────────────────┐
│     SafetySupervisor (mission)   │
│  Subscribes: /localization/status│
│  On LOST → hover → land → critical│
└──────────────────────────────────┘
```

### 当前尚不存在的内容

| 组件 | 契约引用 | 状态 |
|---|---|---|
| EKF 节点（`odom → base_link` TF） | `ros2_contract_manifest.json` | **未实现** |
| `/localization/start_map_session` 服务 | `StartMapSession.srv` | **未实现** |
| `LifecycleNode` 封装 | — | 未使用（标准 `rclpy.node.Node`） |

当前 `SourceSupervisor` 执行的是**直通式源选择**，不是传感器融合。选中源的里程计会直接发布到
`/localization/odom`。

---

## 2. 主定位源：LIO（FAST-LIO）

### 2.1 数据路径

```
Livox Mid-360 → livox_ros_driver2 → /livox/lidar (CustomMsg)
                                     /livox/imu (Imu)
         ↓
    FAST-LIO (external process)
         ↓
    /localization/lio/odom (Odometry)
```

### 2.2 健康监测

`LIOHealthMonitor`（`lio_health.py`）订阅：
- `/localization/lio/odom`，里程计输出
- `/imu/data`，原始 IMU（话题可配置）

以 10 Hz 发布 `/localization/lio/health`（`DiagnosticArray`）。

**健康评估**（`evaluate_health()`，纯函数，可测试）：

| 条件 | 结果 |
|---|---|
| `odom_age > lost_timeout (1.0s)` | `LOST` |
| `imu_age > lost_timeout (1.0s)` | `LOST` |
| `!covariance_finite` | `LOST` |
| 任一对角线元素 `> 1e6` | `LOST` |
| `time_regression`（时钟回退） | `DEGRADED` |
| `odom_age > max_age_active (0.15s)` | `DEGRADED` |
| 其他情况 | `HEALTHY` |

### 2.3 定位源状态评估

`SourceSupervisor.evaluate_source_state()` 按以下规则对每个定位源分类：

| 状态 | 条件 |
|---|---|
| `LOST` | 从未收到消息、协方差非有限、协方差对角线大于 1e6，或超过 1.0 s 未收到消息 |
| `DEGRADED` | 年龄大于 `max_age_degraded`（0.5 s），或年龄大于 `max_age_active`（LIO 为 0.15 s，视觉为 0.20 s），或发生时间回退 |
| `ACTIVE` | 数据新鲜、协方差有限且年龄在阈值内 |

---

## 3. 视觉边界备用源

### 3.1 数据路径

```
Camera (wide) → /camera/wide/image_raw
                      ↓
         BoundaryExtractor (HSV + Canny + Hough)
                      ↓
         compute_boundary_observation()
           • Project image lines to ground plane (z=0)
           • Associate with field profile boundary segments
           • Least-squares correction (dx, dy, dyaw)
                      ↓
         /localization/boundary_observation (BoundaryObservation)
```

### 3.2 BoundaryObservation DOF 掩码

| 位 | DOF | 值 |
|---|---|---|
| X | 1 | X 位置 |
| Y | 2 | Y 位置 |
| Z | 4 | 高度 |
| Roll | 8 | 横滚角 |
| Pitch | 16 | 俯仰角 |
| Yaw | 32 | 航向角 |

完整位姿（X、Y、Yaw）需要至少 2 个不平行的边界约束，且边界线夹角 > 30°。
单条边界线只能提供航向约束（`DOF_YAW` 掩码）。

### 3.3 视觉稳定门

`is_visual_stable()` 要求：
- 至少有 `visual_consecutive_samples`（5）个连续有效观测
- 观测跨度至少为 `visual_stability_duration`（0.5 s）

在通过此门之前，监督器**不会**将视觉源切换为主源。

---

## 4. 定位源切换逻辑

### 4.1 状态机

实现于 `decide_source_switch()`，这是纯函数；相关测试位于
`test_source_supervisor.py`。

```
                    ┌─────────────┐
          LIO ACTIVE│             │Visual stable
          ─────────►│  LIO        │+ hysteresis (2.0s)
                    │  PRIMARY    │─────────┐
                    │             │         │
                    └──────┬──────┘         ▼
                           │         ┌─────────────┐
              LIO LOST     │         │  VISUAL     │
              + visual     │         │  PRIMARY    │
              stable       │         │             │
              + hyst. 2.0s │         └──────┬──────┘
                           │                │
                           ▼                │
                    ┌─────────────┐         │
                    │   NONE      │◄────────┘
                    │  (both lost)│  Both LOST
                    └─────────────┘
```

### 4.2 切换规则

| 当前源 | 条件 | 操作 |
|---|---|---|
| LIO 主源 | LIO ACTIVE | 保持 LIO |
| LIO 主源 | LIO LOST + 视觉稳定 + 滞回（2.0 s） | 切换到 VISUAL |
| LIO 主源 | 两者均 LOST | 切换到 NONE |
| VISUAL 主源 | LIO ACTIVE + 滞回（2.0 s） | 切换到 LIO |
| VISUAL 主源 | 视觉 LOST + LIO 未 LOST | 切换到 LIO（即使为 DEGRADED） |
| VISUAL 主源 | 两者均 LOST | 切换到 NONE |
| NONE | LIO ACTIVE | 切换到 LIO |
| NONE | 视觉稳定 | 切换到 VISUAL |

### 4.3 滞回

`switch_hysteresis_sec`（2.0 s）用于防止定位源之间快速振荡。只有当目标源连续至少 2.0 秒处于所需状态时，才允许切换。

---

## 5. 无跳变约束

### 5.1 位姿对齐门

每次切换定位源前，`poses_aligned()` 都会检查：

| 检查项 | 阈值 |
|---|---|
| 位置差 | ≤ `max_switch_position_diff_m`（0.25 m） |
| 航向差 | ≤ `max_switch_yaw_diff_rad`（10° ≈ 0.175 rad） |

如果任一阈值超出，切换将被**阻止**，监督器保持当前源，即使当前源已降级。这样可防止在 LIO 和视觉里程计之间切换时发生位置跳变。

### 5.2 实现

```python
def poses_aligned(lio_pose, visual_pose,
                  max_position_diff_m=0.25,
                  max_yaw_diff_rad=math.radians(10)) -> bool:
    pos_diff = math.sqrt(
        (lio_pose.x - visual_pose.x)**2 +
        (lio_pose.y - visual_pose.y)**2 +
        (lio_pose.z - visual_pose.z)**2
    )
    yaw_diff = abs(normalize_angle(lio_yaw - visual_yaw))
    return pos_diff <= max_position_diff_m and yaw_diff <= max_yaw_diff_rad
```

### 5.3 无跳变约束阻止恢复时

如果 LIO 恢复但其位姿与视觉估计相差超过 0.25 m，监督器不会切回 LIO。系统继续使用视觉源，直到：
- LIO 重新收敛到容差范围内，或
- 视觉源也丢失（→ NONE → 悬停 → 降落）

---

## 6. 丢失处理：悬停→降落

### 6.1 SafetySupervisor 状态机

来源：`ed_uav_mission/safety_supervisor.py`

```
┌──────────┐   all sources LOST   ┌──────────────────────────┐
│  ACTIVE  │──────────────────────►│ LOCALIZATION_LOST_       │
│          │                      │ HOVERING                 │
└──────────┘                      │ • issues FlightCommand   │
      ▲                           │   HOVER                  │
      │ recovered                 └────────────┬─────────────┘
      │◄───────────────────────────────────────│
      │                                        │ still lost
      │                                        │ after 2.0 s
      │                                        ▼
      │                           ┌──────────────────────────┐
      │                           │ LOCALIZATION_LOST_       │
      │                           │ LANDING                  │
      │                           │ • issues FlightCommand   │
      │                           │   LAND (up to 3 retries) │
      │                           └────────────┬─────────────┘
      │                                        │ retries
      │                                        │ exhausted
      │                                        ▼
      │                           ┌──────────────────────────┐
      │                           │ CRITICAL                 │
      │                           │ • manual takeover needed │
      │                           └──────────────────────────┘
```

### 6.2 转换细节

| 起始状态 | 触发条件 | 操作 |
|---|---|---|
| ACTIVE | `LocalizationStatus.state == LOST` | 发出 `FlightCommand.HOVER` |
| HOVERING | 2.0 s 后仍为 LOST | 发出 `FlightCommand.LAND` |
| HOVERING | `state != LOST`（已恢复） | 返回 ACTIVE |
| LANDING | 检查下降进度 | 继续降落 |
| LANDING | 3 次重试耗尽且没有下降 | 转为 CRITICAL |

### 6.3 CRITICAL 状态

在 CRITICAL 状态下：
- 不执行自动恢复
- 必须由操作员手动接管
- 空中**不会**自动加锁电机（根据契约，系统“绝不会在空中自动加锁电机”）

### 6.4 其他丢失触发条件

| 触发条件 | 操作 |
|---|---|
| FCU 通信丢失 | → CRITICAL |
| 电池电压低 | → LAND |
| AUX 状态过期 | → LAND |
| 任务超时 | → LAND |

---

## 7. 场地锚点（map → odom）

### 7.1 目的

`FieldAnchor` 发布 `map → odom` 变换，将里程计转换到固定的地图坐标系。它是该边**唯一**获授权的发布者。

### 7.2 行为

1. 加载场地配置 YAML（必须是 `KnownFieldProfile`，不能是 `UnknownArenaProfile`）
2. 等待第一条 `/localization/odom` 消息（最长为 `takeoff_timeout_sec`，10 s）
3. 计算：`T_map_odom = T_map_base * T_odom_base_inverse`
4. 通过 `StaticTransformBroadcaster` 发布到 `/tf_static`

### 7.3 场地配置要求

根据 `field_profile/model.py`（Pydantic 验证）：
- 边界线段长度必须非零
- 至少有 2 条不平行线段（夹角 > 10°）
- 禁飞区必须严格位于允许区域内部
- 多边形不得自相交
- 所有元素的标识符必须唯一

---

## 8. 操作员流程

### 8.1 飞行前检查

```bash
# 1. Verify localization source is ACTIVE
ros2 topic echo /localization/status --once
# Expected: state=1 (ACTIVE), source=1 (LIO) or source=2 (VISUAL)

# 2. Verify LIO health
ros2 topic echo /localization/lio/health --once
# Expected: level=0 (OK)

# 3. Verify TF tree
ros2 run tf2_tools view_frames
# Expected: map → odom → base_link → sensor frames

# 4. Verify odom is publishing
ros2 topic hz /localization/odom
# Expected: ~66 Hz (150 ms freshness)
```

### 8.2 飞行期间监测

```bash
# Watch localization status
ros2 topic echo /localization/status

# Watch for source switches (log messages)
ros2 topic echo /rosout | grep -i "source\|switch\|lost\|degraded"
```

### 8.3 飞行后分析

```bash
# Record bag for analysis
ros2 bag record /localization/status /localization/odom \
  /localization/lio/odom /localization/boundary_observation \
  /localization/lio/health /tf /tf_static

# Replay and inspect
ros2 bag play <bag_path>
ros2 topic echo /localization/status
```

---

## 9. 验收标准

| 门 | 标准 | 验证方式 |
|---|---|---|
| LIO 激活 | `/localization/status` 显示 `source=1, state=1` | `ros2 topic echo` |
| 视觉稳定 | 至少 5 个连续有效的边界观测 | `test_source_supervisor.py` |
| 无跳变 | 切换时位置差 ≤ 0.25 m，航向差 ≤ 10° | `test_source_supervisor.py` |
| 丢失后悬停 | 在 1 个周期（50 ms）内发出 `FlightCommand.HOVER` | `test_source_supervisor.py` |
| 持续丢失后降落 | 2.0 s 后发出 `FlightCommand.LAND` | `test_source_supervisor.py` |
| 空中不加锁电机 | 安全监督器在空中从不发出 DISARM | 契约审查 |
| map→odom 权限 | 只有 `field_anchor` 发布该 TF 边 | `verify_static_tf.py` |
