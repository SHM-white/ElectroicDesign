# 地图会话

> **状态**：契约已定义，尚未实现（任务 13 契约层）
> **所有者**：`ed_uav_localization.map_archive`
> **契约**：`/localization/start_map_session`（已在 `ros2_contract_manifest.json` 中冻结）

---

## 1. 概览

地图会话在一次飞行运行期间，从 FAST-LIO 里程计采集新点云归档和轨迹。每个会话都是原子的：归档要么完整有效，要么保留为可安全清理的 `.partial` 暂存目录。

会话系统遵循三个原则：

1. **原子完成**：半写入归档不能被误认为有效地图
2. **新地图规则**：累积期间 LIO 里程计仍然可用
3. **确定性回放**：会话生命周期可由固定测试数据复现

### 当前状态

| 组件 | 文件 | 状态 |
|---|---|---|
| `StartMapSession.srv` 接口 | `ed_uav_interfaces/srv/StartMapSession.srv` | **已定义并构建** |
| 契约清单条目 | `ed_uav_interfaces/contracts/ros2_contract_manifest.json` | **已冻结** |
| `map_session.py` 会话管理器 | `ed_uav_localization/` | **未实现** |
| `test_map_session.py` 测试 | `ed_uav_localization/test/` | **未实现** |
| `.partial` 暂存模式 | `ed_uav_verification/artifacts.py` | **参考实现** |
| LIO 健康监测 | `ed_uav_localization/lio_health.py` | **已实现** |
| 源监督器 | `ed_uav_localization/source_supervisor.py` | **已实现** |

---

## 2. StartMapSession 服务定义

冻结的服务契约定义于 `ed_uav_interfaces/srv/StartMapSession.srv`：

```
# A new session only; saved-map loading and relocalization are not contracts.
string<=64 session_id
string<=128 archive_root
bool record_pointcloud
---
bool accepted
string<=96 reason
string<=128 staging_uri
```

### 请求字段

| 字段 | 类型 | 约束 | 说明 |
|---|---|---|---|
| `session_id` | string | ≤64 字符 | 调用方提供的唯一标识符 |
| `archive_root` | string | ≤128 字符 | 归档存储的文件系统根目录 |
| `record_pointcloud` | bool | — | 是否累积点云数据 |

### 响应字段

| 字段 | 类型 | 约束 | 说明 |
|---|---|---|---|
| `accepted` | bool | — | 会话是否已启动 |
| `reason` | string | ≤96 字符 | `accepted=false` 时的拒绝原因 |
| `staging_uri` | string | ≤128 字符 | `.partial` 暂存目录的路径 |

### 契约清单注册

根据 `ros2_contract_manifest.json`：

```json
{
  "name": "/localization/start_map_session",
  "type": "ed_uav_interfaces/srv/StartMapSession",
  "owner": "ed_uav_localization.map_archive",
  "qos": "command_reliable",
  "units": "session identifiers and filesystem paths",
  "frame": "map",
  "clock": "steady_clock"
}
```

只有 `ed_uav_localization.map_archive` 可以拥有此服务。任何其他节点都不得在 `/localization/start_map_session` 上注册或响应。

---

## 3. 会话生命周期

```
START → ACCUMULATING → FINALIZING → COMPLETE
                ↓              ↓
            CANCELLED     .partial (recoverable)
```

### 3.1 START

客户端携带会话元数据调用 `StartMapSession`。服务验证：
- `session_id` 唯一（不存在相同 ID 的活动会话）
- `archive_root` 可写
- `record_pointcloud` 与当前传感器状态一致

验证通过后返回 `accepted=true`，并将 `staging_uri` 指向
`<archive_root>/<session_id>.partial/`.

### 3.2 ACCUMULATING

LIO 里程计和点云暂存到 `.partial` 目录：
- 如果 `record_pointcloud=true`，则将点云追加到 `pointcloud.pcd`
- 将轨迹位姿追加到 `trajectory.csv`
- 定期更新元数据

**关键不变量**：LIO 里程计不受会话状态门控。无论会话处于何种状态，`/localization/lio/odom` 话题都继续发布。这就是“新地图规则”，见第 6 节。

### 3.3 FINALIZING

会话停止时（客户端调用或超时）：
1. 刷新所有缓冲数据
2. 计算所有文件的校验和
3. 写入包含 SHA-256 哈希的 `manifest.json`
4. 使用 `completion_status: "complete"` 更新 `metadata.json`

### 3.4 COMPLETE

`.partial` 目录以原子方式重命名为最终归档路径：
```
<archive_root>/<session_id>.partial/  →  <archive_root>/<session_id>/
```

在 POSIX 文件系统上，重命名是原子的。任何观察者都不会在最终路径看到未完成的归档。

### 3.5 CANCELLED

如果进程在 ACCUMULATING 或 FINALIZING 期间中断：
- `.partial` 目录保留在磁盘上
- 非 `.partial` 路径下不存在最终归档
- 清理脚本可以按目录年龄检测并删除过期暂存目录

---

## 4. 归档格式

```
<session_id>/
├── metadata.json          # session metadata (timestamps, hashes, versions)
├── trajectory.csv         # stamped poses (timestamp_ns,x,y,z,qx,qy,qz,qw)
├── pointcloud.pcd         # accumulated point cloud (if record_pointcloud=true)
├── config/
│   ├── field_profile.yaml # copy of active field profile
│   ├── calibration.yaml   # calibration hash and status
│   └── lidar_config.yaml  # lidar configuration snapshot
└── manifest.json          # SHA-256 checksums of all files
```

### 4.1 metadata.json 模式

```json
{
  "session_id": "flight-2026-07-23-001",
  "start_time": "2026-07-23T14:30:00Z",
  "end_time": "2026-07-23T14:35:00Z",
  "field_profile_id": "competition-2026",
  "calibration_hash": "e1ec326500451dc318cc55568cbc4f4f1247fe24fd9fb619577c36455310b37c",
  "lidar_firmware": "1.2.3",
  "software_version": "ed_uav_localization 0.1.0",
  "frame_conventions": "ENU, base_link at CoM",
  "completion_status": "complete",
  "record_pointcloud": true,
  "point_count": 1234567,
  "trajectory_poses": 6000
}
```

| 字段 | 类型 | 说明 |
|---|---|---|
| `session_id` | string | 调用方提供的唯一 ID |
| `start_time` | ISO 8601 | 会话开始时间戳 |
| `end_time` | ISO 8601 | 会话结束时间戳 |
| `field_profile_id` | string | 当前场地配置 |
| `calibration_hash` | string | 当前标定的 SHA-256 哈希 |
| `lidar_firmware` | string | LiDAR 固件版本 |
| `software_version` | string | ROS 软件包版本 |
| `frame_conventions` | string | 坐标系约定 |
| `completion_status` | string | `"complete"` 或 `"partial"` |
| `record_pointcloud` | bool | 是否记录点云 |
| `point_count` | int | 累积点总数 |
| `trajectory_poses` | int | 轨迹位姿总数 |

### 4.2 trajectory.csv 格式

```csv
timestamp_ns,x,y,z,qx,qy,qz,qw
1721745000000000000,0.000,0.000,0.000,0.000,0.000,0.000,1.000
1721745000050000000,0.001,0.000,0.000,0.000,0.000,0.000,1.000
...
```

每一行表示一个 LIO 里程计位姿。时间戳是自纪元以来的纳秒数。位置单位为米（ENU）。四元数为（qx、qy、qz、qw）。

### 4.3 manifest.json 模式

```json
{
  "schema_version": 1,
  "session_id": "flight-2026-07-23-001",
  "files": {
    "metadata.json": {"sha256": "abc123...", "size_bytes": 1024},
    "trajectory.csv": {"sha256": "def456...", "size_bytes": 65536},
    "pointcloud.pcd": {"sha256": "ghi789...", "size_bytes": 10485760},
    "config/field_profile.yaml": {"sha256": "jkl012...", "size_bytes": 512},
    "config/calibration.yaml": {"sha256": "mno345...", "size_bytes": 256},
    "config/lidar_config.yaml": {"sha256": "pqr678...", "size_bytes": 128}
  }
}
```

---

## 5. 使用 .partial 暂存实现原子完成

归档采用两阶段提交模式，遵循 `ed_uav_verification/artifacts.py` 中的参考实现。

### 5.1 参考模式

`artifacts.py` 中的 `EventArtifactWriter` 和 `FixtureBagBuilder` 类实现了这一模式：

```python
# From ed_uav_verification/artifacts.py (lines 46-62)
class EventArtifactWriter:
    def write(self, report: ScenarioReport) -> Path:
        if not report.completed:
            raise IncompleteScenarioError()
        if self.path.exists():
            raise ArtifactExistsError(self.path)
        partial = self.path.with_name(f"{self.path.name}.partial")
        if partial.exists():
            raise ArtifactExistsError(partial)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        try:
            partial.write_bytes(report.event_json)
            partial.replace(self.path)  # atomic rename
        except OSError:
            partial.unlink(missing_ok=True)
            raise
        return self.path
```

### 5.2 地图会话实现模式

地图会话应遵循相同的两阶段提交流程：

1. **暂存**：所有文件写入 `<session_id>.partial/`
   - 增量追加写入 `trajectory.csv`
   - 分块写入 `pointcloud.pcd`
   - 定期更新 `metadata.json`

2. **验证**：会话停止时
   - 计算所有文件的 SHA-256
   - 写入包含校验和的 `manifest.json`
   - 验证所有校验和匹配

3. **提交**：原子重命名
   - `partial.replace(final_path)`，执行 POSIX 原子重命名
   - 任何观察者都不会看到未完成的归档

### 5.3 错误处理

| 场景 | 行为 |
|---|---|
| 暂存期间进程崩溃 | `.partial` 目录保留，不存在最终归档 |
| 暂存期间磁盘已满 | `.partial` 目录保留，不存在最终归档 |
| `session_id` 重复 | 以 `accepted=false, reason="session exists"` 拒绝 |
| 上次运行遗留的过期 `.partial` | 以 `accepted=false, reason="stale partial"` 拒绝 |
| 验证失败 | `.partial` 目录保留，不存在最终归档 |

### 5.4 清理策略

过期的 `.partial` 目录应通过以下方式清理：
- 年龄阈值（例如超过 1 小时）
- 手动清理脚本
- 启动新会话前的飞行前检查

---

## 6. 新地图规则

**建图期间位置仍然可用。** LIO 里程计流不受会话完成状态门控。

### 6.1 设计理由

`/localization/lio/odom` 话题由 FAST-LIO 独立发布，与地图会话系统无关。因此：

- 建图累积期间，导航可以使用 LIO 里程计
- 地图归档是飞行的副产物，不是前置条件
- 每次正式运行都可以启动新会话，无需等待归档完成
- 会话失败不会影响导航

### 6.2 会话期间的 LIO 健康状态

LIO 健康监测器（`ed_uav_localization/lio_health.py`）评估：

```python
# From lio_health.py (lines 59-93)
def evaluate_health(
    *,
    odom_age_sec: Optional[float],
    imu_age_sec: Optional[float],
    time_regression: bool,
    covariance_finite: bool,
    covariance_exceeds: bool,
    no_odom_duration_sec: Optional[float],
    thresholds: Optional[HealthThresholds] = None,
) -> LIOHealth:
```

健康状态：
- `HEALTHY`：LIO 里程计新鲜且稳定
- `DEGRADED`：LIO 里程计过期或检测到时间回退
- `LOST`：LIO 里程计缺失或协方差失控

会话管理器应在累积期间监测 LIO 健康状态，但不得根据会话状态门控里程计发布。

### 6.3 与源监督器集成

源监督器（`ed_uav_localization/source_supervisor.py`）管理 LIO/视觉源切换。关键阈值如下：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `lio_max_age_active` | 0.15s | ACTIVE 状态允许的最大 LIO 数据年龄 |
| `lio_max_age_degraded` | 0.50s | DEGRADED 状态允许的最大 LIO 数据年龄 |
| `lost_timeout` | 1.0s | 没有里程计数据达到该时长后进入 LOST |
| `covariance_blowup` | 1e6 | 协方差对角线阈值 |

会话管理器应记录源状态转换，但不得干预源切换决策。

---

## 7. 实现路线图

### 7.1 待创建文件

| 文件 | 用途 |
|---|---|
| `ed_uav_localization/map_session.py` | 会话管理器节点 |
| `ed_uav_localization/test/test_map_session.py` | 单元测试 |
| `ed_uav_localization/config/map_session.yaml` | 默认参数 |

### 7.2 会话管理器节点

`map_session.py` 节点应：

1. 为 `/localization/start_map_session` 注册服务端
2. 管理会话生命周期（START → ACCUMULATING → FINALIZING → COMPLETE）
3. 订阅 `/localization/lio/odom` 获取轨迹
4. 在启用时订阅 `/lidar/points` 获取点云
5. 按照 `.partial` 暂存模式写入归档
6. 在累积期间监测 LIO 健康状态

### 7.3 测试要求

根据任务 13 的验收标准：

- 以实时倍率 ≥1 回放带时间戳的确定性 lidar/IMU 固定数据
- `/localization/lio/odom` 有限、单调，并在会话完成前可用
- 只有指定的定位节点拥有 `odom → base_link`
- 成功停止后以原子方式创建完整的清单/归档
- 模拟磁盘已满或中断时留下可发现的 `.partial` 归档

---

## 8. 验收标准

根据任务 13：

| 标准 | 验证方式 |
|---|---|
| 会话完成前 LIO 里程计可用 | 回放固定数据，检查 `/localization/lio/odom` 是否发布 |
| LIO 里程计有限且单调 | 验证固定数据回放中的时间戳 |
| 只有定位节点拥有 `odom → base_link` | 在 `test_static_tf_ownership.py` 中检查 TF 权限 |
| 原子创建归档 | 停止会话，确认不存在 `.partial` |
| 中断恢复 | 暂存期间终止进程，确认 `.partial` 存在 |
| 确定性回放 | 相同种子生成字节完全一致的归档 |

---

## 9. 相关文档

- `docs/localization/LOCALIZATION_AND_FAILOVER.md`：定位源切换和失效切换
- `docs/architecture/ROS2_CONTRACTS.md`：冻结的 ROS 图契约
- `docs/architecture/ROS2_ARCHITECTURE.md`：系统架构
- `docs/testing/ACCEPTANCE.md`：里程碑验收标准

---

## 10. 参考资料

- `ed_uav_interfaces/srv/StartMapSession.srv`：服务定义
- `ed_uav_interfaces/contracts/ros2_contract_manifest.json`：契约清单
- `ed_uav_verification/artifacts.py`：`.partial` 暂存模式参考实现
- `ed_uav_localization/lio_health.py`：LIO 健康监测
- `ed_uav_localization/source_supervisor.py`：源切换逻辑
- `ed_uav_localization/field_anchor.py`：map→odom TF 广播器
