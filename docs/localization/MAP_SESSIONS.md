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
| `session_id` | string | ≤64 chars | Caller-provided unique identifier |
| `archive_root` | string | ≤128 chars | Filesystem root for archive storage |
| `record_pointcloud` | bool | — | Whether to accumulate point cloud data |

### 响应字段

| 字段 | 类型 | 约束 | 说明 |
|---|---|---|---|
| `accepted` | bool | — | Whether the session was started |
| `reason` | string | ≤96 chars | Rejection reason if `accepted=false` |
| `staging_uri` | string | ≤128 chars | Path to `.partial` staging directory |

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
- `session_id` is unique (no active session with same ID)
- `archive_root` is writable
- `record_pointcloud` is consistent with current sensor state

验证通过后返回 `accepted=true`，并将 `staging_uri` 指向
`<archive_root>/<session_id>.partial/`.

### 3.2 ACCUMULATING

LIO 里程计和点云暂存到 `.partial` 目录：
- Point clouds are appended to `pointcloud.pcd` (if `record_pointcloud=true`)
- Trajectory poses are appended to `trajectory.csv`
- Metadata is updated periodically

**关键不变量**：LIO 里程计不受会话状态门控。无论会话处于何种状态，`/localization/lio/odom` 话题都继续发布。这就是“新地图规则”，见第 6 节。

### 3.3 FINALIZING

会话停止时（客户端调用或超时）：
1. All buffered data is flushed
2. Checksums are computed for all files
3. `manifest.json` is written with SHA-256 hashes
4. `metadata.json` is updated with `completion_status: "complete"`

### 3.4 COMPLETE

`.partial` 目录以原子方式重命名为最终归档路径：
```
<archive_root>/<session_id>.partial/  →  <archive_root>/<session_id>/
```

在 POSIX 文件系统上，重命名是原子的。任何观察者都不会在最终路径看到未完成的归档。

### 3.5 CANCELLED

如果进程在 ACCUMULATING 或 FINALIZING 期间中断：
- The `.partial` directory remains on disk
- No final archive exists at the non-`.partial` path
- Cleanup scripts can detect and remove stale partials by age

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
| `session_id` | string | Caller-provided unique ID |
| `start_time` | ISO 8601 | Session start timestamp |
| `end_time` | ISO 8601 | Session end timestamp |
| `field_profile_id` | string | Active field profile |
| `calibration_hash` | string | SHA-256 of active calibration |
| `lidar_firmware` | string | Lidar firmware version |
| `software_version` | string | ROS package versions |
| `frame_conventions` | string | Coordinate frame conventions |
| `completion_status` | string | "complete" or "partial" |
| `record_pointcloud` | bool | Whether point cloud was recorded |
| `point_count` | int | Total points accumulated |
| `trajectory_poses` | int | Total trajectory poses |

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
   - `trajectory.csv` is appended incrementally
   - `pointcloud.pcd` is written in chunks
   - `metadata.json` is updated periodically

2. **验证**：会话停止时
   - Compute SHA-256 for all files
   - Write `manifest.json` with checksums
   - Verify all checksums match

3. **提交**：原子重命名
   - `partial.replace(final_path)` — POSIX atomic rename
   - No observer can see a half-complete archive

### 5.3 错误处理

| 场景 | 行为 |
|---|---|
| Process crash during staging | `.partial` directory remains, no final archive |
| Disk full during staging | `.partial` directory remains, no final archive |
| Duplicate session_id | Reject with `accepted=false, reason="session exists"` |
| Stale `.partial` from previous run | Reject with `accepted=false, reason="stale partial"` |
| Validation failure | `.partial` directory remains, no final archive |

### 5.4 清理策略

过期的 `.partial` 目录应通过以下方式清理：
- Age threshold (e.g., >1 hour old)
- Manual cleanup script
- Pre-flight check before starting new session

---

## 6. 新地图规则

**建图期间位置仍然可用。** LIO 里程计流不受会话完成状态门控。

### 6.1 设计理由

`/localization/lio/odom` 话题由 FAST-LIO 独立发布，与地图会话系统无关。因此：
地图会话系统。因此：

- Navigation can use LIO odometry during map accumulation
- The map archive is a byproduct of flight, not a prerequisite
- Each formal run may start a new session without waiting for archive completion
- If the session fails, navigation is unaffected

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

| Parameter | Default | Description |
|---|---|---|
| `lio_max_age_active` | 0.15s | Max LIO age for ACTIVE state |
| `lio_max_age_degraded` | 0.50s | Max LIO age for DEGRADED state |
| `lost_timeout` | 1.0s | Seconds without odometry before LOST |
| `covariance_blowup` | 1e6 | Diagonal covariance threshold |

会话管理器应记录源状态转换，但不得干预源切换决策。

---

## 7. 实现路线图

### 7.1 待创建文件

| 文件 | 用途 |
|---|---|
| `ed_uav_localization/map_session.py` | Session manager node |
| `ed_uav_localization/test/test_map_session.py` | Unit tests |
| `ed_uav_localization/config/map_session.yaml` | Default parameters |

### 7.2 会话管理器节点

`map_session.py` 节点应：

1. Register as service server for `/localization/start_map_session`
2. Manage session lifecycle (START → ACCUMULATING → FINALIZING → COMPLETE)
3. Subscribe to `/localization/lio/odom` for trajectory
4. Subscribe to `/lidar/points` for point cloud (if enabled)
5. Write archives following the `.partial` staging pattern
6. Monitor LIO health during accumulation

### 7.3 测试要求

根据任务 13 的验收标准：

- Replay a deterministic timestamped lidar/IMU fixture at real-time factor ≥1
- `/localization/lio/odom` is finite, monotonic and available before session completion
- Only the designated localization node owns `odom → base_link`
- Successful stop atomically creates a complete manifest/archive
- Simulated disk-full/interruption leaves a discoverable `.partial` archive

---

## 8. 验收标准

根据任务 13：

| 标准 | 验证方式 |
|---|---|
| LIO odometry available before session completion | Replay fixture, check `/localization/lio/odom` publishes |
| LIO odometry finite and monotonic | Validate timestamps in fixture replay |
| Only localization node owns `odom → base_link` | TF authority check in `test_static_tf_ownership.py` |
| Atomic archive creation | Stop session, verify no `.partial` remains |
| Interruption recovery | Kill process during staging, verify `.partial` exists |
| Deterministic replay | Same seed produces byte-identical archive |

---

## 9. 相关文档

- `docs/localization/LOCALIZATION_AND_FAILOVER.md` — Source switching and failover
- `docs/architecture/ROS2_CONTRACTS.md` — Frozen ROS graph contract
- `docs/architecture/ROS2_ARCHITECTURE.md` — System architecture
- `docs/testing/ACCEPTANCE.md` — Milestone acceptance criteria

---

## 10. 参考资料

- `ed_uav_interfaces/srv/StartMapSession.srv` — Service definition
- `ed_uav_interfaces/contracts/ros2_contract_manifest.json` — Contract manifest
- `ed_uav_verification/artifacts.py` — Reference `.partial` staging pattern
- `ed_uav_localization/lio_health.py` — LIO health monitoring
- `ed_uav_localization/source_supervisor.py` — Source switching logic
- `ed_uav_localization/field_anchor.py` — map→odom TF broadcaster
