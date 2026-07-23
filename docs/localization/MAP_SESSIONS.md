# Map Sessions

> **Status**: Contract defined, not implemented (Task 13 contract layer)
> **Owner**: `ed_uav_localization.map_archive`
> **Contract**: `/localization/start_map_session` (frozen in `ros2_contract_manifest.json`)

---

## 1. Overview

A map session captures a fresh point-cloud archive and trajectory from FAST-LIO
odometry during a single flight run. Each session is atomic: either the archive
is complete and valid, or it remains a `.partial` staging directory that can be
cleaned up safely.

The session system is designed around three principles:

1. **Atomic completion** — no half-written archives can be mistaken for valid maps
2. **Fresh-map rule** — LIO odometry remains available during accumulation
3. **Deterministic replay** — session lifecycle is reproducible from fixture data

### Current State

| Component | File | Status |
|---|---|---|
| `StartMapSession.srv` interface | `ed_uav_interfaces/srv/StartMapSession.srv` | **Defined and built** |
| Contract manifest entry | `ed_uav_interfaces/contracts/ros2_contract_manifest.json` | **Frozen** |
| `map_session.py` session manager | `ed_uav_localization/` | **Not implemented** |
| `test_map_session.py` tests | `ed_uav_localization/test/` | **Not implemented** |
| `.partial` staging pattern | `ed_uav_verification/artifacts.py` | **Reference implementation** |
| LIO health monitoring | `ed_uav_localization/lio_health.py` | **Implemented** |
| Source supervisor | `ed_uav_localization/source_supervisor.py` | **Implemented** |

---

## 2. StartMapSession Service Definition

The frozen service contract is defined in `ed_uav_interfaces/srv/StartMapSession.srv`:

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

### Request Fields

| Field | Type | Constraint | Description |
|---|---|---|---|
| `session_id` | string | ≤64 chars | Caller-provided unique identifier |
| `archive_root` | string | ≤128 chars | Filesystem root for archive storage |
| `record_pointcloud` | bool | — | Whether to accumulate point cloud data |

### Response Fields

| Field | Type | Constraint | Description |
|---|---|---|---|
| `accepted` | bool | — | Whether the session was started |
| `reason` | string | ≤96 chars | Rejection reason if `accepted=false` |
| `staging_uri` | string | ≤128 chars | Path to `.partial` staging directory |

### Contract Manifest Registration

From `ros2_contract_manifest.json`:

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

Only `ed_uav_localization.map_archive` may own this service. No other node
may register or respond on `/localization/start_map_session`.

---

## 3. Session Lifecycle

```
START → ACCUMULATING → FINALIZING → COMPLETE
                ↓              ↓
            CANCELLED     .partial (recoverable)
```

### 3.1 START

Client calls `StartMapSession` with session metadata. The service validates:
- `session_id` is unique (no active session with same ID)
- `archive_root` is writable
- `record_pointcloud` is consistent with current sensor state

If validation passes, returns `accepted=true` with `staging_uri` pointing to
`<archive_root>/<session_id>.partial/`.

### 3.2 ACCUMULATING

LIO odometry and point clouds are staged to the `.partial` directory:
- Point clouds are appended to `pointcloud.pcd` (if `record_pointcloud=true`)
- Trajectory poses are appended to `trajectory.csv`
- Metadata is updated periodically

**Critical invariant**: LIO odometry is NOT gated by session state. The
`/localization/lio/odom` topic continues to publish regardless of session
status. This is the "fresh-map rule" — see Section 6.

### 3.3 FINALIZING

On session stop (client call or timeout):
1. All buffered data is flushed
2. Checksums are computed for all files
3. `manifest.json` is written with SHA-256 hashes
4. `metadata.json` is updated with `completion_status: "complete"`

### 3.4 COMPLETE

The `.partial` directory is atomically renamed to the final archive path:
```
<archive_root>/<session_id>.partial/  →  <archive_root>/<session_id>/
```

The rename is atomic on POSIX filesystems. No observer can see a half-complete
archive at the final path.

### 3.5 CANCELLED

If the process is interrupted during ACCUMULATING or FINALIZING:
- The `.partial` directory remains on disk
- No final archive exists at the non-`.partial` path
- Cleanup scripts can detect and remove stale partials by age

---

## 4. Archive Format

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

### 4.1 metadata.json Schema

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

| Field | Type | Description |
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

### 4.2 trajectory.csv Format

```csv
timestamp_ns,x,y,z,qx,qy,qz,qw
1721745000000000000,0.000,0.000,0.000,0.000,0.000,0.000,1.000
1721745000050000000,0.001,0.000,0.000,0.000,0.000,0.000,1.000
...
```

Each row is one LIO odometry pose. Timestamps are nanoseconds since epoch.
Positions are in meters (ENU). Quaternions are (qx, qy, qz, qw).

### 4.3 manifest.json Schema

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

## 5. Atomic Completion with .partial Staging

The archive uses a two-phase commit pattern, following the reference
implementation in `ed_uav_verification/artifacts.py`.

### 5.1 Reference Pattern

From `artifacts.py`, the `EventArtifactWriter` and `FixtureBagBuilder` classes
implement this exact pattern:

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

### 5.2 Map Session Implementation Pattern

The map session should follow the same two-phase commit:

1. **Staging**: All files are written to `<session_id>.partial/`
   - `trajectory.csv` is appended incrementally
   - `pointcloud.pcd` is written in chunks
   - `metadata.json` is updated periodically

2. **Validation**: On session stop
   - Compute SHA-256 for all files
   - Write `manifest.json` with checksums
   - Verify all checksums match

3. **Commit**: Atomic rename
   - `partial.replace(final_path)` — POSIX atomic rename
   - No observer can see a half-complete archive

### 5.3 Error Handling

| Scenario | Behavior |
|---|---|
| Process crash during staging | `.partial` directory remains, no final archive |
| Disk full during staging | `.partial` directory remains, no final archive |
| Duplicate session_id | Reject with `accepted=false, reason="session exists"` |
| Stale `.partial` from previous run | Reject with `accepted=false, reason="stale partial"` |
| Validation failure | `.partial` directory remains, no final archive |

### 5.4 Cleanup Policy

Stale `.partial` directories should be cleaned up by:
- Age threshold (e.g., >1 hour old)
- Manual cleanup script
- Pre-flight check before starting new session

---

## 6. Fresh-Map Rule

**Position is available while mapping.** The LIO odometry stream is not gated
by session completion.

### 6.1 Design Rationale

The `/localization/lio/odom` topic is published by FAST-LIO independently of
the map session system. This means:

- Navigation can use LIO odometry during map accumulation
- The map archive is a byproduct of flight, not a prerequisite
- Each formal run may start a new session without waiting for archive completion
- If the session fails, navigation is unaffected

### 6.2 LIO Health During Session

The LIO health monitor (`ed_uav_localization/lio_health.py`) evaluates:

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

Health states:
- `HEALTHY` — LIO odometry fresh and stable
- `DEGRADED` — LIO odometry stale or time regression detected
- `LOST` — LIO odometry absent or covariance blown up

The session manager should monitor LIO health during accumulation but must NOT
gate odometry publication on session state.

### 6.3 Source Supervisor Integration

The source supervisor (`ed_uav_localization/source_supervisor.py`) manages
LIO/visual source switching. Key thresholds:

| Parameter | Default | Description |
|---|---|---|
| `lio_max_age_active` | 0.15s | Max LIO age for ACTIVE state |
| `lio_max_age_degraded` | 0.50s | Max LIO age for DEGRADED state |
| `lost_timeout` | 1.0s | Seconds without odometry before LOST |
| `covariance_blowup` | 1e6 | Diagonal covariance threshold |

The session manager should log source state transitions but must NOT interfere
with source switching decisions.

---

## 7. Implementation Roadmap

### 7.1 Files to Create

| File | Purpose |
|---|---|
| `ed_uav_localization/map_session.py` | Session manager node |
| `ed_uav_localization/test/test_map_session.py` | Unit tests |
| `ed_uav_localization/config/map_session.yaml` | Default parameters |

### 7.2 Session Manager Node

The `map_session.py` node should:

1. Register as service server for `/localization/start_map_session`
2. Manage session lifecycle (START → ACCUMULATING → FINALIZING → COMPLETE)
3. Subscribe to `/localization/lio/odom` for trajectory
4. Subscribe to `/lidar/points` for point cloud (if enabled)
5. Write archives following the `.partial` staging pattern
6. Monitor LIO health during accumulation

### 7.3 Test Requirements

From Task 13 acceptance criteria:

- Replay a deterministic timestamped lidar/IMU fixture at real-time factor ≥1
- `/localization/lio/odom` is finite, monotonic and available before session completion
- Only the designated localization node owns `odom → base_link`
- Successful stop atomically creates a complete manifest/archive
- Simulated disk-full/interruption leaves a discoverable `.partial` archive

---

## 8. Acceptance Criteria

From Task 13:

| Criterion | Verification |
|---|---|
| LIO odometry available before session completion | Replay fixture, check `/localization/lio/odom` publishes |
| LIO odometry finite and monotonic | Validate timestamps in fixture replay |
| Only localization node owns `odom → base_link` | TF authority check in `test_static_tf_ownership.py` |
| Atomic archive creation | Stop session, verify no `.partial` remains |
| Interruption recovery | Kill process during staging, verify `.partial` exists |
| Deterministic replay | Same seed produces byte-identical archive |

---

## 9. Related Documentation

- `docs/localization/LOCALIZATION_AND_FAILOVER.md` — Source switching and failover
- `docs/architecture/ROS2_CONTRACTS.md` — Frozen ROS graph contract
- `docs/architecture/ROS2_ARCHITECTURE.md` — System architecture
- `docs/testing/ACCEPTANCE.md` — Milestone acceptance criteria

---

## 10. References

- `ed_uav_interfaces/srv/StartMapSession.srv` — Service definition
- `ed_uav_interfaces/contracts/ros2_contract_manifest.json` — Contract manifest
- `ed_uav_verification/artifacts.py` — Reference `.partial` staging pattern
- `ed_uav_localization/lio_health.py` — LIO health monitoring
- `ed_uav_localization/source_supervisor.py` — Source switching logic
- `ed_uav_localization/field_anchor.py` — map→odom TF broadcaster
