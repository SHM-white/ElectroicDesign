# Localization and Failover Runbook

> Source: `ros2_ws/src/ed_uav_localization/` (source_supervisor.py, lio_health.py,
> field_anchor.py), `ros2_ws/src/ed_uav_mission/` (safety_supervisor.py),
> `ros2_ws/src/ed_uav_interfaces/msg/LocalizationStatus.msg`.

---

## 1. Architecture Overview

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

### What Does NOT Exist Yet

| Component | Contract Reference | Status |
|---|---|---|
| EKF node (`odom → base_link` TF) | `ros2_contract_manifest.json` | **Not implemented** |
| `/localization/start_map_session` service | `StartMapSession.srv` | **Not implemented** |
| `LifecycleNode` wrappers | — | Not used (standard `rclpy.node.Node`) |

The `SourceSupervisor` currently does **pass-through source selection**, not
sensor fusion. The selected source's odometry is published directly to
`/localization/odom`.

---

## 2. Primary Source: LIO (FAST-LIO)

### 2.1 Data Path

```
Livox Mid-360 → livox_ros_driver2 → /livox/lidar (CustomMsg)
                                     /livox/imu (Imu)
         ↓
    FAST-LIO (external process)
         ↓
    /localization/lio/odom (Odometry)
```

### 2.2 Health Monitoring

`LIOHealthMonitor` (`lio_health.py`) subscribes to:
- `/localization/lio/odom` — odometry output
- `/imu/data` — raw IMU (configurable topic)

Publishes `/localization/lio/health` (`DiagnosticArray`) at 10 Hz.

**Health evaluation** (`evaluate_health()` — pure function, testable):

| Condition | Result |
|---|---|
| `odom_age > lost_timeout (1.0s)` | `LOST` |
| `imu_age > lost_timeout (1.0s)` | `LOST` |
| `!covariance_finite` | `LOST` |
| Any diagonal `> 1e6` | `LOST` |
| `time_regression` (clock jumped backward) | `DEGRADED` |
| `odom_age > max_age_active (0.15s)` | `DEGRADED` |
| Otherwise | `HEALTHY` |

### 2.3 Source State Evaluation

`SourceSupervisor.evaluate_source_state()` classifies each source:

| State | Conditions |
|---|---|
| `LOST` | No messages ever, covariance non-finite, covariance > 1e6 diagonal, no messages for > 1.0 s |
| `DEGRADED` | Age > `max_age_degraded` (0.5 s), or age > `max_age_active` (0.15 s LIO / 0.20 s visual), or time regression |
| `ACTIVE` | Fresh, finite covariance, within age thresholds |

---

## 3. Visual Boundary Fallback

### 3.1 Data Path

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

### 3.2 BoundaryObservation DOF Mask

| Bit | DOF | Value |
|---|---|---|
| X | 1 | Position X |
| Y | 2 | Position Y |
| Z | 4 | Altitude |
| Roll | 8 | Roll angle |
| Pitch | 16 | Pitch angle |
| Yaw | 32 | Yaw angle |

Full pose (X, Y, Yaw) requires ≥ 2 non-parallel boundary constraints with
inter-line angle > 30°. Single line → yaw-only constraint (`DOF_YAW` mask).

### 3.3 Visual Stability Gate

`is_visual_stable()` requires:
- ≥ `visual_consecutive_samples` (5) consecutive valid observations
- Spanning ≥ `visual_stability_duration` (0.5 s)

Until this gate passes, the supervisor will **not** switch to visual as primary.

---

## 4. Source Switching Logic

### 4.1 State Machine

Implemented in `decide_source_switch()` — pure function, 17 unit tests in
`test_source_supervisor.py`.

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

### 4.2 Switching Rules

| Current | Condition | Action |
|---|---|---|
| LIO primary | LIO ACTIVE | Stay LIO |
| LIO primary | LIO LOST + visual stable + hysteresis (2.0 s) | Switch → VISUAL |
| LIO primary | Both LOST | Switch → NONE |
| VISUAL primary | LIO ACTIVE + hysteresis (2.0 s) | Switch → LIO |
| VISUAL primary | Visual LOST + LIO not LOST | Switch → LIO (even DEGRADED) |
| VISUAL primary | Both LOST | Switch → NONE |
| NONE | LIO ACTIVE | Switch → LIO |
| NONE | Visual stable | Switch → VISUAL |

### 4.3 Hysteresis

The `switch_hysteresis_sec` (2.0 s) prevents rapid oscillation between sources.
A switch is only allowed if the target source has been in the required state
continuously for ≥ 2.0 seconds.

---

## 5. No-Jump Constraints

### 5.1 Pose Alignment Gate

Before any source switch, `poses_aligned()` checks:

| Check | Threshold |
|---|---|
| Position difference | ≤ `max_switch_position_diff_m` (0.25 m) |
| Yaw difference | ≤ `max_switch_yaw_diff_rad` (10° ≈ 0.175 rad) |

If either threshold is exceeded, the switch is **blocked** and the supervisor
remains on the current source (even if degraded). This prevents position jumps
when transitioning between LIO and visual odometry.

### 5.2 Implementation

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

### 5.3 When No-Jump Blocks a Recovery

If LIO recovers but its pose has drifted > 0.25 m from the visual estimate, the
supervisor will NOT switch back to LIO. The system remains on visual until either:
- LIO converges back within tolerance, or
- Visual is also lost (→ NONE → hover → land)

---

## 6. Loss Handling: Hover → Land

### 6.1 SafetySupervisor State Machine

Source: `ed_uav_mission/safety_supervisor.py`

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

### 6.2 Transition Details

| From | Trigger | Action |
|---|---|---|
| ACTIVE | `LocalizationStatus.state == LOST` | Issue `FlightCommand.HOVER` |
| HOVERING | Still LOST after 2.0 s | Issue `FlightCommand.LAND` |
| HOVERING | `state != LOST` (recovered) | Return to ACTIVE |
| LANDING | Check descent progress | Continue landing |
| LANDING | 3 retries exhausted, no descent | Transition to CRITICAL |

### 6.3 Critical State

In CRITICAL state:
- No automatic recovery
- Manual operator takeover required
- Motors are **NOT** automatically locked in air (per contract: "it never
  automatically locks motors in air")

### 6.4 Other Loss Triggers

| Trigger | Action |
|---|---|
| FCU communication loss | → CRITICAL |
| Low battery voltage | → LAND |
| Stale AUX status | → LAND |
| Mission timeout | → LAND |

---

## 7. Field Anchor (map → odom)

### 7.1 Purpose

`FieldAnchor` publishes the `map → odom` transform that converts odometry into
a fixed map frame. It is the **only** authorized publisher of this edge.

### 7.2 Behavior

1. Load field profile YAML (must be `KnownFieldProfile`, not `UnknownArenaProfile`)
2. Wait for first `/localization/odom` message (up to `takeoff_timeout_sec`, 10 s)
3. Compute: `T_map_odom = T_map_base * T_odom_base_inverse`
4. Publish on `/tf_static` via `StaticTransformBroadcaster`

### 7.3 Field Profile Requirements

From `field_profile/model.py` (Pydantic validation):
- Boundary segments must have nonzero length
- At least 2 non-parallel segments (angle > 10°)
- No-fly zones strictly inside allowed zone
- No self-intersecting polygons
- Unique identifiers across all elements

---

## 8. Operator Procedures

### 8.1 Pre-Flight Checks

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

### 8.2 Monitoring During Flight

```bash
# Watch localization status
ros2 topic echo /localization/status

# Watch for source switches (log messages)
ros2 topic echo /rosout | grep -i "source\|switch\|lost\|degraded"
```

### 8.3 Post-Flight Analysis

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

## 9. Acceptance Criteria

| Gate | Criterion | Verification |
|---|---|---|
| LIO active | `/localization/status` shows `source=1, state=1` | `ros2 topic echo` |
| Visual stable | ≥ 5 consecutive valid boundary observations | `test_source_supervisor.py` |
| No-jump | Position diff ≤ 0.25 m, yaw diff ≤ 10° at switch | `test_source_supervisor.py` |
| Hover on loss | `FlightCommand.HOVER` issued within 1 cycle (50 ms) | `test_source_supervisor.py` |
| Land on sustained loss | `FlightCommand.LAND` issued after 2.0 s | `test_source_supervisor.py` |
| No motor lock in air | Safety supervisor never issues DISARM while airborne | Contract review |
| map→odom authority | Only `field_anchor` publishes this TF edge | `verify_static_tf.py` |
