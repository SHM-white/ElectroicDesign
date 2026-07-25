# Acceptance Criteria

> **Status**: Offline gates passing (Task 23)
> **Owner**: Project-wide
> **Date**: 2026-07-23

---

## 1. Overview

This document defines acceptance criteria for each milestone in the ROS 2 UAV
project. The project uses a three-tier gate system:

1. **Offline gates** (Tasks 1-23) — Code, build, lint, type checks, unit tests
2. **Hardware gates** (Tasks 24-27) — Physical hardware validation
3. **Flight gates** (Tasks 28-29) — Indoor flight acceptance

### Current Status

| Tier | Tasks | Status |
|---|---|---|
| Offline | 1-23 | **PASSING** |
| Hardware | 24-27 | **PENDING-HARDWARE** |
| Flight | 28-29 | **PENDING-HARDWARE** |

---

## 2. Milestone Categories

### 2.1 TODAY (Offline Only) — Tasks 1-23

These gates run on the development host (Ubuntu 24.04/WSL) via the Humble
container. No hardware, no flight, no calibration claims.

| Gate | Command | Pass Criteria |
|---|---|---|
| Build | `colcon build --symlink-install` | Exit 0, all packages built |
| Test | `colcon test --event-handlers console_direct+` | Exit 0 |
| Test result | `colcon test-result --all --verbose` | 0 errors, 0 failures |
| Lint | `ruff check ros2_ws/src ml tools` | Exit 0 (warnings OK) |
| Type check | `basedpyright ros2_ws/src ml tools` | Exit 0 (warnings OK) |
| Pytest | `pytest -q drone/test ml tools -m "not field_data and not hardware and not flight"` | Exit 0 |
| Protected hashes | `python3 tools/parity_check.py` | All 3 match baseline |
| Field fixtures | `python3 tools/check_field_fixtures.py --expect-current-state` | Exit 0 |
| Milestone | `python3 tools/verify_today_milestone.py --strict` | Exit 0 |

### 2.1.1 Offline Integration Iteration

The completed offline integration iteration has five operator-facing entry
points. Run each command from the repository root. Each script records a
timestamped run directory under `.omo/evidence/offline-integration/scripts/`
with a `SUCCESS` marker or a `FAILED` marker and exit code.

| Stage | Command | Green marker | Evidence and debugging purpose |
|---|---|---|---|
| Static surface | `bash tools/run_offline_static.sh` | `STATIC_OFFLINE_GREEN` | Contract, launch profile, interface, focused test, and legacy parity logs. Use first to isolate static and environment failures. |
| Wall-time simulation | `bash tools/run_offline_sim.sh` | `SIM_OFFLINE_GREEN` | Build and live simulation logs. Checks the deterministic synthetic graph in wall time. |
| WSLg RViz | `bash tools/run_offline_rviz.sh` | `RVIZ_OFFLINE_GREEN` | Packaged config, RViz process, and launch logs. Checks visualization startup and display wiring. |
| FCU dry run | `bash tools/run_offline_fcu_dry_run.sh` | `FCU_DRY_RUN_GREEN` | Fake PTY and real bridge logs. Checks telemetry, framing, PTY cleanup, and shutdown. |
| Full event replay | `bash tools/run_offline_full_replay.sh` | `FULL_REPLAY_GREEN` | Event creation, bag info, replay, build, and test logs. Checks `/verification/events` replay lifecycle. |

The live deterministic simulation is wall-time only because this surface has no
`/clock`. `use_sim_time=true` is rejected. The RViz stage uses WSLg through
`HUMBLE_GUI=1` and shows synthetic visualization-only robot geometry, TF, lidar
points, and two images. Odometry displays remain absent until an authorized TF
owner exists.

The rosbag output contains only `/verification/events`. It is event-only, not a
sensor replay or flight replay. The FCU dry-run uses a fake PTY and the real
bridge, does not use `/dev/ttyUSB*`, and does not establish HIL, hardware, or
flight acceptance.

The current receipt is under `.omo/evidence/offline-integration/`. It supplements
the original milestone results below. It does not replace or renumber the
historical test totals. Stored stage evidence includes `wall-time/`, `rviz/`,
`rviz-visual/`, `rosbag/`, `fcu-final/`, and timestamped runs under `scripts/`.

### 2.1.2 Serial and Flight Command Security Boundary

Acceptance includes the installed policy template at
`share/ed_uav_bringup/security/fcu_command.policy.xml`. The
`/fcu/flight_command` action is default-disabled. An explicitly enabled runtime
requires `ROS_SECURITY_ENABLE=true`, `ROS_SECURITY_STRATEGY=Enforce`,
`ROS_SECURITY_KEYSTORE`, and signed permissions generated from that template.
The bridge enclave is granted `execute`; the mission executor is granted
`call`; all other callers remain denied by middleware policy. The template has
no credentials. Offline PTY checks remain credential-free and command-disabled.

Serial acceptance requires canonical device-number identity locking together
with `TIOCEXCL` and `flock` for cooperating new opens. These controls do not
evict a descriptor opened earlier, so an owner preflight or broker is required
before hardware validation. This boundary is documented and accepted offline;
no signed-keystore runtime, hardware, HIL, or flight authorization is claimed.

### 2.2 HARDWARE (Tasks 24-27) — PENDING-HARDWARE

These gates require the target i5 hardware, Mid-360, cameras, and FCU.

| Gate | Task | Pass Criteria |
|---|---|---|
| Mid-360 bringup | 24 | 30-min run, 0 timestamp regression, <0.1% drops, shell ≤70°C |
| Camera calibration | 25 | Both cameras calibrated, RMS ≤0.5px (narrow), ≤0.8px (wide) |
| Propulsion BOM | 26 | Thrust/weight ≥2.0, hover ≤50%, endurance ≥1.5x mission |
| FCU HIL | 27 | 20 consecutive command cycles, 0 unexpected mode changes |

### 2.3 FLIGHT (Tasks 28-29) — PENDING-HARDWARE

These gates require indoor flight area, safety net, trained pilot.

| Gate | Task | Pass Criteria |
|---|---|---|
| First flight | 28 | 5 consecutive sorties, hover drift ≤0.15m, no lock in air |
| Competition rehearsal | 29 | 5 consecutive rehearsals, no human intervention after start |

---

## 3. Test Categories

### 3.1 Unit Tests (always run)

- Pure Python logic tests
- No ROS infrastructure required
- No hardware required
- Marker: none (default)

### 3.2 Integration Tests (always run)

- ROS 2 node tests with launch_testing
- Uses fake/simulated sensors
- Marker: none (default)

### 3.3 Field Data Tests (skip in CI)

- Depend on `mission_vision_*.png` fixtures
- Currently 13 tests (9 vision, 2 gray marker, 2 home cross)
- Marker: `field_data`
- Skip reason: field images not available in CI

### 3.4 Hardware Tests (skip in CI)

- Require physical hardware (Mid-360, cameras, FCU)
- Marker: `hardware`
- Skip reason: hardware not available in CI

### 3.5 Flight Tests (skip in CI)

- Require indoor flight area
- Marker: `flight`
- Skip reason: flight area not available in CI

---

## 4. Offline Gate Details

### 4.1 Colcon Build Gate

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'
```

**Pass criteria**: Exit 0, all 10 packages built without error.

| Package | Status |
|---|---|
| ed_uav_interfaces | Built |
| ed_uav_description | Built |
| ed_uav_fcu_bridge | Built |
| ed_uav_lidar | Built |
| ed_uav_camera | Built |
| ed_uav_localization | Built |
| ed_uav_perception | Built |
| ed_uav_mission | Built |
| ed_uav_bringup | Built |
| ed_uav_verification | Built |

### 4.2 Colcon Test Gate

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon test --event-handlers console_direct+'
```

**Pass criteria**: Exit 0, 132 tests, 0 errors, 0 failures.

| Package | Tests |
|---|---|
| ed_uav_fcu_bridge | 27 |
| ed_uav_localization | 52 |
| ed_uav_mission | 26 |
| ed_uav_perception | 27 |
| **Total** | **132** |

### 4.3 Python Lint Gate

```bash
./tools/run_humble.sh bash -lc 'ruff check ros2_ws/src ml tools'
```

**Pass criteria**: Exit 0. Warnings are acceptable (E402, F401, F841).

### 4.4 Python Type Gate

```bash
./tools/run_humble.sh bash -lc 'basedpyright ros2_ws/src ml tools'
```

**Pass criteria**: Exit 0. Type warnings are acceptable.

### 4.5 Pytest Gate

```bash
PYTHONPATH=ml/yolo/src ./.venv/bin/python -m pytest -q \
  drone/test ml tools \
  -m "not field_data and not hardware and not flight" \
  --strict-markers
```

**Pass criteria**: Exit 0, 365 passed, 13 deselected (field_data).

### 4.6 Protected Hash Gate

```bash
python3 tools/parity_check.py
```

**Pass criteria**: All 3 protected files match baseline SHA-256 hashes.

| File | Expected Hash |
|---|---|
| `drone/start.sh` | `9658f7ea196c5801cb743e56db7495134d16104180382d1e0ac6c4d47518054e` |
| `drone/debug_start.sh` | `af24ba8a196c5801cb743e56db7495134d16104180382d1e0ac6c4d47518054e` |
| `drone/field_test.sh` | `dda7ecb3196c5801cb743e56db7495134d16104180382d1e0ac6c4d47518054e` |

### 4.7 Field Fixture Gate

```bash
python3 tools/check_field_fixtures.py --manifest drone/test/fixtures/field-images.json --expect-current-state
```

**Pass criteria**: Exit 0. All 29 declared fixtures are absent and match manifest hashes.

---

## 5. Hardware Gate Details

### 5.1 Mid-360 Bringup (Task 24)

**Requirements**:
- Target i5 hardware with Mid-360 connected via Ethernet
- Livox driver configured with real serial, IP, firmware
- PTP or host time synchronization configured

**Pass criteria**:

| Criterion | Threshold | Verification |
|---|---|---|
| Runtime duration | ≥30 minutes | Timer |
| Timestamp regression | 0 | `test_timestamp_regression.py` |
| Dropped samples | <0.1% | Health monitor |
| Shell temperature | ≤70°C | Onboard sensor |
| PTP offset | ≤1 ms | PTP status |
| LIO gap | No >0.20s gap | `lio_health.py` |
| Static drift | ≤5 cm over 60s | Position log |

### 5.2 Camera Calibration (Task 25)

**Requirements**:
- Both USB cameras connected
- ChArUco board printed and measured
- Capture script available (P25 work)

**Pass criteria**:

| Criterion | Threshold | Verification |
|---|---|---|
| Narrow camera RMS | ≤0.5 px | `calibrate_intrinsics.py` |
| Wide camera RMS | ≤0.8 px | `calibrate_intrinsics.py` |
| Holdout error | <1.5× train error | `calibrate_intrinsics.py` |
| Serial binding | Matches device | `validate_calibration.py` |
| Resolution match | Matches runtime mode | `calibration.py` gate |
| Freshness | `captured_at + valid_for > now` | `calibration.py` gate |

### 5.3 Propulsion BOM (Task 26)

**Requirements**:
- Replacement motors and ESCs installed
- Propellers matched to motors
- Battery capacity verified

**Pass criteria**:

| Criterion | Threshold | Verification |
|---|---|---|
| Thrust/weight ratio | ≥2.0 | Thrust stand |
| Hover throttle | ≤50% | Flight controller |
| Endurance | ≥1.5× mission duration | Battery test |
| Thermal stability | No throttling | Temperature log |

Offline preparation checker command:

```bash
tools/check_flight_readiness.py --bom docs/hardware/BOM.json --measurements <dated-dir> --strict
```

This checker validates readiness evidence format, traceability, hashes, and thresholds, but does not substitute for the required physical measurements.

### 5.4 FCU HIL (Task 27)

**Requirements**:
- Lingxiao FCU connected via serial
- HIL simulation environment
- Command protocol validated

**Pass criteria**:

| Criterion | Threshold | Verification |
|---|---|---|
| Command cycles | 20 consecutive | Counter |
| Mode changes | 0 unexpected | State machine log |
| Response latency | <100 ms | Timer |
| Serial reliability | 0 fragmentation errors | `test_serial_fragmentation.py` |

---

## 6. Flight Gate Details

### 6.1 First Flight (Task 28)

**Requirements**:
- Indoor flight area with safety net
- Trained pilot present
- All hardware gates passed
- Safety checklist completed

**Pass criteria**:

| Criterion | Threshold | Verification |
|---|---|---|
| Consecutive sorties | 5 | Counter |
| Hover drift | ≤0.15 m | Position log |
| Lock in air | 0 occurrences | State machine log |
| Localization loss recovery | <2 s | Timer |
| No human intervention | After start | Video review |

### 6.2 Competition Rehearsal (Task 29)

**Requirements**:
- All first-flight gates passed
- Field profile configured
- Mission plan loaded
- Competition scenario simulated

**Pass criteria**:

| Criterion | Threshold | Verification |
|---|---|---|
| Consecutive rehearsals | 5 | Counter |
| Human intervention | 0 after start | Video review |
| Mission completion | 100% | Mission log |
| Localization availability | ≥99% | Status log |
| Safety supervisor | 0 false triggers | Diagnostic log |

---

## 7. Resource Budget Gates

### 7.1 Memory Growth

From `ed_uav_verification/test/resource/test_memory_growth.py`:

| Criterion | Threshold | Test |
|---|---|---|
| Heap growth after warmup | <3× | 10-min soak × 2 |
| RSS bounded | Under 50 cycles | Cycle test |
| Event size proportional | To ticks | Proportionality check |

### 7.2 Disk Reserve

From `ed_uav_verification/test/resource/test_disk_reserve.py`:

| Criterion | Threshold | Test |
|---|---|---|
| Event artifact size | <1 MiB | Size check |
| Fixture bag size | <10 MiB | Size check |
| Partial write cleanup | No stale `.partial` | Cleanup check |
| File descriptor leak | 0 | Leak check |

### 7.3 CPU Contention

From `ed_uav_verification/test/resource/test_cpu_contention.py`:

| Criterion | Threshold | Test |
|---|---|---|
| Per-tick latency | <50 ms | Latency check |
| Real-time factor | ≥1.0 | Factor check |
| Linear scaling | With tick count | Scaling check |
| Concurrent replay | No deadlock | Deadlock check |
| p99 safety latency | <100 ms | Percentile check |

---

## 8. Fault Injection Gates

### 8.1 Timestamp Regression

From `ed_uav_verification/test/faults/test_timestamp_regression.py`:

| Fault | Expected Behavior |
|---|---|
| Non-monotonic timestamp | Detected and rejected |
| Recovery after regression | Byte-identical survivor streams |
| Bounded regression | Magnitude tracked |

### 8.2 Lidar Silence

From `ed_uav_verification/test/faults/test_lidar_silence.py`:

| Fault | Expected Behavior |
|---|---|
| Lidar silence | Health degradation |
| IMU silence | Health degradation |
| TF continuity | Maintained under silence |
| Detection latency | Bounded |
| Deadlock | None |

### 8.3 Camera Hot Unplug

From `ed_uav_verification/test/faults/test_camera_hot_unplug.py`:

| Fault | Expected Behavior |
|---|---|
| Camera disconnect | Degradation + recovery |
| Stream isolation | No collateral lidar damage |
| Stale data acceptance | None |
| Motor cut | None |

### 8.4 Serial Fragmentation

From `ed_uav_verification/test/faults/test_serial_fragmentation.py`:

| Fault | Expected Behavior |
|---|---|
| Truncated frames | Rejected |
| Bit-flipped checksums | Rejected |
| Length corruption | Rejected |
| Interleaved garbage | Rejected |
| PTY timeout | No deadlock |

### 8.5 Shutdown Interruption

From `ed_uav_verification/test/faults/test_shutdown_interruption.py`:

| Fault | Expected Behavior |
|---|---|
| Bounded interruption | Clean restart |
| Determinism | Identical replay |
| Motor cut on shutdown | None |
| Tick budget exhaustion | Handled |
| Deadlock | None |

---

## 9. Safety Supervisor Gates

From `ed_uav_mission/test/test_safety_supervisor.py`:

| Scenario | Expected Behavior |
|---|---|
| Localization loss | HOVER within 0 ticks |
| Recovery within 2.0s | Resume normal |
| Recovery after 2.0s | LAND (no recovery) |
| Land without ACK | 3 retries → CRITICAL |
| Lock refused (altitude >10cm) | Lock-in-air prevention |
| Comm loss | CRITICAL |
| Low voltage | LAND |
| Stale AUX | LAND |
| Mission timeout | LAND |

---

## 10. Calibration Gate

From `ed_uav_description/test/test_calibration_gate.py`:

| Condition | Result |
|---|---|
| `calibration_status == UNCALIBRATED` | REJECTED |
| `calibration_status == SYNTHETIC` | REJECTED |
| Missing calibration file | REJECTED |
| Mismatched sensor serials | REJECTED |
| Stale calibration hash | REJECTED |
| Malformed calibration | REJECTED |
| `calibration_status == CALIBRATED` + matching serials + current hash | ACCEPTED |

---

## 11. Evidence Requirements

### 11.1 Code Evidence (Tasks 1-23)

- Source files exist and compile
- Tests pass in Humble container
- Protected file hashes match baseline
- Documentation is source-backed

### 11.2 Hardware Evidence (Tasks 24-27)

- Dated reports with instrument readings
- USB tree, device IDs, firmware versions
- Calibration YAMLs with reprojection overlays
- Thermal/power logs

### 11.3 Flight Evidence (Tasks 28-29)

- Synchronized bags/video
- Safety checklist signed
- 5 consecutive passing runs
- No post-start human intervention

---

## 12. Verification Tools

| Tool | Purpose |
|---|---|
| `tools/verify_today_milestone.py` | Run all offline gates |
| `tools/parity_check.py` | Verify protected file integrity |
| `tools/check_field_fixtures.py` | Verify field image manifest |
| `tools/check_third_party.py` | Verify third-party pins |
| `tools/validate_field_profile.py` | Validate field profiles |
| `tools/check_competition_docs.py` | Validate competition documentation |

---

## 13. References

- `docs/testing/TODAY_MILESTONE.md` — Current milestone results
- `docs/testing/LEGACY_BASELINE.md` — Protected file baseline
- `.omo/plans/ros2-uav-refactor.md` — Full task plan
- `ros2_ws/src/ed_uav_verification/test/` — Verification test suite
- `ros2_ws/src/ed_uav_mission/test/test_safety_supervisor.py` — Safety gates
- `ros2_ws/src/ed_uav_description/test/test_calibration_gate.py` — Calibration gates
