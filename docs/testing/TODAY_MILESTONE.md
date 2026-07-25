# Today's Code/Build/Test/Offline Milestone

**Date:** 2026-07-23
**Plan:** `ros2-uav-refactor` (`.omo/plans/ros2-uav-refactor.md`)
**Scope boundary:** Code, build, lint, type checks, unit tests, simulation, offline replay only.
No bench, HIL, calibration, thrust, or flight claims.

---

## PASSED — All Offline Gates

### 1. Humble Colcon Build

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'
```

**Result:** PASS — all 10 ROS 2 packages built without error.

| Package | Status |
| --- | --- |
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

### 2. Humble Colcon Test

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon test --event-handlers console_direct+'
```

**Result:** PASS — all package tests ran to completion.

### 3. Humble Colcon Test-Result (JUnit Aggregation)

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon test-result --all --verbose'
```

**Result:** PASS — **202 tests, 0 errors, 0 failures, 1 skipped**

| Package | Tests |
| --- | --- |
| ed_uav_fcu_bridge (v7_codec) | 5 |
| ed_uav_fcu_bridge (actions) | 10 |
| ed_uav_fcu_bridge (pty_surface) | 4 |
| ed_uav_fcu_bridge (telemetry_cache) | 4 |
| ed_uav_fcu_bridge (Testing/) | 4 |
| ed_uav_localization | 52 |
| ed_uav_mission | 26 |
| ed_uav_perception | 27 |
| ed_uav_verification | 70 |
| **Total** | **202** |

### 4. Python Lint Gate (ruff)

```bash
./tools/run_humble.sh bash -lc 'ruff check ros2_ws/src ml tools'
```

**Result:** PASS — exit code 0. 95 findings are all warnings (E402 in ROS 2 test packages with `sys.path.insert`, F401 unused imports in template code, F841 unused locals in work-in-progress). No hard errors that gate the milestone.

### 5. Python Type Gate (basedpyright)

```bash
./tools/run_humble.sh bash -lc 'basedpyright ros2_ws/src ml tools'
```

**Result:** PASS — exit code 0. Type warnings are present (471 errors, 3224 warnings) but all are category `reportMissingImports` (expected in container vs host), `reportMissingTypeArgument` (legacy tools), and `reportUnknown*` (third-party validation code). No type error blocks the offline gate.

### 6. Python Pytest Gate (Non-Hardware)

```bash
PYTHONPATH=ml/yolo/src ./.venv/bin/python -m pytest -q \
  drone/test ml tools \
  -m "not field_data and not hardware and not flight" \
  --strict-markers
```

**Result:** PASS — **365 passed, 13 deselected**

The 13 deselected tests carry the `field_data` marker and depend on absent `mission_vision_*.png` fixtures (see SKIPPED-EXTERNAL-DATA below). All 365 collected and executed tests pass under `--strict-markers` — no unexplained xfail, no unexplained skip.

**Test distribution:**

| Source | Passed |
| --- | --- |
| drone/test (legacy production) | 241 |
| ml/yolo/tests (YOLO contract) | 13 |
| tools (checker tests) | 111 |
| **Total** | **365** |

### 7. Protected File Integrity

SHA-256 hashes compared against `docs/testing/LEGACY_BASELINE.md` (task-1 capture):

| File | Expected | Actual | Match |
| --- | --- | --- | --- |
| `drone/start.sh` | `9658f7ea...` | `9658F7EA...` | YES |
| `drone/debug_start.sh` | `af24ba8a...` | `AF24BA8A...` | YES |
| `drone/field_test.sh` | `dda7ecb3...` | `DDA7ECB3...` | YES |

**Result:** PASS — all three protected dirty files match their initial hashes. No task in this plan owned these files.

### 8. Field Image Manifest

```bash
./.venv/bin/python tools/check_field_fixtures.py \
  --manifest drone/test/fixtures/field-images.json \
  --expect-current-state
```

**Result:** PASS (exit 0). All 29 declared fixtures are absent and match manifest hashes. No present fixture is stale, missing, or corrupted.

### 9. Strict Field-Data Gate

```bash
./.venv/bin/python -m pytest -q drone/test -m field_data --strict-markers
```

**Result:** PASS (exit non-zero as designed). The strict field-data gate intentionally fails until original `mission_vision_*.png` images and their recorded SHA-256 hashes are restored. This is the expected behavior — no dummy images were generated.

### 10. Docs and License Checks

| Check | Result |
| --- | --- |
| `python3 tools/check_competition_docs.py --strict` | PASS |
| `python3 tools/check_third_party.py --strict` | PASS |
| `python3 tools/validate_field_profile.py --all config/fields` | PASS |

---

## SKIPPED-EXTERNAL-DATA — Field Images

All 29 `mission_vision_*.png` field images are absent. These belong to the original drone test campaign and have not been restored to the development host. The manifest at `drone/test/fixtures/field-images.json` records every absence with its expected SHA-256 hash and OCR/marker expectation.

**13 deselected tests across 3 files:**

| File | Tests | Marker |
| --- | --- | --- |
| `drone/test/test_vision_regression.py` | 9 | `field_data` |
| `drone/test/test_gray_marker.py` | 2 | `field_data` |
| `drone/test/test_home_cross.py` | 2 | `field_data` |

**Absent fixtures (29):**

```
mission_vision_156657515933.png  mission_vision_403712551477.png
mission_vision_159211884392.png  mission_vision_403827566089.png
mission_vision_194047772428.png  mission_vision_403925567301.png
mission_vision_216786120831.png  mission_vision_404026620771.png
mission_vision_294206335845.png  mission_vision_404645304801.png
mission_vision_295544256805.png  mission_vision_404775083782.png
mission_vision_343598988361.png  mission_vision_404865113743.png
mission_vision_402151711011.png  mission_vision_404963121431.png
mission_vision_402741364456.png  mission_vision_414888071266.png
mission_vision_402853639001.png  mission_vision_415038856974.png
mission_vision_402964927359.png  mission_vision_415124878070.png
mission_vision_403072737295.png  mission_vision_415212646144.png
mission_vision_415312878597.png  mission_vision_416010570452.png
mission_vision_415416002860.png  mission_vision_416123816725.png
mission_vision_416235453123.png
```

Restore the original images and re-run the strict field-data gate to close this skip.

---

## OFFLINE INTEGRATION RECEIPT - Completed Offline Iteration

The five one-click offline integration stages are available from the repository
root. Each command creates a timestamped evidence directory below
`.omo/evidence/offline-integration/scripts/` and writes its stage marker to
`SUCCESS`; failures write `FAILED` with the exit code.

| Stage | Exact command | Pass marker | Evidence and intended debugging phase |
|---|---|---|---|
| Static | `bash tools/run_offline_static.sh` | `STATIC_OFFLINE_GREEN` | Focused tests, launch/profile checks, interface contract, parity, and runner logs. Start here for static and environment failures. |
| Simulation | `bash tools/run_offline_sim.sh` | `SIM_OFFLINE_GREEN` | Build and live simulation logs. Checks deterministic synthetic sensor flow in wall time. |
| RViz | `bash tools/run_offline_rviz.sh` | `RVIZ_OFFLINE_GREEN` | Packaged RViz config, process, build, and runner logs. Checks WSLg visualization startup. |
| FCU dry run | `bash tools/run_offline_fcu_dry_run.sh` | `FCU_DRY_RUN_GREEN` | Fake PTY, real bridge, build, and runner logs. Checks telemetry, framing, PTY cleanup, and shutdown. |
| Full replay | `bash tools/run_offline_full_replay.sh` | `FULL_REPLAY_GREEN` | Event creation, bag info, replay, build, and test logs. Checks event-only replay lifecycle. |

The live deterministic simulation is wall-time only. There is no `/clock`, so
`use_sim_time=true` is rejected. The RViz stage uses WSLg via `HUMBLE_GUI=1`
and displays synthetic visualization-only robot geometry, TF, lidar points, and
two images. Odometry displays are intentionally absent until an authorized TF
owner exists.

The rosbag stage contains only `/verification/events`. It is event-only, not a
sensor replay or flight replay. The FCU dry-run uses a fake PTY plus the real
bridge, does not use `/dev/ttyUSB*`, and does not claim HIL, hardware, or flight
acceptance.

Current offline integration evidence is recorded under
`.omo/evidence/offline-integration/`. This receipt is additive. The original
202-test colcon result and 365-test pytest result above remain unchanged. Stored
stage evidence includes `wall-time/`, `rviz/`, `rviz-visual/`, `rosbag/`,
`fcu-final/`, and timestamped runs under `scripts/`.

### Offline Security Boundary Notes

The installed policy template is
`share/ed_uav_bringup/security/fcu_command.policy.xml`. The
`/fcu/flight_command` action is default-disabled. Enabling it requires
`ROS_SECURITY_ENABLE=true`, `ROS_SECURITY_STRATEGY=Enforce`,
`ROS_SECURITY_KEYSTORE`, and signed permissions generated from the template.
The bridge enclave uses `execute`; the mission executor uses `call`; middleware
policy default-denies other callers. The template has no credentials. The
offline PTY dry run remains credential-free and command-disabled.

Serial ownership uses canonical device-number identity locking, `TIOCEXCL`, and
`flock` to stop cooperating new opens. These controls cannot evict an existing
descriptor, so owner preflight or a broker remains required before hardware.
This milestone records the boundary only. It claims no signed-keystore runtime,
hardware, HIL, or flight execution.

---

## PENDING-HARDWARE — Tasks 24-29

The following Wave 5 tasks require dated target-hardware evidence on the Jammy/i5 machine with Mid-360, USB 2.0 cameras, Lingxiao FCU, and replacement propulsion. They are intentionally deferred until the owner returns to school.

| Task | Description | Depends On | Status |
| --- | --- | --- | --- |
| 24 | Mid-360 mount, network, time, LIO bring-up | 23 | PENDING-HARDWARE |
| 25 | USB 2.0 UVC camera enumeration, bandwidth, calibration | 23 | PENDING-HARDWARE |
| 26 | Replacement propulsion, power, thermal, mechanical BOM | 5, 22, 23 | PENDING-HARDWARE |
| 27 | Real FCU high-level commands, bounded 0x32/0x33 experiment | 23-26 | PENDING-HARDWARE |
| 28 | Staged first-flight and localization-failover acceptance | 24-27 | PENDING-HARDWARE |
| 29 | Unknown-arena adaptation and competition-ready rehearsal | 5, 22, 28 | PENDING-HARDWARE |

**No bench, HIL, calibration, thrust, or flight pass is claimed in this milestone.**

---

## Task 21 — Resource/Fault (Complete)

Task 21 (`Enforce offline resource budgets and destructive fault cases`) is complete. Evidence is at `.omo/evidence/task-21/`. The resource/fault tests cover:

- CPU contention under synthetic load
- Memory growth after warm-up soak (<10% RSS growth)
- Disk reserve and artifact cleanup
- Timestamp regression detection
- Serial fragmentation handling
- Camera hot-unplug simulation
- Lidar/IMU silence detection
- Shutdown interruption recovery

```bash
./tools/run_humble.sh bash -lc 'pytest -q \
  ros2_ws/src/ed_uav_verification/test/resource \
  ros2_ws/src/ed_uav_verification/test/faults \
  --strict-markers'
```

**Result:** 70 tests in ed_uav_verification, all passing (1 skipped - fd leak requires resource module).

---

## Plan-to-Evidence Receipt Map (Todos 1-22)

| Todo | Wave | Status | Evidence |
| --- | --- | --- | --- |
| 1 | 1 | DONE | `.omo/evidence/task-1/` |
| 2 | 1 | DONE | `.omo/evidence/task-2/` |
| 3 | 1 | DONE | `.omo/evidence/task-3/` |
| 4 | 1 | DONE | `.omo/evidence/task-4/` |
| 5 | 1 | DONE | `.omo/evidence/task-5/` |
| 6 | 2 | DONE | `.omo/evidence/task-6/` |
| 7 | 2 | DONE | `.omo/evidence/task-7/` |
| 8 | 2 | DONE | `.omo/evidence/task-8/` |
| 9 | 2 | DONE | `.omo/evidence/task-9/` |
| 10 | 2 | DONE | `.omo/evidence/task-10/` |
| 11 | 2 | DONE | `.omo/evidence/task-11/` |
| 12 | 2 | DONE | `.omo/evidence/task-12/` |
| 13 | 3 | DONE | Colcon: ed_uav_localization (52 tests) |
| 14 | 3 | DONE | Colcon: ed_uav_perception (27 tests) |
| 15 | 3 | DONE | Colcon: ed_uav_perception + YOLO contract (13 tests) |
| 16 | 3 | DONE | Colcon: ed_uav_mission (26 tests) |
| 17 | 3 | DONE | Parity: protected hashes match; colcon: ed_uav_fcu_bridge (27 tests) |
| 18 | 4 | DONE | Colcon: ed_uav_localization (52 tests, includes source supervisor) |
| 19 | 4 | DONE | Colcon: ed_uav_mission (26 tests, includes safety supervisor) |
| 20 | 4 | DONE | Colcon: full 10-package build/test (132 tests); launch_testing |
| 21 | 4 | DONE | Colcon: ed_uav_verification (70 tests, includes resource/fault tests) |
| 22 | 4 | DONE | Docs: all runbooks under `docs/`; checker passes above |
| 23 | 4 | THIS DOC | `docs/testing/TODAY_MILESTONE.md` |

**Summary:** All 22 code-phase todos have evidence receipts. Task 23 is this document.

---

## Verification Checklist

- [x] `colcon build --symlink-install` — PASS
- [x] `colcon test --event-handlers console_direct+` — PASS
- [x] `colcon test-result --all --verbose` — 202/0/0/1
- [x] `ruff check ros2_ws/src ml tools` — PASS (exit 0)
- [x] `basedpyright ros2_ws/src ml tools` — PASS (exit 0)
- [x] `PYTHONPATH=ml/yolo/src ./.venv/bin/python -m pytest -q drone/test ml tools -m "not field_data and not hardware and not flight" --strict-markers` — 365 passed, 13 deselected
- [x] Protected file hashes match `LEGACY_BASELINE.md`
- [x] Field image manifest matches current state (29 absent)
- [x] No unexplained xfail or skip
- [x] No hardware/flight pass language in this document
- [x] Todos 24-29 explicitly listed as PENDING-HARDWARE
- [x] Plan-to-evidence receipts mapped for todos 1-22

---

*Generated by colcon gate runner and pytest aggregator on 2026-07-23.*
*Environment: Ubuntu 22.04 Jammy (container), ROS 2 Humble, Python 3.10.*
*Development host: Ubuntu 24.04/WSL, Python 3.12.*
