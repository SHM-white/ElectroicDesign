# Bringup and Rollback Runbook

> Source: `ros2_ws/src/ed_uav_bringup/`, `ros2_ws/src/ed_uav_verification/`,
> `ros2_ws/src/ed_uav_fcu_bridge/`, `tools/run_humble.sh`,
> `.github/workflows/ros2-ci.yml`, `tools/test_rollback.py`,
> `tools/parity_check.py`.

---

## 1. Deployment Gates Overview

The system defines three gate tiers. Only the **offline gate** is fully
implemented today.

```
┌─────────────────────────────────────────────────────────────┐
│                     OFFLINE GATE (today)                     │
│  CI build/test → launch surface → calibration → contract    │
│  → deterministic scenario → fault matrix → legacy parity    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼ (future)
┌─────────────────────────────────────────────────────────────┐
│                     TARGET GATE (future)                     │
│  Real-device verification on target hardware (no flight)     │
│  Camera capture, LiDAR scan, FCU serial handshake           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼ (future)
┌─────────────────────────────────────────────────────────────┐
│                     HIL GATE (future)                        │
│  Hardware-in-loop with simulated flight dynamics             │
│  Full mission execution with fault injection                 │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼ (future)
┌─────────────────────────────────────────────────────────────┐
│                     FLIGHT GATE (future)                     │
│  Real flight with safety pilot, progressive autonomy        │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Offline Gate — Implemented

### 2.1 Gate 1: CI Build and Test

**File**: `.github/workflows/ros2-ci.yml`

```yaml
# Triggered on: push to main, PRs
# Runner: ubuntu-24.04, 30 min timeout
steps:
  - run: bash tools/test_run_humble.sh          # Gate 1a: runner selection
  - run: tools/run_humble.sh bash -lc '...'     # Gate 1b: Humble environment
  # Gate 1c: workspace build + test (when packages exist)
  - run: tools/run_humble.sh bash -lc '
      source /opt/ros/humble/setup.bash &&
      rosdep install --from-paths ros2_ws/src --ignore-src -r -y &&
      colcon build --symlink-install &&
      colcon test --event-handlers console_direct+ &&
      colcon test-result --all --verbose'
```

**Runner selection** (`tools/run_humble.sh`):
- Ubuntu 22.04 with `/opt/ros/humble` → native execution
- All other hosts → Docker/Podman container (digest-pinned `ros:humble-ros-base-jammy`)
- Validates pinned base image label `io.ed.humble.base-ref`
- 900 s timeout on container operations
- Mutual-exclusion lock prevents concurrent runs

**Acceptance**: All `colcon test` pass, `colcon test-result` shows 0 failures.

### 2.2 Gate 2: Launch Surface Verification

**File**: `ros2_ws/src/ed_uav_bringup/tools/verify_launch_surface.py`

Static AST analysis (no ROS runtime required):

```bash
python3 ros2_ws/src/ed_uav_bringup/tools/verify_launch_surface.py \
  ros2_ws/src/ed_uav_bringup/launch/bringup.launch.py
```

**Checks**:
1. Exactly 7 P06 launch arguments declared
2. Exactly 4 P06 profiles exist (`offline`, `camera_only`, `lidar`, `competition`)
3. `validate_for_profile()` called **before** `Node()` construction
4. No forbidden TF authorities (`static_transform_publisher`, `map → odom`,
   `odom → base_link` as static joints)

**Output**: `BRINGUP: GREEN` or `BRINGUP: RED: <reason>`

### 2.3 Gate 3: Calibration Validation

**File**: `ros2_ws/src/ed_uav_description/tools/validate_calibration.py`

```bash
python3 ros2_ws/src/ed_uav_description/tools/validate_calibration.py \
  ros2_ws/src/ed_uav_description/config/example_uncalibrated.yaml
```

**Checks**:
- Schema version valid
- Calibration hash matches recomputed hash
- All required transforms present
- Competition profile: status must be `CALIBRATED`, serials must not be `UNSET`/`SYNTHETIC-*`

### 2.4 Gate 4: ROS 2 Contract Verification

**File**: `ros2_ws/src/ed_uav_interfaces/tools/check_contract.py`

```bash
./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_contract.py \
  ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json
```

Validates all approved topics, services, actions, TF edges, QoS profiles,
freshness deadlines, lifecycle ordering, and enum values.

### 2.5 Gate 5: Deterministic Scenario Verification

**File**: `ros2_ws/src/ed_uav_verification/ed_uav_verification/cli.py`

```bash
# Run 60-second deterministic scenario
ed-uav-verify --output scenario_events.json
```

**What it does**:
- Virtual monotonic clock (no real time dependency)
- 8 synthetic sensor streams at 20 Hz
- 6 fault injection modes: DROP, FREEZE, CORRUPTION, LATENCY, TIME_REGRESSION, PROCESS_DEATH
- Atomic event artifact writing
- Fault matrix assertion: every fault → activation + degradation + recovery + stream recovery

**Output**: `SCENARIO: GREEN {sha256, duration, ticks}`

### 2.6 Gate 6: Legacy Parity Check

**File**: `tools/parity_check.py`

```bash
python3 tools/parity_check.py
```

Verifies SHA-256 integrity of protected legacy files:
- `drone/start.sh`
- `drone/debug_start.sh`
- `drone/field_test.sh`

Hashes are pinned in `docs/testing/LEGACY_BASELINE.md`.

### 2.7 Gate 7: Third-Party Provenance

**File**: `tools/check_third_party.py`

```bash
python3 tools/check_third_party.py --strict
```

Validates:
- Pinned git revisions (no floating refs)
- License file hashes match cached copies
- Dataset manifest: `policy.model_weight_downloads: "prohibited"`
- No forbidden copy markers under `ed_*` packages

### 2.8 Gate 8: Rollback Verification

**File**: `tools/test_rollback.py`

```bash
pytest tools/test_rollback.py -v
```

**Checks**:
1. **Legacy imports**: 7 core modules importable (`lx_protocol`, `path_plan`, `state_machine`, `mcu_serial`, `config`, `localization`, `vision`)
2. **Legacy command builders**: All 6 V7 commands produce valid checksummed frames
3. **Legacy path and state**: Grid has 28 blocks, path covers all, `FlightState` has 10+ states
4. **Legacy test discovery**: pytest discovers legacy test suite
5. **Mutual exclusion**: POSIX `fcntl.lockf(LOCK_EX|LOCK_NB)` proves two processes cannot claim the same endpoint
6. **Serial exclusive open**: `TIOCEXCL` ioctl available on kernel

---

## 3. Operator Flow

### 3.1 Build

```bash
# Full workspace build
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && \
  rosdep install --from-paths ros2_ws/src --ignore-src -r -y && \
  colcon build --symlink-install'
```

### 3.2 Test

```bash
# Full test suite
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && \
  colcon test --event-handlers console_direct+ && \
  colcon test-result --all --verbose'
```

### 3.3 Launch (Offline/Simulated)

```bash
# Source the workspace
source ros2_ws/install/setup.bash

# Launch with offline profile (no hardware required)
ros2 launch ed_uav_bringup bringup.launch.py \
  profile:=offline \
  calibration_file:=ros2_ws/src/ed_uav_description/config/example_uncalibrated.yaml
```

### 3.4 Launch (Camera Only — Simulated)

```bash
# Launch cameras with fake devices
ros2 launch ed_uav_camera dual_uvc.launch.py \
  camera_plan:=ros2_ws/src/ed_uav_camera/config/fake_dual_camera_plan.json \
  use_fake_devices:=true

# Launch bringup with camera_only profile
ros2 launch ed_uav_bringup bringup.launch.py \
  profile:=camera_only \
  calibration_file:=path/to/calibrated.yaml
```

### 3.5 Launch (Verification Harness)

```bash
# Deterministic 60-second scenario
ros2 launch ed_uav_verification verification_harness.launch.py \
  seed:=7 duration_seconds:=60 rate_hz:=20
```

### 3.6 One-Click Offline Integration

Run these commands from the repository root. Each script creates a timestamped
directory below `.omo/evidence/offline-integration/scripts/` and writes a
stage-specific `SUCCESS` marker there. A failed run writes `FAILED` with the
exit code. Retain the directories as debugging evidence.

| Stage | Exact command | Pass marker and evidence | Debugging phase |
|---|---|---|---|
| Static contract surface | `bash tools/run_offline_static.sh` | `STATIC_OFFLINE_GREEN` in `SUCCESS`; focused pytest, launch-surface, replay-profile, interface-contract, parity, and runner logs | First check for environment, launch, interface, and legacy-parity regressions before starting processes |
| Live simulation | `bash tools/run_offline_sim.sh` | `SIM_OFFLINE_GREEN` in `SUCCESS`; build, simulation, colcon, and runner logs | Check the live offline graph and deterministic synthetic sensor flow |
| WSLg visualization | `bash tools/run_offline_rviz.sh` | `RVIZ_OFFLINE_GREEN` in `SUCCESS`; packaged config, RViz, build, and runner logs | Check WSLg display startup, visualization topics, and RViz process lifetime |
| FCU bridge dry run | `bash tools/run_offline_fcu_dry_run.sh` | `FCU_DRY_RUN_GREEN` in `SUCCESS`; FCU, build, and runner logs | Check telemetry, bridge framing, PTY cleanup, and shutdown |
| Event replay | `bash tools/run_offline_full_replay.sh` | `FULL_REPLAY_GREEN` in `SUCCESS`; event creation, bag info, replay, test, build, and runner logs | Check event artifact creation, event-only rosbag shape, and replay lifecycle |

The live deterministic simulation is wall-time only. It does not publish
`/clock`, so `use_sim_time=true` is rejected for this surface. The one-click
simulation and visualization commands use `use_sim_time:=false`.

The RViz stage uses WSLg through `HUMBLE_GUI=1`. It displays synthetic,
visualization-only robot geometry, TF, lidar points, and two images. Odometry
displays are intentionally absent until an authorized TF owner exists.

The rosbag stage is event-only. Its approved topic is `/verification/events`;
it is not a sensor replay and it is not a flight replay. The FCU dry-run uses a
fake PTY with the real bridge. It does not open `/dev/ttyUSB*` and makes no HIL,
hardware, or flight claim.
Standalone CLI command tests remain offline-only and do not authorize FCU
hardware commands.

The current offline integration receipt is recorded under
`.omo/evidence/offline-integration/`. This receipt supplements, and does not
replace, the original milestone results in `docs/testing/TODAY_MILESTONE.md`.
The stored stage evidence includes `wall-time/`, `rviz/`, `rviz-visual/`,
`rosbag/`, `fcu-final/`, and timestamped runs under `scripts/`.

### 3.7 FCU Bridge (Standalone)

```bash
# Requires physical FCU on /dev/ttyUSB0
source ros2_ws/install/setup.bash
ros2 run ed_uav_fcu_bridge ed_uav_fcu_bridge \
  --ros-args -p serial_port:=/dev/ttyUSB0 -p baudrate:=500000
```

**Prerequisites**:
- FCU connected via USB-TTL at 500000 baud
- Cooperative serial ownership preflight or broker confirms that no other process
  owns `/dev/ttyUSB0`
- `TIOCEXCL` and the canonical device-number lock are enabled for cooperating
  opens; an already-open legacy file descriptor can remain writable after
  `TIOCEXCL`, so the preflight remains required
- User in `dialout` group (or run as root)

### 3.8 Flight Command Authority

The `/fcu/flight_command` action is disabled by default. Explicit enablement
requires all of the following:

- `ROS_SECURITY_ENABLE=true`
- `ROS_SECURITY_STRATEGY=Enforce`
- `ROS_SECURITY_KEYSTORE` points to the configured keystore
- Signed permissions are generated from the installed template at
  `share/ed_uav_bringup/security/fcu_command.policy.xml`

The bridge enclave has `execute` permission and the mission executor has
`call` permission. Default deny is enforced by the ROS 2 middleware policy. The
policy template contains no credentials. The offline PTY dry run remains
credential-free and keeps flight commands disabled.

---

## 4. Profiles

Defined in `bringup.launch.py`:

| Profile | Calibration Gate | Hardware Required | Use Case |
|---|---|---|---|
| `offline` | Relaxed (any status) | None | CI, development |
| `camera_only` | Relaxed | Cameras only | Camera testing |
| `lidar` | Relaxed | LiDAR only | LiDAR testing |
| `competition` | **Strict** (`CALIBRATED`) | All sensors + FCU | Competition |

### Competition Gate Requirements

- `calibration_status == "CALIBRATED"`
- All `sensor_serials` match actual device serials
- `calibration_hash` matches recomputed hash
- All transforms measured (no zero values except `fcu_link`)

---

## 5. Rollback Procedure

### 5.1 What "Rollback" Means

The project maintains two parallel codebases:

| Codebase | Entry Point | Purpose |
|---|---|---|
| `drone/` (legacy) | `drone/main.py --profile competition` | Python-only, direct serial, no ROS |
| `ros2_ws/` (ROS 2) | `ros2 launch ed_uav_bringup bringup.launch.py` | ROS 2 graph, typed interfaces |

**Rollback** = reverting from the ROS 2 stack to the legacy `drone/` stack.

### 5.2 Rollback Steps

```bash
# 1. Stop ROS 2 processes
pkill -f "ros2 launch"
pkill -f "ed_uav_fcu_bridge"

# 2. Verify legacy code is intact
python3 tools/parity_check.py
# Expected: all hashes match

# 3. Verify legacy imports work
python3 -c "from drone import lx_protocol, path_plan, state_machine, mcu_serial"
# Expected: no import errors

# 4. Start legacy system
cd drone
python main.py --profile competition --serial-port /dev/ttyUSB0
```

### 5.3 Serial Ownership Boundary

The `ExclusiveSerialPort` in `ed_uav_fcu_bridge/serial_port.py` uses a
canonical character-device major/minor identity lock, `TIOCEXCL`, and
`flock(LOCK_EX|LOCK_NB)`. Together these stop cooperating new opens from
claiming the same endpoint.

These mechanisms cannot evict a descriptor that was opened before the boundary
was established. An external owner preflight or serial broker is required
before connecting hardware, especially when a legacy process may already hold
the FCU.

### 5.4 Protected Files

| File | Purpose | Integrity |
|---|---|---|
| `drone/start.sh` | Legacy production launcher | SHA-256 pinned in `docs/testing/LEGACY_BASELINE.md` |
| `drone/debug_start.sh` | Legacy debug launcher | SHA-256 pinned |
| `drone/field_test.sh` | Legacy field test launcher | SHA-256 pinned |

Any modification detected by `tools/parity_check.py` triggers a RED gate.

---

## 6. Docker/Container Deployment

### 6.1 Image

```bash
# Build the Humble toolchain image
docker build -t ed-humble-toolchain -f docker/Dockerfile.humble .
```

Base: `ros:humble-ros-base-jammy` (digest-pinned, linux/amd64).
Includes: vision-msgs, cv-bridge, pytest 8.x, ruff, basedpyright, pydantic 2.x.

### 6.2 Compose

```bash
docker compose -f docker/compose.humble.yml up -d
docker compose -f docker/compose.humble.yml exec humble bash
```

### 6.3 Runner Dispatch

`tools/run_humble.sh` automatically selects:
- **Native**: Ubuntu 22.04 with `/opt/ros/humble` installed
- **Container**: All other hosts (WSL, macOS, Ubuntu 24.04)

Override with `HUMBLE_CONTAINER_RUNTIME=podman` for Podman.

---

## 7. Future Gates (Not Yet Implemented)

### 7.1 Target Gate

Real-device verification without flight:
- Camera capture at all target resolutions
- LiDAR scan with Mid-360
- FCU serial handshake (arm/disarm/mode)
- Sensor timestamp synchronization check

### 7.2 HIL Gate

Hardware-in-loop with simulated dynamics:
- Full mission execution against simulated field
- Fault injection with real sensor feeds
- Localization source switching under load
- Safety supervisor hover→land verification

### 7.3 Flight Gate

Real flight with progressive autonomy:
1. Manual flight with ROS logging only
2. Assisted flight (ROS provides suggestions, pilot overrides)
3. Semi-autonomous (ROS controls, pilot can override)
4. Full autonomous (pilot monitors only)

---

## 8. Acceptance Criteria Summary

| Gate | Criterion | Tool |
|---|---|---|
| CI build | `colcon build` + `colcon test` pass | `ros2-ci.yml` |
| Launch surface | `BRINGUP: GREEN` | `verify_launch_surface.py` |
| Calibration | Hash match, serials bound, competition: `CALIBRATED` | `validate_calibration.py` |
| Contract | All interfaces match manifest | `check_contract.py` |
| Scenario | `SCENARIO: GREEN` with fault matrix pass | `ed-uav-verify` |
| Legacy parity | All SHA-256 hashes match | `parity_check.py` |
| Rollback | Legacy imports + mutual exclusion verified | `test_rollback.py` |
| Provenance | Pinned revisions, license hashes, no copy markers | `check_third_party.py --strict` |
