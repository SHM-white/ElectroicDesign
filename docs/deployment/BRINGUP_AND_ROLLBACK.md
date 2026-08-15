# 启动与回滚运行手册

> 来源：`ros2_ws/src/ed_uav_bringup/`、`ros2_ws/src/ed_uav_verification/`、
> `ros2_ws/src/ed_uav_fcu_bridge/`, `tools/run_humble.sh`,
> `.github/workflows/ros2-ci.yml`, `tools/test_rollback.py`,
> `tools/parity_check.py`.

---

## 1. 部署门控概述

系统定义三层门控。目前只有**离线门控**已完整实现。

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

## 2. 离线门控，已实现

### 2.1 门控 1：CI 构建与测试

**文件**：`.github/workflows/ros2-ci.yml`

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

**运行器选择**（`tools/run_humble.sh`）：
- 安装 `/opt/ros/humble` 的 Ubuntu 22.04 → 原生执行
- 其他主机 → Docker/Podman 容器（摘要固定的 `ros:humble-ros-base-jammy`）
- 验证固定的基础镜像标签 `io.ed.humble.base-ref`
- 容器操作超时 900 s
- 互斥锁阻止并发运行

**验收**：所有 `colcon test` 通过，且 `colcon test-result` 显示 0 个失败。

### 2.2 门控 2：启动面验证

**文件**：`ros2_ws/src/ed_uav_bringup/tools/verify_launch_surface.py`

静态 AST 分析（不需要 ROS 运行时）：

```bash
python3 ros2_ws/src/ed_uav_bringup/tools/verify_launch_surface.py \
  ros2_ws/src/ed_uav_bringup/launch/bringup.launch.py
```

**检查项**：
1. 准确定义 7 个 P06 启动参数
2. 准确存在 4 个 P06 配置（`offline`、`camera_only`、`lidar`、`competition`）
3. `Node()` 构造**之前**调用 `validate_for_profile()`
4. 不得存在禁止的 TF 权威（`static_transform_publisher`、`map → odom`、
    `odom → base_link` 作为静态关节）

**输出**：`BRINGUP: GREEN` 或 `BRINGUP: RED: <reason>`

### 2.3 门控 3：标定验证

**文件**：`ros2_ws/src/ed_uav_description/tools/validate_calibration.py`

```bash
python3 ros2_ws/src/ed_uav_description/tools/validate_calibration.py \
  ros2_ws/src/ed_uav_description/config/example_uncalibrated.yaml
```

**检查项**：
- Schema version 有效
- 标定哈希与重新计算的哈希匹配
- 所有必需变换均存在
- 竞赛配置：状态必须为 `CALIBRATED`，序列号不得为 `UNSET`/`SYNTHETIC-*`

### 2.4 门控 4：ROS 2 契约验证

**文件**：`ros2_ws/src/ed_uav_interfaces/tools/check_contract.py`

```bash
./.venv/bin/python ros2_ws/src/ed_uav_interfaces/tools/check_contract.py \
  ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json
```

验证所有批准的话题、服务、动作、TF 边、QoS 配置、新鲜度截止时间、生命周期顺序和枚举值。

### 2.5 门控 5：确定性场景验证

**文件**：`ros2_ws/src/ed_uav_verification/ed_uav_verification/cli.py`

```bash
# Run 60-second deterministic scenario
ed-uav-verify --output scenario_events.json
```

**功能**：
- 虚拟单调时钟（不依赖真实时间）
- 8 路 20 Hz 合成传感器流
- 6 种故障注入模式：DROP、FREEZE、CORRUPTION、LATENCY、TIME_REGRESSION、PROCESS_DEATH
- 原子写入事件 artifact
- 故障矩阵断言：每个故障都必须完成激活 + 降级 + 恢复 + 流恢复

**输出**：`SCENARIO: GREEN {sha256, duration, ticks}`

### 2.6 门控 6：旧版一致性检查

**文件**：`tools/parity_check.py`

```bash
python3 tools/parity_check.py
```

验证受保护旧版文件的 SHA-256 完整性：
- `drone/start.sh`
- `drone/debug_start.sh`
- `drone/field_test.sh`

哈希固定在 `docs/testing/LEGACY_BASELINE.md` 中。

### 2.7 门控 7：第三方来源验证

**文件**：`tools/check_third_party.py`

```bash
python3 tools/check_third_party.py --strict
```

验证：
- Git 修订版本已固定（无浮动引用）
- 许可证文件哈希与缓存副本匹配
- 数据集清单：`policy.model_weight_downloads: "prohibited"`
- `ed_*` 包下不存在禁止的复制标记

### 2.8 门控 8：回滚验证

**文件**：`tools/test_rollback.py`

```bash
pytest tools/test_rollback.py -v
```

**检查项**：
1. **旧版导入**：7 个核心模块可导入（`lx_protocol`、`path_plan`、`state_machine`、`mcu_serial`、`config`、`localization`、`vision`）
2. **旧版命令构建器**：全部 6 个 V7 命令生成有效校验和帧
3. **旧版路径和状态**：网格有 28 个区块，路径覆盖全部区块，`FlightState` 有 10+ 个状态
4. **旧版测试发现**：pytest 能发现旧版测试套件
5. **互斥**：POSIX `fcntl.lockf(LOCK_EX|LOCK_NB)` 证明两个进程不能声明同一端点
6. **串口独占打开**：内核提供 `TIOCEXCL` ioctl

---

## 3. 操作员流程

### 3.1 构建

```bash
# Full workspace build
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && \
  rosdep install --from-paths ros2_ws/src --ignore-src -r -y && \
  colcon build --symlink-install'
```

### 3.2 测试

```bash
# Full test suite
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && \
  colcon test --event-handlers console_direct+ && \
  colcon test-result --all --verbose'
```

### 3.3 启动（离线/模拟）

```bash
# Source the workspace
source ros2_ws/install/setup.bash

# Launch with offline profile (no hardware required)
ros2 launch ed_uav_bringup bringup.launch.py \
  profile:=offline \
  calibration_file:=ros2_ws/src/ed_uav_description/config/example_uncalibrated.yaml
```

### 3.4 启动（仅相机，模拟）

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

### 3.5 启动（验证工具）

```bash
# Deterministic 60-second scenario
ros2 launch ed_uav_verification verification_harness.launch.py \
  seed:=7 duration_seconds:=60 rate_hz:=20
```

### 3.6 一键离线集成

从仓库根目录运行这些命令。每个脚本都会在 `.omo/evidence/offline-integration/scripts/`
下创建时间戳目录，并在其中写入阶段专用的 `SUCCESS` 标记。失败运行会写入带退出码的
`FAILED`。保留这些目录作为调试证据。

| 阶段 | 精确命令 | 通过标记和证据 | 调试阶段 |
|---|---|---|---|
| 静态契约面 | `bash tools/run_offline_static.sh` | `SUCCESS` 中的 `STATIC_OFFLINE_GREEN`；聚焦 pytest、启动面、回放配置、接口契约、一致性和运行器日志 | 启动进程前首先检查环境、启动、接口和旧版一致性回归 |
| 实时模拟 | `bash tools/run_offline_sim.sh` | `SUCCESS` 中的 `SIM_OFFLINE_GREEN`；构建、模拟、colcon 和运行器日志 | 检查实时离线图和确定性合成传感器流 |
| WSLg 可视化 | `bash tools/run_offline_rviz.sh` | `SUCCESS` 中的 `RVIZ_OFFLINE_GREEN`；打包配置、RViz、构建和运行器日志 | 检查 WSLg 显示启动、可视化话题和 RViz 进程生命周期 |
| FCU 桥接空运行 | `bash tools/run_offline_fcu_dry_run.sh` | `SUCCESS` 中的 `FCU_DRY_RUN_GREEN`；FCU、构建和运行器日志 | 检查遥测、桥接帧、PTY 清理和关闭 |
| 事件回放 | `bash tools/run_offline_full_replay.sh` | `SUCCESS` 中的 `FULL_REPLAY_GREEN`；事件创建、包信息、回放、测试、构建和运行器日志 | 检查事件 artifact 创建、仅事件 rosbag 形状和回放生命周期 |

实时确定性模拟仅使用墙钟时间。它不发布 `/clock`，因此该测试路径拒绝
`use_sim_time=true`。一键模拟和可视化命令使用 `use_sim_time:=false`。

RViz 阶段通过 `HUMBLE_GUI=1` 使用 WSLg。它显示仅用于可视化的合成机器人几何体、TF、
激光雷达点和两幅图像。在存在授权 TF 所有者前，故意不显示里程计。

rosbag 阶段仅包含事件。其批准话题为 `/verification/events`；它不是传感器回放，也不是
飞行回放。FCU 空运行使用真实桥接和伪 PTY，不会打开 `/dev/ttyUSB*`，也不作出 HIL、
硬件或飞行声明。独立 CLI 命令测试仍仅限离线，不授权 FCU 硬件命令。

当前离线集成收据记录在 `.omo/evidence/offline-integration/` 下。该收据补充而不替代
`docs/testing/TODAY_MILESTONE.md` 中的原始里程碑结果。存储的阶段证据包括
`wall-time/`、`rviz/`、`rviz-visual/`、`rosbag/`、`fcu-final/` 以及 `scripts/` 下的
时间戳运行目录。

### 3.7 FCU 桥接（独立运行）

```bash
# Requires physical FCU on /dev/ttyUSB0
source ros2_ws/install/setup.bash
ros2 run ed_uav_fcu_bridge ed_uav_fcu_bridge \
  --ros-args -p serial_port:=/dev/ttyUSB0 -p baudrate:=500000
```

**前置条件**：
- FCU 通过 USB-TTL 以 500000 波特率连接
- 协作式串口所有权预检或代理确认没有其他进程持有 `/dev/ttyUSB0`
- 协作式打开启用 `TIOCEXCL` 和规范设备号锁；已经打开的旧版文件描述符在
  `TIOCEXCL` 后仍可能可写，因此仍必须执行预检
- 用户属于 `dialout` 组（或以 root 运行）

### 3.8 飞行命令启用与紧急锁浆

`/fcu/flight_command` action 默认禁用，实飞 launch 通过显式参数
`enable_flight_commands:=true` 和 `enable_realtime_control:=true` 启用。仓库内不再读取
SROS2 环境变量或注入 enclave；网络访问控制由部署边界负责。唯一的遥控安全锁是
AUX1 `1800..2000 us` 一键紧急锁浆，触发后锁存并抢占当前飞行命令。

---

## 4. 配置档

定义于 `bringup.launch.py`：

| 配置 | 标定门控 | 所需硬件 | 用途 |
|---|---|---|---|
| `offline` | 宽松（任意状态） | 无 | CI、开发 |
| `camera_only` | 宽松 | 仅相机 | 相机测试 |
| `lidar` | 宽松 | 仅激光雷达 | 激光雷达测试 |
| `competition` | **严格**（`CALIBRATED`） | 所有传感器 + FCU | 竞赛 |

### 竞赛门控要求

- `calibration_status == "CALIBRATED"`
- 所有 `sensor_serials` 与实际设备序列号匹配
- `calibration_hash` 与重新计算的哈希匹配
- 所有变换均已测量（除 `fcu_link` 外不得为零值）

---

## 5. 回滚流程

### 5.1 “回滚”的含义

项目维护两套并行代码库：

| 代码库 | 入口 | 用途 |
|---|---|---|
| `drone/`（旧版） | `drone/main.py --profile competition` | 仅 Python、直接串口、无 ROS |
| `ros2_ws/`（ROS 2） | `ros2 launch ed_uav_bringup bringup.launch.py` | ROS 2 图、类型化接口 |

**回滚** = 从 ROS 2 栈切换回旧版 `drone/` 栈。

### 5.2 回滚步骤

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

### 5.3 串口所有权边界

`ed_uav_fcu_bridge/serial_port.py` 中的 `ExclusiveSerialPort` 使用规范字符设备主/次设备
号身份锁、`TIOCEXCL` 和 `flock(LOCK_EX|LOCK_NB)`。这些机制共同阻止协作式的新打开操作
声明同一端点。

这些机制无法驱逐在边界建立前已经打开的描述符。连接硬件前必须执行外部所有者预检
或使用串口代理，尤其是在旧版进程可能已经持有 FCU 时。

### 5.4 受保护文件

| 文件 | 用途 | 完整性 |
|---|---|---|
| `drone/start.sh` | 旧版生产启动器 | SHA-256 固定在 `docs/testing/LEGACY_BASELINE.md` |
| `drone/debug_start.sh` | 旧版调试启动器 | SHA-256 已固定 |
| `drone/field_test.sh` | 旧版现场测试启动器 | SHA-256 已固定 |

`tools/parity_check.py` 检测到任何修改都会触发 RED 门控。

---

## 6. Docker/容器部署

### 6.1 镜像

```bash
# Build the Humble toolchain image
docker build -t ed-humble-toolchain -f docker/Dockerfile.humble .
```

基础镜像：`ros:humble-ros-base-jammy`（摘要固定，linux/amd64）。
包含：vision-msgs、cv-bridge、pytest 8.x、ruff、basedpyright、pydantic 2.x。

### 6.2 Compose

```bash
docker compose -f docker/compose.humble.yml up -d
docker compose -f docker/compose.humble.yml exec humble bash
```

### 6.3 运行器分派

`tools/run_humble.sh` 自动选择：
- **原生**：已安装 `/opt/ros/humble` 的 Ubuntu 22.04
- **容器**：其他所有主机（WSL、macOS、Ubuntu 24.04）

使用 `HUMBLE_CONTAINER_RUNTIME=podman` 可改用 Podman。

---

## 7. 后续门控（尚未实现）

### 7.1 目标门控

不进行飞行的真实设备验证：
- 在所有目标分辨率采集相机图像
- 使用 Mid-360 扫描激光雷达
- FCU 串口握手（解锁/上锁/模式）
- 检查传感器时间戳同步

### 7.2 HIL 门控

使用模拟动力学的硬件在环：
- 在模拟场地执行完整任务
- 对真实传感器流注入故障
- 在负载下切换定位源
- 验证安全监管器 hover→land

### 7.3 飞行门控

逐步增加自主程度的真实飞行：
1. 仅记录 ROS 日志的手动飞行
2. 辅助飞行（ROS 提供建议，飞行员可覆盖）
3. 半自主飞行（ROS 控制，飞行员可覆盖）
4. 全自主飞行（飞行员仅监视）

---

## 8. 验收标准摘要

| 门控 | 标准 | 工具 |
|---|---|---|
| CI 构建 | `colcon build` + `colcon test` 通过 | `ros2-ci.yml` |
| 启动面 | `BRINGUP: GREEN` | `verify_launch_surface.py` |
| 标定 | 哈希匹配、序列号绑定，竞赛状态为 `CALIBRATED` | `validate_calibration.py` |
| 契约 | 所有接口与清单匹配 | `check_contract.py` |
| 场景 | `SCENARIO: GREEN` 且故障矩阵通过 | `ed-uav-verify` |
| 旧版一致性 | 所有 SHA-256 哈希匹配 | `parity_check.py` |
| 回滚 | 旧版导入和互斥已验证 | `test_rollback.py` |
| 来源 | 修订版本固定、许可证哈希匹配、无复制标记 | `check_third_party.py --strict` |
