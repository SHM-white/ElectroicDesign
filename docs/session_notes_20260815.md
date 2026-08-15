# 项目维护文档 - 2026-08-15 会话记录

## 一、本次会话解决的难点问题

### 1. HMAC 验证完全删除

**问题描述**：项目要求 HMAC 密钥文件存在才能启动，但比赛中不需要安全验证。

**涉及文件**（8个）：
- `ros2_ws/src/ed_uav_vehicle_bridge/ed_uav_vehicle_bridge/protocol.py`
- `ros2_ws/src/ed_uav_vehicle_bridge/ed_uav_vehicle_bridge/config.py`
- `tools/run_competition.sh`
- `tools/diagnostics/vehicle_comm_diagnostic.py`
- `tools/sim_competition.py`
- `ros2_ws/src/ed_uav_vehicle_bridge/test/test_protocol.py`
- `ros2_ws/src/ed_uav_vehicle_bridge/test/test_config.py`
- `tools/diagnostics/test_protocol.py`

**关键修改**：
```python
# protocol.py - encode_datagram()
# 原来：计算HMAC标签
tag = hmac.new(key, authenticated_body, hashlib.sha256).digest()[:HMAC_TAG_BYTES]
# 修改后：返回全零标签
return authenticated_body + b'\x00' * HMAC_TAG_BYTES

# protocol.py - decode_datagram()
# 原来：验证HMAC标签
if not hmac.compare_digest(data[-HMAC_TAG_BYTES:], expected_tag):
    raise ProtocolError(ProtocolErrorCode.BAD_HMAC, "authentication tag mismatch")
# 修改后：跳过验证

# config.py - load_hmac_key_file()
# 原来：文件不存在时抛异常
# 修改后：返回默认零密钥 b'\x00' * 32
```

**踩坑点**：
- 测试中的 `GOLDEN_HEX` 值需要重新计算（HMAC标签从实际值变为全零）
- `run_competition.sh` 第176行的检查需要删除
- 多个测试文件需要同步更新

---

### 2. WSL Docker 集成问题

**问题描述**：WSL 中运行 `docker` 命报错 `Input/output error`，`run_humble.sh` 报错 `HUMBLE_GUI must be 1 when GUI forwarding is enabled`。

**根因分析**：
1. Docker Desktop 的 WSL 集成未启用
2. `gui_args()` 函数只接受 `HUMBLE_GUI=1` 或未设置，但 `run_competition.sh` 在 `display=false` 时设置 `HUMBLE_GUI=0`

**解决方案**：

#### 步骤1：启用 Docker Desktop WSL 集成
1. 打开 Docker Desktop
2. 设置 → Resources → WSL Integration
3. 启用 Ubuntu-22.04 集成
4. Apply & Restart

#### 步骤2：修复 gui_args() 函数
```bash
# tools/run_humble_support.sh 第131行
# 原来：
[[ -z "${HUMBLE_GUI:-}" ]] || die "HUMBLE_GUI must be 1 when GUI forwarding is enabled"
# 修改后：
[[ -z "${HUMBLE_GUI:-}" || "${HUMBLE_GUI:-}" == 0 ]] || die "HUMBLE_GUI must be 1 when GUI forwarding is enabled"
```

#### 步骤3：修复 ROS2 setup.bash 的 unbound variable 问题
```bash
# tools/run_competition.sh run_ros() 函数
# 原来：
set -euo pipefail
source /opt/ros/humble/setup.bash
# 修改后（native_humble 和 Docker 两个分支都需要）：
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u
```

---

### 3. 测试文件 GOLDEN_HEX 值更新

**问题描述**：修改 `protocol.py` 后，`test_wire_golden_vector_is_stable` 测试失败。

**原因**：HMAC 标签从实际计算值变为全零，导致整个数据包的十六进制表示改变。

**解决方法**：
```bash
# 运行以下命令获取正确的 GOLDEN_HEX
docker run --rm -v "${PWD}:/workspace" -w /workspace ed-humble-toolchain:jammy-humble \
  python3 -c "
from ed_uav_vehicle_bridge.protocol import encode_datagram
from ed_uav_vehicle_bridge.models import *
frame = OutboundFrame(
    message_type=MessageType.CAR_TELEMETRY,
    sender_id=SenderId(0x43415231),
    boot_id=BootId(0x10203040),
    sequence=Sequence(0xFFFFFFFE),
    source_millis=SourceMillis(0x01020304),
    payload=bytes.fromhex('010102070003002efbffff410183ff0000')
)
encoded = encode_datagram(frame, bytes(range(32)))
print(encoded.hex())
"
```

**正确的值**：
```python
GOLDEN_HEX = "5444010211003152414340302010feffffff04030201010102070003002efbffff410183ff000050ee0000000000000000"
```

---

### 4. 仿真启动信号调试 (2026-08-16)

**问题描述**：`sim_car_controller` 的 `_on_start` 回调收到 `/simulation/competition_start` 消息，但 mission_executor 的 reducer 仍然显示 `started=False`。

**根因**：`start_event` 只在 `start_age_s <= 1.0` 时为 True（1秒窗口），但 reducer 在窗口期内未及时处理。

**调试日志**：
```
[CAR-START] received competition_start: data=True already_started=False
[CAR-TEL] seq=6 started=True start_event=True age=0.10s
...
[REDUCER] vehicle not started yet, waiting...
[REDUCER] vehicle started! → TAKEOFF
```

**结论**：信号最终能到达，但有延迟。reducer 在 waiting_start 阶段等待了几秒后才看到 started=True。

---

## 二、项目设计架构

### 1. 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                      用户层                                  │
│  run_competition.sh / run_gazebo_sim.sh / ...               │
└─────────────────────────┬───────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────┐
│                    容器层                                     │
│  run_humble.sh → Docker (ed-humble-toolchain:jammy-humble)   │
└─────────────────────────┬───────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────┐
│                    Launch 层                                  │
│  sim.launch.py / full_competition.launch.py / ...            │
└─────────────────────────┬───────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────┐
│                    节点层                                     │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐        │
│  │ Gazebo   │ │ FAST-LIO │ │ Mission  │ │ Vehicle  │        │
│  │ Sim      │ │ Lidar    │ │ Executor │ │ Bridge   │        │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘        │
└─────────────────────────────────────────────────────────────┘
```

### 2. 通信架构

```
HMI (192.168.20.3:42002)
    │ TASK_SELECTION / HEARTBEAT
    ▼
NUC (192.168.20.1:42000) ── vehicle_bridge
    │ MISSION_STATUS / HEARTBEAT
    ▼
CAR (192.168.20.2:42001)
    │ CAR_TELEMETRY
    ▼
mission_executor
    │ /fcu/flight_command
    ▼
fcu_bridge ──串口──▶ 飞控
```

### 3. 仿真模式架构

```
sim.launch.py
├── Gazebo (GUI/Headless)
├── ros_gz_bridge (参数桥)
├── bringup.launch.py
├── localization_simulation.launch.py
├── fast_lio_simulation.launch.py
├── planner_only.launch.py
├── target_observation.launch.py
├── mission_executor.launch.py
├── sim_fcu (仿真飞控)
├── sim_car_controller (小车控制器)
└── sim_mission_starter (自动选题器)
```

### 4. 关键配置文件

| 文件 | 用途 |
|------|------|
| `ros2_ws/src/ed_uav_mission/config/missions/d_arena_competition.yaml` | 竞赛任务配置 |
| `ros2_ws/src/ed_uav_localization/config/fields/d_arena_2026.yaml` | 场地配置 (5m×4m) |
| `ros2_ws/src/ed_uav_description/config/synthetic_calibrated.yaml` | 仿真标定文件 |
| `calibration_data/camera_runtime_plan.local.json` | 相机配置 |
| `config/hmac.key.hex` | HMAC密钥（已禁用验证） |

### 5. 启动脚本层次

```
run_competition.sh (统一入口)
├── --simulation → sim.launch.py
├── --real → full_competition.launch.py
└── --build → build_sim_packages.sh

run_gazebo_sim.sh → gazebo_simulation.launch.py
run_stability_test_sim.sh → sim.launch.py (stability config)
run_no_car_mode.sh → ros2 run (no_car_sim + mission_executor + vehicle_bridge)
run_offline_sim.sh → offline_integration.launch.py
```

---

## 三、常用命令

### 仿真启动
```bash
# 无GUI仿真（默认容器模式）
./tools/run_competition.sh --simulation --no-display

# 有GUI仿真
./tools/run_competition.sh --simulation --enable-display

# 指定任务
./tools/run_competition.sh --simulation --no-display --task 1

# Docker host 网络（便于 rqt 调试）
./tools/run_competition.sh --simulation --no-display --network-host

# 强制使用本机 ROS2（需先安装）
FORCE_NATIVE=1 ./tools/run_competition.sh --simulation --no-display
```

### 构建
```bash
# 完整构建
./tools/build_sim_packages.sh

# 单包构建
cd ros2_ws && colcon build --packages-select ed_uav_vehicle_bridge --symlink-install
```

### 测试
```bash
# 运行单元测试
colcon test --packages-select ed_uav_vehicle_bridge

# 运行Python测试
python3 -m pytest src/ed_uav_vehicle_bridge/test/test_protocol.py -v
```

### 硬件测试
```bash
# FCU串口测试
./tools/test_fcu_serial.sh

# 小车链路测试
./tools/test_car_link.sh
```

---

## 四、已知问题和限制

1. **Costmap 警告**：仿真启动时会显示 `Robot is out of bounds of the costmap!`，这是正常的（无人机初始位置在地图边界外）。

2. **Docker 镜像构建**：首次构建需要较长时间（下载依赖），后续使用缓存。

3. **WSL 路径问题**：Docker 无法直接访问 WSL 文件系统，需要使用 Windows 路径或启用 WSL 集成。

4. **HMAC 验证已禁用**：所有 HMAC 相关的验证已被跳过，通信不再安全验证。

---

## 五、文件修改记录

| 日期 | 文件 | 修改内容 |
|------|------|----------|
| 2026-08-15 | protocol.py | 跳过HMAC计算和验证 |
| 2026-08-15 | config.py | 文件不存在时返回默认密钥 |
| 2026-08-15 | run_competition.sh | 删除HMAC检查，修复set -u问题 |
| 2026-08-15 | run_humble_support.sh | 修复gui_args()支持HUMBLE_GUI=0 |
| 2026-08-15 | vehicle_comm_diagnostic.py | 文件不存在时返回默认密钥 |
| 2026-08-15 | sim_competition.py | 文件不存在时返回默认密钥 |
| 2026-08-15 | test_protocol.py | 更新GOLDEN_HEX，移除HMAC测试 |
| 2026-08-15 | test_config.py | 更新测试，移除密钥文件检查 |
| 2026-08-15 | test_protocol.py (diagnostics) | 使用默认密钥 |
| 2026-08-15 | camera_runtime_plan.local.json | 修复硬编码路径 |
| 2026-08-15 | test_fcu_serial.sh | 新增：FCU串口测试 |
| 2026-08-15 | test_car_link.sh | 新增：小车链路测试 |
| 2026-08-16 | run_competition.sh | 修复 native_humble 分支 set +u 问题 |
| 2026-08-16 | sim_car_controller.py | 添加 CAR-START/CAR-TEL 调试日志 |
| 2026-08-16 | sim_mission_starter.py | 添加 start 信号发布日志 |
| 2026-08-16 | run_humble.sh | 添加 HUMBLE_NETWORK 环境变量支持 |
| 2026-08-16 | run_competition.sh | 添加 --network-host 选项，FORCE_NATIVE 开关 |
