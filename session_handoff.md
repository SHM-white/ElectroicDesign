# Codex Session 转交文档 - ElectroicDesign 项目修复

> **最后更新**: 2026-08-16
> **注意**：本文档根据 codex session 日志整理，标注"已完成"的任务需要实际验证，因为 session 中途断开，不确定最终状态。

---

## 一、项目概述

- **仓库**: `https://github.com/SHM-white/ElectroicDesign.git`
- **工作目录**: `/home/shm-white/ElectroicDesign`
- **类型**: 无人机 + 小车协同仿真项目（Gazebo + ROS2），用于比赛 D 题
- **运行环境**: WSL2 Ubuntu 22.04 + Docker

---

## 二、项目核心问题诊断

### 2.1 最严重的问题

1. **真实任务被代码永久禁止**
   - `authority.py:72` 即使能力报告、HMAC、设备身份全部验证通过，最后仍无条件抛出 `programmable field capability remains disabled`
   - 任务执行器转成 `CAPABILITY_BLOCKED`（`d_task_capability.py:35`）
   - 非仿真任务按现有代码不可能通过起飞前检查

2. **仿真模式没有真正组合起来**
   - `--simulation` 只设置 `simulation_only=true`，但 `full_competition.launch.py:203` 仍会启动真实 FCU
   - 任务执行器要求仿真 FCU 必须发布 `SOURCE_SIMULATOR`（`executor.py:127`）
   - 完整 launch 没有启动：fake FCU / FlightCommand server、仿真定位数据源、`/clock` 发布者

3. **默认启动必然碰到缺失文件**
   - `ros2_ws/src/ed_uav_lidar/config/fields/mid360_field_manifest.local.json`
   - `config/hmac.key.hex`
   - `keystore/`
   - `ros2_ws/install/setup.bash`

4. **启动参数与实际行为严重不一致**
   - `run_competition.sh:49` 默认显示 `flight=false`，但 launch 内硬编码 `"enable_flight_commands": True`（`full_competition.launch.py:216`）
   - `--no-h7` 只改变提示文字，launch 仍可能启动 H7
   - `--no-hotspot` 和 dry-run 跳过热点的条件被注释了（`run_competition.sh:210`）

5. **`--build` 构建不出完整链路**
   - `run_competition.sh:126` 的包列表漏了：`ed_uav_camera`、`ed_uav_perception`、`ed_uav_lidar`、`ed_uav_description`
   - `ed_uav_bringup/package.xml` 漏掉 mission、localization、lidar、FCU 等运行依赖

6. **本机配置已经串机**
   - `camera_runtime_plan.local.json:26` 硬编码了 `/home/xtyf/ed/calibration_data/...`，当前工程位于 `/home/shm-white/ElectroicDesign`

### 2.2 为什么测试全绿但跑不了

- 文档中的 202 个测试记录是 2026-07-23
- 当前完整启动脚本是在 2026-08-01 后修改的
- 现有测试没有覆盖 `run_competition.sh` 或 `full_competition.launch.py`
- 仓库承认硬件、HIL 和首次飞行仍是 `PENDING-HARDWARE`（`TODAY_MILESTONE.md:194`）

### 2.3 项目结构根因

同时存在三套互相漂移的入口：
- 已废弃的 `drone/`
- 离线/Gazebo 仿真脚本
- `run_competition.sh` + `full_competition.launch.py`
- `run_task3_flight_test.sh`
- `drone/main.py` 还指向不存在的 `tools/run_competition_sim.sh`

---

## 三、详细执行计划

### 阶段 1：建立失败基线
- 记录各种启动模式的退出码和失败点
- 建立启动模式矩阵：纯软件仿真 / 通信模拟 / 硬件 dry-run / 真实飞行
- **验证方法**：每种模式都有明确的预期行为和当前失败原因

### 阶段 2：启动合约测试
- `run_competition.sh` 测试：
  - `--no-hotspot` 不调用热点工具
  - dry-run 自动跳过热点
  - `--no-h7` 确实传入 launch
  - 没有 `--flight` 时飞控指令保持禁用
  - 后台模拟器启动失败时主脚本退出
  - 参数中包含空格时不会被错误拆分
- `full_competition.launch.py` 测试：
  - 所有脚本参数均被声明和消费
  - 仿真模式不启动真实 FCU、H7 和雷达
  - dry-run 不要求真实 keystore
  - 实飞模式必须要求密钥、keystore、标定和硬件 manifest
  - 默认状态不允许发送飞行指令

### 阶段 3：修复构建和包依赖
- 使用 `colcon build --packages-up-to ed_uav_bringup`
- 给 bringup 补齐：`ed_uav_fcu_bridge`、`ed_uav_lidar`、`ed_uav_localization`、`ed_uav_mission`
- 安装 lidar 的 `config/fields` 文件，不依赖源码绝对路径
- **验证方法**：干净工作区可以构建组合链全部自研包

### 阶段 4：恢复真正的纯仿真闭环
- 仿真模式必须组合：
  - fake FCU 状态，发布 `SOURCE_SIMULATOR`
  - fake `FlightCommand` action server
  - fake localization/odometry
  - fake CAR telemetry
  - 合成标定文件（`SYNTHETIC`）
  - 仿真场地配置
  - fake payload actuator
  - 统一时间模型（墙钟优先，`use_sim_time=false`）
- 仿真模式必须禁止：`/dev/ttyUSB*`、H7 串口、MID-360/FAST-LIO 真实驱动、SROS2 实飞权限

### 阶段 5：修复统一启动脚本
- 恢复 `WITH_HOTSPOT` 条件判断
- dry-run 默认跳过热点
- 为 launch 增加并传递：`with_fcu`、`with_h7`、`with_lidar`、`with_cameras`、`enable_flight_commands`
- 删除字符串拼接参数，使用 Bash 数组直接调用 `ros2 launch`
- `--flight` 必须是唯一能启用飞控指令的入口
- `--no-h7`、`--no-fcu` 必须影响实际节点，而非只改变终端提示
- 检查后台 `sim_competition.py` 是否存活；启动失败立即终止组合链
- 修正 Ctrl+C 和异常退出时的进程清理

### 阶段 6：部署输入预检
- 在创建任何 ROS 节点前验证：
  - HMAC 文件存在、为合法十六进制、至少 32 字节
  - SROS2 keystore 存在且结构完整
  - 标定状态：仿真 `SYNTHETIC` / 实机 `CALIBRATED`
  - 场地 profile 与 mission ID 匹配
  - 雷达 manifest 存在且字段完整
  - FAST-LIO launch 和 driver JSON 可读
  - 相机计划中的 `/dev/v4l/by-id` 路径存在
  - FCU/H7 设备身份不冲突
  - UDP 绑定地址确实属于本机
- 缺失项必须在启动前一次性列出

### 阶段 7：清理机器路径和失效入口
- 修复相机计划中 `/home/xtyf/ed/...` 的绝对路径
- 修正 `drone/main.py` 指向不存在的 `run_competition_sim.sh`
- 将 `drone/` 明确隔离为历史代码
- 明确唯一入口：
  - 软件仿真：`run_competition.sh --simulation`
  - 硬件检查：`run_competition.sh --dry-run`
  - 实飞：`run_task3_flight_test.sh` 或统一后的 `--flight`

### 阶段 8：真实飞行能力门专项处理（需要硬件证据）
- 当前 `require_programmable_capability()` 无条件拒绝真实能力
- 软件阶段先做到：
  - 非 `--flight` 永远不能发送真实指令
  - `--flight` 缺任何授权输入时，在启动前拒绝
  - 不再出现界面显示 `flight=false`、节点实际为 `True` 的情况
- 解除永久阻断需要单独审查和硬件证据，不纳入第一轮无硬件修复

---

## 四、新增需求执行计划

### 4.1 补全 Git 子模块
- 5 个子模块：`readonly`（小车代码）、`fast_lio_ros2`、`livox_sdk2`、`livox_ros_driver2`、`ultralytics`
- 执行命令：
  ```bash
  git submodule sync --recursive
  git submodule update --init --recursive
  git submodule status --recursive
  ```
- 要求：严格检出主仓库记录的 commit，不擅自更新到远端最新版本
- **验证方法**：`git submodule status --recursive` 不再出现前导 `-`

### 4.2 完整审计地面站—NUC—小车链路
- 预期拓扑：
  ```
  HMI 192.168.20.3:42002
      │ TASK_SELECTION
      ▼
  NUC 192.168.20.1:42000
      │ vehicle_bridge → ROS mission
      │ HEARTBEAT / MISSION_STATUS
      ▼
  CAR 192.168.20.2:42001
  ```
- 已发现问题：
  - `node.py:120` 的 CAR→HMI 转发使用了已离开作用域的 `provision`，会触发 `NameError`
  - `sim_competition.py` 和 `vehicle_bridge` 都试图占用 NUC 的 UDP 42000，存在端口/角色冲突

### 4.3 简化通信状态机
- **保留**：CRC/HMAC 包完整性、sender/boot/sequence 去重、数据长度/枚举/数值合法性、CAR 遥测新鲜度状态展示、重复选题的幂等 ACK、START 事件到任务启动的单一路径
- **删除或合并**：多层重复的 `BridgeAuthority` 启动门、Task3 专用身份/preset/revision 重复匹配、选题→预选→提交→FCU armed→AUX→CAR START 的多重授权链、仿真/no-car/immediate-start 三套分叉状态机
- 目标流程：
  - 正常：`HMI 选题 → ROS 确认 → 等待 CAR START → 执行任务`
  - 仿真：`HMI 选题 → ROS 确认 → 自动生成 START → 执行任务`

### 4.4 去除内部冗余安全锁
- **删除**：SROS2 keystore 检查、`ROS_SECURITY_*` 环境注入、`--enclave` 参数、SROS2 policy 对启动的依赖、capability report/provenance authority/capability HMAC、AUX5/AUX6/task3 capability/重复 armed 状态授权、`enable_flight_commands` 的多层配置门
- **保留**：串口帧校验、NaN/溢出/非法枚举/越界指令拒绝、FCU 串口唯一所有权、命令单位转换、通信断开状态上报

### 4.5 实现唯一的一键紧急锁桨
- 通道：`紧急键/AUX → emergency_motor_lock → FCU cmd_lock`
- 设计要求：
  - 独立于任务状态机、地面站选题和定位状态
  - 优先级高于移动、起飞、降落和载荷命令
  - 直接调用凌霄 `cmd_lock`
  - 幂等：重复触发不会产生错误状态
  - 锁桨后清空待发送飞行命令
  - 锁桨状态保持，不因任务重试自动恢复
  - 仿真中只停止虚拟电机，不触碰真实串口

### 4.6 修复组合仿真闭环
- fake FCU + FlightCommand + CAR/HMI + 合成标定 + 仿真定位 + `/clock` + fake payload
- 解决 `sim_competition.py` 与 bridge 的角色冲突：模拟器只模拟 CAR/HMI，不再冒充并占用 ROS 端口

### 4.7 修复二维激光雷达 Z 漂移
- 当前问题：Gazebo 雷达配置了 4 条垂直线，但本质上仍接近平面扫描（`model.sdf:42`），FAST-LIO 对 Z 的观测退化
- 修复方案：
  - FAST-LIO 只提供 `x/y/yaw`
  - Z 使用 Gazebo 高度源或向下测距仪
  - 仿真专用 planar constraint 节点在进入 `source_supervisor` 前融合二者
  - LIO 原始 Z 标记为不可观测并增大协方差
  - 输出保留连续的 `odom → base_link`，避免切换跳变
  - 真实 Mid-360 路径不启用 planar constraint
  - 将雷达垂直层数、normalizer 的固定 `height=4` 和 FAST-LIO `scan_line=4` 统一配置
- 测试场景：静止悬停、纯 XY 方形轨迹、固定高度圆形轨迹、主动升降后继续平移、LIO 重启和短暂丢点
- **验收**：水平飞行时融合 Z 不再随 LIO 漂移，并持续贴近仿真真值

### 4.8 Gazebo 场地重建
- 当前问题：`ed_uav_arena.sdf:18` 是约 16m×16m 的通用障碍场，与 D 题 5m×4m 场地不符
- 建立唯一场地规范：
  - 综合来源：D 题 PDF（4页）、`QA.txt` 官方答疑、`d_arena_2026.yaml`、`readonly` 小车子模块、`画板 1.svg` 靶面图
  - 固定坐标：`H = map 原点`、`X = 5m 长边`、`Y = 4m 短边`、`Z = ENU 向上`
- 扩展场地 profile `d_arena_2026.yaml`：
  - landmarks：H/A/B/C/D
  - 小车路线 polyline
  - 路线事件位置与触发半径
  - 起降区、抛投区和判分区域
  - 黑线宽度（约 2cm）
  - 小车平台尺寸和 AprilTag 位姿
  - 安全高度（2.5m）
- 重建 Gazebo world：
  - 删除：16m 围墙、随机方块、圆柱障碍物、marker gate、随机地面 AprilTag
  - 新增：5m×4m 白色场地底板、黑色循迹路线、A/B/C/D 黑点和文字、H 点标记、抛投靶面、半透明安全网、2.5m 高度边界
- 增加真实任务小车模型：
  - 小车底盘、0.6m×0.6m 起降平台、平台中心 AprilTag、抛投判定圈、碰撞体和接触传感器
  - 沿赛题路线移动：`START → B → D → A/COMPLETE`
- 两种比赛场景：
  - 抛投任务：H 起飞 → B 点伴飞 → D 点抛投 → 30cm/50cm 圈判定 → 返回 H
  - 动态起降任务：跟随平台 → AprilTag 对准 → 移动平台接触/稳定/锁定判定

### 4.9 AprilTag 配置
- Family：`tag36h11`
- ID：`0`
- 有效编码边长：15cm×15cm
- 含白边的 Gazebo 模型尺寸：24cm×24cm
- 固定目标 ID 为 `0`，贯通 detector、launch、Gazebo 和任务配置
- 现有 `apriltag_marker/model.sdf:3` 被写成 `static=true`，不能直接挂到动态小车上；需拆出可复用贴图 visual 作为小车平台 link 的子几何

### 4.10 FAST-LIO 定位验收（简化版）
- 仅需在单次运行中连续输出：`/fast_lio/odometry`、`/localization/lio/odom`、`/localization/odom`
- FAST-LIO 重启后允许重新建立 odom 原点
- 室内围墙和非对称墙面特征保留，用于改善 XY/yaw 可观测性
- Z 仍由仿真高度源或向下测距提供
- 验收重点：连续性、更新频率、平面漂移、Z 稳定、TF 唯一性

---

## 五、session 断开时的工作状态

根据 session 日志，断开时正在进行：

1. **Docker 镜像重建**：最后一次依赖层刷新（添加 `jsonschema`）
2. **SROS2/AUX 门控清除**：已完成，从实飞、空跑与自启动路径中清除
3. **Livox 驱动和 FAST-LIO 编译**：已成功链接
4. **AprilTag 观测节点**：已修复 TypeError，补回早期拒绝消息、相机回调兼容和确定性的 H7 PTY 清理
5. **待执行**：无界面 Gazebo + FAST-LIO 连续里程计实测

---

## 六、已发现并修复的具体问题

| 文件 | 问题 | 修复状态 |
|------|------|----------|
| `node.py:120` | CAR→HMI 转发 `NameError`（`provision` 离开作用域） | 需验证 |
| `sim_competition.py` | 与 `vehicle_bridge` 端口冲突（都占用 UDP 42000） | 需验证 |
| `run_competition.sh:126` | 包列表漏了 camera/perception/lidar/description | 需验证 |
| `run_competition.sh:210` | `--no-hotspot` 和 dry-run 跳过热点的条件被注释 | 需验证 |
| `full_competition.launch.py:203` | `--simulation` 仍启动真实 FCU | 需验证 |
| `full_competition.launch.py:216` | `enable_flight_commands` 硬编码为 True | 需验证 |
| `authority.py:72` | 无条件抛出 `programmable field capability remains disabled` | 需验证（需硬件） |
| `camera_runtime_plan.local.json:26` | 硬编码 `/home/xtyf/ed/...` 路径 | 需验证 |
| `drone/main.py:5` | 指向不存在的 `run_competition_sim.sh` | 需验证 |
| 飞控日志函数 | 会再次执行同一动作，造成命令重复下发 | 需验证 |
| 任务选择回调 | 访问了接口里不存在的字段 | 需验证 |
| mission/profile ID | 小车 START 后生成的 ID 与执行器加载的配置不一致 | 需验证 |

---

## 七、验证清单

以下任务需要在继续工作前验证：

- [ ] `git submodule status --recursive` 子模块状态
- [ ] Docker 镜像是否构建成功（包含 jsonschema）
- [ ] `colcon build` 是否能成功构建所有包
- [ ] `./tools/run_competition.sh --simulation --dry-run --no-hotspot --no-h7 --no-display` 是否能启动完整软件闭环
- [ ] FAST-LIO 是否能连续输出 `/localization/odom`
- [ ] Gazebo 场地是否已更新为 5m×4m
- [ ] AprilTag 检测是否固定为 tag36h11 ID 0
- [ ] 紧急锁桨是否独立于任务状态机
- [ ] 二维雷达 Z 漂移是否通过 planar constraint 解决

---

## 八、技术栈

- ROS2 Humble
- Gazebo 仿真
- FAST-LIO 算法（激光雷达定位）
- Livox 驱动（激光雷达）
- AprilTag 视觉定位
- Docker 容器化构建
- colcon 构建系统

---

## 九、2026-08-16 Session 完成的工作

### 9.1 Docker 强制模式

**问题**: WSL2 环境下 `/opt/ros/humble/setup.bash` 存在，`run_humble.sh` 的 `is_jammy` 检测会走本机路径，而不是 Docker。

**解决方案**: 添加 `HUMBLE_FORCE_CONTAINER` 环境变量和 `--force-container` 命令行选项。

**修改的文件**:
1. `tools/run_humble.sh:114` - 添加 `HUMBLE_FORCE_CONTAINER` 检查
2. `tools/run_competition.sh:11` - 添加 `HUMBLE_FORCE_CONTAINER` 变量定义
3. `tools/run_competition.sh:94` - 添加 `--force-container` 命令行选项
4. `tools/run_competition.sh:105` - `native_humble` 判断排除强制容器模式
5. `tools/run_competition.sh:136` - 构建时传递 `HUMBLE_FORCE_CONTAINER`
6. `tools/run_competition.sh:173` - 运行时传递 `HUMBLE_FORCE_CONTAINER`
7. `tools/build_sim_packages.sh:24` - 修复 `LD_LIBRARY_PATH` 使用相对路径

**使用方式**:
```bash
# 命令行参数
tools/run_competition.sh --simulation --build --force-container

# 或环境变量
HUMBLE_FORCE_CONTAINER=1 tools/run_competition.sh --simulation --build
```

### 9.2 搜索行为修改

**问题**: 原来的 SEARCHING 阶段没有实现向前搜索的逻辑，只是等待目标出现。

**解决方案**: 添加 SEARCH_FORWARD 效果，实现向前搜索，移动一定距离后未找到目标则失败降落。

**修改的文件**:
1. `ed_uav_mission/ed_uav_mission/d_task_model.py` - 添加 `SEARCH_FORWARD` DTaskEffect 和 `SEARCH_DISTANCE_EXCEEDED` DTaskFault
2. `ed_uav_mission/ed_uav_mission/d_task_events.py` - 添加 `search_distance_m` 参数到 DTaskRuntimeConfig
3. `ed_uav_mission/ed_uav_mission/d_task_reducer.py` - 添加 SEARCHING 阶段的处理逻辑
4. `ed_uav_mission/ed_uav_mission/competition_planner.py` - 添加 `search_forward` 方法
5. `ed_uav_mission/ed_uav_mission/competition_runtime.py` - 添加 `search_forward` 回调和 SEARCH_FORWARD 效果执行
6. `ed_uav_mission/ed_uav_mission/executor.py` - 添加 `_search_forward_task1` 方法
7. `ed_uav_mission/ed_uav_mission/mission_model.py` - 添加 `search_distance_m` 参数到 CompetitionParams
8. `ed_uav_mission/config/missions/d_arena_competition.yaml` - 添加 `search_distance_m: 2.0` 配置

**当前行为树流程**:
```
WAITING_START → TAKEOFF → STABILIZING(3s) → MOVE_RIGHT(0.75m) → SEARCHING(向前2m) → [找到目标] → TRACKING/ESCORTING
                                                                                    → [未找到] → SEARCH_DISTANCE_EXCEEDED → SAFE_HOVER → SAFE_RETURN → SAFE_LAND → ABORTED
```

### 9.3 无人机速度调整

**问题**: 搜索的前进速度太慢，追不上小车。

**解决方案**: 增加无人机的移动速度。

**修改的文件**:
1. `ed_uav_gazebo/ed_uav_gazebo/motion_policy.py:22` - `SIMULATOR_MOVE_SPEED_LIMIT_M_S` 从 0.6 增加到 1.0

**注意**: 小车速度保持 0.15 m/s 不变（在 `sim.launch.py:139` 中配置）。

---

## 十、当前项目状态

### 10.1 已完成

1. ✅ Docker 强制模式支持
2. ✅ 搜索行为树修改（SEARCH_FORWARD）
3. ✅ 无人机速度调整（1.0 m/s）
4. ✅ 构建系统修复（`--build` 参数正常工作）
5. ✅ 仿真闭环基本可用

### 10.2 未完成

1. ❌ 搜索距离可能需要调整（当前 2.0m）
2. ❌ 搜索行为可能需要优化（当前只是直线向前）
3. ❌ 调试日志清理（大量 `print(... flush=True)`）
4. ❌ Stage 4 脚本清理
5. ❌ QoS 不兼容问题（`/d_task/target_observation` 发布者用 BEST_EFFORT，订阅者用 RELIABLE）
6. ❌ 仿真流程端到端验证（reducer 转入 takeoff 后的后续流程）
7. ❌ 二维雷达 Z 漂移问题
8. ❌ Gazebo 场地重建（5m×4m）
9. ❌ AprilTag 配置
10. ❌ FAST-LIO 定位验收

### 10.3 已知问题

1. **QoS 不兼容**: `/d_task/target_observation` 发布者用 BEST_EFFORT，订阅者 mission_executor 用 RELIABLE，导致日志报 `No messages will be received`
2. **搜索失败**: 当前搜索距离 2.0m 可能不够，需要根据实际场地调整
3. **调试日志**: 多个文件中有大量 `print(... flush=True)` 调试输出，比赛前需移除或降级

---

## 十一、下一步工作建议

### 11.1 短期（比赛前）

1. **调整搜索距离**: 根据实际场地大小调整 `search_distance_m` 参数
2. **清理调试日志**: 移除或降级 `sim_fcu.py`、`sim_car_controller.py`、`sim_mission_starter.py`、`lio_adapter.py`、`planar_odom_fuser.py`、`sim_localization.py`、`d_task_reducer.py`、`competition_runtime.py` 中的调试输出
3. **修复 QoS 不兼容**: 统一 `RELIABILITY` 策略
4. **端到端验证**: 跑完一次完整 30s 仿真确认闭环

### 11.2 中期（比赛后）

1. **Gazebo 场地重建**: 更新为 5m×4m 场地
2. **AprilTag 配置**: 固定为 tag36h11 ID 0
3. **FAST-LIO 定位验收**: 验证连续输出和 Z 稳定性
4. **二维雷达 Z 漂移**: 通过 planar constraint 解决

---

## 十二、常用命令

```bash
# 构建并运行仿真（强制 Docker）
tools/run_competition.sh --simulation --build --force-container

# 仅构建
tools/run_competition.sh --simulation --build --force-container

# 运行仿真（不显示 GUI）
tools/run_competition.sh --simulation --force-container --no-display

# 运行仿真（手动启动）
tools/run_competition.sh --simulation --force-container --manual-start

# 查看帮助
tools/run_competition.sh --help
```

---

## 十三、关键配置文件

| 文件 | 用途 |
|------|------|
| `ed_uav_mission/config/missions/d_arena_competition.yaml` | 任务配置（搜索距离、高度等） |
| `ed_uav_localization/config/fields/d_arena_2026.yaml` | 场地配置（起飞点、路线等） |
| `ed_uav_gazebo/launch/sim.launch.py` | 仿真启动配置（小车速度等） |
| `ed_uav_gazebo/ed_uav_gazebo/motion_policy.py` | 无人机运动策略（速度限制等） |
| `tools/run_competition.sh` | 主启动脚本 |
| `tools/run_humble.sh` | Docker/本机 ROS2 切换脚本 |
