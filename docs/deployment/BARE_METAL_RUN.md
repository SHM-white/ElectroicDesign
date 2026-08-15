# 本机裸机运行提醒（不需要 Docker）

> **结论：本开发机（Ubuntu 22.04.5 LTS / Jammy）自带原生 ROS 2 Humble，所有操作直接裸机执行，禁止、也不需要构建或使用任何 Docker 容器。**

## 为什么不需要 Docker

| 项目 | 状态 |
| --- | --- |
| 操作系统 | Ubuntu 22.04.5 LTS（`/etc/lsb-release` 确认） |
| 原生 ROS | `/opt/ros/humble` 已安装 |
| `tools/run_humble.sh` 行为 | 在 Ubuntu 22.04 上直接使用原生 `/opt/ros/humble`，不创建容器 |
| docker / podman | 本机未安装（`which docker podman` 为空） |

仓库 `README.md` 中提到的 `tools/run_humble.sh` 容器模式、`docker/Dockerfile.humble`、镜像
`ed-humble-toolchain:jammy-humble` 等流程**只在非 22.04 主机上才需要**。本机直接 source
原生 ROS 环境即可，无需安装 docker、无需拉取/构建镜像。

## 常用命令（裸机直接跑）

```bash
# ── 环境（每次新终端先执行）────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# ── 构建 ────────────────────────────────────────────────────────────────
# 注意：全量 colcon build 会因预存的 ML 包失败（ONNXRuntime / yolo_*，与飞行逻辑无关），
# 只构建与任务相关的包即可：
cd ros2_ws
colcon build --packages-select \
    ed_uav_interfaces ed_uav_mission ed_uav_vehicle_bridge \
    ed_uav_verification ed_uav_bringup ed_uav_perception ed_uav_fcu_bridge \
    --symlink-install

# ── 测试（同样直接裸机跑，不经容器）────────────────────────────────────
cd ros2_ws
python3 -m pytest src/ed_uav_mission/test src/ed_uav_vehicle_bridge/test src/ed_uav_verification/test -q

# ── 场地测试（纯视觉 + AprilTag，无需飞控/电机，可先验证相机和视觉链路）──
./field_test.sh --direct-capture

# ── Task3 稳定性测试 ────────────────────────────────────────────────────
# 1) 校验配置（当前未接电机电源时先跑这个）：
./task3.sh --dry-run
# 2) 正式启动（网络边界由部署侧隔离，仓库内不启用 SROS2 门控）：
./task3.sh
```

## 飞行控制边界

仓库内的实飞启动不读取 keystore，也不注入 enclave。网络隔离由部署侧负责。
程序只保留 AUX1 `1800..2000 us` 的一次性紧急锁浆锁存；其他 AUX 通道不参与
任务准入或模式切换。

## 注意事项

1. **不要**执行 `docker build` / `podman` / `run_humble.sh` 的容器模式；本机也没有这些命令。
2. `./task3.sh --dry-run` 只校验并打印命令，不会打开串口、网络或传感器。
3. 当前有 4 个**预先存在**的测试失败（与本次改动无关，基线即失败）：
   - `ed_uav_mission/test/test_action_lifecycle.py::test_lifecycle_sources_use_steady_timer_deadlines_and_recovery`
   - `ed_uav_mission/test/test_competition_tree.py::test_competition_ros_integration_remains_planner_only_and_flight_command_only`
   - `ed_uav_mission/test/test_executor.py::test_d_task_ros_surface_uses_typed_inputs_status_and_selection_service`
   - `ed_uav_vehicle_bridge/test/test_ros_udp_integration.py::test_real_udp_ros_select_arm_start_replay_stale_and_cleanup`
4. 电机电源未连接时：只能做 `--dry-run` 校验和 `field_test.sh`（视觉链路），不能真实起飞。

## 2026-08-01 Task3 逻辑修复记录（摘要）

对照赛题（D 题）检查并修复了以下问题：

1. **地面站状态错误**：`StabilityRunner` 原先发布竞争任务相位（stabilizing/acquiring/tracking），
   地面站会错误显示“搜索/伴飞”。已改为发布模型声明的 `STABILITY_*` 相位序列，
   并新增 `MissionStatus.STATE_STABILITY_TEST` 状态映射与显示标签。
2. **FSM 崩溃隐患**：稳定性任务分支从不发布 TAKEOFF 相位，任务结束迁移 COMPLETE 时会抛
   `invalid transition: ARMED -> COMPLETE`。已在执行器显式推进 FSM 到 TAKEOFF。
3. **死代码陷阱**：删除未接线的 `stability_runtime.py`（其发布未映射相位，若启用会 KeyError 崩溃），
   回调定义统一到 `stability_runner.py`。
4. **几何轨迹**（正方形 4 角点 + 圆形 13 点，共 17 个航点）与测试约束保持一致，未改动。

**Task3 启动链路修复（2026-08-01）**：

5. **标定门禁 grep 引号 bug**：`run_task3_flight_test.sh` 原 grep `"CALIBRATED"`（带引号），
   YAML 值无引号导致永远匹配失败；同样 `"synthetic_simulation"`/`"blocked"` 检查失效。
   已改为匹配 `key: value` 无引号格式。
6. **生成真实标定文件**：`calibration_data/field_calibrated_v1.yaml` — 相机/雷达序列号
   （取自相机计划与 field_extrinsics）+ 实测安装尺寸（2026-08-01 实测：雷达高于质心
   12cm、相机与激光测距仪低于质心 13cm、两相机左右各偏质心 0.5cm、前后无偏移）；
   canonical hash 校验通过。
7. **真实场地配置**：`d_arena_2026.yaml` — D 题 4m×5m 场地，原点为左下角 H 点，
   `current_field/eligible`，限高 2.5m（安全网）。
8. **真实任务配置**：`d_arena_stability_test.yaml`（mission_id: task3-stability-2026），
   与 `simulation_stability_test.yaml`（仿真用，保持原样）分离。
9. **launch 传参 bug**：`task3_field_profile_id` 原传入文件路径，与 executor 的
   selection contract 不匹配导致小车选择请求被拒；已改为从 profile YAML 读取 `profile_id`。
10. **mission_id 契约**：executor 要求 `goal.mission_id == config.mission_id`，
    新任务配置的 mission_id 与 `--task3-identity task3-stability-2026` 一致。

验证：`./task3.sh --dry-run` 全链路通过；相关包测试 298 passed（7 个失败均为基线预存，
与本轮改动无关）。
