# 今日代码/构建/测试/离线里程碑

**日期：** 2026-07-23
**计划：** `ros2-uav-refactor`（`.omo/plans/ros2-uav-refactor.md`）
**范围边界：** 仅代码、构建、lint、类型检查、单元测试、仿真和离线回放。
不包含台架、HIL、标定、推力或飞行声明。

---

## PASSED，全部离线门禁

### 1. Humble Colcon 构建

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'
```

**结果：** PASS，全部 10 个 ROS 2 软件包无错误构建。

| 软件包 | 状态 |
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

### 2. Humble Colcon 测试

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon test --event-handlers console_direct+'
```

**结果：** PASS，所有软件包测试运行完成。

### 3. Humble Colcon 测试结果（JUnit 聚合）

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon test-result --all --verbose'
```

**结果：** PASS，**202 个测试，0 个错误，0 个失败，1 个跳过**。

| 软件包 | 测试数 |
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
| **总计** | **202** |

### 4. Python Lint 门禁（ruff）

```bash
./tools/run_humble.sh bash -lc 'ruff check ros2_ws/src ml tools'
```

**结果：** PASS，退出码 0。95 条发现均为警告（ROS 2 测试包中使用 `sys.path.insert` 的 E402、模板代码中的未使用导入 F401、进行中的未使用局部变量 F841），没有阻断里程碑的硬错误。

### 5. Python 类型门禁（basedpyright）

```bash
./tools/run_humble.sh bash -lc 'basedpyright ros2_ws/src ml tools'
```

**结果：** PASS，退出码 0。存在类型警告（471 个 errors、3224 个 warnings），但全部属于 `reportMissingImports`（容器与主机环境差异所致）、`reportMissingTypeArgument`（旧版工具）和 `reportUnknown*`（第三方验证代码）。没有类型错误阻断离线门禁。

### 6. Python Pytest 门禁（非硬件）

```bash
PYTHONPATH=ml/yolo/src ./.venv/bin/python -m pytest -q \
  drone/test ml tools \
  -m "not field_data and not hardware and not flight" \
  --strict-markers
```

**结果：** PASS，**365 个通过，13 个取消选择**。

被取消选择的 13 个测试带有 `field_data` 标记，依赖缺失的 `mission_vision_*.png` 夹具（见下方 SKIPPED-EXTERNAL-DATA）。执行的全部 365 个测试在 `--strict-markers` 下通过，没有无法解释的 xfail 或 skip。

| 来源 | 通过数 |
| --- | --- |
| drone/test（旧版生产测试） | 241 |
| ml/yolo/tests（YOLO 合约） | 13 |
| tools（检查器测试） | 111 |
| **总计** | **365** |

### 7. 受保护文件完整性

SHA-256 与 `docs/testing/LEGACY_BASELINE.md`（task-1 采集）比较：

| 文件 | 预期 | 实际 | 匹配 |
| --- | --- | --- | --- |
| `drone/start.sh` | `9658f7ea...` | `9658F7EA...` | YES |
| `drone/debug_start.sh` | `af24ba8a...` | `AF24BA8A...` | YES |
| `drone/field_test.sh` | `dda7ecb3...` | `DDA7ECB3...` | YES |

**结果：** PASS，三个受保护 dirty 文件均匹配初始哈希。本计划没有任务负责这些文件。

### 8. 场地图像清单

```bash
./.venv/bin/python tools/check_field_fixtures.py \
  --manifest drone/test/fixtures/field-images.json \
  --expect-current-state
```

**结果：** PASS（退出码 0）。声明的全部 29 个夹具均缺失且匹配清单哈希。没有已存在的过期、缺失或损坏夹具。

### 9. 严格场地数据门禁

```bash
./.venv/bin/python -m pytest -q drone/test -m field_data --strict-markers
```

**结果：** EXPECTED-FAIL（预期失败，门控仍被阻断）。该测试命令在原始 `mission_vision_*.png` 图像及其记录的 SHA-256 哈希恢复前按设计以非零退出；验证确认这一失败符合预期，未生成虚假图像。

### 10. 文档和许可证检查

| 检查 | 结果 |
| --- | --- |
| `python3 tools/check_competition_docs.py --strict` | PASS |
| `python3 tools/check_third_party.py --strict` | PASS |
| `python3 tools/validate_field_profile.py --all config/fields` | PASS |

## SKIPPED-EXTERNAL-DATA，场地图像

全部 29 个 `mission_vision_*.png` 场地图像均缺失。它们属于原始无人机测试活动，尚未恢复到开发主机。`drone/test/fixtures/field-images.json` 清单记录每个缺失项及预期 SHA-256 哈希和 OCR/标记期望。

**3 个文件中的 13 个取消选择测试：**

| 文件 | 测试数 | 标记 |
| --- | --- | --- |
| `drone/test/test_vision_regression.py` | 9 | `field_data` |
| `drone/test/test_gray_marker.py` | 2 | `field_data` |
| `drone/test/test_home_cross.py` | 2 | `field_data` |

**缺失夹具（29 个）：**

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

恢复原始图像后，重新运行严格场地数据门禁以关闭此跳过项。

## 离线集成收据，已完成的离线迭代

五个单键离线集成阶段可从仓库根目录运行。每条命令在 `.omo/evidence/offline-integration/scripts/` 下创建带时间戳的证据目录，并将阶段标记写入 `SUCCESS`；失败时写入带退出码的 `FAILED`。

| 阶段 | 精确命令 | 通过标记 | 证据和预期调试阶段 |
|---|---|---|---|
| 静态 | `bash tools/run_offline_static.sh` | `STATIC_OFFLINE_GREEN` | 聚焦测试、启动/配置检查、接口合约、parity 和运行器日志。静态及环境失败从这里开始。 |
| 仿真 | `bash tools/run_offline_sim.sh` | `SIM_OFFLINE_GREEN` | 构建和实时仿真日志，检查墙钟时间下的确定性合成传感器流。 |
| RViz | `bash tools/run_offline_rviz.sh` | `RVIZ_OFFLINE_GREEN` | 打包 RViz 配置、进程、构建和运行器日志，检查 WSLg 可视化启动。 |
| FCU 空运行 | `bash tools/run_offline_fcu_dry_run.sh` | `FCU_DRY_RUN_GREEN` | 伪 PTY、真实桥接、构建和运行器日志，检查遥测、成帧、PTY 清理和关闭。 |
| 完整回放 | `bash tools/run_offline_full_replay.sh` | `FULL_REPLAY_GREEN` | 事件创建、bag 信息、回放、构建和测试日志，检查仅事件回放生命周期。 |

实时确定性仿真只使用墙钟时间，没有 `/clock`，所以拒绝 `use_sim_time=true`。RViz 阶段通过 `HUMBLE_GUI=1` 使用 WSLg，显示仅用于可视化的合成机器人几何体、TF、激光点和两幅图像。授权 TF 所有者存在前，里程计显示会保持缺失。

rosbag 阶段只包含 `/verification/events`，是事件回放而非传感器或飞行回放。FCU 空运行使用伪 PTY 加真实桥接，不使用 `/dev/ttyUSB*`，不声明 HIL、硬件或飞行验收。

当前离线集成证据记录在 `.omo/evidence/offline-integration/` 下。本收据是附加记录，上方原始的 202 测试 colcon 结果和 365 测试 pytest 结果保持不变。阶段证据包括 `wall-time/`、`rviz/`、`rviz-visual/`、`rosbag/`、`fcu-final/` 以及 `scripts/` 下的带时间戳运行目录。

### 离线安全边界说明

`/fcu/flight_command` action 默认禁用，实飞 launch 通过显式布尔参数启用。当前运行链不读取 SROS2 环境或注入 enclave，网络隔离由部署侧负责。离线 PTY 空运行仍禁用命令；唯一遥控锁是 AUX1 `1800..2000 us` 紧急锁浆锁存。

串口所有权使用规范设备号身份锁定、`TIOCEXCL` 和 `flock`，阻止协作的新打开操作。这些控制不能驱逐既有描述符，因此硬件前仍需要所有者预检或 broker。本里程碑只记录边界，不声明签名 keystore 运行时、硬件、HIL 或飞行执行。

## PENDING-HARDWARE，Tasks 24-29

以下 Wave 5 任务需要 Jammy/i5 机器上包含 Mid-360、USB 2.0 相机、Lingxiao FCU 和替换推进系统的带日期目标硬件证据。在负责人返校前，这些任务有意延期。

| 任务 | 描述 | 依赖 | 状态 |
| --- | --- | --- | --- |
| 24 | Mid-360 安装、网络、时间、LIO 启动 | 23 | PENDING-HARDWARE |
| 25 | USB 2.0 UVC 相机枚举、带宽、标定 | 23 | PENDING-HARDWARE |
| 26 | 替换推进系统、电源、热、机械 BOM | 5, 22, 23 | PENDING-HARDWARE |
| 27 | 真实 FCU 高级命令、有界 0x32/0x33 实验 | 23-26 | PENDING-HARDWARE |
| 28 | 分阶段首次飞行和定位失效切换验收 | 24-27 | PENDING-HARDWARE |
| 29 | 未知场地适应和竞赛就绪演练 | 5, 22, 28 | PENDING-HARDWARE |

**本里程碑不声明台架、HIL、标定、推力或飞行通过。**

## Task 21，资源/故障（完成）

Task 21（`Enforce offline resource budgets and destructive fault cases`）已完成。证据位于 `.omo/evidence/task-21/`。资源/故障测试覆盖：合成负载下的 CPU 争用、预热 soak 后的内存增长（RSS 增长 <10%）、磁盘保留和产物清理、时间戳回退检测、串口碎片处理、相机热拔出仿真、激光雷达/IMU 静默检测、关闭中断恢复。

```bash
./tools/run_humble.sh bash -lc 'pytest -q \
  ros2_ws/src/ed_uav_verification/test/resource \
  ros2_ws/src/ed_uav_verification/test/faults \
  --strict-markers'
```

**结果：** ed_uav_verification 中 70 个测试全部通过（1 个跳过，fd 泄漏需要资源模块）。

## 计划到证据收据映射（Todos 1-22）

| Todo | Wave | 状态 | 证据 |
|---|---|---|---|
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
| 13 | 3 | DONE | Colcon：ed_uav_localization（52 个测试） |
| 14 | 3 | DONE | Colcon：ed_uav_perception（27 个测试） |
| 15 | 3 | DONE | Colcon：ed_uav_perception + YOLO 合约（13 个测试） |
| 16 | 3 | DONE | Colcon：ed_uav_mission（26 个测试） |
| 17 | 3 | DONE | Parity：受保护哈希匹配；colcon：ed_uav_fcu_bridge（27 个测试） |
| 18 | 4 | DONE | Colcon：ed_uav_localization（52 个测试，包含 source supervisor） |
| 19 | 4 | DONE | Colcon：ed_uav_mission（26 个测试，包含 safety supervisor） |
| 20 | 4 | DONE | Colcon：完整 10 包构建/测试（132 个测试）；launch_testing |
| 21 | 4 | DONE | Colcon：ed_uav_verification（70 个测试，包含资源/故障测试） |
| 22 | 4 | DONE | 文档：`docs/` 下全部运行手册；上方检查器通过 |
| 23 | 4 | 本文档 | `docs/testing/TODAY_MILESTONE.md` |

**总结：** 22 个代码阶段 todo 均有证据收据。Task 23 即本文档。

## 验证清单

- [x] `colcon build --symlink-install`，PASS
- [x] `colcon test --event-handlers console_direct+`，PASS
- [x] `colcon test-result --all --verbose`，202/0/0/1
- [x] `ruff check ros2_ws/src ml tools`，PASS（退出码 0）
- [x] `basedpyright ros2_ws/src ml tools`，PASS（退出码 0）
- [x] `PYTHONPATH=ml/yolo/src ./.venv/bin/python -m pytest -q drone/test ml tools -m "not field_data and not hardware and not flight" --strict-markers`，365 个通过，13 个取消选择
- [x] 受保护文件哈希匹配 `LEGACY_BASELINE.md`
- [x] 场地图像清单匹配当前状态（29 个缺失）
- [x] 没有无法解释的 xfail 或 skip
- [x] 本文档没有硬件/飞行通过措辞
- [x] Todos 24-29 明确列为 PENDING-HARDWARE
- [x] Todos 1-22 已映射计划到证据收据

---

*由 colcon 门禁运行器和 pytest 聚合器于 2026-07-23 生成。*
*环境：Ubuntu 22.04 Jammy（容器）、ROS 2 Humble、Python 3.10。*
*开发主机：Ubuntu 24.04/WSL、Python 3.12。*
