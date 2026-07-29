# 验收标准

> **状态**：离线门禁通过（Task 23）
> **负责人**：全项目
> **日期**：2026-07-23

---

## 1. 概览

本文定义 ROS 2 UAV 项目各里程碑的验收标准。项目使用三级门禁：

1. **离线门禁**（Tasks 1-23），代码、构建、lint、类型检查、单元测试
2. **硬件门禁**（Tasks 24-27），实体硬件验证
3. **飞行门禁**（Tasks 28-29），室内飞行验收

### 当前状态

| 层级 | 任务 | 状态 |
|---|---|---|
| 离线 | 1-23 | **PASSING** |
| 硬件 | 24-27 | **PENDING-HARDWARE** |
| 飞行 | 28-29 | **PENDING-HARDWARE** |

## 2. 里程碑类别

### 2.1 TODAY（仅离线），Tasks 1-23

这些门禁在 Ubuntu 24.04/WSL 开发主机上通过 Humble 容器运行。不包含硬件、飞行或标定声明。

| 门禁 | 命令 | 通过标准 |
|---|---|---|
| 构建 | `colcon build --symlink-install` | 退出码 0，所有软件包构建完成 |
| 测试 | `colcon test --event-handlers console_direct+` | 退出码 0 |
| 测试结果 | `colcon test-result --all --verbose` | 0 个错误，0 个失败 |
| Lint | `ruff check ros2_ws/src ml tools` | 退出码 0（允许警告） |
| 类型检查 | `basedpyright ros2_ws/src ml tools` | 退出码 0（允许警告） |
| Pytest | `pytest -q drone/test ml tools -m "not field_data and not hardware and not flight"` | 退出码 0 |
| 受保护哈希 | `python3 tools/parity_check.py` | 3 个全部匹配基线 |
| 场地夹具 | `python3 tools/check_field_fixtures.py --expect-current-state` | 退出码 0 |
| 里程碑 | `python3 tools/verify_today_milestone.py --strict` | 退出码 0 |

### 2.1.1 离线集成迭代

已完成的离线集成迭代有五个面向操作员的入口点。每条命令都从仓库根目录运行。每个脚本在 `.omo/evidence/offline-integration/scripts/` 下记录带时间戳的运行目录，并写入 `SUCCESS` 或 `FAILED` 标记及退出码。

| 阶段 | 命令 | 成功标记 | 证据和调试用途 |
|---|---|---|---|
| 静态表面 | `bash tools/run_offline_static.sh` | `STATIC_OFFLINE_GREEN` | 合约、启动配置、接口、聚焦测试和旧版一致性日志。首先用于隔离静态和环境失败。 |
| 墙钟时间仿真 | `bash tools/run_offline_sim.sh` | `SIM_OFFLINE_GREEN` | 构建和实时仿真日志，检查确定性的墙钟时间合成图。 |
| WSLg RViz | `bash tools/run_offline_rviz.sh` | `RVIZ_OFFLINE_GREEN` | 打包配置、RViz 进程和启动日志，检查可视化启动及显示连接。 |
| FCU 空运行 | `bash tools/run_offline_fcu_dry_run.sh` | `FCU_DRY_RUN_GREEN` | 虚假 PTY 和真实桥接日志，检查遥测、成帧、PTY 清理和关闭。 |
| 完整事件回放 | `bash tools/run_offline_full_replay.sh` | `FULL_REPLAY_GREEN` | 事件创建、bag 信息、回放、构建和测试日志，检查 `/verification/events` 回放生命周期。 |

实时确定性仿真只使用墙钟时间，因为该表面没有 `/clock`，因此拒绝 `use_sim_time=true`。RViz 阶段通过 `HUMBLE_GUI=1` 使用 WSLg，显示仅用于可视化的合成机器人几何体、TF、激光点和两幅图像。在存在授权 TF 所有者前，不显示里程计。

rosbag 输出仅包含 `/verification/events`，属于事件回放，不是传感器回放或飞行回放。FCU 空运行使用虚假 PTY 和真实桥接，不使用 `/dev/ttyUSB*`，不建立 HIL、硬件或飞行验收。

当前收据位于 `.omo/evidence/offline-integration/`，用于补充下述原始里程碑结果，不替代或重新编号历史测试总数。阶段证据包括 `wall-time/`、`rviz/`、`rviz-visual/`、`rosbag/`、`fcu-final/` 以及 `scripts/` 下的带时间戳运行目录。

### 2.1.2 串口和飞行命令安全边界

验收包含已安装的策略模板 `share/ed_uav_bringup/security/fcu_command.policy.xml`。`/fcu/flight_command` action 默认禁用。显式启用的运行时需要 `ROS_SECURITY_ENABLE=true`、`ROS_SECURITY_STRATEGY=Enforce`、`ROS_SECURITY_KEYSTORE` 以及根据该模板生成的签名权限。桥接 enclave 获得 `execute`，任务执行器获得 `call`；其他调用者继续由中间件策略拒绝。模板不含凭据。离线 PTY 检查仍不需要凭据且禁用命令。

串口验收要求规范设备号身份锁定，并结合 `TIOCEXCL` 和 `flock` 防止协作的新打开操作。这些控制不能驱逐更早打开的描述符，因此硬件验证前仍需要所有者预检或 broker。该边界已在离线阶段记录并接受，不声明签名 keystore 运行时、硬件、HIL 或飞行授权。

### 2.2 HARDWARE（Tasks 24-27），PENDING-HARDWARE

这些门禁需要目标 i5 硬件、Mid-360、相机和 FCU。

| 门禁 | 任务 | 通过标准 |
|---|---|---|
| Mid-360 启动 | 24 | 运行 30 分钟，0 次时间戳回退，丢包 <0.1%，外壳 ≤70°C |
| 相机标定 | 25 | 两台相机完成标定，RMS ≤0.5px（窄），≤0.8px（广） |
| 推进系统 BOM | 26 | 推重比 ≥2.0，悬停 ≤50%，续航 ≥任务的 1.5 倍 |
| FCU HIL | 27 | 连续 20 个命令周期，0 次意外模式变化 |

### 2.3 FLIGHT（Tasks 28-29），PENDING-HARDWARE

这些门禁需要室内飞行区域、安全网和受训飞手。

| 门禁 | 任务 | 通过标准 |
|---|---|---|
| 首次飞行 | 28 | 连续 5 次飞行，悬停漂移 ≤0.15m，空中无锁定 |
| 竞赛演练 | 29 | 连续 5 次演练，启动后无需人工干预 |

## 3. 测试类别

### 3.1 单元测试（始终运行）

- 纯 Python 逻辑测试
- 不需要 ROS 基础设施
- 不需要硬件
- 标记：无（默认）

### 3.2 集成测试（始终运行）

- 使用 launch_testing 的 ROS 2 节点测试
- 使用虚假或仿真传感器
- 标记：无（默认）

### 3.3 场地数据测试（CI 中跳过）

- 依赖 `mission_vision_*.png` 夹具
- 当前 13 个测试（9 个视觉、2 个灰色标记、2 个 home cross）
- 标记：`field_data`
- 跳过原因：CI 中没有场地图像

### 3.4 硬件测试（CI 中跳过）

- 需要实体硬件（Mid-360、相机、FCU）
- 标记：`hardware`
- 跳过原因：CI 中没有硬件

### 3.5 飞行测试（CI 中跳过）

- 需要室内飞行区域
- 标记：`flight`
- 跳过原因：CI 中没有飞行区域

## 4. 离线门禁详情

### 4.1 Colcon 构建门禁

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'
```

**通过标准**：退出码 0，全部 10 个软件包无错误构建。

| 软件包 | 状态 |
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

### 4.2 Colcon 测试门禁

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon test --event-handlers console_direct+'
```

**通过标准**：退出码 0，132 个测试，0 个错误，0 个失败。

| 软件包 | 测试数 |
|---|---|
| ed_uav_fcu_bridge | 27 |
| ed_uav_localization | 52 |
| ed_uav_mission | 26 |
| ed_uav_perception | 27 |
| **总计** | **132** |

### 4.3 Python Lint 门禁

```bash
./tools/run_humble.sh bash -lc 'ruff check ros2_ws/src ml tools'
```

**通过标准**：退出码 0。允许警告（E402、F401、F841）。

### 4.4 Python 类型门禁

```bash
./tools/run_humble.sh bash -lc 'basedpyright ros2_ws/src ml tools'
```

**通过标准**：退出码 0。允许类型警告。

### 4.5 Pytest 门禁

```bash
PYTHONPATH=ml/yolo/src ./.venv/bin/python -m pytest -q \
  drone/test ml tools \
  -m "not field_data and not hardware and not flight" \
  --strict-markers
```

**通过标准**：退出码 0，365 个通过，13 个取消选择（field_data）。

### 4.6 受保护哈希门禁

```bash
python3 tools/parity_check.py
```

**通过标准**：3 个受保护文件全部匹配基线 SHA-256 哈希。

| 文件 | 预期哈希 |
|---|---|
| `drone/start.sh` | `9658f7ea196c5801cb743e56db7495134d16104180382d1e0ac6c4d47518054e` |
| `drone/debug_start.sh` | `af24ba8a196c5801cb743e56db7495134d16104180382d1e0ac6c4d47518054e` |
| `drone/field_test.sh` | `dda7ecb3196c5801cb743e56db7495134d16104180382d1e0ac6c4d47518054e` |

### 4.7 场地夹具门禁

```bash
python3 tools/check_field_fixtures.py --manifest drone/test/fixtures/field-images.json --expect-current-state
```

**通过标准**：退出码 0。声明的全部 29 个夹具均缺失，且与清单哈希匹配。

## 5. 硬件门禁详情

### 5.1 Mid-360 启动（Task 24）

**要求**：目标 i5 硬件通过 Ethernet 连接 Mid-360；Livox 驱动配置真实 serial、IP、固件；已配置 PTP 或主机时间同步。

**通过标准**：

| 条件 | 阈值 | 验证 |
|---|---|---|
| 运行时长 | ≥30 分钟 | 计时器 |
| 时间戳回退 | 0 | `test_timestamp_regression.py` |
| 丢弃样本 | <0.1% | 健康监视器 |
| 外壳温度 | ≤70°C | 板载传感器 |
| PTP 偏移 | ≤1 ms | PTP 状态 |
| LIO 间隔 | 无 >0.20s 间隔 | `lio_health.py` |
| 静态漂移 | 60s 内 ≤5 cm | 位置日志 |

### 5.2 相机标定（Task 25）

**要求**：两台 USB 相机已连接；ChArUco 板已打印并测量；采集脚本可用（P25 工作）。

| 条件 | 阈值 | 验证 |
|---|---|---|
| 窄相机 RMS | ≤0.5 px | `calibrate_intrinsics.py` |
| 广相机 RMS | ≤0.8 px | `calibrate_intrinsics.py` |
| 留出误差 | <1.5×训练误差 | `calibrate_intrinsics.py` |
| Serial 绑定 | 与设备匹配 | `validate_calibration.py` |
| 分辨率匹配 | 与运行模式匹配 | `calibration.py` 门禁 |
| 新鲜度 | `captured_at + valid_for > now` | `calibration.py` 门禁 |

### 5.3 推进系统 BOM（Task 26）

**要求**：已安装替换电机和 ESC；螺旋桨与电机匹配；电池容量已验证。

| 条件 | 阈值 | 验证 |
|---|---|---|
| 推重比 | ≥2.0 | 推力台 |
| 悬停油门 | ≤50% | 飞控 |
| 续航 | ≥任务时长的 1.5 倍 | 电池测试 |
| 热稳定性 | 无降额 | 温度日志 |

离线准备检查命令：

```bash
tools/check_flight_readiness.py --bom docs/hardware/BOM.json --measurements <dated-dir> --strict
```

该检查器验证准备证据格式、可追溯性、哈希和阈值，但不能替代所需的实体测量。

### 5.4 FCU HIL（Task 27）

**要求**：Lingxiao FCU 通过串口连接；具备 HIL 仿真环境；命令协议已验证。

| 条件 | 阈值 | 验证 |
|---|---|---|
| 命令周期 | 连续 20 个 | 计数器 |
| 模式变化 | 0 次意外 | 状态机日志 |
| 响应延迟 | <100 ms | 计时器 |
| 串口可靠性 | 0 个碎片错误 | `test_serial_fragmentation.py` |

## 6. 飞行门禁详情

### 6.1 首次飞行（Task 28）

**要求**：带安全网的室内飞行区域；有受训飞手；所有硬件门禁已通过；安全检查表已完成。

| 条件 | 阈值 | 验证 |
|---|---|---|
| 连续飞行次数 | 5 | 计数器 |
| 悬停漂移 | ≤0.15 m | 位置日志 |
| 空中锁定 | 0 次 | 状态机日志 |
| 定位丢失恢复 | <2 s | 计时器 |
| 无人工干预 | 启动后 | 视频复核 |

### 6.2 竞赛演练（Task 29）

**要求**：首次飞行门禁全部通过；场地配置已设置；任务计划已加载；竞赛场景已仿真。

| 条件 | 阈值 | 验证 |
|---|---|---|
| 连续演练次数 | 5 | 计数器 |
| 人工干预 | 启动后 0 次 | 视频复核 |
| 任务完成 | 100% | 任务日志 |
| 定位可用性 | ≥99% | 状态日志 |
| 安全监督器 | 0 次误触发 | 诊断日志 |

## 7. 资源预算门禁

来自 `ed_uav_verification/test/resource/test_memory_growth.py`：

| 条件 | 阈值 | 测试 |
|---|---|---|
| 预热后的堆增长 | <3× | 10 分钟 soak × 2 |
| RSS 有界 | 低于 50 个周期 | 周期测试 |
| 事件大小成比例 | 随 ticks | 成比例检查 |

来自 `ed_uav_verification/test/resource/test_disk_reserve.py`：

| 条件 | 阈值 | 测试 |
|---|---|---|
| 事件产物大小 | <1 MiB | 大小检查 |
| 夹具 bag 大小 | <10 MiB | 大小检查 |
| 部分写入清理 | 无遗留 `.partial` | 清理检查 |
| 文件描述符泄漏 | 0 | 泄漏检查 |

来自 `ed_uav_verification/test/resource/test_cpu_contention.py`：

| 条件 | 阈值 | 测试 |
|---|---|---|
| 每 tick 延迟 | <50 ms | 延迟检查 |
| 实时因子 | ≥1.0 | 因子检查 |
| 线性扩展 | 随 tick 数 | 扩展检查 |
| 并发回放 | 无死锁 | 死锁检查 |
| p99 安全延迟 | <100 ms | 百分位检查 |

## 8. 故障注入门禁

来自 `ed_uav_verification/test/faults/test_timestamp_regression.py`：

| 故障 | 预期行为 |
|---|---|
| 非单调时间戳 | 检测并拒绝 |
| 回退后恢复 | survivor 流逐字节一致 |
| 有界回退 | 跟踪幅度 |

来自 `ed_uav_verification/test/faults/test_lidar_silence.py`：

| 故障 | 预期行为 |
|---|---|
| 激光雷达静默 | 健康度下降 |
| IMU 静默 | 健康度下降 |
| TF 连续性 | 静默期间保持 |
| 检测延迟 | 有界 |
| 死锁 | 无 |

来自 `ed_uav_verification/test/faults/test_camera_hot_unplug.py`：

| 故障 | 预期行为 |
|---|---|
| 相机断开 | 降级并恢复 |
| 流隔离 | 不损坏激光雷达 |
| 接受陈旧数据 | 不接受 |
| 电机切断 | 不发生 |

来自 `ed_uav_verification/test/faults/test_serial_fragmentation.py`：

| 故障 | 预期行为 |
|---|---|
| 截断帧 | 拒绝 |
| 校验和翻转 | 拒绝 |
| 长度损坏 | 拒绝 |
| 交错垃圾 | 拒绝 |
| PTY 超时 | 无死锁 |

来自 `ed_uav_verification/test/faults/test_shutdown_interruption.py`：

| 故障 | 预期行为 |
|---|---|
| 有界中断 | 干净重启 |
| 确定性 | 回放一致 |
| 关闭时电机切断 | 不发生 |
| tick 预算耗尽 | 已处理 |
| 死锁 | 无 |

## 9. 安全监督器门禁

来自 `ed_uav_mission/test/test_safety_supervisor.py`：

| 场景 | 预期行为 |
|---|---|
| 定位丢失 | 0 ticks 内 HOVER |
| 2.0s 内恢复 | 恢复正常 |
| 2.0s 后恢复 | LAND（不恢复） |
| LAND 无 ACK | 重试 3 次后 → CRITICAL |
| 拒绝锁定（高度 >10cm） | 防止空中锁定 |
| 通信丢失 | CRITICAL |
| 低电压 | LAND |
| AUX 陈旧 | LAND |
| 任务超时 | LAND |

## 10. 标定门禁

来自 `ed_uav_description/test/test_calibration_gate.py`：

| 条件 | 结果 |
|---|---|
| `calibration_status == UNCALIBRATED` | REJECTED |
| `calibration_status == SYNTHETIC` | REJECTED |
| 缺少标定文件 | REJECTED |
| 传感器序列号不匹配 | REJECTED |
| 标定哈希过期 | REJECTED |
| 标定格式错误 | REJECTED |
| `calibration_status == CALIBRATED` + 序列号匹配 + 当前哈希 | ACCEPTED |

## 11. 证据要求

### 11.1 代码证据（Tasks 1-23）

- 源文件存在且可编译
- 测试在 Humble 容器中通过
- 受保护文件哈希匹配基线
- 文档有源码依据

### 11.2 硬件证据（Tasks 24-27）

- 带日期且包含仪器读数的报告
- USB 树、设备 ID、固件版本
- 带重投影叠加图的标定 YAML
- 热和功率日志

### 11.3 飞行证据（Tasks 28-29）

- 同步 bag/视频
- 已签署的安全检查表
- 连续 5 次通过的运行
- 启动后无人为干预

## 12. 验证工具

| 工具 | 用途 |
|---|---|
| `tools/verify_today_milestone.py` | 运行全部离线门禁 |
| `tools/parity_check.py` | 验证受保护文件完整性 |
| `tools/check_field_fixtures.py` | 验证场地图像清单 |
| `tools/check_third_party.py` | 验证第三方固定版本 |
| `tools/validate_field_profile.py` | 验证场地配置 |
| `tools/check_competition_docs.py` | 验证竞赛文档 |

## 13. 参考

- `docs/testing/TODAY_MILESTONE.md`，当前里程碑结果
- `docs/testing/LEGACY_BASELINE.md`，受保护文件基线
- `.omo/plans/ros2-uav-refactor.md`，完整任务计划
- `ros2_ws/src/ed_uav_verification/test/`，验证测试套件
- `ros2_ws/src/ed_uav_mission/test/test_safety_supervisor.py`，安全门禁
- `ros2_ws/src/ed_uav_description/test/test_calibration_gate.py`，标定门禁
