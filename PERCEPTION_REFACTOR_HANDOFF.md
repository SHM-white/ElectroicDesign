# 感知框架重构 - 转交文档

**日期**: 2026-08-16  
**状态**: 进行中，部分完成  
**作者**: MiMo-V2.5-Pro (AI Assistant)

---

## 📋 项目概述

### 目标
将原有的双相机检测架构重构为独立节点架构，支持：
- 窄相机和广角相机独立检测
- 双相机融合（带跳变过滤）
- 透明可视化调试
- 性能优化

### 项目结构
```
ElectroicDesign/
├── ros2_ws/src/
│   ├── ed_uav_perception/     # 感知包
│   │   ├── ed_uav_perception/
│   │   │   ├── single_camera_detector_node.py  # 单相机检测基类
│   │   │   ├── narrow_detector_node.py         # 窄相机入口
│   │   │   ├── wide_detector_node.py           # 广角相机入口
│   │   │   ├── target_fusion_node.py           # 双相机融合
│   │   │   ├── perception_visualizer_node.py   # 可视化调试
│   │   │   ├── apriltag_detector.py            # ArUco 检测器
│   │   │   └── kalman_tracker.py               # Kalman 跟踪器
│   │   └── launch/
│   │       ├── narrow_detector.launch.py
│   │       ├── wide_detector.launch.py
│   │       └── target_fusion.launch.py
│   ├── ed_uav_gazebo/         # Gazebo 仿真包
│   │   ├── launch/sim.launch.py
│   │   └── models/ed_quadrotor/model.sdf
│   └── ed_uav_interfaces/     # 消息定义
│       └── msg/
│           ├── PerceptionDiagnostics.msg
│           └── FusionDiagnostics.msg
└── tools/
    └── run_competition.sh     # 启动脚本
```

---

## ✅ 已完成的工作

### 1. 消息类型定义
**文件**: `ed_uav_interfaces/msg/`

- `PerceptionDiagnostics.msg` - 单相机诊断消息
- `FusionDiagnostics.msg` - 融合诊断消息

### 2. 单相机检测节点
**文件**: `ed_uav_perception/ed_uav_perception/single_camera_detector_node.py`

**功能**:
- 独立订阅相机图像和相机信息
- 订阅车辆遥测数据 (`/vehicle/telemetry`)
- 发布检测结果、标注图像、诊断信息
- 支持多线程处理
- 支持动态分辨率调整

**关键修改**:
```python
# 订阅话题
self.create_subscription(Image, f'/camera/{camera_role}/image_raw', ...)
self.create_subscription(CameraInfo, f'/camera/{camera_role}/camera_info', ...)
self.create_subscription(VehicleTelemetry, '/vehicle/telemetry', ...)  # 修复：从 /d_task/vehicle/telemetry 改为 /vehicle/telemetry

# 发布话题
self.detection_pub = self.create_publisher(TargetObservation, f'/perception/{camera_role}/detection', ...)
self.annotated_pub = self.create_publisher(Image, f'/perception/{camera_role}/annotated_image', ...)
self.diagnostics_pub = self.create_publisher(PerceptionDiagnostics, f'/perception/{camera_role}/diagnostics', ...)
```

### 3. 双相机融合节点
**文件**: `ed_uav_perception/ed_uav_perception/target_fusion_node.py`

**功能**:
- 订阅窄相机和广角相机的检测结果
- 时间同步（使用定时器，而非 message_filters）
- 跳变过滤（50cm 阈值）
- Kalman 预测

**关键修改**:
```python
# 移除了 message_filters，改用定时器融合
self.create_timer(0.05, self._fusion_timer)  # 20Hz

def _fusion_timer(self) -> None:
    now = time.monotonic()
    narrow_fresh = (now - self.last_narrow_time) < self.fusion_timeout
    wide_fresh = (now - self.last_wide_time) < self.fusion_timeout
    ...
```

### 4. 可视化调试节点
**文件**: `ed_uav_perception/ed_uav_perception/perception_visualizer_node.py`

**功能**:
- 三个窗口：窄相机、广角相机、融合状态
- 实时覆盖层：FPS、延迟、检测状态
- 录制功能（按 r 键）
- 截图功能（按 s 键）

**性能优化**:
```python
# 降低显示分辨率
display_width = 640

# 降低更新频率
self.create_timer(0.067, self._update_display)  # ~15fps
```

### 5. Launch 文件
**文件**: `ed_uav_gazebo/launch/sim.launch.py`

**修改**:
- 替换了旧的 `target_observation.launch.py` 和 `camera_debug` 节点
- 添加了新的检测节点、融合节点、可视化节点

### 6. 性能优化
**文件**: `ed_uav_perception/ed_uav_perception/apriltag_detector.py`

**修改**:
- 优化了 ArUco 检测参数：`adaptiveThreshWinSizeMax` 从 201 降到 49

---

## 🔧 已修复的问题

### 问题 1: FPS 为 0
**原因**: 检测节点订阅了错误的话题 `/d_task/vehicle/telemetry`，但实际发布的话题是 `/vehicle/telemetry`

**修复**: 修改 `single_camera_detector_node.py` 中的订阅话题

```python
# 修改前
self.create_subscription(VehicleTelemetry, '/d_task/vehicle/telemetry', ...)

# 修改后
self.create_subscription(VehicleTelemetry, '/vehicle/telemetry', ...)
```

### 问题 2: 可视化窗口黑屏
**原因**: 早期拒绝时没有发布标注图像

**修复**: 在 `_publish_rejection` 方法中添加标注图像发布（已回滚，因为可能导致崩溃）

### 问题 3: 显示卡顿
**原因**: 图像分辨率高（1280x960/1280x720），更新频率高（30fps）

**修复**: 降低显示分辨率到 640px，更新频率到 15fps

### 问题 4: 无人机不起飞
**原因**: 错误的 remap 修改导致 mission_executor 收不到车辆数据

**修复**: 撤销 `sim.launch.py` 中的 remap 修改

---

## ⚠️ 待解决的问题

### 问题 1: narrow_detector 可能崩溃
**现象**: `narrow_detector` 在运行一段时间后崩溃（exit code 1）

**可能原因**:
1. 内存问题 - 处理大图像时内存不足
2. OpenCV 问题 - 某些操作可能导致段错误
3. 多线程问题 - `_executor.submit()` 可能导致竞态条件

**调试建议**:
```bash
# 检查崩溃日志
docker exec <container> cat /opt/ed-ros-home/log/python3_49_*.log

# 检查进程状态
docker exec <container> ps aux | grep narrow_detector
```

### 问题 2: WSL 和 Docker 之间的 DDS 发现
**现象**: 使用 `--network-host` 后，容器外的 rqt 仍然看不到容器内的话题

**原因**: WSL2 的网络隔离导致 DDS 多播发现不工作

**临时解决方案**:
```bash
# 在容器内使用 ros2 命令行工具
docker exec <container> /tmp/ros_topics.sh

# 或者创建快捷脚本
./tools/ros2_exec.sh "ros2 topic hz /camera/wide/image_raw"
```

**长期解决方案**:
1. 配置 Fast DDS 使用单播发现
2. 或者在容器内安装 rqt

### 问题 3: 检测性能优化
**当前状态**: 检测频率约 5-8 Hz

**优化建议**:
1. 降低图像分辨率（从 1280x960 降到 640x480）
2. 使用 GPU 加速 ArUco 检测
3. 优化多线程处理

---

## 📁 文件清单

### 新增文件（12个）
```
ros2_ws/src/ed_uav_interfaces/msg/PerceptionDiagnostics.msg
ros2_ws/src/ed_uav_interfaces/msg/FusionDiagnostics.msg
ros2_ws/src/ed_uav_perception/ed_uav_perception/single_camera_detector_node.py
ros2_ws/src/ed_uav_perception/ed_uav_perception/narrow_detector_node.py
ros2_ws/src/ed_uav_perception/ed_uav_perception/wide_detector_node.py
ros2_ws/src/ed_uav_perception/ed_uav_perception/target_fusion_node.py
ros2_ws/src/ed_uav_perception/ed_uav_perception/perception_visualizer_node.py
ros2_ws/src/ed_uav_perception/launch/narrow_detector.launch.py
ros2_ws/src/ed_uav_perception/launch/wide_detector.launch.py
ros2_ws/src/ed_uav_perception/launch/target_fusion.launch.py
ros2_ws/src/ed_uav_perception/debug_recordings/.gitignore
PERCEPTION_REFACTOR_PLAN.md
```

### 修改文件（7个）
```
ros2_ws/src/ed_uav_gazebo/models/ed_quadrotor/model.sdf          # 相机分辨率
ros2_ws/src/ed_uav_interfaces/CMakeLists.txt                      # 新消息
ros2_ws/src/ed_uav_perception/setup.py                            # 新入口点
ros2_ws/src/ed_uav_perception/package.xml                         # 新依赖
ros2_ws/src/ed_uav_gazebo/launch/sim.launch.py                    # 新架构
ros2_ws/src/ed_uav_perception/ed_uav_perception/apriltag_detector.py  # 性能优化
ros2_ws/src/ed_uav_perception/ed_uav_perception/single_camera_detector_node.py  # 话题修复
```

---

## 🚀 启动命令

### 仿真模式
```bash
# 构建并启动仿真
./tools/run_competition.sh --build --simulation --enable-display --force-container

# 仅启动仿真（不构建）
./tools/run_competition.sh --simulation --enable-display --force-container
```

### 调试命令
```bash
# 检查容器状态
docker ps --format "{{.Names}}"

# 在容器内检查话题
docker exec <container> /tmp/ros_topics.sh

# 检查节点状态
docker exec <container> ros2 node list

# 检查话题频率
docker exec <container> ros2 topic hz /camera/wide/image_raw
```

---

## 📊 当前性能指标

| 指标 | 值 | 说明 |
|------|-----|------|
| 车辆遥测频率 | ~8 Hz | ✅ 正常 |
| 窄相机 FPS | ~5-8 Hz | ⚠️ 可优化 |
| 广角相机 FPS | ~3-5 Hz | ⚠️ 可优化 |
| 显示帧率 | 15 fps | ✅ 流畅 |
| 显示分辨率 | 640px | ✅ 清晰 |

---

## 🎯 下一步工作

### 优先级 1：稳定性
1. 调查并修复 `narrow_detector` 崩溃问题
2. 添加更详细的错误处理和日志

### 优先级 2：性能优化
1. 降低图像处理分辨率
2. 使用 GPU 加速 ArUco 检测
3. 优化多线程处理

### 优先级 3：调试工具
1. 解决 WSL 和 Docker 之间的 DDS 发现问题
2. 在容器内安装 rqt
3. 添加更详细的性能监控

### 优先级 4：功能完善
1. 添加动态分辨率调整
2. 添加检测结果可视化
3. 添加录制和回放功能

---

## 📞 联系方式

如有问题，请参考：
- 项目文档：`PERCEPTION_REFACTOR_PLAN.md`
- 故障排除：`TROUBLESHOOTING.md`
- 代理协作：`AGENTS.md`

---

**文档版本**: 1.0  
**最后更新**: 2026-08-16
