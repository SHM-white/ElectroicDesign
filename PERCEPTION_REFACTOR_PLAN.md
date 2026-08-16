# 相机识别框架重构计划

> **创建时间**: 2026-08-16
> **状态**: 执行中

---

## 一、目标

重构相机识别框架，解决以下问题：
1. 双相机不是真正独立运行
2. 调试信息不足，像黑箱
3. QoS配置不兼容
4. 性能需要优化

### 用户需求
- 广角和窄视角两个相机在单独节点运行识别，互不阻塞
- 发给决策ROS包的topic只有识别结果
- 支持发布原始图片和处理过程中的图片及对应信息用于调试
- 可视化窗口用另一个ROS包接收这些话题并绘制识别过程和结果
- 录制调试视频，放在文件夹下添加gitignore
- 优先精度，720p可接受，必要时降分辨率保流畅

---

## 二、新架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          数据源层                                       │
├─────────────────────────────────────────────────────────────────────────┤
│  /camera/narrow/image_raw ──┐                                           │
│  /camera/narrow/camera_info ┼─→ [narrow_detector_node]                  │
│  /d_task/vehicle/telemetry ─┘         │                                 │
│                                       │                                 │
│  /camera/wide/image_raw ────┐         │                                 │
│  /camera/wide/camera_info ──┼─→ [wide_detector_node]                    │
│  /d_task/vehicle/telemetry ─┘         │                                 │
└───────────────────────────────────────┼─────────────────────────────────┘
                                        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          检测层（独立运行）                             │
├─────────────────────────────────────────────────────────────────────────┤
│  [narrow_detector_node]                    [wide_detector_node]         │
│  ├─ 订阅: /camera/narrow/*                 ├─ 订阅: /camera/wide/*      │
│  ├─ 检测: AprilTag (独立)                  ├─ 检测: AprilTag (独立)     │
│  ├─ 发布:                                  ├─ 发布:                     │
│  │   /perception/narrow/detection          │   /perception/wide/detection│
│  │   /perception/narrow/annotated_image    │   /perception/wide/annotated│
│  │   /perception/narrow/diagnostics        │   /perception/wide/diag    │
│  └─ QoS: BEST_EFFORT (对齐硬件)           └─ QoS: BEST_EFFORT          │
└───────────────────────────────────────┬─────────────────────────────────┘
                                        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          融合层                                         │
├─────────────────────────────────────────────────────────────────────────┤
│  [target_fusion_node]                                                   │
│  ├─ 订阅: /perception/{narrow,wide}/detection                          │
│  ├─ 时间同步: ApproximateTimeSynchronizer                              │
│  ├─ 跳变过滤: 50cm阈值 + Kalman预测                                    │
│  ├─ 发布:                                                              │
│  │   /d_task/target_observation (决策用，BE)                           │
│  │   /perception/fusion/diagnostics (调试用)                          │
│  └─ QoS: BEST_EFFORT (对齐合同)                                        │
└───────────────────────────────────────┬─────────────────────────────────┘
                                        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          可视化调试层                                   │
├─────────────────────────────────────────────────────────────────────────┤
│  [perception_visualizer_node]                                           │
│  ├─ 三个窗口: 窄相机、广角、融合状态                                   │
│  ├─ 实时覆盖层: FPS、延迟、检测状态                                    │
│  ├─ 录制功能: 按r键开始/停止                                           │
│  └─ 截图功能: 按s键保存                                                │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 三、文件清单

### 3.1 新增文件（12个）

**消息类型（2个）**
```
ros2_ws/src/ed_uav_interfaces/msg/PerceptionDiagnostics.msg
ros2_ws/src/ed_uav_interfaces/msg/FusionDiagnostics.msg
```

**检测节点（3个）**
```
ros2_ws/src/ed_uav_perception/ed_uav_perception/single_camera_detector_node.py
ros2_ws/src/ed_uav_perception/ed_uav_perception/narrow_detector_node.py
ros2_ws/src/ed_uav_perception/ed_uav_perception/wide_detector_node.py
```

**融合节点（1个）**
```
ros2_ws/src/ed_uav_perception/ed_uav_perception/target_fusion_node.py
```

**可视化节点（1个）**
```
ros2_ws/src/ed_uav_perception/ed_uav_perception/perception_visualizer_node.py
```

**Launch文件（3个）**
```
ros2_ws/src/ed_uav_perception/launch/narrow_detector.launch.py
ros2_ws/src/ed_uav_perception/launch/wide_detector.launch.py
ros2_ws/src/ed_uav_perception/launch/target_fusion.launch.py
```

**调试录制目录（2个）**
```
ros2_ws/src/ed_uav_perception/debug_recordings/.gitignore
```

### 3.2 修改文件（6个）

```
ros2_ws/src/ed_uav_interfaces/CMakeLists.txt
ros2_ws/src/ed_uav_perception/setup.py
ros2_ws/src/ed_uav_perception/package.xml
ros2_ws/src/ed_uav_gazebo/models/ed_quadrotor/model.sdf
ros2_ws/src/ed_uav_gazebo/launch/sim.launch.py
ros2_ws/src/ed_uav_perception/ed_uav_perception/apriltag_detector.py
```

### 3.3 删除文件（2个）

```
ros2_ws/src/ed_uav_perception/ed_uav_perception/target_observation_node.py
ros2_ws/src/ed_uav_gazebo/ed_uav_gazebo/camera_debug.py
```

---

## 四、实施步骤

### 步骤1：基础设施准备（30分钟）
- 修改model.sdf恢复分辨率1280×960
- 创建新消息类型文件
- 更新CMakeLists.txt

### 步骤2：单相机检测节点（2小时）
- 创建single_camera_detector_node.py
- 创建narrow_detector_node.py和wide_detector_node.py
- 创建launch文件
- 更新setup.py添加入口点

### 步骤3：融合节点（1.5小时）
- 创建target_fusion_node.py
- 实现跳变过滤逻辑
- 创建launch文件

### 步骤4：可视化节点（1.5小时）
- 创建perception_visualizer_node.py
- 实现录制功能
- 实现截图功能

### 步骤5：集成测试（1小时）
- 更新sim.launch.py
- 运行仿真测试
- 验证端到端流程

### 步骤6：清理与优化（30分钟）
- 删除旧文件
- 创建.gitignore
- 更新文档

---

## 五、关键技术决策

### 5.1 QoS策略
- 图像订阅：qos_profile_sensor_data (BEST_EFFORT)
- 诊断发布：qos_profile_sensor_data (BEST_EFFORT)
- 融合结果：qos_profile_sensor_data (BEST_EFFORT)

### 5.2 性能优化策略
- 多线程：ThreadPoolExecutor处理图像写入
- ArUco参数：降低adaptiveThreshWinSizeMax到49
- 动态分辨率：FPS<10时自动降级
- 异步处理：标注图像异步发布

### 5.3 融合策略
- 时间同步：ApproximateTimeSynchronizer，slop=0.1s
- 跳变过滤：50cm阈值，使用Kalman预测
- 质量加权：根据检测质量加权融合
- Kalman滤波：恒速模型，平滑输出

---

## 六、预期效果

### 6.1 性能指标
- 检测帧率：窄相机~15fps，广角~10fps
- 处理延迟：<50ms（含检测和发布）
- 融合延迟：<10ms

### 6.2 调试体验
- 可视化：实时看到检测结果和状态
- 诊断：每帧都有详细的诊断信息
- 录制：可以录制调试视频供后续分析
- 透明：不再是黑箱，能看到每个环节

---

## 七、时间估算

| 步骤 | 任务 | 时间 |
|------|------|------|
| 1 | 基础设施准备 | 30分钟 |
| 2 | 单相机检测节点 | 2小时 |
| 3 | 融合节点 | 1.5小时 |
| 4 | 可视化节点 | 1.5小时 |
| 5 | 集成测试 | 1小时 |
| 6 | 清理与优化 | 30分钟 |
| **总计** | | **7小时** |
