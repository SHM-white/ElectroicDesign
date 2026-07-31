# ED UAV 感知

`target_observation_node` 同时消费 `/camera/narrow/image_raw` 和
`/camera/wide/image_raw`（以及对应的 `CameraInfo`）和
`/d_task/vehicle/telemetry`。

## 双相机融合策略

- **窄幅优先**：窄幅相机检测到目标时以其结果为主。
- **宽幅回退**：窄幅未检测到时自动切换到宽幅相机结果。
- **双目融合**：两相机同时检测到时，在机体坐标系中按质量加权平均平移，
  再通过 EMA 滤波平滑输出。
- **旋转**：始终使用窄幅相机的旋转向量（降落只需 yaw 分量）。

## 检测目标

默认检测 AprilTag 36h11（`d2026-apriltag-v1`），标签边长 15cm。
也支持旧版十字同心圆（`d2026-circle-cross-v1`）。

使用 `ros2 launch ed_uav_perception target_observation.launch.py` 直接启动。

每张处理后的图像都会发布有效性/状态、候选数量、重投影 RMS、质量、
协方差策略和有界的拒绝原因。

合成测试和驱动产物仅用于描述软件行为；它们不是物理相机或飞行精度证据。
