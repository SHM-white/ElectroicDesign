# ED UAV 感知

`target_observation_node` 消费 `/camera/narrow/image_raw`、匹配的 `CameraInfo` 和
`/d_task/vehicle/telemetry`。它只检测冻结的 `d2026-circle-cross-v1` 几何图形，使用带原始 `K/D`
的原始畸变像素，并在 `/d_task/target_observation` 上发布带类型的有效或拒绝观测。

使用 `ros2 launch ed_uav_perception target_observation.launch.py` 直接启动。
相机和飞行器话题参数可通过同一启动命令重新映射。

目标具有四重对称性。`VehicleTelemetry.heading_rad` 和带符号的 `yaw_rate_rad_s` 用于预测图像采集时的航向；
保留的新鲜先验也会限制时间跳变。缺少消歧信息、标定、新鲜度或几何信息时，会发布带类型的拒绝结果，
而不是选择一个位姿。

每张处理后的图像都会发布有效性/状态、候选数量、重投影 RMS、质量、协方差策略和有界的拒绝原因。
诊断参数与最新的带类型消息保持一致。

合成测试和驱动产物仅用于描述软件行为；它们不是物理相机或飞行精度证据。
