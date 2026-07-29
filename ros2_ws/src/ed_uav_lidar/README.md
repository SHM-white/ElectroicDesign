# ED UAV 激光雷达传输

`ed_uav_lidar` 有三种启动模式：`disabled`、`generic` 和 `mid360`。
默认模式为 disabled，因此不会导入或启动 Livox 软件。

`mid360` 使用 P04 固定的 `livox_ros_driver2` 作为独立 ROS 进程，并设置 `xfer_format=1`。
`/livox/lidar` 仍是 FAST-LIO 的直接 `CustomMsg` 输入。`mid360_adapter` 是旁路节点，将 `/lidar/points`
发布为标准 `PointCloud2` 字段 `x`、`y`、`z`、`intensity` 以及原始 `offset_time`；它不会改写直接消息。
`/livox/imu` 会中继到冻结的 `/lidar/imu` 话题。

Mid-360 的序列号、IP、固件和驱动 JSON 值起初都是占位符。只有在它们全部变为非占位值，且内置的
`mid360_driver.json` 已替换为经过现场验证的路径后，启动计划才会启动供应商驱动。
`ptp` 只报告 `PTP_CONFIGURED_UNVERIFIED`；`host` 报告 `HOST_TIME_UNVERIFIED`。
这两种设置都不表示经过实测的同步或点质量。

无硬件手动重放接口如下：

```bash
PYTHONPATH=ros2_ws/src/ed_uav_lidar \
  python3 -m ed_uav_lidar.replay sample.json
```

对于格式错误的输入、缺失或倒退的逐点计时、过期 IMU、驱动退出以及驱动看门狗挂起，它会产生确定性的 RED 结果。
