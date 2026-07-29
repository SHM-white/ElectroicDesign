# ED UAV 相机传输

`ed_uav_camera` 根据已经完成能力探测的运行计划启动两个彼此独立的单目 `v4l2_camera` 节点。
该软件包不会映射数字形式的 `/dev/video*` 路径，也不会假定物理相机支持候选格式。

已安装的 `config/camera_profiles.yaml` 仅包含规划候选项：
`full_calibration` 将其 2592x1944 候选项保持在 2 Hz 或更低，并且两个实时配置都将 MJPEG
排在降级的未压缩回退候选项之前。
`fake_dual_camera_plan.json` 是合成测试数据，不用于目标硬件枚举或实测标定。

真实设备启动要求 `camera_plan` 中包含 P25 生成的证据：

```bash
ros2 launch ed_uav_camera dual_uvc.launch.py camera_plan:=/secure/p25-runtime-plan.json
```

计划必须准确包含 narrow 和 wide 的 `/dev/v4l/by-id` 绑定、已观测序列号、控制器标识符、所选候选模式，
以及绑定序列号/光栅/新鲜度的 camera-info 元数据。V4L2 驱动接收
`use_v4l2_buffer_timestamps:=true` 和 `camera_info_url`；其
`camera_info_manager` 会在每个命名空间的 `image_raw` 流旁发布匹配的锁存 `camera_info`。每个驱动进程都会独立重启。

对于离线仿真源，请使用明确的仅测试计划：

```bash
ros2 launch ed_uav_camera dual_uvc.launch.py \
  camera_plan:=.../fake_dual_camera_plan.json use_fake_devices:=true
```

仅主机端的 fake 接口不依赖 ROS 或相机：

```bash
python3 -m ed_uav_camera.fake_cli --duration-seconds 600 \
  --wide-unplug-at-seconds 120 --wide-reconnect-at-seconds 180 --restart-wide
```

不会启动立体处理。控制器拓扑、支持模式探测和实测带宽仍属于外部预检工作。
`tools/calibration/calibrate_chessboard.py` 中的选定相机引导程序会直接枚举稳定的 V4L2 by-id 设备，
并创建此严格启动路径所需的序列号/光栅绑定标定输入；它不会启动 ROS，也不会绕过门禁。
无论提供的序列号或 by-id 文本是什么，录制视频和合成运行都会标记为非生产，并被正式硬件运行门禁拒绝。

进行物理标定时，普通视角相机运行 `./tools/calibration/run_camera_calibration.sh 1`，
广角相机使用 `2`。直接 V4L2 接口请求 30 fps 的 MJPG，打开实时角点/进度预览，并接受
`q` 或 Escape 取消。如果相机没有 `ID_SERIAL_SHORT`，设备发现会使用明确的
`usb-revision:VID:PID:REV` 回退方式，并拒绝重复项。该回退方式可以区分已安装的 revision-0122 和
revision-0708 相机，但无法区分携带相同元组的未来替换设备；此类替换设备必须重新标定。
