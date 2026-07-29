# 相机标定运行手册

> 状态：**已实现选定相机棋盘格引导流程。**
> `tools/calibration/calibrate_chessboard.py` 会直接打开一个稳定的 V4L2 by-id 设备，
> 然后才启动受标定门控的 ROS。它也接受录制视频用于确定性验证，但录制和合成输出
> 明确不用于生产，并且无法通过正式硬件运行时门控。
> 操作员入口是 `tools/calibration/run_camera_calibration.sh`。

---

## 1. 概述

系统使用两个单目 UVC 相机（窄视场和宽视场）。每个相机都需要：

1. **内参标定**：由 `ed_uav_perception/rectifier.py` 使用相机矩阵 `K` 和畸变系数 `D`
2. **逐分辨率标定**：每个采集分辨率使用独立的 `camera_info`，由
   `ed_uav_camera/calibration.py` 门控
3. **机体外参**：标定 YAML 中由 `ed_uav_description` 使用的
   `base_link → camera_*_optical_frame` 变换

### 当前状态（已有内容）

| 组件 | 文件 | 状态 |
|---|---|---|
| `CameraCalibration` dataclass (K, D, width, height, model) | `ed_uav_perception/rectifier.py` | 已实现 |
| 针孔校正（`cv2.getOptimalNewCameraMatrix` + `cv2.undistort`） | `ed_uav_perception/rectifier.py` | 已实现 |
| 鱼眼校正（`cv2.fisheye.*`） | `ed_uav_perception/rectifier.py` | 已实现 |
| 标定门控（serial、resolution、freshness） | `ed_uav_camera/calibration.py` | 已实现 |
| `full_calibration` 配置（2592×1944 @ 2 Hz） | `ed_uav_camera/config/camera_profiles.yaml` | 已实现 |
| 机体外参 YAML 和验证 | `ed_uav_description/calibration.py` | 已实现 |
| 棋盘格直接采集 CLI | `tools/calibration/calibrate_chessboard.py` | 已实现 |
| `cv2.calibrateCamera` 训练/留出求解器 | `ed_uav_camera/chessboard_solver.py` | 已实现 |
| 重投影叠加图 | artifact `overlays/` 目录 | 已实现 |
| **camera_info → CameraCalibration 桥接** | — | **尚未实现** |

引导流程特意与 `dual_uvc.launch.py` 分开。正式相机启动仍需要包含已接受
`camera_info` 的运行时计划；不会绕过启动门控来创建初始标定。

---

## 2. 棋盘格规格

### 推荐棋盘格

| 参数 | 值 |
|---|---|
| 物理列数 | 11 |
| 物理行数 | 8 |
| OpenCV 内角点 | `(10, 7)` |
| 方格尺寸 | 15.0 mm |

将棋盘格安装在刚性平面基板上。每次运行前都用卡尺测量方格。CLI 要求显式指定
`--confirm-square-mm 15.0`；`(8,11)` 不是 OpenCV 的角点模式，会被拒绝。

---

## 3. 采集流程

### 3.1 前置条件

```bash
# 构建包含相机包的工作区
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && \
  colcon build --packages-select ed_uav_camera && \
  colcon test --packages-select ed_uav_camera'

# 用户加入 video 组后刷新一次相机权限。
newgrp video
```

### 3.2 直接采集一个相机

```bash
# 1 = 普通视场/窄相机；2 = 广角相机。
./tools/calibration/run_camera_calibration.sh 1
```

不带参数运行时，会交互选择 `1` 或 `2`。启动器默认使用 1280x720，在
`calibration_data/` 下创建时间戳目录并打开实时预览。叠加层显示已接受观测和棋盘格状态；
按 `q` 或 Escape 取消。只有明确要为另一运行模式标定时，才覆盖栅格尺寸：

```bash
CAMERA_CALIBRATION_WIDTH=2592 CAMERA_CALIBRATION_HEIGHT=1944 \
  ./tools/calibration/run_camera_calibration.sh 1
```

该命令只枚举 `/dev/v4l/by-id/*-video-index0`，显示有效身份和稳定路径，并提示输入设备
编号。启动器将选项 `1` 映射为普通视场/窄相机，将 `2` 映射为广角相机；使用 OpenCV
的 V4L2 后端打开选定的 by-id 路径。持久化输出绝不使用 `/dev/videoN`。每个相机和每个
确切运行时栅格都要运行一次。

相机提供 `ID_SERIAL_SHORT` 时，身份使用该值。对于已知的两个无序列号 W19 相机，回退值是
规范化 USB 元组
`usb-revision:VID:PID:REV`: the normal-view unit is
`usb-revision:0ac8:3460:0122`, and the wide-angle unit is
`usb-revision:0ac8:3460:0708`。这能在正常重启和 USB 端口变化后区分两个设备。它不是全球
唯一的制造序列号：无法区分另一个 VID、PID 和修订号相同的设备，因此更换为相同修订号的
设备前必须重新标定。

生产来源不是命令行选项。只有 CLI 通过直接 V4L2 路径打开枚举出的稳定 by-id 设备时，才会
推导生产来源。`--input-video` 运行始终输出 `recorded_video_fixture` 和
`production_eligible: false`，即使传入的序列号和 by-id 文本与真实硬件匹配。合成 fixture
使用 `synthetic_fixture`；两者都无法通过正式硬件运行时门控。

### 3.3 采集检查表

移动棋盘格，直到至少 15 个清晰且不重复的观测自动通过。接受 24 个观测后停止采集。
过滤条件如下：

| 标准 | 最小值 |
|---|---|
| 棋盘格可见 | 全部 70 个 `(10, 7)` 内角点 |
| 模糊度 | Laplacian 方差 >= 80 |
| 重复分离度 | 归一化角点 RMS >= 0.008 |
| 覆盖范围 | 棋盘格中心占据 3x3 图像网格中的 >=4 个单元格 |
| 尺度多样性 | 棋盘格面积比例跨度 >=0.025 |

### 3.4 图像命名约定

```
calibration_data/
├── narrow_2592x1944/
│   ├── narrow_001.png
│   ├── narrow_002.png
│   └── ...
├── wide_2592x1944/
│   ├── wide_001.png
│   └── ...
└── narrow_1280x720/    # 如需第二种分辨率
    └── ...
```

---

## 4. 逐分辨率标定

### 4.1 为什么要逐分辨率标定

`ed_uav_camera/calibration.py` 中的 `CalibrationDescriptor` 会验证标定分辨率与运行模式
分辨率匹配。2592×1944 标定**不能**用于 1280×720 运行模式，因为主点偏移和按像素
缩放的焦距不同。

### 4.2 要标定的分辨率

来源：`camera_profiles.yaml`：

| 配置 | 分辨率 | 帧率 | 用途 |
|---|---|---|---|
| `full_calibration` | 2592×1944 | 2 Hz MJPEG | 标定采集 |
| `wide_live` | 1280×720 | 15 Hz MJPEG | 宽视场运行 |
| `narrow_live` | 1280×720 | 20 Hz MJPEG | 窄视场运行 |

两种分辨率都要标定。2592×1944 标定提供最高精度；运行时配置需要 1280×720 标定。

### 4.3 缩放内参（替代方案）

如果只有高分辨率标定结果，可以缩放内参：

```
fx_720 = fx_2592 * (1280 / 2592)
cx_720 = cx_2592 * (1280 / 2592)
```

该引导流程不接受缩放后的内参。应直接为每个选定栅格标定；描述符和现有运行时门控会
将结果绑定到该栅格。

---

## 5. 畸变模型比较

### 5.1 针孔模型（5 参数）

系数：`[k1, k2, p1, p2, k3]`

由 `rectifier._rectify_pinhole()` 使用：
```python
new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
undistorted = cv2.undistort(image, K, D, None, new_K)
```

**适用条件**：Focal length / FOV < 8（不是极端广角）。

### 5.2 有理模型（8 参数）

系数：`[k1, k2, p1, p2, k3, k4, k5, k6]`

OpenCV 的 `cv2.calibrateCamera` 通过 `CALIB_RATIONAL_MODEL` 标志支持该模型。
`rectifier.py` 当前不能直接使用它，需要扩展 `DistortionModel` 字面量类型。

**适用条件**：高畸变镜头，且 5 参数残差 >0.5 px。

### 5.3 鱼眼 / Kannala-Brandt（4 参数）

系数：`[k1, k2, k3, k4]`

由 `rectifier._rectify_fisheye()` 使用：
```python
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), None)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, None, new_K, (w, h), cv2.CV_16SC2)
undistorted = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
```

**适用条件**：FOV > 120°（无人机广角镜头的典型情况）。

### 5.4 选择指南

| 标准 | 针孔（5） | 有理（8） | 鱼眼（4） |
|---|---|---|---|
| FOV < 90° | ✅ 首选 | 过度复杂 | ❌ 错误模型 |
| FOV 90°–120° | ⚠️ 临界 | ✅ 首选 | ⚠️ 检查残差 |
| FOV > 120° | ❌ 不足 | ⚠️ 可能可用 | ✅ 首选 |
| 重投影误差目标 | < 0.5 px | < 0.3 px | < 0.5 px |
| `rectifier.py` 支持 | ✅ `_rectify_pinhole` | ❌ 尚未支持 | ✅ `_rectify_fisheye` |

---

## 6. 标定输出格式

### 6.1 内参文件（camera_info YAML）

标准 ROS `camera_info_url` 格式：

```yaml
image_width: 2592
image_height: 1944
camera_name: narrow_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_model: plumb_bob   # or "rational_polynomial" or "equidistant"
distortion_coefficients:
  rows: 1
  cols: 5                      # 5 for pinhole, 4 for fisheye
  data: [k1, k2, p1, p2, k3]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]   # identity for monocular
projection_matrix:
  rows: 3
  cols: 4
  data: [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
```

### 6.2 运行时计划条目

运行时计划 JSON 中的每个相机都必须包含：

```json
{
  "calibration": {
    "serial": "DEVICE-SERIAL-HERE",
    "width": 2592,
    "height": 1944,
    "captured_at_ns": 1721700000000000000,
    "valid_for_ns": 7776000000000000000,
    "camera_info_url": "file:///secure/narrow_2592x1944.yaml",
    "capture_provenance": "direct_v4l2",
    "observed_serial": "DEVICE-SERIAL-HERE",
    "observed_by_id": "/dev/v4l/by-id/usb-camera-video-index0"
  }
}
```

`calibration.py` 门控验证：
- `serial` 与设备硬件序列号匹配
- `width`/`height` 与选定模式匹配
- `captured_at_ns + valid_for_ns > now_ns`（新鲜度）
- `camera_info_url` 以 `file://` 开头
- `capture_provenance` 必须准确为 `direct_v4l2`
- 观测到的序列号和稳定 by-id 与运行时相机绑定完全匹配

---

## 7. 重投影误差和留出验证

### 7.1 重投影误差

计算所有标定图像的平均重投影误差：

```python
mean_error = 0
for i in range(len(obj_points)):
    img_points2, _ = cv2.projectPoints(
        obj_points[i], rvecs[i], tvecs[i], K, D
    )
    error = cv2.norm(img_points[i], img_points2, cv2.NORM_L2) / len(img_points2)
    mean_error += error
mean_error /= len(obj_points)
```

**验收标准**：

| 模型 | 平均误差 | 单图最大误差 |
|---|---|---|
| Pinhole (5) | < 0.5 px | < 1.0 px |
| Rational (8) | < 0.3 px | < 0.7 px |
| Fisheye (4) | < 0.5 px | < 1.0 px |

### 7.2 留出验证

确定性划分会将每第五个已接受观测分配到留出集。标定只使用训练观测；每个留出姿态都
使用固定内参独立求解。验收要求留出集平均值 <=0.5 px，最大值 <=1.0 px。

### 7.3 叠加可视化

生成叠加图像，显示：
- 检测到的角点（绿色圆圈）
- 重投影角点（红色十字）
- 每个角点的误差向量（黄色线段）

输出目录包含 `camera_info.yaml`、`descriptor.json`、`descriptor.json.sha256` 和
`overlays/`。文件先写入隐藏的同级暂存目录，完整目录随后重命名到目标位置。重试会替换
中断的部分输出并删除过期暂存状态；已有的完整 artifact 绝不会被静默覆盖。

描述符包含序列号、稳定 by-id、确切栅格、已接受帧索引、指标、采集时间/有效期、相机信息
URI，以及两个有意不同的哈希：

- `descriptor_hash.algorithm = ed-canonical-json-v1`：对 UTF-8 紧凑 JSON 计算 SHA-256；该
  JSON 使用排序键、ASCII 转义和有限数字，并省略顶层 `descriptor_hash` 字段。
- `descriptor.json.sha256`：对实际输出的美化打印 `descriptor.json` 文件字节计算 SHA-256，
  采用标准校验和文件格式。

---

## 8. 机体外参

### 8.1 标定 YAML 格式

由 `ed_uav_description/calibration.py` 定义，并由 `bringup.launch.py` 使用：

```yaml
schema_version: 1
calibration_id: FIELD-CALIBRATED-001
calibration_status: CALIBRATED    # required for competition profile
calibration_hash: <sha256>        # hash of JSON excluding hash field
sensor_serials:
  camera_narrow: REAL-SERIAL-001
  camera_wide: REAL-SERIAL-002
  lidar: REAL-LIDAR-001
transforms:
  fcu_link:                    {xyz_m: [0.0, 0.0, 0.0], rpy_rad: [0.0, 0.0, 0.0]}
  lidar_link:                  {xyz_m: [0.12, 0.0, 0.08], rpy_rad: [0.0, 0.0, 0.0]}
  camera_narrow_optical_frame: {xyz_m: [0.08, 0.04, -0.02], rpy_rad: [0.0, 0.0, 0.0]}
  camera_wide_optical_frame:   {xyz_m: [0.08, -0.04, -0.02], rpy_rad: [0.0, 0.0, 0.0]}
  rangefinder_link:            {xyz_m: [0.0, 0.0, -0.06], rpy_rad: [0.0, 0.0, 0.0]}
```

### 8.2 测量流程

从 `base_link`（无人机质心）测量到每个传感器坐标系原点：

1. **位置（xyz_m）**：用尺或卡尺以米为单位测量。X=前，Y=左，Z=上（ENU）。
2. **姿态（rpy_rad）**：滚转/俯仰/偏航，单位为弧度。对于水平且朝前安装的相机，rpy
   通常为 `[0, 0, 0]` 或 `[0, 0, π]`（按光学坐标系约定旋转 180°）。

### 8.3 验证

```bash
# Validate calibration file
python3 ros2_ws/src/ed_uav_description/tools/validate_calibration.py \
  ros2_ws/src/ed_uav_description/config/example_uncalibrated.yaml

# Dump expected static TF
python3 ros2_ws/src/ed_uav_description/tools/dump_static_model.py \
  path/to/calibrated.yaml
```

### 8.4 竞赛门控

`bringup.launch.py` 中的 `competition` 配置要求：
- `calibration_status == "CALIBRATED"`
- All `sensor_serials` match actual device serials (not `UNSET` or `SYNTHETIC-*`)
- `calibration_hash` matches recomputed hash

---

## 9. 旧版硬编码值（待替换）

非 ROS 的 `drone/config.py` 包含占位内参：

```python
CAMERA_FOCAL_X_PX = 800.0        # NEEDS ACTUAL CALIBRATION
CAMERA_FOCAL_Y_PX = 800.0        # NEEDS ACTUAL CALIBRATION
CAMERA_PRINCIPAL_X_PX = 720.0    # center of 1440×1080
CAMERA_PRINCIPAL_Y_PX = 540.0
CAMERA_TAIL_FORWARD_OFFSET_CM = 25.0  # estimated body offset
```

竞赛前必须用实测标定输出替换这些值。

---

## 10. 验收标准摘要

| 门控 | 标准 | 工具 |
|---|---|---|
| 内参精度 | 平均重投影 < 0.5 px（针孔）或 < 0.5 px（鱼眼） | `calibrate_intrinsics.py` |
| 留出稳健性 | 留出误差 < 训练误差的 1.5× | `calibrate_intrinsics.py` |
| 序列号绑定 | 标定序列号与设备序列号匹配 | `validate_calibration.py` |
| 分辨率匹配 | 标定分辨率与运行模式匹配 | `calibration.py` 门控 |
| 新鲜度 | `captured_at + valid_for > now` | `calibration.py` 门控 |
| 外参测量 | 所有变换均非零（`fcu_link` 除外） | `validate_calibration.py` |
| 竞赛门控 | `calibration_status == CALIBRATED` | `bringup.launch.py` |
