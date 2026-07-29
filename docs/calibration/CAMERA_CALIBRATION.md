# 相机标定运行手册

> 状态：**已实现选定相机棋盘格引导流程。**
> `tools/calibration/calibrate_chessboard.py` opens one stable V4L2 by-id device
> directly, before the calibration-gated ROS launch. It also accepts a recorded
> video for deterministic verification, but recorded and synthetic outputs are
> explicitly non-production and fail the formal hardware runtime gate.
> The operator entry point is `tools/calibration/run_camera_calibration.sh`.

---

## 1. 概述

系统使用两个单目 UVC 相机（窄视场和宽视场）。每个相机都需要：

1. **Intrinsic calibration** — camera matrix `K` and distortion coefficients `D`
   consumed by `ed_uav_perception/rectifier.py`
2. **Per-resolution calibration** — separate `camera_info` per capture resolution,
   gated by `ed_uav_camera/calibration.py`
3. **Body extrinsics** — `base_link → camera_*_optical_frame` transforms in the
   calibration YAML consumed by `ed_uav_description`

### 当前状态（已有内容）

| Component | File | Status |
|---|---|---|
| `CameraCalibration` dataclass (K, D, width, height, model) | `ed_uav_perception/rectifier.py` | Implemented |
| Pinhole rectification (`cv2.getOptimalNewCameraMatrix` + `cv2.undistort`) | `ed_uav_perception/rectifier.py` | Implemented |
| Fisheye rectification (`cv2.fisheye.*`) | `ed_uav_perception/rectifier.py` | Implemented |
| Calibration gate (serial, resolution, freshness) | `ed_uav_camera/calibration.py` | Implemented |
| `full_calibration` profile (2592×1944 @ 2 Hz) | `ed_uav_camera/config/camera_profiles.yaml` | Implemented |
| Body extrinsic YAML + validation | `ed_uav_description/calibration.py` | Implemented |
| Chessboard direct-capture CLI | `tools/calibration/calibrate_chessboard.py` | Implemented |
| `cv2.calibrateCamera` train/holdout solver | `ed_uav_camera/chessboard_solver.py` | Implemented |
| Reprojection overlays | artifact `overlays/` directory | Implemented |
| **camera_info → CameraCalibration bridge** | — | **Not implemented** |

引导流程特意与 `dual_uvc.launch.py` 分开。正式相机启动仍需要包含已接受
`camera_info` 的运行时计划；不会绕过启动门控来创建初始标定。

---

## 2. 棋盘格规格

### 推荐棋盘格

| Parameter | Value |
|---|---|
| Physical columns | 11 |
| Physical rows | 8 |
| OpenCV inner corners | `(10, 7)` |
| Square size | 15.0 mm |

将棋盘格安装在刚性平面基板上。每次运行前都用卡尺测量方格。CLI 要求显式指定
`--confirm-square-mm 15.0`；`(8,11)` 不是 OpenCV 的角点模式，会被拒绝。

---

## 3. 采集流程

### 3.1 前置条件

```bash
# Build workspace with camera package
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && \
  colcon build --packages-select ed_uav_camera && \
  colcon test --packages-select ed_uav_camera'

# Refresh camera permissions once after the user joins the video group.
newgrp video
```

### 3.2 直接采集一个相机

```bash
# 1 = normal-view/narrow camera; 2 = wide-angle camera.
./tools/calibration/run_camera_calibration.sh 1
```

Run without an argument to choose `1` or `2` interactively. The launcher uses
1280x720 by default, creates a timestamped directory under `calibration_data/`,
and opens a live preview. The overlay reports accepted observations and board
state; press `q` or Escape to cancel. Override the raster only when deliberately
calibrating another runtime mode:

```bash
CAMERA_CALIBRATION_WIDTH=2592 CAMERA_CALIBRATION_HEIGHT=1944 \
  ./tools/calibration/run_camera_calibration.sh 1
```

The command enumerates only `/dev/v4l/by-id/*-video-index0`, displays the effective
identity and stable path, and prompts for the device number. The launcher maps
choice `1` to normal-view/narrow and `2` to wide-angle. It opens the selected by-id
path with OpenCV's V4L2 backend. Persisted output never uses `/dev/videoN`. Run it
once for every camera and every exact runtime raster.

Identity uses `ID_SERIAL_SHORT` when the camera provides one. For the two known
serialless W19 cameras, the fallback is the normalized USB tuple
`usb-revision:VID:PID:REV`: the normal-view unit is
`usb-revision:0ac8:3460:0122`, and the wide-angle unit is
`usb-revision:0ac8:3460:0708`. This distinguishes these two units across normal
restarts and USB-port changes. It is not a globally unique manufacturing serial:
another unit with the same VID, PID, and revision cannot be distinguished, so
do not replace a camera with a same-revision unit without recalibrating it.

Production provenance is not a command-line option. It is derived only when the
CLI opens an enumerated stable by-id device through the direct V4L2 path. An
`--input-video` run always emits `recorded_video_fixture` and
`production_eligible: false`, even if supplied serial and by-id text match real
hardware. Synthetic fixtures use `synthetic_fixture`; both fail the formal
hardware runtime gate.

### 3.3 采集检查表

移动棋盘格，直到至少 15 个清晰且不重复的观测自动通过。接受 24 个观测后停止采集。
过滤条件如下：

| Criterion | Minimum |
|---|---|
| Board visible | All 70 `(10, 7)` inner corners |
| Blur | Laplacian variance >= 80 |
| Duplicate separation | Normalized corner RMS >= 0.008 |
| Coverage | Board centers occupy >=4 cells of a 3x3 image grid |
| Scale diversity | Board area fraction span >=0.025 |

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
└── narrow_1280x720/    # second resolution if needed
    └── ...
```

---

## 4. 逐分辨率标定

### 4.1 为什么要逐分辨率标定

`ed_uav_camera/calibration.py` 中的 `CalibrationDescriptor` 会验证标定分辨率与运行模式
分辨率匹配。2592×1944 标定**不能**用于 1280×720 运行模式，因为主点偏移和按像素
缩放的焦距不同。

### 4.2 要标定的分辨率

From `camera_profiles.yaml`:

| Profile | Resolution | Frame Rate | Purpose |
|---|---|---|---|
| `full_calibration` | 2592×1944 | 2 Hz MJPEG | Calibration capture |
| `wide_live` | 1280×720 | 15 Hz MJPEG | Wide runtime |
| `narrow_live` | 1280×720 | 20 Hz MJPEG | Narrow runtime |

两种分辨率都要标定。2592×1944 标定提供最高精度；运行时配置需要 1280×720 标定。

### 4.3 缩放内参（替代方案）

If only the high-resolution calibration is available, intrinsics can be scaled:

```
fx_720 = fx_2592 * (1280 / 2592)
cx_720 = cx_2592 * (1280 / 2592)
```

该引导流程不接受缩放后的内参。应直接为每个选定栅格标定；描述符和现有运行时门控会
将结果绑定到该栅格。

---

## 5. 畸变模型比较

### 5.1 针孔模型（5 参数）

Coefficients: `[k1, k2, p1, p2, k3]`

Consumed by `rectifier._rectify_pinhole()`:
```python
new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
undistorted = cv2.undistort(image, K, D, None, new_K)
```

**Use when**: Focal length / FOV < 8 (not extreme wide-angle).

### 5.2 有理模型（8 参数）

Coefficients: `[k1, k2, p1, p2, k3, k4, k5, k6]`

OpenCV's `cv2.calibrateCamera` supports this via `CALIB_RATIONAL_MODEL` flag.
Not directly consumed by `rectifier.py` — would require extending the
`DistortionModel` literal type.

**Use when**: High-distortion lenses where 5-param residuals are >0.5 px.

### 5.3 鱼眼 / Kannala-Brandt（4 参数）

Coefficients: `[k1, k2, k3, k4]`

Consumed by `rectifier._rectify_fisheye()`:
```python
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), None)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, None, new_K, (w, h), cv2.CV_16SC2)
undistorted = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
```

**Use when**: FOV > 120° (typical for drone wide-angle lenses).

### 5.4 选择指南

| Criterion | Pinhole (5) | Rational (8) | Fisheye (4) |
|---|---|---|---|
| FOV < 90° | ✅ Preferred | Overkill | ❌ Wrong model |
| FOV 90°–120° | ⚠️ Marginal | ✅ Preferred | ⚠️ Check residuals |
| FOV > 120° | ❌ Inadequate | ⚠️ May work | ✅ Preferred |
| Reproj error target | < 0.5 px | < 0.3 px | < 0.5 px |
| `rectifier.py` support | ✅ `_rectify_pinhole` | ❌ Not yet | ✅ `_rectify_fisheye` |

---

## 6. 标定输出格式

### 6.1 内参文件（camera_info YAML）

Standard ROS `camera_info_url` format:

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

Each camera in the runtime plan JSON must include:

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

The `calibration.py` gate validates:
- `serial` matches the device's hardware serial
- `width`/`height` match the selected mode
- `captured_at_ns + valid_for_ns > now_ns` (freshness)
- `camera_info_url` starts with `file://`
- `capture_provenance` is exactly `direct_v4l2`
- observed serial and stable by-id exactly match the runtime camera binding

---

## 7. 重投影误差和留出验证

### 7.1 重投影误差

Compute mean reprojection error across all calibration images:

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

**Acceptance criteria**:

| Model | Mean error | Max per-image error |
|---|---|---|
| Pinhole (5) | < 0.5 px | < 1.0 px |
| Rational (8) | < 0.3 px | < 0.7 px |
| Fisheye (4) | < 0.5 px | < 1.0 px |

### 7.2 留出验证

The deterministic split assigns every fifth accepted observation to holdout.
Calibration uses only the train observations; each holdout pose is independently
solved against the fixed intrinsics. Acceptance requires holdout mean <=0.5 px
and holdout maximum <=1.0 px.

### 7.3 叠加可视化

Generate overlay images showing:
- Detected corners (green circles)
- Reprojected corners (red crosses)
- Per-corner error vectors (yellow lines)

The output directory contains `camera_info.yaml`, `descriptor.json`,
`descriptor.json.sha256`, and `overlays/`. Files are written into a hidden sibling
staging directory and the complete directory is renamed into place. A retry
replaces interrupted partial output and removes stale staging state; an existing
complete artifact is never silently overwritten.

The descriptor includes serial, stable by-id, exact raster, accepted frame
indices, metrics, capture time/lifetime, camera-info URI, and two deliberately
different hashes:

- `descriptor_hash.algorithm = ed-canonical-json-v1`: SHA-256 of UTF-8 compact
  JSON with sorted keys, ASCII escaping, finite numbers, and the top-level
  `descriptor_hash` field omitted.
- `descriptor.json.sha256`: SHA-256 of the exact emitted pretty-printed
  `descriptor.json` file bytes, in standard checksum-file format.

---

## 8. 机体外参

### 8.1 标定 YAML 格式

Defined in `ed_uav_description/calibration.py` and consumed by `bringup.launch.py`:

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

Measure from `base_link` (drone center of mass) to each sensor frame origin:

1. **Position (xyz_m)**: Measure with ruler/calipers in meters. X=forward,
   Y=left, Z=up (ENU).
2. **Orientation (rpy_rad)**: Roll/pitch/yaw in radians. For cameras mounted
   level and forward-facing, rpy is typically `[0, 0, 0]` or `[0, 0, π]`
   (180° rotation for optical frame convention).

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

The `competition` profile in `bringup.launch.py` requires:
- `calibration_status == "CALIBRATED"`
- All `sensor_serials` match actual device serials (not `UNSET` or `SYNTHETIC-*`)
- `calibration_hash` matches recomputed hash

---

## 9. 旧版硬编码值（待替换）

The non-ROS `drone/config.py` contains placeholder intrinsics:

```python
CAMERA_FOCAL_X_PX = 800.0        # NEEDS ACTUAL CALIBRATION
CAMERA_FOCAL_Y_PX = 800.0        # NEEDS ACTUAL CALIBRATION
CAMERA_PRINCIPAL_X_PX = 720.0    # center of 1440×1080
CAMERA_PRINCIPAL_Y_PX = 540.0
CAMERA_TAIL_FORWARD_OFFSET_CM = 25.0  # estimated body offset
```

These must be replaced with measured calibration output before competition.

---

## 10. 验收标准摘要

| Gate | Criterion | Tool |
|---|---|---|
| Intrinsic accuracy | Mean reproj < 0.5 px (pinhole) or < 0.5 px (fisheye) | `calibrate_intrinsics.py` |
| Holdout robustness | Holdout error < 1.5× train error | `calibrate_intrinsics.py` |
| Serial binding | Calibration serial matches device serial | `validate_calibration.py` |
| Resolution match | Calibration resolution matches runtime mode | `calibration.py` gate |
| Freshness | `captured_at + valid_for > now` | `calibration.py` gate |
| Extrinsic measurement | All transforms non-zero (except fcu_link) | `validate_calibration.py` |
| Competition gate | `calibration_status == CALIBRATED` | `bringup.launch.py` |
