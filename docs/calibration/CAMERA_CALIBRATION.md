# Camera Calibration Runbook

> Status: **Selected-camera chessboard bootstrap implemented.**
> `tools/calibration/calibrate_chessboard.py` opens one stable V4L2 by-id device
> directly, before the calibration-gated ROS launch. It also accepts a recorded
> video for deterministic verification, but recorded and synthetic outputs are
> explicitly non-production and fail the formal hardware runtime gate.

---

## 1. Overview

The system uses two monocular UVC cameras (narrow and wide). Each requires:

1. **Intrinsic calibration** — camera matrix `K` and distortion coefficients `D`
   consumed by `ed_uav_perception/rectifier.py`
2. **Per-resolution calibration** — separate `camera_info` per capture resolution,
   gated by `ed_uav_camera/calibration.py`
3. **Body extrinsics** — `base_link → camera_*_optical_frame` transforms in the
   calibration YAML consumed by `ed_uav_description`

### Current State (what exists)

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

The bootstrap is intentionally separate from `dual_uvc.launch.py`. Formal camera
launch still requires a runtime plan containing accepted `camera_info`; no launch
gate is bypassed to create the initial calibration.

---

## 2. Chessboard Specification

### Recommended Board

| Parameter | Value |
|---|---|
| Physical columns | 11 |
| Physical rows | 8 |
| OpenCV inner corners | `(10, 7)` |
| Square size | 15.0 mm |

Mount the board on a rigid, flat substrate. Measure a square with calipers before
every run. The CLI requires an explicit `--confirm-square-mm 15.0`; `(8,11)` is
not the OpenCV corner pattern and is rejected.

---

## 3. Capture Procedure

### 3.1 Prerequisites

```bash
# Build workspace with camera package
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && \
  colcon build --packages-select ed_uav_camera && \
  colcon test --packages-select ed_uav_camera'
```

### 3.2 Capture One Camera Directly

```bash
PYTHONPATH=ros2_ws/src/ed_uav_camera \
python3 tools/calibration/calibrate_chessboard.py \
  --width 1280 --height 720 \
  --confirm-square-mm 15.0 \
  --output-dir /secure/calibration/narrow-1280x720
```

The command enumerates only `/dev/v4l/by-id/*-video-index0`, displays serial and
stable path, and prompts for narrow/wide plus the selected camera. It opens that
by-id path with OpenCV's V4L2 backend. Persisted output never uses `/dev/videoN`.
Run it once for every camera and every exact runtime raster.

Production provenance is not a command-line option. It is derived only when the
CLI opens an enumerated stable by-id device through the direct V4L2 path. An
`--input-video` run always emits `recorded_video_fixture` and
`production_eligible: false`, even if supplied serial and by-id text match real
hardware. Synthetic fixtures use `synthetic_fixture`; both fail the formal
hardware runtime gate.

### 3.3 Capture Checklist

Move the board until at least 15 sharp, unique observations pass automatically.
Capture stops at 24 accepted observations. The filters require:

| Criterion | Minimum |
|---|---|
| Board visible | All 70 `(10, 7)` inner corners |
| Blur | Laplacian variance >= 80 |
| Duplicate separation | Normalized corner RMS >= 0.008 |
| Coverage | Board centers occupy >=4 cells of a 3x3 image grid |
| Scale diversity | Board area fraction span >=0.025 |

### 3.4 Image Naming Convention

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

## 4. Per-Resolution Calibration

### 4.1 Why Per-Resolution

The `CalibrationDescriptor` in `ed_uav_camera/calibration.py` validates that the
calibration resolution matches the runtime mode resolution. A 2592×1944
calibration **cannot** be used with a 1280×720 runtime mode — the principal point
offsets and pixel-scaled focal lengths differ.

### 4.2 Resolutions to Calibrate

From `camera_profiles.yaml`:

| Profile | Resolution | Frame Rate | Purpose |
|---|---|---|---|
| `full_calibration` | 2592×1944 | 2 Hz MJPEG | Calibration capture |
| `wide_live` | 1280×720 | 15 Hz MJPEG | Wide runtime |
| `narrow_live` | 1280×720 | 20 Hz MJPEG | Narrow runtime |

Calibrate at **both** resolutions. The 2592×1944 calibration provides maximum
accuracy; the 1280×720 calibration is needed for the runtime profile.

### 4.3 Scaling Intrinsics (alternative)

If only the high-resolution calibration is available, intrinsics can be scaled:

```
fx_720 = fx_2592 * (1280 / 2592)
cx_720 = cx_2592 * (1280 / 2592)
```

Scaled intrinsics are not accepted by this bootstrap. Calibrate directly at each
selected raster; the descriptor and existing runtime gate bind the result to it.

---

## 5. Distortion Model Comparison

### 5.1 Pinhole Model (5-param)

Coefficients: `[k1, k2, p1, p2, k3]`

Consumed by `rectifier._rectify_pinhole()`:
```python
new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
undistorted = cv2.undistort(image, K, D, None, new_K)
```

**Use when**: Focal length / FOV < 8 (not extreme wide-angle).

### 5.2 Rational Model (8-param)

Coefficients: `[k1, k2, p1, p2, k3, k4, k5, k6]`

OpenCV's `cv2.calibrateCamera` supports this via `CALIB_RATIONAL_MODEL` flag.
Not directly consumed by `rectifier.py` — would require extending the
`DistortionModel` literal type.

**Use when**: High-distortion lenses where 5-param residuals are >0.5 px.

### 5.3 Fisheye / Kannala-Brandt (4-param)

Coefficients: `[k1, k2, k3, k4]`

Consumed by `rectifier._rectify_fisheye()`:
```python
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), None)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, None, new_K, (w, h), cv2.CV_16SC2)
undistorted = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
```

**Use when**: FOV > 120° (typical for drone wide-angle lenses).

### 5.4 Selection Guide

| Criterion | Pinhole (5) | Rational (8) | Fisheye (4) |
|---|---|---|---|
| FOV < 90° | ✅ Preferred | Overkill | ❌ Wrong model |
| FOV 90°–120° | ⚠️ Marginal | ✅ Preferred | ⚠️ Check residuals |
| FOV > 120° | ❌ Inadequate | ⚠️ May work | ✅ Preferred |
| Reproj error target | < 0.5 px | < 0.3 px | < 0.5 px |
| `rectifier.py` support | ✅ `_rectify_pinhole` | ❌ Not yet | ✅ `_rectify_fisheye` |

---

## 6. Calibration Output Format

### 6.1 Intrinsic File (camera_info YAML)

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

### 6.2 Runtime Plan Entry

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

## 7. Reprojection Error & Holdout Validation

### 7.1 Reprojection Error

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

### 7.2 Holdout Validation

The deterministic split assigns every fifth accepted observation to holdout.
Calibration uses only the train observations; each holdout pose is independently
solved against the fixed intrinsics. Acceptance requires holdout mean <=0.5 px
and holdout maximum <=1.0 px.

### 7.3 Overlay Visualization

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

## 8. Body Extrinsics

### 8.1 Calibration YAML Format

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

### 8.2 Measurement Procedure

Measure from `base_link` (drone center of mass) to each sensor frame origin:

1. **Position (xyz_m)**: Measure with ruler/calipers in meters. X=forward,
   Y=left, Z=up (ENU).
2. **Orientation (rpy_rad)**: Roll/pitch/yaw in radians. For cameras mounted
   level and forward-facing, rpy is typically `[0, 0, 0]` or `[0, 0, π]`
   (180° rotation for optical frame convention).

### 8.3 Validation

```bash
# Validate calibration file
python3 ros2_ws/src/ed_uav_description/tools/validate_calibration.py \
  ros2_ws/src/ed_uav_description/config/example_uncalibrated.yaml

# Dump expected static TF
python3 ros2_ws/src/ed_uav_description/tools/dump_static_model.py \
  path/to/calibrated.yaml
```

### 8.4 Competition Gate

The `competition` profile in `bringup.launch.py` requires:
- `calibration_status == "CALIBRATED"`
- All `sensor_serials` match actual device serials (not `UNSET` or `SYNTHETIC-*`)
- `calibration_hash` matches recomputed hash

---

## 9. Legacy Hardcoded Values (to be replaced)

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

## 10. Acceptance Criteria Summary

| Gate | Criterion | Tool |
|---|---|---|
| Intrinsic accuracy | Mean reproj < 0.5 px (pinhole) or < 0.5 px (fisheye) | `calibrate_intrinsics.py` |
| Holdout robustness | Holdout error < 1.5× train error | `calibrate_intrinsics.py` |
| Serial binding | Calibration serial matches device serial | `validate_calibration.py` |
| Resolution match | Calibration resolution matches runtime mode | `calibration.py` gate |
| Freshness | `captured_at + valid_for > now` | `calibration.py` gate |
| Extrinsic measurement | All transforms non-zero (except fcu_link) | `validate_calibration.py` |
| Competition gate | `calibration_status == CALIBRATED` | `bringup.launch.py` |
