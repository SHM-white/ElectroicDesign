# Physical landing-marker recognition

## Overview

The landing marker recognition system now uses **AprilTag 36h11** as the primary detection method.

### Detection Methods

| Revision | Method | Status |
|----------|--------|--------|
| `d2026-apriltag-v1` | AprilTag 36h11 (OpenCV ArUco) | **Default, Recommended** |
| `d2026-circle-cross-v1` | Custom circle-cross marker | Legacy, still supported |

## AprilTag Configuration

### Tag Specifications
- **Family**: tag36h11
- **Size**: 15.3cm (0.153m) - update `APRILTAG_SIZE_M` in `target_detector.py` if different
- **Print**: Use `tmp/tag36h11_0_print.png`

### Key Files
```
ros2_ws/src/ed_uav_perception/ed_uav_perception/
├── apriltag_detector.py    # AprilTag detection using OpenCV ArUco
├── target_detector.py      # Main detector (dispatches to AprilTag or circle-cross)
├── target_pipeline.py      # Freshness and calibration boundary
├── target_pose.py          # PnP pose estimation
└── target_observation_node.py  # ROS 2 node (default: d2026-apriltag-v1)
```

### Test Scripts
```bash
# WSL with camera forwarded
cd /home/shm-white/ed
source .venv/bin/activate
python3 tmp/test_apriltag.py

# OpenCV ArUco version (no extra dependencies)
python3 tmp/test_apriltag_opencv.py
```

## ROS Launch

From the repository root, launch the physical cameras, observer, and RViz:

```bash
./tools/run_landing_marker_recognition.sh --camera-plan config/cameras/landing_marker_camera_plan.local.json
```

The documented root-level colcon build must already have produced a readable
`install/setup.bash`; the runner fails clearly before ROS launch when that
overlay is missing.

The command requires exactly narrow and wide stable by-id bindings. In
container mode it mounts `/dev/v4l/by-id` read-only and forwards only the two
resolved `/dev/videoN` character devices. It fails before ROS startup when a
plan is missing, unreadable, outside the workspace, or still contains a
placeholder controller ID. Device validation also runs before native Jammy
dispatch, but native commands receive no container flags.

The tracked plan keeps container-portable
`file:///workspace/calibration_data/...` camera-info URLs. For native Jammy,
the runner writes a temporary copy with those URLs mapped to this repository's
`calibration_data/` directory, passes that copy to ROS, and removes it when the
command exits. The supplied plan is never modified. Use `--help` for the
command summary.

## Custom Circle-Cross Marker (Legacy)

### Marker Geometry
- Outer circle: 50cm diameter (25cm radius)
- Inner circle: 30cm diameter (15cm radius)
- Cross: 2cm wide lines extending from center to outer circle

### Usage
To use the legacy circle-cross detector, set the target revision parameter:
```bash
ros2 param set /target_observation_node target_revision d2026-circle-cross-v1
```

Or in launch file:
```python
Node(
    package='ed_uav_perception',
    executable='target_observation_node',
    parameters=[{'target_revision': 'd2026-circle-cross-v1'}],
)
```

## QA Compliance

Per competition Q&A:
- Q8: "能不能在小车平台加AprilTag" → 答：可以
- Q13: "能否在小车平台增加AprilTag、二维码或彩色方向标志？" → 答：可以
- Q22: "小车降落平台能不能贴类似二维码的标签？" → 答：可
