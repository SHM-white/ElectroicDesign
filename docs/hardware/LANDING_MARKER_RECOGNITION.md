# Physical landing-marker recognition

## Overview

The landing marker recognition system now uses **AprilTag 36h11** as the primary detection method with **visual servo precision landing**.

### Detection Methods

| Revision | Method | Status |
|----------|--------|--------|
| `d2026-apriltag-v1` | AprilTag 36h11 (OpenCV ArUco) | **Default, Recommended** |
| `d2026-circle-cross-v1` | Custom circle-cross marker | Legacy, still supported |

## AprilTag Configuration

### Tag Specifications
- **Family**: tag36h11
- **Size**: 15cm (0.15m) — edge length of the square AprilTag
- **Print**: Use `tmp/tag36h11_0_print.png`

### Key Files
```
ros2_ws/src/ed_uav_perception/ed_uav_perception/
├── apriltag_detector.py    # AprilTag detection using OpenCV ArUco
├── target_detector.py      # Main detector (dispatches to AprilTag or circle-cross)
├── target_pipeline.py      # Freshness and calibration boundary
├── target_pose.py          # PnP pose estimation
├── visual_servo.py         # Visual servo controller for precision landing
├── visual_servo_node.py    # ROS 2 node for visual servo
└── target_observation_node.py  # ROS 2 node (default: d2026-apriltag-v1)
```

## Visual Servo Precision Landing

### Overview
The visual servo controller uses the detected marker pose to compute velocity corrections for precise landing. It implements a PD controller with phase-dependent gains.

### Landing Phases

| Phase | Distance | Description | Gains |
|-------|----------|-------------|-------|
| APPROACH | > 2m | Coarse positioning | Low gains, high max velocity |
| DESCENT | 0.5-2m | Medium precision | Medium gains |
| FINAL | 0.1-0.5m | High precision | High gains, low max velocity |
| TOUCHDOWN | < 0.1m | Minimal corrections | Very high gains, very low velocity |

### Launch
```bash
# Standalone visual servo node
ros2 launch ed_uav_perception visual_servo.launch.py

# With landing marker recognition (includes visual servo)
./tools/run_landing_marker_recognition.sh --use-visual-servo true
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_topic` | `/d_task/target_observation` | Target observation topic |
| `velocity_topic` | `/cmd_vel_stamped` | Velocity command output topic |
| `approach_kp_xy` | 0.3 | XY proportional gain for approach phase |
| `descent_kp_xy` | 0.5 | XY proportional gain for descent phase |
| `final_kp_xy` | 0.8 | XY proportional gain for final phase |
| `touchdown_kp_xy` | 1.0 | XY proportional gain for touchdown phase |
| `position_tolerance_m` | 0.02 | Position tolerance for convergence (m) |
| `stable_time_sec` | 0.5 | Time to remain stable before declaring landed |
| `enabled` | true | Enable visual servo on startup |

### Integration with Mission Executor

The mission executor (`executor.py`) automatically uses the visual servo controller for precision landing when available:

1. **Detection**: Target observation published to `/d_task/target_observation`
2. **Tracking**: Mission system tracks target using `track_target()` 
3. **Precision Landing**: Final descent uses visual servo for precise positioning
4. **Convergence**: System waits for stable position before sending land command

## ROS Launch

From the repository root, launch the physical cameras, observer, and RViz:

```bash
./tools/run_landing_marker_recognition.sh --camera-plan config/cameras/landing_marker_camera_plan.local.json
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `camera_plan` | (required) | Camera configuration file |
| `target_revision` | `d2026-apriltag-v1` | Detection method |
| `use_rviz` | `true` | Launch RViz visualization |
| `use_visual_servo` | `true` | Launch visual servo node |
| `velocity_topic` | `/cmd_vel_stamped` | Velocity command topic |

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
