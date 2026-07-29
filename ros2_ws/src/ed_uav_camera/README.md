# ED UAV Camera Transport

`ed_uav_camera` launches two independent monocular `v4l2_camera` nodes from a
runtime plan that has already been capability-probed. The package never maps a
numeric `/dev/video*` path and never assumes a physical camera supports a
candidate format.

The installed `config/camera_profiles.yaml` contains planning candidates only:
`full_calibration` keeps its 2592x1944 candidates at 2 Hz or lower, and both
live profiles order MJPEG before reduced uncompressed fallback candidates.
`fake_dual_camera_plan.json` is synthetic test data, not target hardware
enumeration or measured calibration.

Real-device launch requires P25-produced evidence in `camera_plan`:

```bash
ros2 launch ed_uav_camera dual_uvc.launch.py camera_plan:=/secure/p25-runtime-plan.json
```

The plan must contain exactly narrow and wide `/dev/v4l/by-id` bindings,
observed serials, controller identifiers, selected candidate modes, and
serial/raster/freshness-bound camera-info metadata. The V4L2 driver receives
`use_v4l2_buffer_timestamps:=true` and `camera_info_url`; its
`camera_info_manager` publishes matching latched `camera_info` beside each
namespaced `image_raw` stream. Each driver process respawns independently.

For an offline simulated source, use the explicit test-only plan:

```bash
ros2 launch ed_uav_camera dual_uvc.launch.py \
  camera_plan:=.../fake_dual_camera_plan.json use_fake_devices:=true
```

The host-only fake surface has no ROS or camera dependency:

```bash
python3 -m ed_uav_camera.fake_cli --duration-seconds 600 \
  --wide-unplug-at-seconds 120 --wide-reconnect-at-seconds 180 --restart-wide
```

No stereo processing is launched. Controller topology, supported-mode probing,
and measured bandwidth remain external preflight work. The selected-camera
bootstrap at `tools/calibration/calibrate_chessboard.py` directly enumerates
stable V4L2 by-id devices and creates the serial/raster-bound calibration input
required by this strict launch path; it does not launch ROS or bypass the gate.
Recorded-video and synthetic runs are marked non-production and are rejected by
the formal hardware runtime gate regardless of supplied serial or by-id text.
