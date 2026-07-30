# Physical landing-marker recognition

Create a local runtime plan from
`config/cameras/landing_marker_camera_plan.example.json`. Keep the tracked
serials, stable by-id paths, measured modes, and calibration URLs unchanged.
Replace both `REPLACE_WITH_P25_CONTROLLER_ID` values with controller IDs from
the P25 capability and topology measurement; do not guess them.

From the repository root, launch the physical cameras, observer, and RViz:

```bash
./tools/run_landing_marker_recognition.sh --camera-plan config/cameras/landing_marker_camera_plan.local.json
```

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
