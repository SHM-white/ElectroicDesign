# ED UAV perception

`target_observation_node` consumes `/camera/narrow/image_raw`, matching
`CameraInfo`, and `/d_task/vehicle_telemetry`. It detects only the frozen
`d2026-circle-cross-v1` geometry, uses raw distorted pixels with raw `K/D`, and
publishes accepted camera-frame poses on `/d_task/target_observation`.

Launch it directly with `ros2 launch ed_uav_perception
target_observation.launch.py initial_vehicle_heading_rad:=<radians>`. Camera
and vehicle topic arguments are remappable from the same launch command.

The target is fourfold symmetric. The first observation therefore needs
`initial_vehicle_heading_rad`; later observations may use the retained prior
pose and `VehicleTelemetry.turn_class`. Missing disambiguation, calibration,
freshness, or geometry rejects instead of selecting the lowest-error pose.

The frozen `TargetObservation` contract has `confidence` and pose covariance,
but no candidate-count, reprojection-RMS, or reject-reason fields. The node
does not overload unrelated message fields. Those diagnostics are retained in
the typed `last_result` and exposed through `last_candidate_count`,
`last_reprojection_rms_px`, `last_quality`, and `last_reject_reason` parameters.

Synthetic tests and driver artifacts characterize software behavior only;
they are not physical camera or flight-accuracy evidence.
