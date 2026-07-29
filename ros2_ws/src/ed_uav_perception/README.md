# ED UAV perception

`target_observation_node` consumes `/camera/narrow/image_raw`, matching
`CameraInfo`, and `/d_task/vehicle/telemetry`. It detects only the frozen
`d2026-circle-cross-v1` geometry, uses raw distorted pixels with raw `K/D`, and
publishes typed valid or rejected observations on `/d_task/target_observation`.

Launch it directly with `ros2 launch ed_uav_perception
target_observation.launch.py`. Camera and vehicle topic arguments are
remappable from the same launch command.

The target is fourfold symmetric. `VehicleTelemetry.heading_rad` and signed
`yaw_rate_rad_s` predict heading at image acquisition; a fresh retained prior
also bounds temporal jumps. Missing disambiguation, calibration, freshness, or
geometry publishes a typed rejection instead of selecting a pose.

Every processed image publishes validity/status, candidate count, reprojection
RMS, quality, covariance policy, and a bounded rejection reason. Diagnostic
parameters mirror the latest typed message.

Synthetic tests and driver artifacts characterize software behavior only;
they are not physical camera or flight-accuracy evidence.
