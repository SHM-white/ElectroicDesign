# ED UAV Lidar Transport

`ed_uav_lidar` has three launch modes: `disabled`, `generic`, and `mid360`.
The default is disabled, so it does not import or launch Livox software.

`mid360` uses the P04-pinned `livox_ros_driver2` as a separate ROS process
with `xfer_format=1`. `/livox/lidar` remains the direct `CustomMsg` input for
FAST-LIO. `mid360_adapter` is a side branch that publishes `/lidar/points` as
standard `PointCloud2` fields `x`, `y`, `z`, `intensity`, and raw
`offset_time`; it never rewrites the direct message. `/livox/imu` is relayed to
the frozen `/lidar/imu` topic.

The Mid-360 serial, IP, firmware, and driver JSON values begin as placeholders.
The launch plan starts no vendor driver until they are all non-placeholder and
the built-in `mid360_driver.json` has been replaced with a field-verified path.
`ptp` only
reports `PTP_CONFIGURED_UNVERIFIED`; `host` reports `HOST_TIME_UNVERIFIED`.
Neither setting claims measured synchronization or point quality.

The hardware-free manual replay surface is:

```bash
PYTHONPATH=ros2_ws/src/ed_uav_lidar \
  python3 -m ed_uav_lidar.replay sample.json
```

It has deterministic RED results for malformed input, missing or regressed
per-point timing, stale IMU, driver exit, and a hung driver watchdog.
