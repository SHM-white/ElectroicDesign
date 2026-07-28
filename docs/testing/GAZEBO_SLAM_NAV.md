# Gazebo FAST-LIO Planner-Only Mission

Run the integrated, simulation-only FAST-LIO and Nav2 planner-only competition
mission from a WSLg session with `DISPLAY`, `WAYLAND_DISPLAY`,
`XDG_RUNTIME_DIR`, and `/mnt/wslg` available:

```bash
./tools/run_gazebo_slam_nav.sh
```

The command uses `tools/run_humble.sh` in GUI and interactive mode. On its
first execution it imports only the revisions pinned in
`ros2_ws/dependencies.repos` for `livox_sdk2`, `livox_ros_driver2`, and
`fast_lio_ros2`, including FAST-LIO's `ikd-Tree` submodule. It configures,
builds, and installs Livox SDK2 under the run evidence, records each SDK log,
and passes that private library and include directory to the driver build. The
runner applies `tools/patches/fast_lio_simulation.patch` to the evidence-local
FAST-LIO checkout before building. That patch uses a two-second, 200-sample IMU
initialization window and initializes the previous lidar scan end timestamp;
it applies with zero fuzz and fails if the pinned source no longer matches. The
source imports, build, install, colcon log, launch log, topic/action readiness
logs, and action results remain under `.omo/evidence/gazebo/<run-id>/`; it does
not write third-party sources to `ros2_ws/src` or install SDK2 system-wide.

The runner builds the ROS 2 variants of Livox and FAST-LIO, starts
`ed_uav_gazebo gazebo_simulation.launch.py` with
`localization_mode:=fast_lio`, Gazebo GUI, and RViz, and waits for the LIO,
map, planner, FCU, and mission interfaces. It requires an ACTIVE localization
status with a valid `map -> odom` transform before arming through
`/fcu/flight_command`, then sends `simulation-competition` to
`/mission/execute`. It records successful action results and confirms that the
simulator FCU is disarmed before the run becomes successful.

After the mission succeeds, Gazebo and RViz remain open for inspection. Press
`Ctrl-C` to close the session; this is a clean completion after a successful
mission and stops the launch process group.

## Limits

This is not a hardware, HIL, firmware, sensor-calibration, or flight-safety
test. The simulated generic `PointCloud2` stream has no physical Livox Mid-360
per-point timing fidelity. Nav2 is planner-only and plans fixed-altitude XY
paths: it does not start a Nav2 controller, `bt_navigator`, or publish
`cmd_vel`. The mission uses the simulator `FlightCommand` action rather than a
hardware driver and does not qualify hardware flight.
