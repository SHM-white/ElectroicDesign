# ROS 2 Workspace

`ros2_ws/src` contains the ED UAV ROS 2 packages. Build and test the workspace
through the Humble runner, not by installing Humble on the Ubuntu 24.04
development host.

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon list --base-paths ros2_ws/src && ros2 doctor --report'
```

`colcon list --base-paths ros2_ws/src` reports the package workspace. Use the
same runner for the focused CI build and test gate:

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && rosdep install --from-paths ros2_ws/src --ignore-src -r -y && colcon build --symlink-install && colcon test --event-handlers console_direct+ && colcon test-result --all --verbose'
```

The image is `linux/amd64`, based on the digest-pinned
`ros:humble-ros-base-jammy` manifest declared in `docker/Dockerfile.humble`.
`tools/run_humble.sh` uses native `/opt/ros/humble` only on Ubuntu 22.04; all
other hosts use Docker or Podman. The image records installed package and Python
tool versions in `/usr/local/share/ed-humble-toolchain-versions.txt`.

For the interactive simulation-only Gazebo FAST-LIO and Nav2 planner-only
competition mission, run `./tools/run_gazebo_slam_nav.sh`. It stores isolated
third-party source and build evidence per run; see
[`docs/testing/GAZEBO_SLAM_NAV.md`](../docs/testing/GAZEBO_SLAM_NAV.md) for
prerequisites and limitations.

For an externally provisioned real Livox/FAST-LIO/localization chain, use
`./tools/run_lidar_odometry_accuracy_demo.sh` from the repository root. It
preflights `/localization/odom`, captures a bounded odometry trial, and does
not launch hardware, FAST-LIO, FCU, mission, actions, or Gazebo. See
[`docs/localization/REAL_LIDAR_ODOMETRY_DEMO.md`](../docs/localization/REAL_LIDAR_ODOMETRY_DEMO.md)
for the one-command stationary, loop, and straight-line workflows.
