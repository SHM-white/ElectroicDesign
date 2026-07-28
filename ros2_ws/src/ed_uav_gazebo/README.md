# ED UAV Gazebo Fortress simulator

`ed_uav_gazebo` launches a hardware-free Gazebo Fortress arena with a local
quadrotor model, native `MulticopterVelocityControl`, four native
`MulticopterMotorModel` systems, cameras, GPU lidar, IMU, downward ray sensor,
ground-truth odometry, and the ROS bridge contract used by the ED stack.

The supported one-command path imports the pinned FAST-LIO, Livox driver, and
Livox SDK2 sources into an isolated evidence directory, builds the overlay, and
starts Gazebo plus RViz:

```bash
./tools/run_gazebo_slam_nav.sh
```

After those dependencies and the workspace have already been built and sourced,
the launch file can also be started directly:

```bash
ros2 launch ed_uav_gazebo gazebo_simulation.launch.py use_sim_time:=true use_rviz:=true
```

The simulator FCU action server is the only owner of `/fcu/flight_command`.
It consumes actual `/simulation/ground_truth/odom`, publishes simulator FCU
state and diagnostics, but integrated launches disable its TF output.
`ed_uav_localization.source_supervisor` is the sole dynamic
`odom -> base_link` publisher, while `field_anchor` owns `map -> odom`.
Static sensor transforms remain owned by `robot_state_publisher`. GPU lidar
publishes `PointCloudPacked`, and the bridge converts it to canonical
`/lidar/points` `PointCloud2`; the default simulation mode feeds that stream
and `/lidar/imu` to FAST-LIO.

This package validates ROS/task/sensor integration and native Gazebo vehicle
dynamics. It does not validate V7 firmware behavior, HIL timing, serial
hardware, or flight safety.
