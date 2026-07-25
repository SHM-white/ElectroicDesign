# ED UAV Gazebo Fortress simulator

`ed_uav_gazebo` launches a hardware-free Gazebo Fortress arena with a local
quadrotor model, native `MulticopterVelocityControl`, four native
`MulticopterMotorModel` systems, cameras, GPU lidar, IMU, downward ray sensor,
ground-truth odometry, and the ROS bridge contract used by the ED stack.

Run it from the Humble workspace after sourcing the workspace:

```bash
ros2 launch ed_uav_gazebo gazebo_simulation.launch.py use_sim_time:=true use_rviz:=true
```

The simulator FCU action server is the only owner of `/fcu/flight_command`.
It consumes actual `/simulation/ground_truth/odom`, publishes simulator FCU
state and diagnostics, and owns the dynamic `odom -> base_link` transform.
Static sensor transforms remain owned by `robot_state_publisher` from the
existing offline bringup launch. GPU lidar publishes `PointCloudPacked` and
the bridge converts it to canonical `/lidar/points` `PointCloud2`.

This package validates ROS/task/sensor integration and native Gazebo vehicle
dynamics. It does not validate V7 firmware behavior, HIL timing, serial
hardware, or flight safety.
