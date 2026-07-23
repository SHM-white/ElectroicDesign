# ROS 2 Workspace

`ros2_ws/src` is intentionally empty until the package-owning tasks add source
packages. The workspace is built and tested through the Humble runner, not by
installing Humble on the Ubuntu 24.04 development host.

```bash
mkdir -p ros2_ws/src
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && colcon list --base-paths ros2_ws/src && ros2 doctor --report'
```

With no packages, `colcon list --base-paths ros2_ws/src` emits no package rows;
`ros2 doctor --report` still reports the pinned Humble runtime. Once packages
exist, use the same CI build gate:

```bash
./tools/run_humble.sh bash -lc 'source /opt/ros/humble/setup.bash && rosdep install --from-paths ros2_ws/src --ignore-src -r -y && colcon build --symlink-install && colcon test --event-handlers console_direct+ && colcon test-result --all --verbose'
```

The image is `linux/amd64`, based on the digest-pinned
`ros:humble-ros-base-jammy` manifest declared in `docker/Dockerfile.humble`.
`tools/run_humble.sh` uses native `/opt/ros/humble` only on Ubuntu 22.04; all
other hosts use Docker or Podman. The image records installed package and Python
tool versions in `/usr/local/share/ed-humble-toolchain-versions.txt`.
