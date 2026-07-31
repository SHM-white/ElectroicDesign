"""Target-following simulation: Gazebo + AprilTag perception + visual servo.

Combines the Gazebo Fortress arena (with ground markers) and the full
perception → visual-servo pipeline.  Uses ground-truth localization to
avoid FAST-LIO z-axis drift.  Publishes synthetic vehicle telemetry so
the perception pipeline's motion-context gates pass without a physical
ground vehicle.

Usage
-----
    ros2 launch ed_uav_gazebo target_following_sim.launch.py

Override the target tag revision::

    ros2 launch ed_uav_gazebo target_following_sim.launch.py \\
        target_revision:=d2026-apriltag-v1
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("ed_uav_gazebo"))
    bringup_share = Path(get_package_share_directory("ed_uav_bringup"))
    description_share = Path(get_package_share_directory("ed_uav_description"))
    localization_share = Path(get_package_share_directory("ed_uav_localization"))
    perception_share = Path(get_package_share_directory("ed_uav_perception"))
    world = package_share / "worlds" / "ed_uav_arena.sdf"
    bridge = package_share / "config" / "bridge.yaml"
    rviz = package_share / "rviz" / "target_following_sim.rviz"
    profile = localization_share / "config" / "fields" / "simulation_arena.yaml"
    calibration = description_share / "config" / "synthetic_calibrated.yaml"

    return LaunchDescription(
        [
            # ── Arguments ──────────────────────────────────────────────
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "target_revision",
                default_value="d2026-apriltag-v1",
                description="Marker revision to detect (d2026-apriltag-v1 or d2026-circle-cross-v1)",
            ),

            # ── Environment ────────────────────────────────────────────
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", str(package_share / "models")
            ),
            SetEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", str(package_share / "models")
            ),

            # ── Gazebo ─────────────────────────────────────────────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        Path(get_package_share_directory("ros_gz_sim"))
                        / "launch"
                        / "gz_sim.launch.py"
                    )
                ),
                launch_arguments={"gz_args": f"-r {world}"}.items(),
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        Path(get_package_share_directory("ros_gz_sim"))
                        / "launch"
                        / "gz_sim.launch.py"
                    )
                ),
                launch_arguments={"gz_args": f"-r -s {world}"}.items(),
                condition=IfCondition("false"),  # headless only when gui=false
            ),

            # ── ROS–Gazebo bridge ──────────────────────────────────────
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gazebo_bridge",
                output="screen",
                parameters=[
                    {
                        "config_file": str(bridge),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),

            # ── Bringup (robot description, TF, camera calibration) ────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bringup_share / "launch" / "bringup.launch.py")
                ),
                launch_arguments={
                    "profile": "offline",
                    "calibration_file": str(calibration),
                    "camera_narrow_serial": "SYNTHETIC-NARROW-001",
                    "camera_wide_serial": "SYNTHETIC-WIDE-001",
                    "lidar_serial": "SYNTHETIC-LIDAR-001",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),

            # ── Localization (ground truth — fixes z-axis drift) ───────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        localization_share
                        / "launch"
                        / "localization_simulation.launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "profile_path": str(profile),
                }.items(),
            ),

            # ── Simulator FCU ──────────────────────────────────────────
            Node(
                package="ed_uav_gazebo",
                executable="sim_fcu",
                name="sim_fcu",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "publish_odom_to_base_link_tf": False,
                    }
                ],
            ),

            # ── Ground truth localization relay ─────────────────────────
            Node(
                package="ed_uav_gazebo",
                executable="sim_localization",
                name="sim_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}
                ],
            ),

            # ── Simulated vehicle telemetry ────────────────────────────
            Node(
                package="ed_uav_gazebo",
                executable="sim_vehicle_telemetry",
                name="sim_vehicle_telemetry",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}
                ],
            ),

            # ── Target observation (perception pipeline) ───────────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(perception_share / "launch" / "target_observation.launch.py")
                ),
                launch_arguments={
                    "target_revision": LaunchConfiguration("target_revision"),
                    "max_reprojection_rms_px": "2.0",
                }.items(),
            ),

            # ── Visual servo ───────────────────────────────────────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(perception_share / "launch" / "visual_servo.launch.py")
                ),
                launch_arguments={
                    "target_topic": "/d_task/target_observation",
                    "velocity_topic": "/cmd_vel_stamped",
                    "enabled": "true",
                }.items(),
            ),

            # ── RViz ───────────────────────────────────────────────────
            Node(
                package="rviz2",
                executable="rviz2",
                name="target_following_rviz",
                output="screen",
                arguments=["-d", str(rviz)],
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}
                ],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
