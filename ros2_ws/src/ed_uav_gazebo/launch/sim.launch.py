"""Launch the hardware-free Fortress arena, adapters, static TF, and optional RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the complete simulator graph with one Gazebo clock source."""
    package_share = Path(get_package_share_directory("ed_uav_gazebo"))
    bringup_share = Path(get_package_share_directory("ed_uav_bringup"))
    description_share = Path(get_package_share_directory("ed_uav_description"))
    localization_share = Path(get_package_share_directory("ed_uav_localization"))
    mission_share = Path(get_package_share_directory("ed_uav_mission"))
    navigation_share = Path(get_package_share_directory("ed_uav_navigation"))
    perception_share = Path(get_package_share_directory("ed_uav_perception"))
    world = package_share / "worlds" / "ed_uav_arena.sdf"
    bridge = package_share / "config" / "bridge.yaml"
    rviz = package_share / "rviz" / "sim.rviz"
    default_profile = localization_share / "config" / "fields" / "d_arena_2026.yaml"
    default_mission = mission_share / "config" / "missions" / "d_arena_competition.yaml"
    calibration = description_share / "config" / "synthetic_calibrated.yaml"
    localization_mode = LaunchConfiguration("localization_mode")
    fast_lio_mode = PythonExpression(["'", localization_mode, "' == 'fast_lio'"])
    ground_truth_mode = PythonExpression(["'", localization_mode, "' == 'ground_truth'"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("simulation_task", default_value="1"),
            DeclareLaunchArgument("profile_path", default_value=str(default_profile)),
            DeclareLaunchArgument(
                "mission_config",
                default_value=str(default_mission),
                description="Path to mission config YAML (e.g. simulation_stability_test.yaml)",
            ),
            DeclareLaunchArgument(
                "localization_mode",
                default_value="ground_truth",
                choices=("fast_lio", "ground_truth"),
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(package_share / "models")),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", str(package_share / "models")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")),
                launch_arguments={"gz_args": f"-r {world}"}.items(),
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")),
                launch_arguments={"gz_args": f"-r -s {world}"}.items(),
                condition=UnlessCondition(LaunchConfiguration("gui")),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gazebo_bridge",
                output="screen",
                parameters=[{"config_file": str(bridge), "use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup_share / "launch" / "bringup.launch.py")),
                launch_arguments={
                    "profile": "offline",
                    "calibration_file": str(calibration),
                    "camera_narrow_serial": "SYNTHETIC-NARROW-001",
                    "camera_wide_serial": "SYNTHETIC-WIDE-001",
                    "lidar_serial": "SYNTHETIC-LIDAR-001",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(localization_share / "launch" / "localization_simulation.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "profile_path": LaunchConfiguration("profile_path"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(package_share / "launch" / "fast_lio_simulation.launch.py")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "calibration_file": str(calibration),
                }.items(),
                condition=IfCondition(fast_lio_mode),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(navigation_share / "launch" / "planner_only.launch.py")),
                launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(perception_share / "launch" / "target_observation.launch.py")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "vehicle_topic": "/vehicle/telemetry",
                    "target_revision": "d2026-apriltag-v1",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(mission_share / "launch" / "mission_executor.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "profile_path": LaunchConfiguration("profile_path"),
                    "mission_config_path": LaunchConfiguration("mission_config"),
                    "calibration_file": str(calibration),
                    "simulation_only": "true",
                }.items(),
            ),
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
            Node(
                package="ed_uav_gazebo",
                executable="sim_car_controller",
                name="sim_car_controller",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time"), "speed_m_s": 0.15}],
            ),
            Node(
                package="ed_uav_gazebo",
                executable="sim_mission_starter",
                name="sim_mission_starter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "mission_id": "d-arena-competition-2026",
                        "field_profile_id": "d-arena-2026",
                        "mission_profile_id": "d2026-competition",
                        "deployment_preset_id": "field-2026",
                        "target_revision": "d2026-apriltag-v1",
                        "task": ParameterValue(LaunchConfiguration("simulation_task"), value_type=int),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("auto_start")),
            ),
            Node(
                package="ed_uav_gazebo",
                executable="sim_localization",
                name="sim_localization",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                condition=IfCondition(ground_truth_mode),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="sim_rviz",
                output="screen",
                arguments=["-d", str(rviz)],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
