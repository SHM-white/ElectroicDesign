from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    profile_path = LaunchConfiguration("profile_path")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "profile_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ed_uav_localization"), "config", "fields", "simulation_arena.yaml"]
                ),
            ),
            Node(
                package="ed_uav_localization",
                executable="source_supervisor",
                name="source_supervisor",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="ed_uav_localization",
                executable="field_anchor",
                name="field_anchor",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time, "profile_path": profile_path}
                ],
            ),
        ]
    )
