from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    profile_path = LaunchConfiguration("profile_path")
    mission_config_path = LaunchConfiguration("mission_config_path")
    calibration_file = LaunchConfiguration("calibration_file")
    simulation_only = LaunchConfiguration("simulation_only")
    payload_config_path = LaunchConfiguration("payload_config_path")
    capability_report = LaunchConfiguration("programmable_capability_report")
    fcu_device_identity = LaunchConfiguration("fcu_device_identity")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "profile_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ed_uav_localization"), "config", "fields", "simulation_arena.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "mission_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ed_uav_mission"), "config", "missions", "simulation_patrol.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ed_uav_description"), "config", "example_uncalibrated.yaml"]
                ),
            ),
            DeclareLaunchArgument("simulation_only", default_value="false"),
            DeclareLaunchArgument(
                "payload_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ed_uav_mission"), "config", "payload_adapter.yaml"]
                ),
            ),
            DeclareLaunchArgument("programmable_capability_report", default_value=""),
            DeclareLaunchArgument("fcu_device_identity", default_value=""),
            Node(
                package="ed_uav_mission",
                executable="mission_executor",
                name="mission_executor",
                output="screen",
                arguments=["--ros-args", "--enclave", "/ed_uav_mission_executor"],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "profile_path": profile_path,
                        "mission_config_path": mission_config_path,
                        "calibration_file": calibration_file,
                        "simulation_only": simulation_only,
                        "payload_config_path": payload_config_path,
                        "programmable_capability_report": capability_report,
                        "fcu_device_identity": fcu_device_identity,
                    }
                ],
            ),
        ]
    )
