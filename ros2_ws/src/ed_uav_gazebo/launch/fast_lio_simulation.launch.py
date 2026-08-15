from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("ed_uav_gazebo"))
    config = package_share / "config" / "fast_lio_gazebo.yaml"
    use_sim_time = LaunchConfiguration("use_sim_time")
    calibration_file = LaunchConfiguration("calibration_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ed_uav_description"), "config", "synthetic_calibrated.yaml"]
                ),
            ),
            Node(
                package="ed_uav_gazebo",
                executable="gazebo_pointcloud_normalizer",
                name="gazebo_pointcloud_normalizer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": "/lidar/points_raw",
                        "output_topic": "/lidar/points",
                        "scan_rate_hz": 10.0,
                    }
                ],
            ),
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fastlio_mapping",
                output="screen",
                parameters=[str(config), {"use_sim_time": use_sim_time}],
                remappings=[
                    ("/Odometry", "/fast_lio/odometry"),
                    ("/cloud_registered", "/fast_lio/cloud_registered"),
                    ("/Laser_map", "/fast_lio/laser_map"),
                    ("/path", "/fast_lio/path"),
                    ("/tf", "/fast_lio/tf"),
                ],
            ),
            Node(
                package="ed_uav_localization",
                executable="lio_adapter",
                name="lio_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "calibration_file": calibration_file,
                        "input_topic": "/fast_lio/odometry",
                        "output_topic": "/localization/lio/planar_raw",
                        "cloud_input_topic": "/fast_lio/cloud_registered",
                        "map_input_topic": "/fast_lio/laser_map",
                        "path_input_topic": "/fast_lio/path",
                        "cloud_output_topic": "/localization/lio/cloud_registered",
                        "map_output_topic": "/localization/lio/map",
                        "path_output_topic": "/localization/lio/path",
                    }
                ],
            ),
            Node(
                package="ed_uav_gazebo",
                executable="planar_odom_fuser",
                name="planar_odom_fuser",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": "/localization/lio/planar_raw",
                        "output_topic": "/localization/lio/odom",
                        "altitude_topic": "/simulation/ground_truth/odom",
                        "altitude_variance": 0.0025,
                        "maximum_vertical_rate_m_s": 3.0,
                    }
                ],
            ),
        ]
    )
