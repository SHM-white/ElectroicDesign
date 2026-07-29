"""Launch the prescribed target observer with explicit context parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    vehicle_topic = LaunchConfiguration("vehicle_topic")
    heading = LaunchConfiguration("initial_vehicle_heading_rad")
    rms = LaunchConfiguration("max_reprojection_rms_px")
    revision = LaunchConfiguration("target_revision")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic", default_value="/camera/narrow/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/narrow/camera_info"
            ),
            DeclareLaunchArgument(
                "vehicle_topic", default_value="/d_task/vehicle_telemetry"
            ),
            DeclareLaunchArgument(
                "target_revision", default_value="d2026-circle-cross-v1"
            ),
            DeclareLaunchArgument(
                "initial_vehicle_heading_rad", default_value="nan"
            ),
            DeclareLaunchArgument(
                "max_reprojection_rms_px", default_value="2.0"
            ),
            Node(
                package="ed_uav_perception",
                executable="target_observation_node",
                name="target_observation_node",
                output="screen",
                parameters=[
                    {"target_revision": revision},
                    {
                        "initial_vehicle_heading_rad": ParameterValue(
                            heading, value_type=float
                        )
                    },
                    {
                        "max_reprojection_rms_px": ParameterValue(
                            rms, value_type=float
                        )
                    },
                ],
                remappings=[
                    ("/camera/narrow/image_raw", image_topic),
                    ("/camera/narrow/camera_info", camera_info_topic),
                    ("/d_task/vehicle_telemetry", vehicle_topic),
                ],
            ),
        ]
    )
