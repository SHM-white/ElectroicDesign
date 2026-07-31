"""Launch the prescribed target observer with dual-camera fusion."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    vehicle_topic = LaunchConfiguration("vehicle_topic")
    rms = LaunchConfiguration("max_reprojection_rms_px")
    revision = LaunchConfiguration("target_revision")
    ema = LaunchConfiguration("fusion_ema_alpha")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle_topic", default_value="/d_task/vehicle/telemetry"
            ),
            DeclareLaunchArgument(
                "target_revision", default_value="d2026-apriltag-v1"
            ),
            DeclareLaunchArgument(
                "max_reprojection_rms_px", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "fusion_ema_alpha", default_value="0.6"
            ),
            Node(
                package="ed_uav_perception",
                executable="target_observation_node",
                name="target_observation_node",
                output="screen",
                parameters=[
                    {"target_revision": revision},
                    {
                        "max_reprojection_rms_px": ParameterValue(
                            rms, value_type=float
                        )
                    },
                    {
                        "fusion_ema_alpha": ParameterValue(
                            ema, value_type=float
                        )
                    },
                ],
                remappings=[
                    ("/d_task/vehicle/telemetry", vehicle_topic),
                ],
            ),
        ]
    )
