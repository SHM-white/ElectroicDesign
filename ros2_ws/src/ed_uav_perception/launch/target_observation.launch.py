"""Launch the prescribed target observer with dual-camera Kalman fusion."""

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
    kn_pos = LaunchConfiguration("kalman_process_noise_pos")
    kn_vel = LaunchConfiguration("kalman_process_noise_vel")
    kn_age = LaunchConfiguration("kalman_max_predict_age_sec")
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
            DeclareLaunchArgument(
                "kalman_process_noise_pos", default_value="0.05"
            ),
            DeclareLaunchArgument(
                "kalman_process_noise_vel", default_value="0.3"
            ),
            DeclareLaunchArgument(
                "kalman_max_predict_age_sec", default_value="0.5"
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
                    {
                        "kalman_process_noise_pos": ParameterValue(
                            kn_pos, value_type=float
                        )
                    },
                    {
                        "kalman_process_noise_vel": ParameterValue(
                            kn_vel, value_type=float
                        )
                    },
                    {
                        "kalman_max_predict_age_sec": ParameterValue(
                            kn_age, value_type=float
                        )
                    },
                ],
                remappings=[
                    ("/d_task/vehicle/telemetry", vehicle_topic),
                ],
            ),
        ]
    )
