"""Launch the finite virtual-time ED UAV verification publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Expose the canonical seeded virtual 60-second/20Hz launch surface."""
    seed = LaunchConfiguration("seed")
    duration_seconds = LaunchConfiguration("duration_seconds")
    rate_hz = LaunchConfiguration("rate_hz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("seed", default_value="7"),
            DeclareLaunchArgument("duration_seconds", default_value="60"),
            DeclareLaunchArgument("rate_hz", default_value="20"),
            Node(
                package="ed_uav_verification",
                executable="ed-uav-verify-ros",
                arguments=["--seed", seed, "--duration-seconds", duration_seconds, "--rate-hz", rate_hz],
                output="screen",
            ),
        ]
    )
