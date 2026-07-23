"""Launch the static description after its profile-specific calibration gate."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ed_uav_description.calibration import ExpectedSerials, load_calibration, validate_for_profile
from ed_uav_description.model import render_robot_description


P06_PROFILES = ("offline", "camera_only", "lidar", "competition")
LIFECYCLE_ORDER = ("calibration_gate", "robot_state_publisher", "hardware_owners", "localization")


def _build_actions(context) -> list[Node]:
    profile = LaunchConfiguration("profile").perform(context)
    calibration_path = Path(LaunchConfiguration("calibration_file").perform(context))
    calibration = load_calibration(calibration_path)
    validate_for_profile(
        calibration,
        profile,
        ExpectedSerials(
            LaunchConfiguration("camera_narrow_serial").perform(context),
            LaunchConfiguration("camera_wide_serial").perform(context),
            LaunchConfiguration("lidar_serial").perform(context),
        ),
    )
    description_share = Path(get_package_share_directory("ed_uav_description"))
    robot_description = render_robot_description(calibration, description_share / "urdf" / "ed_uav.urdf.xacro")
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    description_share = Path(get_package_share_directory("ed_uav_description"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="offline", choices=P06_PROFILES),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=str(description_share / "config" / "example_uncalibrated.yaml"),
            ),
            DeclareLaunchArgument("camera_narrow_serial", default_value="UNSET"),
            DeclareLaunchArgument("camera_wide_serial", default_value="UNSET"),
            DeclareLaunchArgument("lidar_serial", default_value="UNSET"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_build_actions),
        ]
    )
