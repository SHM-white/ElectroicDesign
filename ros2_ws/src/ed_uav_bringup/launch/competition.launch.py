"""Full competition mode — calibration-gated with strict sensor serial matching.

This profile REFUSES to activate when the calibration status is not CALIBRATED
or when any sensor serial does not match the expected hardware identity.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ed_uav_description.calibration import ExpectedSerials, load_calibration, validate_for_profile
from ed_uav_description.model import render_robot_description

PROFILE = "competition"
LIFECYCLE_ORDER = ("calibration_gate", "robot_state_publisher", "hardware_owners", "localization")


def _build_actions(context) -> list:
    calibration_path = Path(LaunchConfiguration("calibration_file").perform(context))
    calibration = load_calibration(calibration_path)
    validate_for_profile(
        calibration,
        PROFILE,
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
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    description_share = Path(get_package_share_directory("ed_uav_description"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration_file",
                default_value="",
                description="Path to a CALIBRATED calibration YAML — required for competition.",
            ),
            DeclareLaunchArgument(
                "camera_narrow_serial",
                default_value="",
                description="Expected narrow-camera serial — must match calibration.",
            ),
            DeclareLaunchArgument(
                "camera_wide_serial",
                default_value="",
                description="Expected wide-camera serial — must match calibration.",
            ),
            DeclareLaunchArgument(
                "lidar_serial",
                default_value="",
                description="Expected lidar serial — must match calibration.",
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_build_actions),
        ]
    )
