"""Offline replay from rosbag2 — no live hardware required."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ed_uav_description.calibration import ExpectedSerials, load_calibration, validate_for_profile
from ed_uav_description.model import render_robot_description

PROFILE = "offline"
LIFECYCLE_ORDER = ("calibration_gate", "robot_state_publisher", "replay")


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
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "play",
                LaunchConfiguration("bag_path"),
                "--rate", LaunchConfiguration("bag_rate"),
            ],
            output="screen",
            name="rosbag_replay",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    description_share = Path(get_package_share_directory("ed_uav_description"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="Absolute path to rosbag2 directory for replay.",
            ),
            DeclareLaunchArgument(
                "bag_rate",
                default_value="1.0",
                description="Replay rate multiplier (1.0 = real-time).",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=str(description_share / "config" / "example_uncalibrated.yaml"),
            ),
            DeclareLaunchArgument("camera_narrow_serial", default_value="UNSET"),
            DeclareLaunchArgument("camera_wide_serial", default_value="UNSET"),
            DeclareLaunchArgument("lidar_serial", default_value="UNSET"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "authority_token",
                default_value="ed-uav-offline-replay",
                description="Single control-authority token for this profile.",
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
