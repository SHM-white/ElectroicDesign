"""Launch the deterministic offline integration surface."""

from __future__ import annotations

import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WALL_TIME_ERROR = (
    "live deterministic publisher has no /clock and requires use_sim_time=false"
)


def _build_actions(context: LaunchContext) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    if use_sim_time:
        raise RuntimeError(WALL_TIME_ERROR)

    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    if use_rviz and shutil.which("rviz2") is None:
        raise RuntimeError("use_rviz=true requested but rviz2 is unavailable")

    bringup_share = Path(get_package_share_directory("ed_uav_bringup"))
    verification_share = Path(get_package_share_directory("ed_uav_verification"))
    actions: list[Action] = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / "launch" / "bringup.launch.py")
            ),
            launch_arguments={
                "profile": "offline",
                "calibration_file": LaunchConfiguration("calibration_file"),
                "camera_narrow_serial": LaunchConfiguration("camera_narrow_serial"),
                "camera_wide_serial": LaunchConfiguration("camera_wide_serial"),
                "lidar_serial": LaunchConfiguration("lidar_serial"),
                "namespace": LaunchConfiguration("namespace"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(verification_share / "launch" / "verification_harness.launch.py")
            ),
            launch_arguments={
                "seed": LaunchConfiguration("seed"),
                "duration_seconds": LaunchConfiguration("duration_seconds"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
    ]
    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="offline_integration_rviz",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                output="screen",
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Expose deterministic offline publishing, static TF, and optional RViz."""
    description_share = Path(get_package_share_directory("ed_uav_description"))
    bringup_share = Path(get_package_share_directory("ed_uav_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("seed", default_value="7"),
            DeclareLaunchArgument("duration_seconds", default_value="60"),
            DeclareLaunchArgument("rate_hz", default_value="20"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(bringup_share / "rviz" / "offline_integration.rviz"),
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=str(
                    description_share / "config" / "synthetic_calibrated.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "camera_narrow_serial", default_value="SYNTHETIC-NARROW-001"
            ),
            DeclareLaunchArgument(
                "camera_wide_serial", default_value="SYNTHETIC-WIDE-001"
            ),
            DeclareLaunchArgument("lidar_serial", default_value="SYNTHETIC-LIDAR-001"),
            DeclareLaunchArgument("namespace", default_value=""),
            OpaqueFunction(function=_build_actions),
        ]
    )
