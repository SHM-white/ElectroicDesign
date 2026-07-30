"""Compose dual UVC cameras, target observation, and annotated-image RViz."""

from __future__ import annotations

import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _requested(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _build_actions(context: LaunchContext) -> list[Action]:
    """Build the camera, perception, and optional visualization actions."""
    use_rviz = _requested(LaunchConfiguration("use_rviz").perform(context))
    if use_rviz and shutil.which("rviz2") is None:
        raise RuntimeError("use_rviz=true requested but rviz2 is unavailable")

    camera_share = Path(get_package_share_directory("ed_uav_camera"))
    perception_share = Path(get_package_share_directory("ed_uav_perception"))
    actions: list[Action] = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(camera_share / "launch" / "dual_uvc.launch.py")
            ),
            launch_arguments={
                "camera_plan": LaunchConfiguration("camera_plan"),
                "profile_catalog": LaunchConfiguration("profile_catalog"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(perception_share / "launch" / "target_observation.launch.py")
            ),
            launch_arguments={
                "vehicle_topic": LaunchConfiguration("vehicle_topic"),
                "target_revision": LaunchConfiguration("target_revision"),
                "max_reprojection_rms_px": LaunchConfiguration(
                    "max_reprojection_rms_px"
                ),
            }.items(),
        ),
    ]
    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="landing_marker_recognition_rviz",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Expose the one-command landing-marker recognition display."""
    camera_share = Path(get_package_share_directory("ed_uav_camera"))
    bringup_share = Path(get_package_share_directory("ed_uav_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_plan", default_value=""),
            DeclareLaunchArgument(
                "profile_catalog",
                default_value=str(camera_share / "config" / "camera_profiles.yaml"),
            ),
            DeclareLaunchArgument(
                "vehicle_topic", default_value="/d_task/vehicle/telemetry"
            ),
            DeclareLaunchArgument(
                "target_revision", default_value="d2026-circle-cross-v1"
            ),
            DeclareLaunchArgument(
                "max_reprojection_rms_px", default_value="2.0"
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    bringup_share / "rviz" / "landing_marker_recognition.rviz"
                ),
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
