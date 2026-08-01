"""Field-test launch — standalone target-tracking validation without FCU.

Launches a narrow camera and the field-test node that runs AprilTag
detection, PnP pose estimation, Kalman filtering, and simulated visual
servo control with a real-time display window.

Usage:
  ros2 launch ed_uav_mission field_test.launch.py
  ros2 launch ed_uav_mission field_test.launch.py camera_plan:=/path/to/plan.json
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_actions(context):
    """Build the field-test node graph."""
    camera_share = Path(get_package_share_directory("ed_uav_camera"))

    camera_plan = LaunchConfiguration("camera_plan").perform(context)
    tag_size_m = LaunchConfiguration("tag_size_m").perform(context)
    tag_family = LaunchConfiguration("tag_family").perform(context)
    target_tag_id = LaunchConfiguration("target_tag_id").perform(context)
    max_display_width = LaunchConfiguration("max_display_width").perform(context)
    camera_yaw_offset = LaunchConfiguration("camera_yaw_offset_rad").perform(context)
    wide_camera_yaw_offset = LaunchConfiguration("wide_camera_yaw_offset_rad").perform(context)
    use_direct_capture = LaunchConfiguration("use_direct_capture").perform(context)
    camera_device = LaunchConfiguration("camera_device").perform(context)
    wide_camera_device = LaunchConfiguration("wide_camera_device").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)

    actions = []

    # 1. Camera — dual UVC (skip if direct capture mode)
    if use_direct_capture != "true":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(camera_share / "launch" / "dual_uvc.launch.py")
                ),
                launch_arguments={"camera_plan": camera_plan}.items(),
            )
        )

    # 2. Field test node
    actions.append(
        Node(
            package="ed_uav_mission",
            executable="field_test",
            name="field_test",
            output="screen",
            parameters=[
                {
                    "tag_size_m": float(tag_size_m),
                    "tag_family": tag_family,
                    "target_tag_id": int(target_tag_id),
                    "max_display_width": int(max_display_width),
                    "headless_log_interval_sec": 2.0,
                    "log_interval_sec": 1.0,
                    "camera_yaw_offset_rad": float(camera_yaw_offset),
                    "wide_camera_yaw_offset_rad": float(wide_camera_yaw_offset),
                    "use_direct_capture": use_direct_capture == "true",
                    "camera_device": camera_device,
                    "wide_camera_device": wide_camera_device,
                    "odom_topic": odom_topic,
                }
            ],
        )
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_plan",
                description="Camera runtime plan JSON (required)",
            ),
            DeclareLaunchArgument(
                "tag_size_m",
                default_value="0.15",
                description="AprilTag edge length in metres",
            ),
            DeclareLaunchArgument(
                "tag_family",
                default_value="tag36h11",
                description="AprilTag family name",
            ),
            DeclareLaunchArgument(
                "target_tag_id",
                default_value="-1",
                description="Target tag ID (-1 = detect any)",
            ),
            DeclareLaunchArgument(
                "max_display_width",
                default_value="960",
                description="Max display window width (px)",
            ),
            DeclareLaunchArgument(
                "camera_yaw_offset_rad",
                default_value="-1.5708",
                description="Narrow camera mounting yaw offset (default=-π/2)",
            ),
            DeclareLaunchArgument(
                "wide_camera_yaw_offset_rad",
                default_value="1.5708",
                description="Wide camera mounting yaw offset (default=+π/2)",
            ),
            DeclareLaunchArgument(
                "use_direct_capture",
                default_value="false",
                description="Use direct OpenCV capture for MJPG cameras (bypass v4l2_camera)",
            ),
            DeclareLaunchArgument(
                "camera_device",
                default_value="/dev/video2",
                description="Narrow camera device path for direct capture mode",
            ),
            DeclareLaunchArgument(
                "wide_camera_device",
                default_value="/dev/video0",
                description="Wide camera device path for direct capture mode",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="",
                description="Odometry topic (empty = auto-detect from /localization/odom, /localization/lio/odom, etc.)",
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
