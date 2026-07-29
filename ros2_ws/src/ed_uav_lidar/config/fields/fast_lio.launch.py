"""FAST-LIO launch for field MID-360 deployment.

This launch file is referenced by mid360_field_manifest.local.json and
starts fastlio_mapping with the project-standard mid360 config, remapping
output topics so the localization pipeline (lio_adapter, source_supervisor)
can consume them without collision.
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    fast_lio_share = Path(get_package_share_directory("fast_lio"))
    default_config_path = fast_lio_share / "config"

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")

    # Fix: MVS SDK 的 libusb 覆盖了系统版本，导致 PCL 符号查找失败
    system_lib = "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu"
    current_ld = os.environ.get("LD_LIBRARY_PATH", "")
    fixed_ld = f"{system_lib}:{current_ld}" if current_ld else system_lib

    return LaunchDescription(
        [
            SetEnvironmentVariable("LD_LIBRARY_PATH", fixed_ld),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("config_path", default_value=str(default_config_path)),
            DeclareLaunchArgument("config_file", default_value="mid360.yaml"),
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fastlio_mapping",
                output="screen",
                parameters=[
                    PathJoinSubstitution([config_path, config_file]),
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("/Odometry", "/fast_lio/odometry"),
                    ("/cloud_registered", "/fast_lio/cloud_registered"),
                    ("/Laser_map", "/fast_lio/laser_map"),
                    ("/path", "/fast_lio/path"),
                ],
            ),
        ]
    )
