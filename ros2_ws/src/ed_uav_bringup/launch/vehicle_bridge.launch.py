"""Expose the dedicated authenticated vehicle bridge through bringup."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    bridge_share = Path(get_package_share_directory("ed_uav_vehicle_bridge"))
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bridge_share / "launch" / "vehicle_bridge.launch.py")
                )
            )
        ]
    )
