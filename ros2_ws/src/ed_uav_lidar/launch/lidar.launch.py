"""Optional Mid-360 or generic lidar launch guarded by field configuration."""

from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ed_uav_lidar.config import ConfigurationError, normalize_config
from ed_uav_lidar.launch_plan import build_launch_plan


def _materialize(context: LaunchContext) -> list[LogInfo | Node | RegisterEventHandler]:
    values = {
        key: LaunchConfiguration(key).perform(context)
        for key in (
            "lidar_enabled",
            "transport",
            "serial_number",
            "sensor_ip",
            "firmware_version",
            "time_authority",
            "driver_config_path",
        )
    }
    try:
        config = normalize_config(values)
    except ConfigurationError as error:
        return [LogInfo(msg=f"LIDAR_CONFIGURATION_RED: {error}")]
    plan = build_launch_plan(config)
    if not plan.nodes:
        return [LogInfo(msg=plan.code)]
    actions: list[LogInfo | Node | RegisterEventHandler] = [LogInfo(msg=plan.code)]
    driver_action: Node | None = None
    for node in plan.nodes:
        action = Node(
            package=node.package,
            executable=node.executable,
            output="screen",
            parameters=[dict(node.parameters)],
        )
        actions.append(action)
        if node.package == "livox_ros_driver2":
            driver_action = action
    if driver_action is not None:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=driver_action,
                    on_exit=[LogInfo(msg="LIDAR_DRIVER_DEAD")],
                )
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Create a camera-safe optional transport launch description."""
    share_directory = get_package_share_directory("ed_uav_lidar")
    defaults = {
        "lidar_enabled": "false",
        "transport": "disabled",
        "serial_number": "UNSET",
        "sensor_ip": "0.0.0.0",
        "firmware_version": "UNSET",
        "time_authority": "host",
        "driver_config_path": f"{share_directory}/config/mid360_driver.json",
    }
    arguments = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    return LaunchDescription([*arguments, OpaqueFunction(function=_materialize)])
