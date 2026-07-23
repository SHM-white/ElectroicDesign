"""ROS-independent launch planning for optional lidar transport."""

from __future__ import annotations

from dataclasses import dataclass

from .config import LidarConfig, Transport


@dataclass(frozen=True, slots=True)
class NodeSpec:
    package: str
    executable: str
    parameters: tuple[tuple[str, str | int | float], ...]


@dataclass(frozen=True, slots=True)
class LaunchPlan:
    code: str
    nodes: tuple[NodeSpec, ...]
    fastlio_custom_topic: str


def build_launch_plan(config: LidarConfig) -> LaunchPlan:
    """Return the exact optional driver and adapter process plan for one mode."""
    if not config.enabled:
        return LaunchPlan(code="LIDAR_DISABLED", nodes=(), fastlio_custom_topic="")
    if not config.field_check.ready:
        return LaunchPlan(
            code=config.field_check.code,
            nodes=(),
            fastlio_custom_topic=config.fastlio_custom_topic,
        )
    match config.transport:
        case Transport.GENERIC:
            return LaunchPlan(
                code=config.time_status,
                nodes=(
                    NodeSpec(
                        package="ed_uav_lidar",
                        executable="generic_monitor",
                        parameters=(
                            ("input_topic", config.generic_input_topic),
                            ("monitoring_topic", config.monitoring_topic),
                        ),
                    ),
                ),
                fastlio_custom_topic="",
            )
        case Transport.MID360:
            return LaunchPlan(
                code=config.time_status,
                nodes=(
                    NodeSpec(
                        package="livox_ros_driver2",
                        executable="livox_ros_driver2_node",
                        parameters=(
                            ("xfer_format", 1),
                            ("multi_topic", 0),
                            ("data_src", 0),
                            ("publish_freq", 10.0),
                            ("output_data_type", 0),
                            ("frame_id", "lidar_link"),
                            ("user_config_path", config.driver_config_path),
                        ),
                    ),
                    NodeSpec(
                        package="ed_uav_lidar",
                        executable="mid360_adapter",
                        parameters=(
                            ("custom_topic", config.fastlio_custom_topic),
                            ("monitoring_topic", config.monitoring_topic),
                            ("imu_topic", config.imu_topic),
                        ),
                    ),
                ),
                fastlio_custom_topic=config.fastlio_custom_topic,
            )
        case Transport.DISABLED:
            return LaunchPlan(code="LIDAR_DISABLED", nodes=(), fastlio_custom_topic="")
