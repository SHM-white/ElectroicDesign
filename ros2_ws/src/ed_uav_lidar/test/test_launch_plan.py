"""Launch-plan tests independent of ROS and Livox installation."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_lidar.config import normalize_config
from ed_uav_lidar.launch_plan import build_launch_plan


def test_mid360_launch_keeps_fastlio_custom_topic_direct() -> None:
    # Given: a field-complete Mid-360 configuration.
    config = normalize_config(
        {
            "lidar_enabled": True,
            "transport": "mid360",
            "serial_number": "MID360-EXAMPLE",
            "sensor_ip": "192.168.1.12",
            "firmware_version": "FIELD-VERIFY",
            "time_authority": "host",
            "driver_config_path": "/etc/ed_uav/mid360-field.json",
        }
    )

    # When: the ROS-independent plan is generated.
    plan = build_launch_plan(config)

    # Then: the vendor driver feeds FAST-LIO directly and the monitor is a side branch.
    assert plan.code == "HOST_TIME_UNVERIFIED"
    assert plan.fastlio_custom_topic == "/livox/lidar"
    assert tuple(node.package for node in plan.nodes) == (
        "livox_ros_driver2",
        "ed_uav_lidar",
    )


def test_generic_launch_never_requires_livox() -> None:
    # Given: an enabled generic PointCloud2 configuration.
    config = normalize_config({"lidar_enabled": True, "transport": "generic"})

    # When: its launch plan is generated.
    plan = build_launch_plan(config)

    # Then: only the project-owned generic monitor is required.
    assert tuple(node.package for node in plan.nodes) == ("ed_uav_lidar",)
