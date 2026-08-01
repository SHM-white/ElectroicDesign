"""ROS parameter declaration for fail-closed bridge provisioning."""

from pathlib import Path

from rclpy.node import Node

from .config import BridgeProvisioning
from .models import Endpoint, SenderId


def declare_bridge_provisioning(node: Node) -> BridgeProvisioning:
    defaults = {
        "bind_host": "0.0.0.0",
        "bind_port": 0,
        "car_peer_host": "",
        "car_peer_port": 0,
        "hmi_peer_host": "",
        "hmi_peer_port": 0,
        "car_sender_id": 0x43415231,
        "hmi_sender_id": 0x484D4931,
        "bridge_sender_id": 0x524F5331,
        "hmac_key_file": "",
        "mission_timeout_seconds": 90.0,
        "telemetry_stale_seconds": 0.75,
        "task3_flight_test_mode": False,
        "no_car_mode": False,
        "task3_mission_id": "",
        "task3_field_profile_id": "",
        "task3_mission_profile_id": "",
        "task3_deployment_preset_id": "",
        "task3_target_revision": "",
        "task3_timeout_seconds": 120.0,
    }
    for name, default in defaults.items():
        node.declare_parameter(name, default)

    def value(name: str):
        return node.get_parameter(name).value

    return BridgeProvisioning(
        bind=Endpoint(str(value("bind_host")), int(value("bind_port"))),
        car_peer=Endpoint(str(value("car_peer_host")), int(value("car_peer_port"))),
        hmi_peer=Endpoint(str(value("hmi_peer_host")), int(value("hmi_peer_port"))),
        car_sender_id=SenderId(int(value("car_sender_id"))),
        hmi_sender_id=SenderId(int(value("hmi_sender_id"))),
        bridge_sender_id=SenderId(int(value("bridge_sender_id"))),
        hmac_key_file=Path(str(value("hmac_key_file"))),
        mission_timeout_seconds=float(value("mission_timeout_seconds")),
        telemetry_stale_seconds=float(value("telemetry_stale_seconds")),
    )
