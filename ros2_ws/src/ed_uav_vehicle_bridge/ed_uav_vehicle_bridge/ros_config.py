"""ROS parameter declaration for fail-closed bridge provisioning."""

from pathlib import Path

from rclpy.node import Node

from .config import BridgeProvisioning
from .models import Endpoint


def declare_bridge_provisioning(node: Node) -> BridgeProvisioning:
    defaults = {
        "bind_host": "0.0.0.0",
        "bind_port": 0,
        "car_peer_host": "",
        "car_peer_port": 0,
        "hmi_peer_host": "",
        "hmi_peer_port": 0,
        "car_sender_id": "",
        "hmi_sender_id": "",
        "bridge_sender_id": "",
        "hmac_key_file": "",
        "mission_timeout_seconds": 90.0,
        "telemetry_stale_seconds": 0.75,
    }
    for name, default in defaults.items():
        node.declare_parameter(name, default)

    def value(name: str):
        return node.get_parameter(name).value

    return BridgeProvisioning(
        bind=Endpoint(str(value("bind_host")), int(value("bind_port"))),
        car_peer=Endpoint(str(value("car_peer_host")), int(value("car_peer_port"))),
        hmi_peer=Endpoint(str(value("hmi_peer_host")), int(value("hmi_peer_port"))),
        car_sender_id=str(value("car_sender_id")),
        hmi_sender_id=str(value("hmi_sender_id")),
        bridge_sender_id=str(value("bridge_sender_id")),
        hmac_key_file=Path(str(value("hmac_key_file"))),
        mission_timeout_seconds=float(value("mission_timeout_seconds")),
        telemetry_stale_seconds=float(value("telemetry_stale_seconds")),
    )
