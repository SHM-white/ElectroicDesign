"""Finite deterministic fake-car UDP process for ROS integration."""

from pathlib import Path

import rclpy
from rclpy.node import Node

from .config import load_hmac_key_file
from .errors import BridgeConfigError
from .fake import FakeVehicleConfig, build_fake_vehicle_datagrams
from .models import BootEpoch, Endpoint
from .udp_socket import BoundUdpSocket


class FakeVehicleSourceNode(Node):
    """Send a finite 20 Hz signed telemetry sequence and exit cleanly."""

    def __init__(self) -> None:
        super().__init__("fake_vehicle_source")
        defaults = {
            "destination_host": "",
            "destination_port": 0,
            "source_host": "",
            "source_port": 0,
            "sender_id": "",
            "boot_epoch": 0,
            "frame_count": 5,
            "hmac_key_file": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def value(name: str):
            return self.get_parameter(name).value

        config = FakeVehicleConfig(
            destination=Endpoint(
                str(value("destination_host")), int(value("destination_port"))
            ),
            source=Endpoint(str(value("source_host")), int(value("source_port"))),
            sender_id=str(value("sender_id")),
            boot_epoch=BootEpoch(int(value("boot_epoch"))),
            frame_count=int(value("frame_count")),
        )
        if config.destination.port <= 0 or config.source.port <= 0:
            raise BridgeConfigError("fake UDP ports", "must be explicitly provisioned")
        key = load_hmac_key_file(Path(str(value("hmac_key_file"))))
        self._packets = build_fake_vehicle_datagrams(config, key)
        self._destination = config.destination
        self._socket = BoundUdpSocket(config.source)
        self._index = 0
        self.done = False
        self._timer = self.create_timer(0.05, self._send_next)

    def _send_next(self) -> None:
        if self._index >= len(self._packets):
            self.done = True
            self._timer.cancel()
            self.get_logger().info("fake_vehicle_source.complete")
            return
        self._socket.send(self._packets[self._index], self._destination)
        self._index += 1

    def destroy_node(self) -> None:
        self._socket.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakeVehicleSourceNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
