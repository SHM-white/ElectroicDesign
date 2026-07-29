import socket
import select

from ed_uav_vehicle_bridge.models import Endpoint
from ed_uav_vehicle_bridge.udp_socket import BoundUdpSocket


def test_real_loopback_udp_receive_is_bounded_and_socket_closes() -> None:
    # Given: a real bridge-owned loopback UDP socket on an ephemeral port.
    with BoundUdpSocket(Endpoint("127.0.0.1", 0)) as receiver:
        descriptor = receiver.fileno()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(b"first", (receiver.endpoint.host, receiver.endpoint.port))
            sender.sendto(b"second", (receiver.endpoint.host, receiver.endpoint.port))

        # When: one bounded receive tick is processed.
        readable, _, _ = select.select([receiver.fileno()], [], [], 1.0)
        assert readable == [receiver.fileno()]
        first_tick = receiver.receive(maximum_packets=1)
        second_tick = receiver.receive(maximum_packets=1)

        # Then: exactly one packet is consumed per tick without blocking.
        assert tuple(packet.data for packet in first_tick) == (b"first",)
        assert tuple(packet.data for packet in second_tick) == (b"second",)

    # Then: context teardown closes the owned descriptor deterministically.
    assert receiver.fileno() == -1
    assert descriptor >= 0


def test_send_uses_configured_unicast_destination() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as capture:
        capture.bind(("127.0.0.1", 0))
        capture.settimeout(1.0)
        host, port = capture.getsockname()
        with BoundUdpSocket(Endpoint("127.0.0.1", 0)) as sender:
            sender.send(b"status", Endpoint(str(host), int(port)))
        data, source = capture.recvfrom(64)

    assert data == b"status"
    assert source[0] == "127.0.0.1"
