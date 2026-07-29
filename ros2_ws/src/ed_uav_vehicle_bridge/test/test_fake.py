from ed_uav_vehicle_bridge.fake import FakeVehicleConfig, build_fake_vehicle_datagrams
from ed_uav_vehicle_bridge.models import (
    BootEpoch,
    Endpoint,
    MessageType,
    RouteStage,
    Sequence,
)
from ed_uav_vehicle_bridge.payloads import decode_vehicle_telemetry
from ed_uav_vehicle_bridge.protocol import decode_datagram


KEY = bytes(range(32))


def test_fake_source_is_deterministic_and_emits_one_start() -> None:
    config = FakeVehicleConfig(
        destination=Endpoint("127.0.0.1", 40100),
        source=Endpoint("127.0.0.1", 40101),
        sender_id="CAR-01",
        boot_epoch=BootEpoch(0x1234),
        frame_count=5,
    )

    first = build_fake_vehicle_datagrams(config, KEY)
    second = build_fake_vehicle_datagrams(config, KEY)
    decoded = [decode_datagram(packet, KEY) for packet in first]
    telemetry = [decode_vehicle_telemetry(packet.frame.payload) for packet in decoded]

    assert first == second
    assert [packet.frame.sequence for packet in decoded] == [Sequence(index) for index in range(5)]
    assert all(packet.frame.message_type is MessageType.CAR_TELEMETRY for packet in decoded)
    assert sum(value.start_event for value in telemetry) == 1
    assert all(value.route_stage is RouteStage.START for value in telemetry)
