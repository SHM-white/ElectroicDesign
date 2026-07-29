"""Deterministic fake car datagrams for host and ROS integration tests."""

from dataclasses import dataclass

from .errors import BridgeConfigError
from .models import (
    BootEpoch,
    Endpoint,
    MessageType,
    MotionKind,
    OutboundFrame,
    RouteStage,
    Sequence,
    SourceMillis,
    TurnClass,
    VehicleTelemetryValue,
)
from .payloads import encode_vehicle_telemetry
from .protocol import encode_datagram


@dataclass(frozen=True, slots=True)
class FakeVehicleConfig:
    destination: Endpoint
    source: Endpoint
    sender_id: str
    boot_epoch: BootEpoch
    frame_count: int


def build_fake_vehicle_datagrams(
    config: FakeVehicleConfig, key: bytes
) -> tuple[bytes, ...]:
    if not 2 <= config.frame_count <= 1024:
        raise BridgeConfigError("frame_count", "must be in range 2-1024")
    packets: list[bytes] = []
    for index in range(config.frame_count):
        telemetry = VehicleTelemetryValue(
            contract_version=1,
            vehicle_id="fake-car",
            start_event=index == 1,
            heartbeat_alive=True,
            motion_kind=MotionKind.DISPLACEMENT,
            displacement_m=index * 0.01,
            wheel_speed_m_s=0.2 if index > 0 else 0.0,
            turn_class=TurnClass.STRAIGHT,
            route_stage=RouteStage.START,
            lap_complete=False,
            frame_id="vehicle_start",
        )
        frame = OutboundFrame(
            message_type=MessageType.CAR_TELEMETRY,
            sender_id=config.sender_id,
            boot_epoch=config.boot_epoch,
            sequence=Sequence(index),
            source_millis=SourceMillis(index * 50),
            payload=encode_vehicle_telemetry(telemetry),
        )
        packets.append(encode_datagram(frame, key))
    return tuple(packets)
