"""Deterministic fake car datagrams for host and ROS integration tests."""

from dataclasses import dataclass

from .errors import BridgeConfigError
from .models import (
    BootId,
    CarState,
    Endpoint,
    FaultFlag,
    MessageType,
    OutboundFrame,
    QualityFlag,
    RouteEvent,
    SenderId,
    Sequence,
    SourceMillis,
    TurnClass,
    VehicleTelemetryValue,
)
from .payloads import encode_car_telemetry
from .protocol import encode_datagram


@dataclass(frozen=True, slots=True)
class FakeVehicleConfig:
    destination: Endpoint
    source: Endpoint
    sender_id: SenderId
    boot_id: BootId
    frame_count: int


def build_fake_vehicle_datagrams(
    config: FakeVehicleConfig, key: bytes
) -> tuple[bytes, ...]:
    if not 2 <= config.frame_count <= 1024:
        raise BridgeConfigError("frame_count", "must be in range 2-1024")
    packets: list[bytes] = []
    for index in range(config.frame_count):
        telemetry = VehicleTelemetryValue(
            state=CarState.RUNNING if index > 0 else CarState.READY,
            turn=TurnClass.STRAIGHT,
            event=RouteEvent.START if index == 1 else RouteEvent.NONE,
            event_id=1 if index == 1 else 0,
            quality_flags=QualityFlag.LINE_VALID | QualityFlag.ENCODER_VALID,
            displacement_mm=int(index * 10),
            velocity_mm_s=200 if index > 0 else 0,
            line_error_milli=0,
            fault_flags=FaultFlag.NONE,
        )
        frame = OutboundFrame(
            message_type=MessageType.CAR_TELEMETRY,
            sender_id=config.sender_id,
            boot_id=config.boot_id,
            sequence=Sequence(index),
            source_millis=SourceMillis(index * 50),
            payload=encode_car_telemetry(telemetry),
        )
        packets.append(encode_datagram(frame, key))
    return tuple(packets)
