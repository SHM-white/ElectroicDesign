"""Typed UDP v1 payload codecs."""

from __future__ import annotations

import math
import struct
from typing import Final

from .errors import ProtocolError, ProtocolErrorCode
from .models import (
    AuthorityState,
    BootEpoch,
    DTask,
    MissionSelectionValue,
    MissionStatusValue,
    MotionKind,
    RouteStage,
    SelectionAckValue,
    SelectionId,
    Sequence,
    TurnClass,
    VehicleTelemetryValue,
)


TELEMETRY_FIXED: Final = struct.Struct(">HBBffffBB")
SELECTION_FIXED: Final = struct.Struct(">HQQB")
ACK_FIXED: Final = struct.Struct(">HQQBBB")
STATUS_FIXED: Final = struct.Struct(">HIBBB")
AUTHORITY_STATES: Final = tuple(AuthorityState)


def encode_vehicle_telemetry(value: VehicleTelemetryValue) -> bytes:
    _require_contract_version(value.contract_version)
    if not math.isfinite(value.displacement_m) or not math.isfinite(value.wheel_speed_m_s):
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "telemetry floats must be finite")
    _require_vehicle_motion(value.heading_rad, value.yaw_rate_rad_s, value.turn_class)
    flags = (
        int(value.start_event)
        | (int(value.heartbeat_alive) << 1)
        | (int(value.lap_complete) << 2)
    )
    return TELEMETRY_FIXED.pack(
        value.contract_version,
        flags,
        int(value.motion_kind),
        value.displacement_m,
        value.wheel_speed_m_s,
        value.heading_rad,
        value.yaw_rate_rad_s,
        int(value.turn_class),
        int(value.route_stage),
    ) + _encode_string(value.vehicle_id, 32) + _encode_string(value.frame_id, 32)


def decode_vehicle_telemetry(payload: bytes) -> VehicleTelemetryValue:
    reader = _PayloadReader(payload)
    (
        version,
        flags,
        raw_motion,
        displacement,
        speed,
        heading,
        yaw_rate,
        raw_turn,
        raw_route,
    ) = reader.unpack(TELEMETRY_FIXED)
    _require_contract_version(version)
    if flags & ~0x07:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "unknown telemetry flags")
    try:
        motion = MotionKind(raw_motion)
        turn = TurnClass(raw_turn)
        route = RouteStage(raw_route)
    except ValueError as error:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "unknown telemetry enum") from error
    if not math.isfinite(displacement) or not math.isfinite(speed):
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "telemetry floats must be finite")
    _require_vehicle_motion(heading, yaw_rate, turn)
    value = VehicleTelemetryValue(
        contract_version=version,
        vehicle_id=reader.string(32),
        start_event=bool(flags & 0x01),
        heartbeat_alive=bool(flags & 0x02),
        motion_kind=motion,
        displacement_m=displacement,
        wheel_speed_m_s=speed,
        heading_rad=heading,
        yaw_rate_rad_s=yaw_rate,
        turn_class=turn,
        route_stage=route,
        lap_complete=bool(flags & 0x04),
        frame_id=reader.string(32),
    )
    reader.finish()
    return value


def _require_vehicle_motion(
    heading_rad: float, yaw_rate_rad_s: float, turn_class: TurnClass
) -> None:
    if not math.isfinite(heading_rad) or not math.isfinite(yaw_rate_rad_s):
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "vehicle motion must be finite")
    if not -math.pi <= heading_rad <= math.pi or abs(yaw_rate_rad_s) > 20.0:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "vehicle motion is out of range")
    if turn_class is TurnClass.STRAIGHT and abs(yaw_rate_rad_s) > 0.15:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "invalid straight turn yaw rate")
    if turn_class is not TurnClass.STRAIGHT and abs(yaw_rate_rad_s) < 0.01:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "turn requires signed yaw rate")


def encode_mission_selection(value: MissionSelectionValue) -> bytes:
    _require_contract_version(value.contract_version)
    return SELECTION_FIXED.pack(
        value.contract_version,
        value.selection_id,
        value.car_boot_epoch,
        int(value.task),
    ) + b"".join(
        (
            _encode_string(value.mission_id, 64),
            _encode_string(value.mission_profile_id, 64),
            _encode_string(value.deployment_preset_id, 64),
            _encode_string(value.target_revision, 32),
        )
    )


def decode_mission_selection(payload: bytes) -> MissionSelectionValue:
    reader = _PayloadReader(payload)
    version, selection_id, car_epoch, raw_task = reader.unpack(SELECTION_FIXED)
    _require_contract_version(version)
    if selection_id == 0 or car_epoch == 0:
        raise ProtocolError(
            ProtocolErrorCode.BAD_PAYLOAD,
            "selection ID and car epoch must be nonzero",
        )
    try:
        task = DTask(raw_task)
    except ValueError as error:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "unknown D-task") from error
    value = MissionSelectionValue(
        contract_version=version,
        selection_id=SelectionId(selection_id),
        car_boot_epoch=BootEpoch(car_epoch),
        mission_id=reader.string(64),
        mission_profile_id=reader.string(64),
        deployment_preset_id=reader.string(64),
        target_revision=reader.string(32),
        task=task,
    )
    reader.finish()
    return value


def encode_selection_ack(value: SelectionAckValue) -> bytes:
    _require_contract_version(value.contract_version)
    return ACK_FIXED.pack(
        value.contract_version,
        value.selection_id,
        value.car_boot_epoch,
        int(value.accepted),
        AUTHORITY_STATES.index(value.state),
        0,
    ) + _encode_string(value.reason, 96)


def decode_selection_ack(payload: bytes) -> SelectionAckValue:
    reader = _PayloadReader(payload)
    version, selection_id, car_epoch, accepted, state_index, reserved = reader.unpack(ACK_FIXED)
    _require_contract_version(version)
    if accepted not in (0, 1) or reserved != 0 or state_index >= len(AUTHORITY_STATES):
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "invalid acknowledgement flags")
    value = SelectionAckValue(
        contract_version=version,
        selection_id=SelectionId(selection_id),
        car_boot_epoch=BootEpoch(car_epoch),
        accepted=bool(accepted),
        state=AUTHORITY_STATES[state_index],
        reason=reader.string(96),
    )
    reader.finish()
    return value


def encode_mission_status(value: MissionStatusValue) -> bytes:
    _require_contract_version(value.contract_version)
    if not 0 <= value.state <= 10:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "mission state is outside contract")
    return STATUS_FIXED.pack(
        value.contract_version,
        value.source_sequence,
        value.state,
        int(value.route_stage),
        int(value.complete),
    ) + _encode_string(value.mission_id, 64) + _encode_string(value.reason, 96)


def decode_mission_status(payload: bytes) -> MissionStatusValue:
    reader = _PayloadReader(payload)
    version, sequence, state, raw_route, complete = reader.unpack(STATUS_FIXED)
    _require_contract_version(version)
    if state > 10 or complete not in (0, 1):
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "invalid mission status flags")
    try:
        route = RouteStage(raw_route)
    except ValueError as error:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "unknown route stage") from error
    value = MissionStatusValue(
        contract_version=version,
        source_sequence=Sequence(sequence),
        mission_id=reader.string(64),
        state=state,
        route_stage=route,
        complete=bool(complete),
        reason=reader.string(96),
    )
    reader.finish()
    return value


def _require_contract_version(version: int) -> None:
    if version != 1:
        raise ProtocolError(
            ProtocolErrorCode.BAD_PAYLOAD,
            f"unsupported contract version {version}",
        )


def _encode_string(value: str, maximum_bytes: int) -> bytes:
    try:
        encoded = value.encode("utf-8")
    except UnicodeEncodeError as error:
        raise ProtocolError(
            ProtocolErrorCode.BAD_PAYLOAD, "string is not valid UTF-8"
        ) from error
    if not encoded or len(encoded) > maximum_bytes:
        raise ProtocolError(
            ProtocolErrorCode.BAD_PAYLOAD,
            f"string must contain 1-{maximum_bytes} bytes",
        )
    return bytes((len(encoded),)) + encoded


class _PayloadReader:
    """Cursor that consumes each untrusted payload byte exactly once."""

    __slots__ = ("_payload", "_offset")

    def __init__(self, payload: bytes) -> None:
        self._payload = payload
        self._offset = 0

    def unpack(self, shape: struct.Struct) -> tuple[int | float, ...]:
        if self._offset + shape.size > len(self._payload):
            raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "payload ended inside fixed field")
        values = shape.unpack_from(self._payload, self._offset)
        self._offset += shape.size
        return values

    def string(self, maximum_bytes: int) -> str:
        if self._offset >= len(self._payload):
            raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "payload ended before string length")
        length = self._payload[self._offset]
        self._offset += 1
        end = self._offset + length
        if not 1 <= length <= maximum_bytes or end > len(self._payload):
            raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "bounded string length is invalid")
        raw = self._payload[self._offset:end]
        self._offset = end
        try:
            return raw.decode("utf-8")
        except UnicodeDecodeError as error:
            raise ProtocolError(
                ProtocolErrorCode.BAD_PAYLOAD, "string is not valid UTF-8"
            ) from error

    def finish(self) -> None:
        if self._offset != len(self._payload):
            raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "payload has trailing bytes")
