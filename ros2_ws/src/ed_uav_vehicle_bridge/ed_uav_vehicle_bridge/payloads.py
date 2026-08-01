"""Typed UDP v1 payload codecs."""

from __future__ import annotations

import struct
from typing import Final

from .errors import ProtocolError, ProtocolErrorCode
from .models import (
    BootId,
    CarState,
    DTask,
    FaultFlag,
    MissionPhase,
    MissionSelectionValue,
    MissionStatusFlag,
    MissionStatusValue,
    QualityFlag,
    RouteEvent,
    SelectionId,
    TaskMode,
    TurnClass,
    VehicleTelemetryValue,
)


CAR_TELEMETRY: Final = struct.Struct("<BBBHHihhH")
TASK_SELECTION: Final = struct.Struct("<IIBB")  # +mode: 1=实飞, 2=模拟飞
TASK_SELECTION_LEGACY: Final = struct.Struct("<IIB")
MISSION_STATUS: Final = struct.Struct("<IIIBBHH")
KNOWN_MISSION_STATUS_FLAGS: Final = int(
    MissionStatusFlag.DRONE_LINK_OK
    | MissionStatusFlag.DRONE_ARMED
    | MissionStatusFlag.VISION_VALID
    | MissionStatusFlag.ROS_READY
)


def encode_car_telemetry(value: VehicleTelemetryValue) -> bytes:
    _require_uint8(int(value.state), "car state", 3)
    _require_uint8(int(value.turn), "turn", 2)
    _require_uint8(int(value.event), "route event", 5)
    _require_uint16(value.event_id, "event ID")
    _require_uint16(int(value.quality_flags), "quality flags")
    _require_int32(value.displacement_mm, "displacement")
    _require_int16(value.velocity_mm_s, "velocity")
    _require_int16(value.line_error_milli, "line error")
    _require_uint16(int(value.fault_flags), "fault flags")
    return CAR_TELEMETRY.pack(
        int(value.state),
        int(value.turn),
        int(value.event),
        value.event_id,
        int(value.quality_flags),
        value.displacement_mm,
        value.velocity_mm_s,
        value.line_error_milli,
        int(value.fault_flags),
    )


def decode_car_telemetry(payload: bytes) -> VehicleTelemetryValue:
    values = _unpack_exact(payload, CAR_TELEMETRY)
    state, turn, event, event_id, quality_flags, displacement, velocity, line_error, fault_flags = values
    try:
        return VehicleTelemetryValue(
            state=CarState(state),
            turn=TurnClass(turn),
            event=RouteEvent(event),
            event_id=event_id,
            quality_flags=QualityFlag(quality_flags),
            displacement_mm=displacement,
            velocity_mm_s=velocity,
            line_error_milli=line_error,
            fault_flags=FaultFlag(fault_flags),
        )
    except ValueError as error:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "unknown telemetry enum") from error


def encode_task_selection(value: MissionSelectionValue) -> bytes:
    _require_uint32(value.selection_id, "selection ID")
    _require_uint32(value.car_boot_id, "car boot ID")
    _require_uint8(int(value.task), "task", 3)
    if int(value.task) not in (1, 2, 3):
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "task must be 1, 2, or 3")
    return TASK_SELECTION.pack(
        value.selection_id, value.car_boot_id, int(value.task), int(value.mode)
    )


def decode_task_selection(payload: bytes) -> MissionSelectionValue:
    # 兼容旧帧: 12B (无 mode) 按实飞处理
    if len(payload) == TASK_SELECTION_LEGACY.size:
        selection_id, car_boot_id, raw_task = _unpack_exact(payload, TASK_SELECTION_LEGACY)
        raw_mode = int(TaskMode.REAL)
    else:
        selection_id, car_boot_id, raw_task, raw_mode = _unpack_exact(payload, TASK_SELECTION)
    if raw_task not in (1, 2, 3):
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "task must be 1, 2, or 3")
    if raw_mode not in (int(TaskMode.REAL), int(TaskMode.SIMULATED)):
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "mode must be 1 or 2")
    return MissionSelectionValue(
        selection_id=SelectionId(selection_id),
        car_boot_id=BootId(car_boot_id),
        task=DTask(raw_task),
        mode=TaskMode(raw_mode),
    )


def encode_mission_status(value: MissionStatusValue) -> bytes:
    _require_uint32(value.selection_id, "selection ID")
    _require_uint32(value.car_boot_id, "car boot ID")
    _require_uint32(value.hmi_boot_id, "HMI boot ID")
    _require_uint8(int(value.phase), "mission phase", 5)
    _require_uint8(value.selected_task, "selected task", 3)
    _require_uint16(value.reason_flags, "reason flags")
    _require_uint16(int(value.status_flags), "status flags")
    if int(value.status_flags) & ~KNOWN_MISSION_STATUS_FLAGS:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "unknown mission status flags")
    return MISSION_STATUS.pack(
        value.selection_id,
        value.car_boot_id,
        value.hmi_boot_id,
        int(value.phase),
        value.selected_task,
        value.reason_flags,
        int(value.status_flags),
    )


def decode_mission_status(payload: bytes) -> MissionStatusValue:
    selection_id, car_boot_id, hmi_boot_id, raw_phase, selected_task, reason_flags, status_flags = _unpack_exact(
        payload, MISSION_STATUS
    )
    if selected_task > 3 or status_flags & ~KNOWN_MISSION_STATUS_FLAGS:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "invalid mission status flags")
    try:
        phase = MissionPhase(raw_phase)
    except ValueError as error:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "unknown mission phase") from error
    return MissionStatusValue(
        selection_id=SelectionId(selection_id),
        car_boot_id=BootId(car_boot_id),
        hmi_boot_id=BootId(hmi_boot_id),
        phase=phase,
        selected_task=selected_task,
        reason_flags=reason_flags,
        status_flags=MissionStatusFlag(status_flags),
    )


def _unpack_exact(payload: bytes, shape: struct.Struct) -> tuple[int, ...]:
    if len(payload) != shape.size:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, "payload length does not match fixed width")
    return shape.unpack(payload)


def _require_uint8(value: int, field: str, maximum: int) -> None:
    if not 0 <= value <= maximum:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, f"{field} is outside contract")


def _require_uint16(value: int, field: str) -> None:
    if not 0 <= value <= 0xFFFF:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, f"{field} is outside contract")


def _require_uint32(value: int, field: str) -> None:
    if not 0 <= value <= 0xFFFFFFFF:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, f"{field} is outside contract")


def _require_int16(value: int, field: str) -> None:
    if not -0x8000 <= value <= 0x7FFF:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, f"{field} is outside contract")


def _require_int32(value: int, field: str) -> None:
    if not -0x80000000 <= value <= 0x7FFFFFFF:
        raise ProtocolError(ProtocolErrorCode.BAD_PAYLOAD, f"{field} is outside contract")
