"""Typed values crossing the vehicle/HMI UDP boundary."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag, unique
from typing import NewType

from .string_enum import StringEnum


SenderId = NewType("SenderId", int)
BootId = NewType("BootId", int)
BootEpoch = BootId
ReceiptSeconds = NewType("ReceiptSeconds", float)
SelectionId = NewType("SelectionId", int)
Sequence = NewType("Sequence", int)
SourceMillis = NewType("SourceMillis", int)


@unique
class MessageType(IntEnum):
    HEARTBEAT = 1
    CAR_TELEMETRY = 2
    TASK_SELECTION = 3
    MISSION_STATUS = 4
    DIAGNOSTIC = 5


@unique
class CarState(IntEnum):
    READY = 0
    RUNNING = 1
    COMPLETE = 2
    SAFE_STOP = 3


@unique
class TurnClass(IntEnum):
    STRAIGHT = 0
    SMALL = 1
    LARGE = 2


@unique
class RouteEvent(IntEnum):
    NONE = 0
    START = 1
    B = 2
    D = 3
    A = 4
    COMPLETE = 5


@unique
class MissionPhase(IntEnum):
    PRESTART = 0
    SELECTION_ACKED = 1
    ARMED_READY = 2
    CAR_RUNNING = 3
    COMPLETE = 4
    FAULT = 5


@unique
class QualityFlag(IntFlag):
    LINE_VALID = 1 << 0
    ENCODER_VALID = 1 << 1
    WIFI_CONNECTED = 1 << 2
    SELECTION_COMMITTED = 1 << 3


@unique
class FaultFlag(IntFlag):
    NONE = 0
    WIFI_TIMEOUT = 1 << 0
    LINE_LOST = 1 << 1
    ENCODER_DISAGREE = 1 << 2
    PID_OVERRUN = 1 << 3
    BUTTON_STUCK = 1 << 4
    MOTOR = 1 << 5
    STALE_DATA = 1 << 6
    PROTOCOL = 1 << 7
    NO_COMMITTED_SELECTION = 1 << 8
    BROWNOUT = 1 << 9


@unique
class MissionStatusFlag(IntFlag):
    DRONE_LINK_OK = 1 << 0
    DRONE_ARMED = 1 << 1
    VISION_VALID = 1 << 2
    ROS_READY = 1 << 3


@unique
class DTask(IntEnum):
    PAYLOAD_DROP = 1
    DYNAMIC_LANDING = 2
    STABILITY_TEST = 3


@unique
class AuthorityState(StringEnum):
    BOOT_LOCKED = "BOOT_LOCKED"
    PRESTART = "PRESTART"
    SELECT_PENDING = "SELECT_PENDING"
    SELECTED = "SELECTED"
    ARMED_READY = "ARMED_READY"
    CAR_RUNNING = "CAR_RUNNING"
    FAULT = "FAULT"


@unique
class RejectCode(StringEnum):
    NO_CAR_SESSION = "NO_CAR_SESSION"
    NO_COMMITTED_SELECTION = "NO_COMMITTED_SELECTION"
    FCU_ALREADY_ARMED = "FCU_ALREADY_ARMED"
    FCU_NOT_ARMED = "FCU_NOT_ARMED"
    CAR_EPOCH_MISMATCH = "CAR_EPOCH_MISMATCH"
    SELECTION_ALREADY_COMMITTED = "SELECTION_ALREADY_COMMITTED"
    SELECTION_NOT_PENDING = "SELECTION_NOT_PENDING"
    SELECTION_ID_MISMATCH = "SELECTION_ID_MISMATCH"
    SELECTION_REJECTED = "SELECTION_REJECTED"
    ARMED_DURING_SELECTION = "ARMED_DURING_SELECTION"
    START_ALREADY_CONSUMED = "START_ALREADY_CONSUMED"
    READ_ONLY_AFTER_START = "READ_ONLY_AFTER_START"
    FAULT_LATCHED = "FAULT_LATCHED"
    TELEMETRY_STALE = "TELEMETRY_STALE"


@dataclass(frozen=True, slots=True)
class Endpoint:
    host: str
    port: int


@dataclass(frozen=True, slots=True)
class OutboundFrame:
    message_type: MessageType
    sender_id: SenderId
    boot_id: BootId
    sequence: Sequence
    source_millis: SourceMillis
    payload: bytes


@dataclass(frozen=True, slots=True)
class AuthenticatedDatagram:
    frame: OutboundFrame
    checksum_crc16: int


@dataclass(frozen=True, slots=True)
class VehicleTelemetryValue:
    state: CarState
    turn: TurnClass
    event: RouteEvent
    event_id: int
    quality_flags: QualityFlag
    displacement_mm: int
    velocity_mm_s: int
    line_error_milli: int
    fault_flags: FaultFlag


@dataclass(frozen=True, slots=True)
class MissionSelectionValue:
    selection_id: SelectionId
    car_boot_id: BootId
    task: DTask


@dataclass(frozen=True, slots=True)
class MissionStatusValue:
    selection_id: SelectionId
    car_boot_id: BootId
    hmi_boot_id: BootId
    phase: MissionPhase
    selected_task: int
    reason_flags: int
    status_flags: MissionStatusFlag


@dataclass(frozen=True, slots=True)
class SelectMissionCommand:
    selection: MissionSelectionValue


@dataclass(frozen=True, slots=True)
class ExecuteMissionCommand:
    mission_id: str
    field_profile_id: str
    timeout_seconds: float


@dataclass(frozen=True, slots=True)
class TelemetryFault:
    code: RejectCode
    age_seconds: float
    car_boot_epoch: BootId


@dataclass(frozen=True, slots=True)
class AuthorityDecision:
    accepted: bool
    state: AuthorityState
    reason: str
    select_command: SelectMissionCommand | None = None
    acknowledgement: MissionStatusValue | None = None
    execute_command: ExecuteMissionCommand | None = None


@dataclass(frozen=True, slots=True)
class AcceptedPacket:
    datagram: AuthenticatedDatagram
    session_changed: bool
