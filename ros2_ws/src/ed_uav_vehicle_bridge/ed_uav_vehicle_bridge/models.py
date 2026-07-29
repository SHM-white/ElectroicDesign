"""Typed values crossing the vehicle/HMI UDP boundary."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, unique
from typing import NewType

from .string_enum import StringEnum


BootEpoch = NewType("BootEpoch", int)
ReceiptSeconds = NewType("ReceiptSeconds", float)
SelectionId = NewType("SelectionId", int)
Sequence = NewType("Sequence", int)
SourceMillis = NewType("SourceMillis", int)


@unique
class MessageType(IntEnum):
    CAR_TELEMETRY = 1
    HMI_SELECTION = 2
    SELECTION_ACK = 3
    MISSION_STATUS = 4


@unique
class MotionKind(IntEnum):
    DISPLACEMENT = 1
    WHEEL_SPEED = 2


@unique
class TurnClass(IntEnum):
    STRAIGHT = 0
    SMALL = 1
    LARGE = 2


@unique
class RouteStage(IntEnum):
    START = 0
    B = 1
    D = 2
    A = 3
    COMPLETE = 4


@unique
class DTask(IntEnum):
    PAYLOAD_DROP = 1
    DYNAMIC_LANDING = 2


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
    sender_id: str
    boot_epoch: BootEpoch
    sequence: Sequence
    source_millis: SourceMillis
    payload: bytes


@dataclass(frozen=True, slots=True)
class AuthenticatedDatagram:
    frame: OutboundFrame
    checksum_crc16: int


@dataclass(frozen=True, slots=True)
class VehicleTelemetryValue:
    contract_version: int
    vehicle_id: str
    start_event: bool
    heartbeat_alive: bool
    motion_kind: MotionKind
    displacement_m: float
    wheel_speed_m_s: float
    turn_class: TurnClass
    route_stage: RouteStage
    lap_complete: bool
    frame_id: str


@dataclass(frozen=True, slots=True)
class MissionSelectionValue:
    contract_version: int
    selection_id: SelectionId
    car_boot_epoch: BootEpoch
    mission_id: str
    mission_profile_id: str
    deployment_preset_id: str
    target_revision: str
    task: DTask


@dataclass(frozen=True, slots=True)
class SelectionAckValue:
    contract_version: int
    selection_id: SelectionId
    car_boot_epoch: BootEpoch
    accepted: bool
    state: AuthorityState
    reason: str


@dataclass(frozen=True, slots=True)
class MissionStatusValue:
    contract_version: int
    source_sequence: Sequence
    mission_id: str
    state: int
    route_stage: RouteStage
    complete: bool
    reason: str


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
    car_boot_epoch: BootEpoch


@dataclass(frozen=True, slots=True)
class AuthorityDecision:
    accepted: bool
    state: AuthorityState
    reason: str
    select_command: SelectMissionCommand | None = None
    acknowledgement: SelectionAckValue | None = None
    execute_command: ExecuteMissionCommand | None = None


@dataclass(frozen=True, slots=True)
class AcceptedPacket:
    datagram: AuthenticatedDatagram
    session_changed: bool
