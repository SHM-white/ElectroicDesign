"""Map delegated UDP values to the existing D-task ROS contracts."""

from builtin_interfaces.msg import Time
from ed_uav_interfaces.action import ExecuteMission
from ed_uav_interfaces.msg import MissionStatus, VehicleTelemetry
from ed_uav_interfaces.srv import SelectDTaskMission
from typing_extensions import assert_never

from .models import (
    AuthenticatedDatagram,
    CarState,
    ExecuteMissionCommand,
    MissionPhase,
    MissionSelectionValue,
    MissionStatusFlag,
    MissionStatusValue,
    RouteEvent,
    Sequence,
    Task3FlightTestIdentity,
    VehicleTelemetryValue,
)
from .payloads import encode_mission_status


_ROUTE_STAGES: dict[RouteEvent, int] = {
    RouteEvent.NONE: 0,
    RouteEvent.START: 0,
    RouteEvent.B: 1,
    RouteEvent.D: 2,
    RouteEvent.A: 3,
    RouteEvent.COMPLETE: 4,
}


def to_vehicle_message(
    value: VehicleTelemetryValue,
    datagram: AuthenticatedDatagram,
    acquisition_stamp: Time,
    start_stamp: Time,
) -> VehicleTelemetry:
    message = VehicleTelemetry()
    message.contract_version = VehicleTelemetry.CONTRACT_VERSION
    message.start_stamp = start_stamp
    message.acquisition_stamp = acquisition_stamp
    message.source_sequence = datagram.frame.sequence
    message.checksum_crc16 = datagram.checksum_crc16
    message.vehicle_id = str(datagram.frame.sender_id)
    message.start_event = value.event is RouteEvent.START
    message.heartbeat_alive = True
    message.motion_kind = VehicleTelemetry.MOTION_DISPLACEMENT
    message.displacement_m = value.displacement_mm / 1000.0
    message.wheel_speed_m_s = value.velocity_mm_s / 1000.0
    message.heading_rad = 0.0
    message.yaw_rate_rad_s = 0.0
    message.turn_class = int(value.turn)
    message.route_stage = _ROUTE_STAGES[value.event]
    message.lap_complete = value.event is RouteEvent.COMPLETE or value.state is CarState.COMPLETE
    message.frame_id = "vehicle_start"
    return message


def to_stale_vehicle_message(
    value: VehicleTelemetryValue,
    source_sequence: Sequence,
    acquisition_stamp: Time,
    start_stamp: Time,
) -> VehicleTelemetry:
    message = VehicleTelemetry()
    message.contract_version = VehicleTelemetry.CONTRACT_VERSION
    message.start_stamp = start_stamp
    message.acquisition_stamp = acquisition_stamp
    message.source_sequence = source_sequence
    message.checksum_crc16 = 0
    message.vehicle_id = ""
    message.start_event = False
    message.heartbeat_alive = False
    message.motion_kind = VehicleTelemetry.MOTION_DISPLACEMENT
    message.displacement_m = value.displacement_mm / 1000.0
    message.wheel_speed_m_s = value.velocity_mm_s / 1000.0
    message.heading_rad = 0.0
    message.yaw_rate_rad_s = 0.0
    message.turn_class = int(value.turn)
    message.route_stage = _ROUTE_STAGES[value.event]
    message.lap_complete = False
    message.frame_id = "vehicle_start"
    return message


def to_selection_request(
    value: MissionSelectionValue,
    identity: Task3FlightTestIdentity | None = None,
) -> SelectDTaskMission.Request:
    request = SelectDTaskMission.Request()
    request.contract_version = SelectDTaskMission.Request.CONTRACT_VERSION
    request.task = int(value.task)
    if identity is not None:
        request.mission_id = identity.mission_id
        request.field_profile_id = identity.field_profile_id
        request.mission_profile_id = identity.mission_profile_id
        request.deployment_preset_id = identity.deployment_preset_id
        request.target_revision = identity.target_revision
    return request


def to_execute_goal(command: ExecuteMissionCommand) -> ExecuteMission.Goal:
    goal = ExecuteMission.Goal()
    goal.mission_id = command.mission_id
    goal.field_profile_id = command.field_profile_id
    goal.timeout_sec = command.timeout_seconds
    return goal


def encode_mission_status_for_hmi(
    message: MissionStatus | MissionStatusValue,
    selection: MissionSelectionValue | None = None,
) -> bytes:
    match message:
        case MissionStatusValue():
            return encode_mission_status(message)
        case MissionStatus():
            if message.complete:
                phase = MissionPhase.COMPLETE
            else:
                phase = {
                    MissionStatus.STATE_PRE_ARM: MissionPhase.PRESTART,
                    MissionStatus.STATE_ABORTED: MissionPhase.FAULT,
                }.get(message.state, MissionPhase.CAR_RUNNING)
            value = MissionStatusValue(
                selection_id=0 if selection is None else selection.selection_id,
                car_boot_id=0 if selection is None else selection.car_boot_id,
                hmi_boot_id=0,
                phase=phase,
                selected_task=0 if selection is None else int(selection.task),
                reason_flags=0,
                status_flags=MissionStatusFlag(0),
            )
            return encode_mission_status(value)
        case unreachable:
            assert_never(unreachable)
