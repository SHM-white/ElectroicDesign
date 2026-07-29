"""Single typed mapping between UDP domain values and Todo 1 ROS contracts."""

from builtin_interfaces.msg import Time
from ed_uav_interfaces.action import ExecuteMission
from ed_uav_interfaces.msg import MissionStatus, VehicleTelemetry
from ed_uav_interfaces.srv import SelectDTaskMission

from .models import (
    AuthenticatedDatagram,
    ExecuteMissionCommand,
    MissionSelectionValue,
    MissionStatusValue,
    RouteStage,
    Sequence,
    VehicleTelemetryValue,
)


def to_vehicle_message(
    value: VehicleTelemetryValue,
    datagram: AuthenticatedDatagram,
    acquisition_stamp: Time,
    start_stamp: Time,
) -> VehicleTelemetry:
    message = VehicleTelemetry()
    message.contract_version = value.contract_version
    message.start_stamp = start_stamp
    message.acquisition_stamp = acquisition_stamp
    message.source_sequence = datagram.frame.sequence
    message.checksum_crc16 = datagram.checksum_crc16
    message.vehicle_id = value.vehicle_id
    message.start_event = value.start_event
    message.heartbeat_alive = value.heartbeat_alive
    message.motion_kind = int(value.motion_kind)
    message.displacement_m = value.displacement_m
    message.wheel_speed_m_s = value.wheel_speed_m_s
    message.turn_class = int(value.turn_class)
    message.route_stage = int(value.route_stage)
    message.lap_complete = value.lap_complete
    message.frame_id = value.frame_id
    return message


def to_stale_vehicle_message(
    value: VehicleTelemetryValue,
    source_sequence: Sequence,
    acquisition_stamp: Time,
    start_stamp: Time,
) -> VehicleTelemetry:
    message = VehicleTelemetry()
    message.contract_version = value.contract_version
    message.start_stamp = start_stamp
    message.acquisition_stamp = acquisition_stamp
    message.source_sequence = source_sequence
    message.checksum_crc16 = 0
    message.vehicle_id = value.vehicle_id
    message.start_event = False
    message.heartbeat_alive = False
    message.motion_kind = int(value.motion_kind)
    message.displacement_m = value.displacement_m
    message.wheel_speed_m_s = value.wheel_speed_m_s
    message.turn_class = int(value.turn_class)
    message.route_stage = int(value.route_stage)
    message.lap_complete = value.lap_complete
    message.frame_id = value.frame_id
    return message


def to_selection_request(value: MissionSelectionValue) -> SelectDTaskMission.Request:
    request = SelectDTaskMission.Request()
    request.contract_version = value.contract_version
    request.mission_id = value.mission_id
    request.mission_profile_id = value.mission_profile_id
    request.deployment_preset_id = value.deployment_preset_id
    request.target_revision = value.target_revision
    request.task = int(value.task)
    return request


def to_execute_goal(command: ExecuteMissionCommand) -> ExecuteMission.Goal:
    goal = ExecuteMission.Goal()
    goal.mission_id = command.mission_id
    goal.field_profile_id = command.field_profile_id
    goal.timeout_sec = command.timeout_seconds
    return goal


def from_mission_status(message: MissionStatus) -> MissionStatusValue:
    return MissionStatusValue(
        contract_version=message.contract_version,
        source_sequence=Sequence(message.source_sequence),
        mission_id=message.mission_id,
        state=message.state,
        route_stage=RouteStage(message.route_stage),
        complete=message.complete,
        reason=message.reason,
    )
