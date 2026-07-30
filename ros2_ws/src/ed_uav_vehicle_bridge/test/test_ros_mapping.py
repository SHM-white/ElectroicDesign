import pytest


pytest.importorskip("rclpy")

from builtin_interfaces.msg import Time
from ed_uav_interfaces.msg import MissionStatus

from ed_uav_vehicle_bridge.models import (
    AuthenticatedDatagram,
    BootId,
    CarState,
    DTask,
    ExecuteMissionCommand,
    FaultFlag,
    MessageType,
    MissionPhase,
    MissionSelectionValue,
    OutboundFrame,
    QualityFlag,
    RouteEvent,
    SelectionId,
    Sequence,
    SenderId,
    SourceMillis,
    TurnClass,
    VehicleTelemetryValue,
)
from ed_uav_vehicle_bridge.payloads import decode_mission_status
from ed_uav_vehicle_bridge.ros_mapping import (
    encode_mission_status_for_hmi,
    to_execute_goal,
    to_selection_request,
    to_stale_vehicle_message,
    to_vehicle_message,
)


def test_udp_values_map_once_to_d_task_ros_contracts() -> None:
    telemetry = VehicleTelemetryValue(
        state=CarState.RUNNING,
        turn=TurnClass.SMALL,
        event=RouteEvent.START,
        event_id=1,
        quality_flags=QualityFlag.LINE_VALID | QualityFlag.ENCODER_VALID,
        displacement_mm=1250,
        velocity_mm_s=500,
        line_error_milli=0,
        fault_flags=FaultFlag.NONE,
    )
    frame = OutboundFrame(
        MessageType.CAR_TELEMETRY,
        SenderId(0x43415231),
        BootId(9),
        Sequence(7),
        SourceMillis(100),
        b"typed",
    )
    datagram = AuthenticatedDatagram(frame, 0x1234)
    stamp = Time(sec=3, nanosec=4)
    selection = MissionSelectionValue(
        SelectionId(8),
        BootId(9),
        DTask.PAYLOAD_DROP,
    )

    vehicle = to_vehicle_message(telemetry, datagram, stamp, stamp)
    stale = to_stale_vehicle_message(telemetry, Sequence(8), stamp, stamp)
    request = to_selection_request(selection)
    goal = to_execute_goal(ExecuteMissionCommand("mission-8", "field-8", 90.0))

    assert vehicle.source_sequence == 7
    assert vehicle.checksum_crc16 == 0x1234
    assert vehicle.start_event is True
    assert vehicle.displacement_m == pytest.approx(1.25)
    assert vehicle.wheel_speed_m_s == pytest.approx(0.5)
    assert stale.source_sequence == 8
    assert stale.heartbeat_alive is False
    assert request.task == 1
    assert request.contract_version == 1
    assert goal.mission_id == "mission-8"
    assert goal.field_profile_id == "field-8"


def test_mission_status_maps_to_bounded_hmi_value() -> None:
    message = MissionStatus()
    message.contract_version = 1
    message.source_sequence = 5
    message.mission_id = "mission-5"
    message.state = MissionStatus.STATE_PRE_ARM
    message.complete = False
    message.reason = "idle"

    value = decode_mission_status(encode_mission_status_for_hmi(message))

    assert value.phase is MissionPhase.PRESTART
    assert value.selected_task == 0
    assert value.reason_flags == 0
