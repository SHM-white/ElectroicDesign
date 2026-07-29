import pytest


pytest.importorskip("rclpy")

from builtin_interfaces.msg import Time
from ed_uav_interfaces.msg import MissionStatus

from ed_uav_vehicle_bridge.models import (
    AuthenticatedDatagram,
    BootEpoch,
    DTask,
    ExecuteMissionCommand,
    MessageType,
    MissionSelectionValue,
    MotionKind,
    OutboundFrame,
    RouteStage,
    SelectionId,
    Sequence,
    SourceMillis,
    TurnClass,
    VehicleTelemetryValue,
)
from ed_uav_vehicle_bridge.ros_mapping import (
    from_mission_status,
    to_execute_goal,
    to_selection_request,
    to_stale_vehicle_message,
    to_vehicle_message,
)


def test_udp_values_map_once_to_todo1_ros_contracts() -> None:
    telemetry = VehicleTelemetryValue(
        contract_version=1,
        vehicle_id="car-1",
        start_event=True,
        heartbeat_alive=True,
        motion_kind=MotionKind.DISPLACEMENT,
        displacement_m=1.25,
        wheel_speed_m_s=0.5,
        heading_rad=0.4,
        yaw_rate_rad_s=0.2,
        turn_class=TurnClass.SMALL,
        route_stage=RouteStage.START,
        lap_complete=False,
        frame_id="vehicle_start",
    )
    frame = OutboundFrame(
        MessageType.CAR_TELEMETRY,
        "CAR-01",
        BootEpoch(9),
        Sequence(7),
        SourceMillis(100),
        b"typed",
    )
    datagram = AuthenticatedDatagram(frame, 0x1234)
    stamp = Time(sec=3, nanosec=4)
    selection = MissionSelectionValue(
        1,
        SelectionId(8),
        BootEpoch(9),
        "mission-8",
        "profile-8",
        "field-8",
        "target-v1",
        DTask.PAYLOAD_DROP,
    )

    vehicle = to_vehicle_message(telemetry, datagram, stamp, stamp)
    stale = to_stale_vehicle_message(telemetry, Sequence(8), stamp, stamp)
    request = to_selection_request(selection)
    goal = to_execute_goal(ExecuteMissionCommand("mission-8", "field-8", 90.0))

    assert vehicle.source_sequence == 7
    assert vehicle.checksum_crc16 == 0x1234
    assert vehicle.start_event is True
    assert vehicle.heading_rad == pytest.approx(0.4)
    assert vehicle.yaw_rate_rad_s == pytest.approx(0.2)
    assert stale.source_sequence == 8
    assert stale.heartbeat_alive is False
    assert request.task == 1
    assert request.deployment_preset_id == "field-8"
    assert goal.mission_id == "mission-8"
    assert goal.field_profile_id == "field-8"


def test_mission_status_maps_to_bounded_hmi_value() -> None:
    message = MissionStatus()
    message.contract_version = 1
    message.source_sequence = 5
    message.mission_id = "mission-5"
    message.state = MissionStatus.STATE_PRE_ARM
    message.route_stage = RouteStage.START
    message.complete = False
    message.reason = "idle"

    value = from_mission_status(message)

    assert value.source_sequence == Sequence(5)
    assert value.route_stage is RouteStage.START
    assert value.reason == "idle"
