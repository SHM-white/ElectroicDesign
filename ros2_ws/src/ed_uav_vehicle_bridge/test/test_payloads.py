from dataclasses import replace

import pytest

from ed_uav_vehicle_bridge.errors import ProtocolError, ProtocolErrorCode
from ed_uav_vehicle_bridge.models import (
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
from ed_uav_vehicle_bridge.payloads import (
    decode_mission_selection,
    decode_mission_status,
    decode_selection_ack,
    decode_vehicle_telemetry,
    encode_mission_selection,
    encode_mission_status,
    encode_selection_ack,
    encode_vehicle_telemetry,
)


TELEMETRY = VehicleTelemetryValue(
    contract_version=1,
    vehicle_id="car-1",
    start_event=True,
    heartbeat_alive=True,
    motion_kind=MotionKind.DISPLACEMENT,
    displacement_m=-1.25,
    wheel_speed_m_s=0.75,
    turn_class=TurnClass.SMALL,
    route_stage=RouteStage.START,
    lap_complete=False,
    frame_id="vehicle_start",
)
SELECTION = MissionSelectionValue(
    contract_version=1,
    selection_id=SelectionId(44),
    car_boot_epoch=BootEpoch(99),
    mission_id="d-task-run-44",
    mission_profile_id="d2026-payload-drop",
    deployment_preset_id="field-a",
    target_revision="circle-cross-v1",
    task=DTask.PAYLOAD_DROP,
)


def test_payloads_round_trip_into_typed_values_once() -> None:
    # Given: values for every UDP payload exposed by the bridge.
    ack = SelectionAckValue(1, SelectionId(44), BootEpoch(99), True, AuthorityState.SELECTED, "ACK")
    status = MissionStatusValue(1, Sequence(7), "d-task-run-44", 3, RouteStage.B, False, "tracking")

    # When: each value crosses its explicit binary codec.
    decoded = (
        decode_vehicle_telemetry(encode_vehicle_telemetry(TELEMETRY)),
        decode_mission_selection(encode_mission_selection(SELECTION)),
        decode_selection_ack(encode_selection_ack(ack)),
        decode_mission_status(encode_mission_status(status)),
    )

    # Then: downstream code receives only typed values.
    assert decoded == (TELEMETRY, SELECTION, ack, status)


def test_payload_rejects_unknown_enum_and_trailing_bytes() -> None:
    # Given: valid telemetry plus one byte that cannot belong to its schema.
    payload = encode_vehicle_telemetry(TELEMETRY)

    # When/Then: unknown variant and trailing data reject at the boundary.
    with pytest.raises(ProtocolError) as enum_error:
        decode_vehicle_telemetry(payload[:3] + b"\xff" + payload[4:])
    with pytest.raises(ProtocolError) as trailing_error:
        decode_vehicle_telemetry(payload + b"\x00")
    assert enum_error.value.code is ProtocolErrorCode.BAD_PAYLOAD
    assert trailing_error.value.code is ProtocolErrorCode.BAD_PAYLOAD


def test_ros_contract_string_bounds_are_enforced_before_encoding() -> None:
    oversized = replace(SELECTION, target_revision="x" * 33)
    with pytest.raises(ProtocolError) as raised:
        encode_mission_selection(oversized)
    assert raised.value.code is ProtocolErrorCode.BAD_PAYLOAD
