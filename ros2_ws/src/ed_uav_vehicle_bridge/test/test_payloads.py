from dataclasses import replace

import pytest

from ed_uav_vehicle_bridge.errors import ProtocolError, ProtocolErrorCode
from ed_uav_vehicle_bridge.models import (
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
    TurnClass,
    VehicleTelemetryValue,
)
from ed_uav_vehicle_bridge.payloads import (
    decode_car_telemetry,
    decode_mission_status,
    decode_task_selection,
    encode_car_telemetry,
    encode_mission_status,
    encode_task_selection,
)


TELEMETRY = VehicleTelemetryValue(
    state=CarState.RUNNING,
    turn=TurnClass.SMALL,
    event=RouteEvent.B,
    event_id=7,
    quality_flags=QualityFlag.LINE_VALID | QualityFlag.ENCODER_VALID,
    displacement_mm=-1234,
    velocity_mm_s=321,
    line_error_milli=-125,
    fault_flags=FaultFlag.NONE,
)
SELECTION = MissionSelectionValue(
    selection_id=SelectionId(42),
    car_boot_id=BootId(0x12345678),
    task=DTask.PAYLOAD_DROP,
)
STATUS = MissionStatusValue(
    selection_id=SelectionId(99),
    car_boot_id=BootId(0xAABBCCDD),
    hmi_boot_id=BootId(0x11223344),
    phase=MissionPhase.ARMED_READY,
    selected_task=2,
    reason_flags=0,
    status_flags=MissionStatusFlag.DRONE_LINK_OK | MissionStatusFlag.VISION_VALID,
)


def test_payloads_round_trip_into_fixed_width_values() -> None:
    # Given: one value for each delegated payload schema.
    encoded = (
        encode_car_telemetry(TELEMETRY),
        encode_task_selection(SELECTION),
        encode_mission_status(STATUS),
    )

    # When: each fixed-width value is decoded.
    decoded = (
        decode_car_telemetry(encoded[0]),
        decode_task_selection(encoded[1]),
        decode_mission_status(encoded[2]),
    )

    # Then: the values retain their typed fields and exact widths.
    # TASK_SELECTION 现含 mode 字节 (<IIBB = 10B), 兼容旧 9B 帧
    assert tuple(map(len, encoded)) == (17, 10, 18)
    assert decoded == (TELEMETRY, SELECTION, STATUS)


def test_car_telemetry_matches_delegated_little_endian_layout() -> None:
    # Given: the C++ reference telemetry value.
    # When: it is encoded by the Python bridge.
    encoded = encode_car_telemetry(TELEMETRY)

    # Then: all multi-byte fields are little-endian in the fixed payload.
    assert encoded.hex() == "010102070003002efbffff410183ff0000"


@pytest.mark.parametrize(
    ("decoder", "payload"),
    [
        (decode_car_telemetry, b"\x00" * 16),
        (decode_task_selection, b"\x00" * 10),
        (decode_mission_status, b"\x00" * 17),
    ],
)
def test_payload_decoders_require_fixed_width(decoder, payload: bytes) -> None:
    # Given: a payload with one byte missing or appended.
    # When/Then: decoding stops at the schema boundary.
    with pytest.raises(ProtocolError) as raised:
        decoder(payload)
    assert raised.value.code is ProtocolErrorCode.BAD_PAYLOAD


def test_payload_rejects_unknown_enum_and_status_flags() -> None:
    # Given: valid payloads with unknown delegated enum or flag bits.
    telemetry = bytearray(encode_car_telemetry(TELEMETRY))
    telemetry[0] = 0xFF
    status = bytearray(encode_mission_status(STATUS))
    status[-1] |= 0x80

    # When/Then: unknown values reject at the payload boundary.
    with pytest.raises(ProtocolError) as telemetry_error:
        decode_car_telemetry(bytes(telemetry))
    with pytest.raises(ProtocolError) as status_error:
        decode_mission_status(bytes(status))
    assert telemetry_error.value.code is ProtocolErrorCode.BAD_PAYLOAD
    assert status_error.value.code is ProtocolErrorCode.BAD_PAYLOAD


def test_payload_encoder_rejects_invalid_task() -> None:
    # Given: a task selection with a value outside the delegated 1..3 range.
    invalid = replace(SELECTION, task=4)

    # When/Then: serialization rejects the invalid task.
    with pytest.raises(ProtocolError) as raised:
        encode_task_selection(invalid)
    assert raised.value.code is ProtocolErrorCode.BAD_PAYLOAD

    # Task 0 is also invalid
    invalid_zero = replace(SELECTION, task=0)
    with pytest.raises(ProtocolError):
        encode_task_selection(invalid_zero)
