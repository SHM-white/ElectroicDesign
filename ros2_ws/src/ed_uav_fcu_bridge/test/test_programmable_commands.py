from __future__ import annotations

import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge import actions, v7_codec


@pytest.mark.parametrize(
    ("builder_name", "arguments", "expected_hex"),
    (
        ("cmd_target_position", (1234, -5678), "AAFFE00B100101D2040000D2E9FFFF353B"),
        ("cmd_target_height", (-250,), "AAFFE00B10010206FFFFFF00000000AAB6"),
        ("cmd_ascend", (120, 30), "AAFFE00B10020178001E00000000003D0D"),
        ("cmd_descend", (80, 20), "AAFFE00B10020250001400000000000C9A"),
    ),
)
def test_documented_programmable_commands_are_byte_exact(
    builder_name: str,
    arguments: tuple[int, ...],
    expected_hex: str,
) -> None:
    # Given: a manual-defined programmable command and independent full-frame vector.
    builder = getattr(v7_codec, builder_name, None)

    # When: the package exposes its encoder.
    assert callable(builder), f"missing documented V7 builder {builder_name}"
    actual = builder(*arguments)

    # Then: CID, command bytes, padding, and checksums are byte-identical.
    assert actual.hex().upper() == expected_hex


def test_programmable_request_rejects_overlap_bad_ack_and_late_ack() -> None:
    # Given: a programmable target-position command owns the single ACK slot.
    request_factory = getattr(actions.CommandRequest, "target_position", None)
    assert callable(request_factory), "missing typed target-position request"
    written: list[bytes] = []
    controller = actions.FlightActionController(written.append)
    request = request_factory(1234, -5678)
    pending = controller.start(request, steady_now=10.0, timeout_s=0.5)

    # When: overlap, a bad ACK, timeout, and a late matching ACK occur in order.
    with pytest.raises(actions.CommandRejectedError):
        controller.start(actions.CommandRequest.hover(), steady_now=10.1, timeout_s=0.5)
    bad_ack = v7_codec.decode_frame(v7_codec.build_frame(0xFF, 0x00, b"\xE0\x00\x00"))
    bad_result = controller.handle_frame(bad_ack, steady_now=10.2)
    timeout = controller.tick(steady_now=10.501)
    sent = v7_codec.decode_frame(pending.raw)
    matching = v7_codec.decode_frame(
        v7_codec.build_frame(0xFF, 0x00, bytes((sent.frame_id, sent.sum_check, sent.add_check)))
    )
    late_result = controller.handle_frame(matching, steady_now=10.6)

    # Then: no adversarial input completes or overlaps the timed-out command.
    assert bad_result is None
    assert timeout is not None and timeout.code is actions.ResultCode.TIMEOUT
    assert late_result is None
    assert written == [pending.raw]


def test_realtime_control_frame_is_byte_exact() -> None:
    # Given: seven signed V7 realtime fields with positive and negative values.
    fields_type = getattr(v7_codec, "RealtimeControlFields", None)
    encoder = getattr(v7_codec, "cmd_realtime_control", None)
    assert fields_type is not None, "missing typed V7 realtime-control fields"
    assert callable(encoder), "missing V7 realtime-control encoder"
    fields = fields_type(
        roll=0,
        pitch=0,
        thr=0,
        yaw_dps=0,
        spd_x=100,
        spd_y=-200,
        spd_z=300,
    )

    # When: the native V7 realtime frame is encoded.
    actual = encoder(fields)

    # Then: AA FF 41 0E, seven little-endian int16 fields, SC, and AC match.
    assert actual.hex().upper() == "AAFF410E0000000000000000640038FF2C01C053"


def test_realtime_control_frame_never_enters_the_ack_controller() -> None:
    # Given: one legacy command awaiting ACK and one valid zero-velocity 0x41 frame.
    written: list[bytes] = []
    controller = actions.FlightActionController(written.append)
    pending = controller.start(actions.CommandRequest.unlock(), steady_now=1.0, timeout_s=0.5)
    fields = v7_codec.RealtimeControlFields(0, 0, 0, 0, 0, 0, 0)
    realtime_frame = v7_codec.decode_frame(v7_codec.cmd_realtime_control(fields))

    # When: the realtime frame is offered to the ACK correlation path.
    result = controller.handle_frame(realtime_frame, steady_now=1.1)

    # Then: only ID 0x00 can resolve the existing 0xE0 command.
    assert result is None
    assert controller.pending is pending
    assert written == [pending.raw]
