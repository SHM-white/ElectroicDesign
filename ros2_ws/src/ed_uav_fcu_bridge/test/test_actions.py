from __future__ import annotations

import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.actions import (  # noqa: E402
    CommandKind,
    CommandRequest,
    FlightActionController,
    ResultCode,
)
from ed_uav_fcu_bridge.v7_codec import build_frame, decode_frame  # noqa: E402


def acknowledgement_for(raw: bytes) -> bytes:
    frame = decode_frame(raw)
    return build_frame(0xFF, 0x00, bytes((frame.frame_id, frame.sum_check, frame.add_check)))


def test_command_reports_success_only_for_its_matching_ack() -> None:
    # Given: an action controller and a sent move command.
    written: list[bytes] = []
    controller = FlightActionController(written.append)
    pending = controller.start(CommandRequest.move(100, 30, 90), steady_now=10.0, timeout_s=0.5)

    # When: the matching V7 checksum acknowledgement arrives.
    result = controller.handle_frame(decode_frame(acknowledgement_for(written[0])), steady_now=10.1)

    # Then: the action exposes an ACK-backed terminal success.
    assert pending.command == CommandKind.MOVE
    assert result is not None
    assert result.code is ResultCode.SUCCEEDED
    assert result.acknowledged


def test_ack_timeout_stays_terminal_when_late_or_duplicate_acks_arrive() -> None:
    # Given: a hover action with a short acknowledgement deadline.
    written: list[bytes] = []
    controller = FlightActionController(written.append)
    controller.start(CommandRequest.hover(), steady_now=20.0, timeout_s=0.5)
    matching_ack = decode_frame(acknowledgement_for(written[0]))

    # When: the deadline passes and the same ACK arrives twice afterward.
    timeout = controller.tick(steady_now=20.501)
    late = controller.handle_frame(matching_ack, steady_now=20.6)
    duplicate = controller.handle_frame(matching_ack, steady_now=20.7)

    # Then: timeout remains the outcome; late and duplicate ACKs cannot revive it.
    assert timeout is not None
    assert timeout.code is ResultCode.TIMEOUT
    assert late is None
    assert duplicate is None
    assert controller.last_result == timeout


def test_unrelated_ack_cannot_complete_the_pending_command() -> None:
    # Given: a sent unlock command and another valid command's ACK.
    written: list[bytes] = []
    controller = FlightActionController(written.append)
    controller.start(CommandRequest.unlock(), steady_now=30.0, timeout_s=0.5)
    unrelated = decode_frame(acknowledgement_for(CommandRequest.land().to_frame()))

    # When: an ACK with a different checksum signature arrives.
    result = controller.handle_frame(unrelated, steady_now=30.1)

    # Then: it is ignored and the original command remains pending.
    assert result is None
    assert controller.pending is not None
    assert controller.pending.command is CommandKind.UNLOCK


@pytest.mark.parametrize(
    "command_request",
    (
        CommandRequest.unlock(),
        CommandRequest(CommandKind.SET_MODE, mode=3),
        CommandRequest(CommandKind.TAKEOFF, height_cm=150),
        CommandRequest.move(100, 30, 90),
        CommandRequest.hover(),
        CommandRequest.land(),
        CommandRequest(CommandKind.LOCK),
    ),
)
def test_each_supported_high_level_action_reports_its_own_ack_result(command_request: CommandRequest) -> None:
    # Given: one of every public high-level V7 action.
    written: list[bytes] = []
    controller = FlightActionController(written.append)
    controller.start(command_request, steady_now=40.0, timeout_s=0.5)

    # When: the FCU returns the checksum acknowledgement for that exact wire frame.
    result = controller.handle_frame(decode_frame(acknowledgement_for(written[0])), steady_now=40.1)

    # Then: every action has the same ACK-correlated success surface.
    assert result is not None
    assert result.command is command_request.command
    assert result.code is ResultCode.SUCCEEDED
