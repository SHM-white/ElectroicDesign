from dataclasses import replace

import pytest

from ed_uav_vehicle_bridge.errors import ProtocolError, ProtocolErrorCode
from ed_uav_vehicle_bridge.models import (
    AuthenticatedDatagram,
    BootEpoch,
    Endpoint,
    MessageType,
    MotionKind,
    OutboundFrame,
    ReceiptSeconds,
    RouteStage,
    Sequence,
    SourceMillis,
    TurnClass,
    VehicleTelemetryValue,
)
from ed_uav_vehicle_bridge.session import PeerPolicy, RouteTracker, SessionTracker


SOURCE = Endpoint("127.0.0.1", 41001)
POLICY = PeerPolicy("CAR-01", SOURCE, frozenset({MessageType.CAR_TELEMETRY}))
FRAME = OutboundFrame(
    MessageType.CAR_TELEMETRY,
    "CAR-01",
    BootEpoch(100),
    Sequence(10),
    SourceMillis(1000),
    b"payload",
)
DATAGRAM = AuthenticatedDatagram(FRAME, 0x1234)
TELEMETRY = VehicleTelemetryValue(
    1,
    "car-1",
    False,
    True,
    MotionKind.DISPLACEMENT,
    0.0,
    0.0,
    0.0,
    0.0,
    TurnClass.STRAIGHT,
    RouteStage.START,
    False,
    "vehicle_start",
)


def test_source_session_sequence_wrap_and_reboot_are_bounded() -> None:
    # Given: one endpoint-bound peer near uint32 wrap.
    tracker = SessionTracker(POLICY)
    near_wrap = replace(FRAME, sequence=Sequence(0xFFFFFFFF))

    # When: wrap and a fresh random boot epoch arrive from the bound endpoint.
    first = tracker.accept(AuthenticatedDatagram(near_wrap, 1), SOURCE, ReceiptSeconds(1.0))
    wrapped = tracker.accept(
        AuthenticatedDatagram(replace(FRAME, sequence=Sequence(0)), 2),
        SOURCE,
        ReceiptSeconds(1.1),
    )
    rebooted = tracker.accept(
        AuthenticatedDatagram(replace(FRAME, boot_epoch=BootEpoch(200), sequence=Sequence(0)), 3),
        SOURCE,
        ReceiptSeconds(1.2),
    )

    # Then: modulo progression passes and only the new epoch reports a reset.
    assert first.session_changed is True
    assert wrapped.session_changed is False
    assert rebooted.session_changed is True


@pytest.mark.parametrize(
    ("candidate", "source", "expected"),
    [
        (DATAGRAM, Endpoint("127.0.0.1", 41002), ProtocolErrorCode.SOURCE_MISMATCH),
        (DATAGRAM, SOURCE, ProtocolErrorCode.REPLAY),
        (AuthenticatedDatagram(replace(FRAME, sequence=Sequence(9)), 1), SOURCE, ProtocolErrorCode.REORDERED),
        (AuthenticatedDatagram(replace(FRAME, sequence=Sequence(5000)), 1), SOURCE, ProtocolErrorCode.SEQUENCE_GAP),
    ],
)
def test_source_replay_reorder_and_gap_are_rejected(candidate, source, expected) -> None:
    # Given: an established authenticated session.
    tracker = SessionTracker(POLICY)
    tracker.accept(DATAGRAM, SOURCE, ReceiptSeconds(1.0))

    # When: an endpoint or sequence invariant is violated.
    with pytest.raises(ProtocolError) as raised:
        tracker.accept(candidate, source, ReceiptSeconds(1.1))

    # Then: the packet cannot reach typed payload handling.
    assert raised.value.code is expected


def test_retired_boot_epoch_cannot_replay_start_after_reboot() -> None:
    tracker = SessionTracker(POLICY)
    tracker.accept(DATAGRAM, SOURCE, ReceiptSeconds(1.0))
    tracker.accept(
        AuthenticatedDatagram(replace(FRAME, boot_epoch=BootEpoch(200), sequence=Sequence(0)), 1),
        SOURCE,
        ReceiptSeconds(1.1),
    )
    with pytest.raises(ProtocolError) as raised:
        tracker.accept(DATAGRAM, SOURCE, ReceiptSeconds(1.2))
    assert raised.value.code is ProtocolErrorCode.RETIRED_BOOT_EPOCH


def test_freshness_uses_local_steady_receipt_time_once() -> None:
    tracker = SessionTracker(POLICY)
    tracker.accept(DATAGRAM, SOURCE, ReceiptSeconds(20.0))

    assert tracker.telemetry_fault_if_stale(ReceiptSeconds(20.75), 0.75) is None
    fault = tracker.telemetry_fault_if_stale(ReceiptSeconds(20.751), 0.75)
    assert fault is not None
    assert fault.age_seconds == pytest.approx(0.751)
    assert tracker.telemetry_fault_if_stale(ReceiptSeconds(21.0), 0.75) is None


def test_route_tracker_rejects_d_before_b_and_repeated_start() -> None:
    tracker = RouteTracker()
    tracker.accept(replace(TELEMETRY, start_event=True))

    with pytest.raises(ProtocolError) as order_error:
        tracker.accept(replace(TELEMETRY, route_stage=RouteStage.D))
    with pytest.raises(ProtocolError) as start_error:
        tracker.accept(replace(TELEMETRY, start_event=True))
    assert order_error.value.code is ProtocolErrorCode.INVALID_ROUTE_ORDER
    assert start_error.value.code is ProtocolErrorCode.START_EVENT_REPEATED
