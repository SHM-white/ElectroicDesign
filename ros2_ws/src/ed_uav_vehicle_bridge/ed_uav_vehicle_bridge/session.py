"""Endpoint-bound boot sessions, sequence replay checks, and freshness."""

from collections import deque
from dataclasses import dataclass
from typing import Final

from .errors import ProtocolError, ProtocolErrorCode
from .models import (
    AcceptedPacket,
    AuthenticatedDatagram,
    BootEpoch,
    Endpoint,
    MessageType,
    ReceiptSeconds,
    RejectCode,
    RouteStage,
    TelemetryFault,
    VehicleTelemetryValue,
)


MAX_FORWARD_SEQUENCE_GAP: Final = 1024
RETIRED_EPOCH_LIMIT: Final = 32


@dataclass(frozen=True, slots=True)
class PeerPolicy:
    sender_id: str
    endpoint: Endpoint
    allowed_types: frozenset[MessageType]


class SessionTracker:
    """Mutable replay state for one configured UDP peer."""

    def __init__(self, policy: PeerPolicy) -> None:
        self._policy = policy
        self._epoch: int | None = None
        self._last_sequence: int | None = None
        self._retired_epochs: deque[int] = deque(maxlen=RETIRED_EPOCH_LIMIT)
        self._last_receipt: float | None = None
        self._stale_reported = False

    def accept(
        self,
        datagram: AuthenticatedDatagram,
        source: Endpoint,
        receipt_time: ReceiptSeconds,
    ) -> AcceptedPacket:
        frame = datagram.frame
        if source != self._policy.endpoint or frame.sender_id != self._policy.sender_id:
            raise ProtocolError(
                ProtocolErrorCode.SOURCE_MISMATCH,
                "sender endpoint is not the provisioned peer",
            )
        if frame.message_type not in self._policy.allowed_types:
            raise ProtocolError(
                ProtocolErrorCode.MESSAGE_TYPE_FORBIDDEN,
                "message type is not allowed for peer",
            )

        session_changed = self._epoch != frame.boot_epoch
        if session_changed:
            if frame.boot_epoch in self._retired_epochs:
                raise ProtocolError(
                    ProtocolErrorCode.RETIRED_BOOT_EPOCH,
                    "boot epoch was already retired",
                )
            if self._epoch is not None:
                self._retired_epochs.append(self._epoch)
            self._epoch = frame.boot_epoch
            self._last_sequence = None

        if self._last_sequence is not None:
            delta = (frame.sequence - self._last_sequence) & 0xFFFFFFFF
            if delta == 0:
                raise ProtocolError(ProtocolErrorCode.REPLAY, "sequence already accepted")
            if delta >= 0x80000000:
                raise ProtocolError(
                    ProtocolErrorCode.REORDERED,
                    "sequence is older than accepted head",
                )
            if delta > MAX_FORWARD_SEQUENCE_GAP:
                raise ProtocolError(
                    ProtocolErrorCode.SEQUENCE_GAP,
                    "forward sequence gap exceeds window",
                )

        self._last_sequence = frame.sequence
        self._last_receipt = receipt_time
        self._stale_reported = False
        return AcceptedPacket(datagram=datagram, session_changed=session_changed)

    def telemetry_fault_if_stale(
        self, now: ReceiptSeconds, stale_after_seconds: float
    ) -> TelemetryFault | None:
        if self._last_receipt is None or self._epoch is None or self._stale_reported:
            return None
        age = now - self._last_receipt
        if age <= stale_after_seconds:
            return None
        self._stale_reported = True
        return TelemetryFault(
            code=RejectCode.TELEMETRY_STALE,
            age_seconds=age,
            car_boot_epoch=BootEpoch(self._epoch),
        )


class RouteTracker:
    """Mutable one-run route order and start-event guard."""

    def __init__(self) -> None:
        self._stage: RouteStage | None = None
        self._started = False

    def accept(self, telemetry: VehicleTelemetryValue) -> None:
        if telemetry.start_event:
            if self._started:
                raise ProtocolError(
                    ProtocolErrorCode.START_EVENT_REPEATED,
                    "start event is one-shot",
                )
            if telemetry.route_stage is not RouteStage.START:
                raise ProtocolError(
                    ProtocolErrorCode.INVALID_ROUTE_ORDER,
                    "start event must use START stage",
                )
            self._started = True

        if telemetry.lap_complete != (telemetry.route_stage is RouteStage.COMPLETE):
            raise ProtocolError(
                ProtocolErrorCode.INVALID_ROUTE_ORDER,
                "completion flag and stage disagree",
            )
        if self._stage is None:
            if telemetry.route_stage is not RouteStage.START:
                raise ProtocolError(
                    ProtocolErrorCode.INVALID_ROUTE_ORDER,
                    "first route stage must be START",
                )
            self._stage = RouteStage.START
            return
        if telemetry.route_stage is self._stage:
            return
        if not self._started or int(telemetry.route_stage) != int(self._stage) + 1:
            raise ProtocolError(
                ProtocolErrorCode.INVALID_ROUTE_ORDER,
                "route must advance START-B-D-A-COMPLETE",
            )
        self._stage = telemetry.route_stage

    def reset(self) -> None:
        self._stage = None
        self._started = False
