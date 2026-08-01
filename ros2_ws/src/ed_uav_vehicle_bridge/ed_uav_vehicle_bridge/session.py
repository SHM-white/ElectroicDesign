"""Endpoint-bound boot sessions, sequence replay checks, and freshness."""

from collections import deque
from dataclasses import dataclass
from typing import Final

from .errors import ProtocolError, ProtocolErrorCode
from .models import (
    AcceptedPacket,
    AuthenticatedDatagram,
    BootId,
    Endpoint,
    MessageType,
    ReceiptSeconds,
    RejectCode,
    RouteEvent,
    SenderId,
    TelemetryFault,
    VehicleTelemetryValue,
)


RETIRED_EPOCH_LIMIT: Final = 32


@dataclass(frozen=True, slots=True)
class PeerPolicy:
    sender_id: SenderId
    endpoint: Endpoint
    allowed_types: frozenset[MessageType]


class SessionTracker:
    """Mutable replay state for one configured UDP peer."""

    def __init__(self, policy: PeerPolicy) -> None:
        self._policy = policy
        self._boot_id: BootId | None = None
        self._last_sequence: int | None = None
        self._retired_boot_ids: deque[BootId] = deque(maxlen=RETIRED_EPOCH_LIMIT)
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
                f"sender endpoint is not the provisioned peer "
                f"(source={source} policy={self._policy.endpoint} "
                f"sender={frame.sender_id} policy_sender={self._policy.sender_id})",
            )
        if frame.message_type not in self._policy.allowed_types:
            raise ProtocolError(
                ProtocolErrorCode.MESSAGE_TYPE_FORBIDDEN,
                "message type is not allowed for peer",
            )

        session_changed = self._boot_id != frame.boot_id
        if session_changed:
            if frame.boot_id in self._retired_boot_ids:
                raise ProtocolError(
                    ProtocolErrorCode.RETIRED_BOOT_EPOCH,
                    "boot epoch was already retired",
                )
            if self._boot_id is not None:
                self._retired_boot_ids.append(self._boot_id)
            self._boot_id = frame.boot_id
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
        self._last_sequence = frame.sequence
        self._last_receipt = receipt_time
        self._stale_reported = False
        return AcceptedPacket(datagram=datagram, session_changed=session_changed)

    def telemetry_fault_if_stale(
        self, now: ReceiptSeconds, stale_after_seconds: float
    ) -> TelemetryFault | None:
        if self._last_receipt is None or self._boot_id is None or self._stale_reported:
            return None
        age = now - self._last_receipt
        if age <= stale_after_seconds:
            return None
        self._stale_reported = True
        return TelemetryFault(
            code=RejectCode.TELEMETRY_STALE,
            age_seconds=age,
            car_boot_epoch=BootId(self._boot_id),
        )


class RouteTracker:
    """Mutable one-run route order and start-event guard."""

    def __init__(self) -> None:
        self._event: RouteEvent | None = None
        self._event_id: int | None = None

    def accept(self, telemetry: VehicleTelemetryValue) -> None:
        event = telemetry.event
        if event is RouteEvent.NONE:
            return

        if self._event is None:
            if event is not RouteEvent.START:
                raise ProtocolError(
                    ProtocolErrorCode.INVALID_ROUTE_ORDER,
                    "first route event must be START",
                )
            self._event = event
            self._event_id = telemetry.event_id
            return

        if event is self._event:
            if telemetry.event_id == self._event_id:
                return
            if event is RouteEvent.START:
                raise ProtocolError(
                    ProtocolErrorCode.START_EVENT_REPEATED,
                    "start event is one-shot",
                )
            raise ProtocolError(
                ProtocolErrorCode.INVALID_ROUTE_ORDER,
                "route event ID changed without advancing the route",
            )

        if telemetry.event_id == self._event_id or int(event) != int(self._event) + 1:
            raise ProtocolError(
                ProtocolErrorCode.INVALID_ROUTE_ORDER,
                "route must advance START-B-D-A-COMPLETE",
            )
        self._event = event
        self._event_id = telemetry.event_id

    def reset(self) -> None:
        self._event = None
        self._event_id = None
