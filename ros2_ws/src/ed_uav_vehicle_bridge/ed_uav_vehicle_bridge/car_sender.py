"""Authenticated outbound UDP sequencing for the CAR peer."""

from __future__ import annotations

import secrets
import time

from .models import (
    BootId,
    Endpoint,
    MessageType,
    OutboundFrame,
    SenderId,
    Sequence,
    SourceMillis,
)
from .protocol import encode_datagram
from .udp_socket import BoundUdpSocket


class CarSender:
    """Own the bridge→CAR boot epoch and monotonic outbound sequence."""

    def __init__(
        self,
        socket: BoundUdpSocket,
        destination: Endpoint,
        sender_id: SenderId,
        key: bytes,
    ) -> None:
        self._socket = socket
        self._destination = destination
        self._sender_id = sender_id
        self._key = key
        self._boot_epoch = BootId(secrets.randbits(32) or 1)
        self._sequence = 0

    def send(self, message_type: MessageType, payload: bytes) -> None:
        frame = OutboundFrame(
            message_type=message_type,
            sender_id=self._sender_id,
            boot_id=self._boot_epoch,
            sequence=Sequence(self._sequence),
            source_millis=SourceMillis(int(time.monotonic() * 1000) & 0xFFFFFFFF),
            payload=payload,
        )
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        self._socket.send(
            encode_datagram(frame, self._key),
            self._destination,
        )
