"""Authenticated outbound UDP sequencing for the HMI peer."""

from dataclasses import dataclass
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


@dataclass(frozen=True, slots=True)
class HmiSenderConfig:
    socket: BoundUdpSocket
    destination: Endpoint
    sender_id: SenderId
    key: bytes


class HmiSender:
    """Own the bridge boot epoch and monotonic outbound sequence."""

    def __init__(self, config: HmiSenderConfig) -> None:
        self._config = config
        self._boot_epoch = BootId(secrets.randbits(32) or 1)
        self._sequence = 0

    def send(self, message_type: MessageType, payload: bytes) -> None:
        frame = OutboundFrame(
            message_type=message_type,
            sender_id=self._config.sender_id,
            boot_id=self._boot_epoch,
            sequence=Sequence(self._sequence),
            source_millis=SourceMillis(int(time.monotonic() * 1000) & 0xFFFFFFFF),
            payload=payload,
        )
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        self._config.socket.send(
            encode_datagram(frame, self._config.key),
            self._config.destination,
        )
