"""Owned nonblocking UDP socket boundary."""

from dataclasses import dataclass
import socket
from types import TracebackType

from .errors import BridgeConfigError, SocketClosedError
from .models import Endpoint
from .protocol import MAXIMUM_DATAGRAM_BYTES


@dataclass(frozen=True, slots=True)
class ReceivedDatagram:
    data: bytes
    source: Endpoint


class BoundUdpSocket:
    """Context-managed UDP socket owned exclusively by the vehicle bridge."""

    def __init__(self, endpoint: Endpoint) -> None:
        if not 0 <= endpoint.port <= 65535:
            raise BridgeConfigError("port", "must be in range 0-65535")
        owned = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            owned.bind((endpoint.host, endpoint.port))
            owned.setblocking(False)
        except OSError:
            owned.close()
            raise
        host, port = owned.getsockname()
        self._socket: socket.socket | None = owned
        self._endpoint = Endpoint(str(host), int(port))

    def __enter__(self) -> "BoundUdpSocket":
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    @property
    def endpoint(self) -> Endpoint:
        return self._endpoint

    def fileno(self) -> int:
        return -1 if self._socket is None else self._socket.fileno()

    def receive(self, maximum_packets: int) -> tuple[ReceivedDatagram, ...]:
        if not 1 <= maximum_packets <= 1024:
            raise BridgeConfigError("maximum_packets", "must be in range 1-1024")
        owned = self._require_socket("receive")
        packets: list[ReceivedDatagram] = []
        for _ in range(maximum_packets):
            try:
                data, source = owned.recvfrom(MAXIMUM_DATAGRAM_BYTES + 1)
            except BlockingIOError:
                break
            packets.append(
                ReceivedDatagram(
                    data=data,
                    source=Endpoint(str(source[0]), int(source[1])),
                )
            )
        return tuple(packets)

    def send(self, data: bytes, destination: Endpoint) -> None:
        self._require_socket("send").sendto(data, (destination.host, destination.port))

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def _require_socket(self, operation: str) -> socket.socket:
        if self._socket is None:
            raise SocketClosedError(operation)
        return self._socket
