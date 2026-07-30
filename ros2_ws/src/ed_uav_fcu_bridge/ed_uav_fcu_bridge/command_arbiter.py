"""Shared semantic command arbitration and complete-frame serialization."""

from __future__ import annotations

import threading
from typing import Protocol


class WireWriter(Protocol):
    def __call__(self, data: bytes) -> int | None: ...


class CommandArbiter:
    """Allow one legacy ACK command or realtime stream to own the FCU."""

    def __init__(self) -> None:
        self._lock = threading.Lock()

    def try_acquire(self) -> bool:
        """Acquire command ownership without waiting for another operation."""
        return self._lock.acquire(blocking=False)

    def release(self) -> None:
        """Release ownership after ACK or realtime terminal stop frames."""
        self._lock.release()


class SerializedWireWriter:
    """Serialize complete frame writes after semantic ownership is granted."""

    def __init__(self, writer: WireWriter) -> None:
        self._writer = writer
        self._lock = threading.Lock()

    def __call__(self, data: bytes) -> int | None:
        with self._lock:
            return self._writer(data)
