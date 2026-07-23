"""Pseudo-terminal V7 FCU fake with deterministic legacy-compatible telemetry."""

from __future__ import annotations

import os
import select
import struct
import termios
import tty
from dataclasses import dataclass
from types import TracebackType

from .v7 import V7Frame, encode_v7_frame


@dataclass(frozen=True, slots=True)
class FcuReadTimeout(Exception):
    """Raised when the PTY slave does not receive its bounded fake FCU frame."""

    def __str__(self) -> str:
        return "timed out waiting for deterministic FCU PTY frame"


def position_v7_frame(x_cm: int, y_cm: int) -> bytes:
    """Build the legacy-characterized native `0x08` optical-flow position frame."""
    return encode_v7_frame(V7Frame(address=0xFF, frame_id=0x08, payload=struct.pack("<ii", x_cm, y_cm)))


class DeterministicPtyFcu:
    """A context-managed PTY endpoint; it owns no hardware and closes both FDs."""

    def __init__(self) -> None:
        self._master_fd, self._slave_fd = os.openpty()
        tty.setraw(self._slave_fd, when=termios.TCSANOW)
        self._closed = False

    @property
    def slave_path(self) -> str:
        """Return the temporary serial endpoint path exposed to a bridge under test."""
        return os.ttyname(self._slave_fd)

    @property
    def closed(self) -> bool:
        """Return whether both pseudo-terminal descriptors are closed."""
        return self._closed

    def emit_position(self, x_cm: int, y_cm: int) -> bytes:
        """Write one valid native V7 `0x08` position sample to the master endpoint."""
        frame = position_v7_frame(x_cm=x_cm, y_cm=y_cm)
        os.write(self._master_fd, frame)
        return frame

    def read_slave_frame(self) -> bytes:
        """Read one bounded fake FCU frame from the bridge-facing slave endpoint."""
        readable, _, _ = select.select((self._slave_fd,), (), (), 0.2)
        if not readable:
            raise FcuReadTimeout()
        return os.read(self._slave_fd, 512)

    def close(self) -> None:
        """Close the exact temporary descriptors created by this fake."""
        if self._closed:
            return
        os.close(self._master_fd)
        os.close(self._slave_fd)
        self._closed = True

    def __enter__(self) -> DeterministicPtyFcu:
        return self

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()
