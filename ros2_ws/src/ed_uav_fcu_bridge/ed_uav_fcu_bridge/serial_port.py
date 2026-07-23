"""OS-locked, exclusive native serial endpoint ownership."""

from __future__ import annotations

import fcntl
import hashlib
import os
import termios
import tty
from pathlib import Path
from types import TracebackType


class SerialOwnershipError(RuntimeError):
    """Raised when another process already owns the requested FCU endpoint."""


class SerialOpenError(RuntimeError):
    """Raised when an endpoint cannot be opened and configured as a serial TTY."""


class ExclusiveSerialPort:
    """Mutable RAII owner for one serial endpoint and its inter-process lock file."""

    def __init__(self, device: str, baudrate: int = 500000, lock_dir: Path = Path("/tmp")) -> None:
        self.device = device
        self.baudrate = baudrate
        self.lock_dir = lock_dir
        self._lock_fd: int | None = None
        self._serial_fd: int | None = None

    def __enter__(self) -> ExclusiveSerialPort:
        self.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        """Whether this object currently owns and has opened its endpoint."""
        return self._serial_fd is not None

    @property
    def fileno(self) -> int:
        """Return the native serial descriptor for selector-based callers."""
        if self._serial_fd is None:
            raise SerialOpenError("serial endpoint is not open")
        return self._serial_fd

    def open(self) -> None:
        """Acquire lock-file and TTY exclusivity before exposing the endpoint."""
        if self.is_open:
            return
        self._acquire_lock()
        try:
            serial_fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
            tty.setraw(serial_fd)
            attributes = termios.tcgetattr(serial_fd)
            speed = getattr(termios, f"B{self.baudrate}", None)
            if speed is None:
                raise SerialOpenError(f"unsupported baudrate: {self.baudrate}")
            attributes[4] = speed
            attributes[5] = speed
            termios.tcsetattr(serial_fd, termios.TCSANOW, attributes)
            fcntl.ioctl(serial_fd, termios.TIOCEXCL)
            self._serial_fd = serial_fd
        except OSError as error:
            self._release_lock()
            raise SerialOpenError(f"cannot exclusively open {self.device}: {error}") from error
        except SerialOpenError:
            self._release_lock()
            raise

    def read(self, maximum_bytes: int = 4096) -> bytes:
        """Read currently available endpoint bytes without waiting."""
        try:
            return os.read(self.fileno, maximum_bytes)
        except BlockingIOError:
            return b""

    def write(self, data: bytes) -> int:
        """Write a complete native V7 frame or raise on a short write."""
        written = os.write(self.fileno, data)
        if written != len(data):
            raise SerialOpenError("serial endpoint performed a short write")
        return written

    def close(self) -> None:
        """Release the TTY claim and lock file even during interrupted shutdown."""
        serial_fd = self._serial_fd
        self._serial_fd = None
        if serial_fd is not None:
            try:
                fcntl.ioctl(serial_fd, termios.TIOCNXCL)
            except OSError:
                pass
            os.close(serial_fd)
        self._release_lock()

    def _acquire_lock(self) -> None:
        self.lock_dir.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha256(self.device.encode("utf-8")).hexdigest()
        lock_path = self.lock_dir / f"ed-uav-fcu-{digest}.lock"
        lock_fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o600)
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            os.close(lock_fd)
            raise SerialOwnershipError(f"FCU serial endpoint is already owned: {self.device}") from error
        self._lock_fd = lock_fd

    def _release_lock(self) -> None:
        lock_fd = self._lock_fd
        self._lock_fd = None
        if lock_fd is not None:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
            os.close(lock_fd)
