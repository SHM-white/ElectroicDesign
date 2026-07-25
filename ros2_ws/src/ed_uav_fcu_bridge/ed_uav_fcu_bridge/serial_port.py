"""OS-locked, exclusive native serial endpoint ownership."""

from __future__ import annotations

import fcntl
import os
import stat
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
        canonical_path, identity = self._resolve_endpoint()
        self._acquire_lock(identity)
        serial_fd: int | None = None
        try:
            try:
                serial_fd = os.open(
                    canonical_path,
                    os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW,
                )
                opened_stat = os.fstat(serial_fd)
                opened_identity = (os.major(opened_stat.st_rdev), os.minor(opened_stat.st_rdev))
                if not stat.S_ISCHR(opened_stat.st_mode) or opened_identity != identity:
                    raise SerialOpenError(f"serial endpoint identity changed: {self.device}")
                tty.setraw(serial_fd)
                attributes = termios.tcgetattr(serial_fd)
                speed = getattr(termios, f"B{self.baudrate}", None)
                if speed is None:
                    raise SerialOpenError(f"unsupported baudrate: {self.baudrate}")
                attributes[4] = speed
                attributes[5] = speed
                termios.tcsetattr(serial_fd, termios.TCSANOW, attributes)
                fcntl.ioctl(serial_fd, termios.TIOCEXCL)
            except OSError as error:
                raise SerialOpenError(f"cannot exclusively open {self.device}: {error}") from error
            self._serial_fd = serial_fd
            serial_fd = None
        finally:
            try:
                if serial_fd is not None:
                    os.close(serial_fd)
            finally:
                if self._serial_fd is None:
                    self._release_lock()

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
        try:
            if serial_fd is not None:
                try:
                    fcntl.ioctl(serial_fd, termios.TIOCNXCL)
                except OSError:
                    pass
                finally:
                    os.close(serial_fd)
        finally:
            self._release_lock()

    def _resolve_endpoint(self) -> tuple[Path, tuple[int, int]]:
        try:
            canonical_path = Path(self.device).resolve(strict=True)
            endpoint_stat = canonical_path.stat(follow_symlinks=False)
        except (OSError, RuntimeError) as error:
            raise SerialOpenError(f"cannot resolve serial endpoint {self.device}: {error}") from error
        if not stat.S_ISCHR(endpoint_stat.st_mode):
            raise SerialOpenError(f"serial endpoint is not a character device: {self.device}")
        identity = (os.major(endpoint_stat.st_rdev), os.minor(endpoint_stat.st_rdev))
        return canonical_path, identity

    def _acquire_lock(self, identity: tuple[int, int]) -> None:
        self.lock_dir.mkdir(parents=True, exist_ok=True)
        major, minor = identity
        lock_path = self.lock_dir / f"ed-uav-fcu-{major}-{minor}.lock"
        lock_fd = os.open(
            lock_path,
            os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW,
            0o600,
        )
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            os.close(lock_fd)
            raise SerialOwnershipError(f"FCU serial endpoint is already owned: {self.device}") from error
        except OSError:
            os.close(lock_fd)
            raise
        self._lock_fd = lock_fd

    def _release_lock(self) -> None:
        lock_fd = self._lock_fd
        self._lock_fd = None
        if lock_fd is not None:
            try:
                fcntl.flock(lock_fd, fcntl.LOCK_UN)
            finally:
                os.close(lock_fd)
