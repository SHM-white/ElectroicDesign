"""Stable by-id and serial binding rules for independent UVC devices."""

from __future__ import annotations

from dataclasses import dataclass

from .model import CameraRole


@dataclass(frozen=True, slots=True)
class DuplicateSerialError(Exception):
    """Raised when a serial would bind more than one camera namespace."""

    serial: str

    def __str__(self) -> str:
        return f"duplicate camera serial: {self.serial}"


@dataclass(frozen=True, slots=True)
class SerialMismatchError(Exception):
    """Raised when a declared by-id path resolves to the wrong serial."""

    expected_serial: str
    observed_serial: str
    by_id: str

    def __str__(self) -> str:
        return (
            f"camera serial mismatch at {self.by_id}: expected {self.expected_serial}, "
            f"observed {self.observed_serial}"
        )


@dataclass(frozen=True, slots=True)
class MissingCameraError(Exception):
    """Raised when a declared stable by-id path was not observed during preflight."""

    by_id: str

    def __str__(self) -> str:
        return f"camera not observed at {self.by_id}"


@dataclass(frozen=True, slots=True)
class CameraBinding:
    """One camera role's immutable stable device identity contract."""

    role: CameraRole
    serial: str
    by_id: str


@dataclass(frozen=True, slots=True)
class ObservedCamera:
    """Enumeration output supplied by the hardware-owned preflight step."""

    serial: str
    by_id: str


@dataclass(frozen=True, slots=True)
class BoundCamera:
    """A validated role-to-device relation suitable for stream startup."""

    role: CameraRole
    serial: str
    by_id: str


def bind_observed_cameras(
    bindings: tuple[CameraBinding, ...], observed_cameras: tuple[ObservedCamera, ...]
) -> tuple[BoundCamera, ...]:
    """Validate unique expected serials and exact matching at each declared by-id path."""
    expected_serials = tuple(binding.serial for binding in bindings)
    duplicate_expected = _first_duplicate(expected_serials)
    if duplicate_expected is not None:
        raise DuplicateSerialError(duplicate_expected)

    observed_serials = tuple(camera.serial for camera in observed_cameras)
    duplicate_observed = _first_duplicate(observed_serials)
    if duplicate_observed is not None:
        raise DuplicateSerialError(duplicate_observed)

    observed_by_path = {camera.by_id: camera for camera in observed_cameras}
    result: list[BoundCamera] = []
    for binding in bindings:
        if not binding.by_id.startswith("/dev/v4l/by-id/"):
            raise MissingCameraError(binding.by_id)
        observed = observed_by_path.get(binding.by_id)
        if observed is None:
            raise MissingCameraError(binding.by_id)
        if observed.serial != binding.serial:
            raise SerialMismatchError(binding.serial, observed.serial, binding.by_id)
        result.append(BoundCamera(binding.role, binding.serial, binding.by_id))
    return tuple(result)


def _first_duplicate(values: tuple[str, ...]) -> str | None:
    seen: set[str] = set()
    for value in values:
        if value in seen:
            return value
        seen.add(value)
    return None
