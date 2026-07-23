"""Serial- and resolution-bound CameraInfo calibration gates."""

from __future__ import annotations

from dataclasses import dataclass

from .identity import CameraBinding
from .profiles import CameraMode


@dataclass(frozen=True, slots=True)
class CalibrationSerialMismatchError(Exception):
    """Raised when camera-info metadata belongs to another physical camera."""

    expected_serial: str
    calibration_serial: str

    def __str__(self) -> str:
        return (
            f"calibration serial mismatch: expected {self.expected_serial}, "
            f"received {self.calibration_serial}"
        )


@dataclass(frozen=True, slots=True)
class CalibrationResolutionMismatchError(Exception):
    """Raised when camera-info raster dimensions do not match the selected profile."""

    calibration_width: int
    calibration_height: int
    stream_width: int
    stream_height: int

    def __str__(self) -> str:
        return (
            f"calibration {self.calibration_width}x{self.calibration_height} does not match "
            f"stream {self.stream_width}x{self.stream_height}"
        )


@dataclass(frozen=True, slots=True)
class StaleCalibrationError(Exception):
    """Raised when a serial-bound calibration has exceeded its validity interval."""

    serial: str
    expires_at_ns: int

    def __str__(self) -> str:
        return f"calibration for {self.serial} is stale after {self.expires_at_ns}"


@dataclass(frozen=True, slots=True)
class CalibrationDescriptor:
    """Metadata required before passing a calibration URL to camera_info_manager."""

    serial: str
    width: int
    height: int
    captured_at_ns: int
    valid_for_ns: int
    camera_info_url: str


def validate_calibration(
    binding: CameraBinding,
    calibration: CalibrationDescriptor,
    mode: CameraMode,
    now_ns: int,
) -> None:
    """Require exact serial/raster binding and an unexpired camera-info descriptor."""
    if calibration.serial != binding.serial:
        raise CalibrationSerialMismatchError(binding.serial, calibration.serial)
    if calibration.width != mode.width or calibration.height != mode.height:
        raise CalibrationResolutionMismatchError(
            calibration.width,
            calibration.height,
            mode.width,
            mode.height,
        )
    expires_at_ns = calibration.captured_at_ns + calibration.valid_for_ns
    if now_ns > expires_at_ns:
        raise StaleCalibrationError(binding.serial, expires_at_ns)
    if not calibration.camera_info_url.startswith("file://"):
        raise StaleCalibrationError(binding.serial, expires_at_ns)
