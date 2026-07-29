"""Stable V4L2 camera enumeration through Linux by-id and udev properties."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess

from .calibration_models import CalibrationBootstrapError


@dataclass(frozen=True, slots=True)
class StableVideoDevice:
    """One persistent V4L2 endpoint and its hardware serial."""

    serial: str
    by_id: str
    resolved_device: str


def enumerate_stable_video_devices(
    by_id_root: Path = Path("/dev/v4l/by-id"),
) -> tuple[StableVideoDevice, ...]:
    """Enumerate deterministic index-zero capture endpoints without persisting video numbers."""
    if not by_id_root.is_dir():
        raise CalibrationBootstrapError(f"stable V4L2 directory is absent: {by_id_root}")
    devices: list[StableVideoDevice] = []
    for path in sorted(by_id_root.glob("*-video-index0")):
        properties = _udev_properties(path)
        serial = properties.get("ID_SERIAL_SHORT")
        if not serial:
            raise CalibrationBootstrapError(f"camera at {path} has no ID_SERIAL_SHORT")
        devices.append(StableVideoDevice(serial, str(path), str(path.resolve(strict=True))))
    if not devices:
        raise CalibrationBootstrapError(f"no stable index-zero V4L2 devices found in {by_id_root}")
    serials = tuple(device.serial for device in devices)
    if len(serials) != len(set(serials)):
        raise CalibrationBootstrapError("stable V4L2 enumeration contains duplicate serials")
    return tuple(devices)


def _udev_properties(path: Path) -> dict[str, str]:
    try:
        completed = subprocess.run(
            ("udevadm", "info", "--query=property", "--name", str(path)),
            capture_output=True,
            text=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        raise CalibrationBootstrapError(f"cannot query udev identity for {path}: {error}") from error
    properties: dict[str, str] = {}
    for line in completed.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            properties[key] = value
    return properties
