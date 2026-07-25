"""Strict, hash-bound calibration parsing for the static sensor model."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Final

from ed_uav_description.yaml_boundary import StrictYamlError, load_strict_yaml


SCHEMA_VERSION: Final = 1
SENSOR_NAMES: Final = ("camera_narrow", "camera_wide", "lidar")
FRAME_NAMES: Final = (
    "fcu_link",
    "lidar_link",
    "camera_narrow_optical_frame",
    "camera_wide_optical_frame",
    "rangefinder_link",
)
SUPPORTED_PROFILES: Final = ("offline", "camera_only", "lidar", "competition")
REQUIRED_FIELDS: Final = (
    "schema_version",
    "calibration_id",
    "calibration_status",
    "calibration_hash",
    "sensor_serials",
    "transforms",
)


@dataclass(frozen=True, slots=True)
class CalibrationError(Exception):
    reason: str

    def __str__(self) -> str:
        return self.reason


@dataclass(frozen=True, slots=True)
class Transform:
    xyz_m: tuple[float, float, float]
    rpy_rad: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class Calibration:
    calibration_id: str
    calibration_status: str
    calibration_hash: str
    sensor_serials: tuple[tuple[str, str], ...]
    transforms: tuple[tuple[str, Transform], ...]

    def serial_for(self, sensor_name: str) -> str:
        return dict(self.sensor_serials)[sensor_name]

    def transform_for(self, frame_name: str) -> Transform:
        return dict(self.transforms)[frame_name]


@dataclass(frozen=True, slots=True)
class ExpectedSerials:
    camera_narrow: str
    camera_wide: str
    lidar: str

    def values(self) -> tuple[tuple[str, str], ...]:
        return (
            ("camera_narrow", self.camera_narrow),
            ("camera_wide", self.camera_wide),
            ("lidar", self.lidar),
        )


def _mapping(value: object, label: str) -> dict[str, object]:
    if not isinstance(value, dict):
        raise CalibrationError(f"{label} must be a mapping")
    if not all(isinstance(key, str) for key in value):
        raise CalibrationError(f"{label} has a non-string key")
    return value


def _string(mapping: dict[str, object], field_name: str) -> str:
    value = mapping.get(field_name)
    if not isinstance(value, str) or not value.strip():
        raise CalibrationError(f"{field_name} must be a non-empty string")
    return value


def _triple(value: object, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise CalibrationError(f"{label} must contain exactly three numeric values")
    numbers: list[float] = []
    for number in value:
        if isinstance(number, bool) or not isinstance(number, (int, float)):
            raise CalibrationError(f"{label} must contain only numeric values")
        numeric = float(number)
        if not math.isfinite(numeric):
            raise CalibrationError(f"{label} must contain finite values")
        numbers.append(numeric)
    return (numbers[0], numbers[1], numbers[2])


def calibration_hash(document: dict[str, object]) -> str:
    unsigned_document = dict(document)
    unsigned_document.pop("calibration_hash", None)
    canonical = json.dumps(unsigned_document, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def load_calibration(path: Path) -> Calibration:
    try:
        source = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise CalibrationError(f"missing calibration: {path}") from exc
    try:
        document = _mapping(load_strict_yaml(source, str(path)), "calibration")
    except StrictYamlError as exc:
        raise CalibrationError(f"malformed calibration: {exc.reason}") from exc

    if set(document) != set(REQUIRED_FIELDS):
        raise CalibrationError("calibration fields do not match schema version 1")
    if document.get("schema_version") != SCHEMA_VERSION:
        raise CalibrationError("unsupported calibration schema_version")
    recorded_hash = _string(document, "calibration_hash")
    if recorded_hash != calibration_hash(document):
        raise CalibrationError("calibration hash mismatch")

    sensor_serials = _mapping(document["sensor_serials"], "sensor_serials")
    if set(sensor_serials) != set(SENSOR_NAMES):
        raise CalibrationError("sensor_serials do not match required sensor identities")
    transforms = _mapping(document["transforms"], "transforms")
    if set(transforms) != set(FRAME_NAMES):
        raise CalibrationError("transforms do not match approved static frames")

    parsed_transforms: list[tuple[str, Transform]] = []
    for frame_name in FRAME_NAMES:
        transform = _mapping(transforms[frame_name], f"transforms.{frame_name}")
        if set(transform) != {"xyz_m", "rpy_rad"}:
            raise CalibrationError(f"transforms.{frame_name} must contain xyz_m and rpy_rad")
        parsed_transforms.append(
            (frame_name, Transform(_triple(transform["xyz_m"], f"{frame_name}.xyz_m"), _triple(transform["rpy_rad"], f"{frame_name}.rpy_rad")))
        )

    return Calibration(
        calibration_id=_string(document, "calibration_id"),
        calibration_status=_string(document, "calibration_status"),
        calibration_hash=recorded_hash,
        sensor_serials=tuple((sensor_name, _string(sensor_serials, sensor_name)) for sensor_name in SENSOR_NAMES),
        transforms=tuple(parsed_transforms),
    )


def validate_for_profile(calibration: Calibration, profile: str, expected_serials: ExpectedSerials) -> None:
    if profile not in SUPPORTED_PROFILES:
        raise CalibrationError(f"unsupported profile: {profile}")
    if profile != "competition":
        return
    if calibration.calibration_status != "CALIBRATED":
        raise CalibrationError("calibration status must be CALIBRATED for competition")
    for sensor_name, expected_serial in expected_serials.values():
        if not expected_serial or expected_serial == "UNSET":
            raise CalibrationError(f"expected serial missing: {sensor_name}")
        if calibration.serial_for(sensor_name) != expected_serial:
            raise CalibrationError(f"sensor serial mismatch: {sensor_name}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate an ED UAV calibration before launch.")
    parser.add_argument("--profile", required=True, choices=SUPPORTED_PROFILES)
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--camera-narrow-serial", default="UNSET")
    parser.add_argument("--camera-wide-serial", default="UNSET")
    parser.add_argument("--lidar-serial", default="UNSET")
    arguments = parser.parse_args(argv)
    try:
        calibration = load_calibration(arguments.calibration)
        validate_for_profile(
            calibration,
            arguments.profile,
            ExpectedSerials(arguments.camera_narrow_serial, arguments.camera_wide_serial, arguments.lidar_serial),
        )
    except CalibrationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"CALIBRATION: GREEN profile={arguments.profile} calibration_id={calibration.calibration_id}")
    return 0
