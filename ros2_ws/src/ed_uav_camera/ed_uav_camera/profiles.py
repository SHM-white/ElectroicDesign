"""Candidate UVC profile parsing and USB2 controller-budget arithmetic."""

from __future__ import annotations

from dataclasses import dataclass
from enum import unique
import json
from pathlib import Path
from typing import TypeAlias

from typing_extensions import assert_never

from .model import CameraRole
from .string_enum import StrEnum

JsonValue: TypeAlias = str | int | float | bool | None | list["JsonValue"] | dict[str, "JsonValue"]
JsonObject: TypeAlias = dict[str, JsonValue]


@unique
class Compression(StrEnum):
    """Transport classes used by a profile candidate, not hardware claims."""

    MJPEG = "mjpeg"
    UNCOMPRESSED = "uncompressed"


@unique
class ProfileName(StrEnum):
    """Approved camera roles for calibration and live operation."""

    FULL_CALIBRATION = "full_calibration"
    WIDE_LIVE = "wide_live"
    NARROW_LIVE = "narrow_live"


@dataclass(frozen=True, slots=True)
class MalformedProfileError(Exception):
    """Raised when untrusted profile configuration cannot become a typed candidate."""

    detail: str

    def __str__(self) -> str:
        return f"malformed camera profile: {self.detail}"


@dataclass(frozen=True, slots=True)
class UnsupportedProfileError(Exception):
    """Raised when no declared candidate was observed during capability probing."""

    profile: ProfileName

    def __str__(self) -> str:
        return f"profile {self.profile.value} has no supported declared mode"


@dataclass(frozen=True, slots=True)
class ControllerBudgetError(Exception):
    """Raised when candidates overbook a single USB2 controller budget."""

    controller_id: str
    load_mbit_s: float
    budget_mbit_s: float

    def __str__(self) -> str:
        return (
            f"controller {self.controller_id} load {self.load_mbit_s:.3f} Mbit/s "
            f"exceeds budget {self.budget_mbit_s:.3f} Mbit/s"
        )


@dataclass(frozen=True, slots=True)
class CameraMode:
    """A requested or probed V4L2 mode with explicit planning payload evidence."""

    fourcc: str
    width: int
    height: int
    frames_per_second: int
    compression: Compression
    bits_per_pixel: int | None
    declared_peak_mbit_s: float | None

    def payload_mbit_s(self) -> float:
        """Calculate uncompressed payload or return declared compressed planning load."""
        match self.compression:
            case Compression.UNCOMPRESSED:
                if self.bits_per_pixel is None or self.bits_per_pixel <= 0:
                    raise MalformedProfileError("uncompressed mode requires positive bits_per_pixel")
                return self.width * self.height * self.frames_per_second * self.bits_per_pixel / 1_000_000
            case Compression.MJPEG:
                if self.declared_peak_mbit_s is None or self.declared_peak_mbit_s <= 0:
                    raise MalformedProfileError("mjpeg mode requires declared_peak_mbit_s")
                return self.declared_peak_mbit_s
            case unreachable:
                assert_never(unreachable)


@dataclass(frozen=True, slots=True)
class CameraProfile:
    """Ordered candidates for exactly one independent camera namespace."""

    name: ProfileName
    role: CameraRole | None
    candidates: tuple[CameraMode, ...]


@dataclass(frozen=True, slots=True)
class ControllerAssignment:
    """A negotiated camera mode located behind one physical host controller."""

    controller_id: str
    role: CameraRole
    mode: CameraMode


def select_supported_mode(profile: CameraProfile, supported_modes: tuple[CameraMode, ...]) -> CameraMode:
    """Select the first configured candidate explicitly reported by capability probing."""
    for candidate in profile.candidates:
        if candidate in supported_modes:
            return candidate
    raise UnsupportedProfileError(profile.name)


def evaluate_controller_budget(
    assignments: tuple[ControllerAssignment, ...], budget_mbit_s: float
) -> tuple[tuple[str, float], ...]:
    """Aggregate all cameras by host controller, irrespective of downstream hubs."""
    if budget_mbit_s <= 0:
        raise MalformedProfileError("controller budget must be positive")

    loads: dict[str, float] = {}
    for assignment in assignments:
        if not assignment.controller_id:
            raise MalformedProfileError("controller_id must be non-empty")
        loads[assignment.controller_id] = loads.get(assignment.controller_id, 0.0) + assignment.mode.payload_mbit_s()

    ordered_loads = tuple(sorted(loads.items()))
    for controller_id, load_mbit_s in ordered_loads:
        if load_mbit_s > budget_mbit_s:
            raise ControllerBudgetError(controller_id, load_mbit_s, budget_mbit_s)
    return ordered_loads


def parse_profile_catalog(catalog: JsonObject) -> tuple[CameraProfile, ...]:
    """Parse untrusted JSON-compatible profile data into immutable profile candidates."""
    raw_profiles = catalog.get("profiles")
    if not isinstance(raw_profiles, list) or not raw_profiles:
        raise MalformedProfileError("profiles must be a non-empty list")

    profiles = tuple(_parse_profile(raw_profile) for raw_profile in raw_profiles)
    names = tuple(profile.name for profile in profiles)
    if len(set(names)) != len(names):
        raise MalformedProfileError("profile names must be unique")
    return profiles


def load_profile_catalog(path: Path) -> tuple[CameraProfile, ...]:
    """Load a JSON-compatible YAML profile catalog without inventing hardware modes."""
    try:
        raw_catalog: JsonValue = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise MalformedProfileError(str(error)) from error
    if not isinstance(raw_catalog, dict):
        raise MalformedProfileError("catalog root must be an object")
    return parse_profile_catalog(raw_catalog)


def parse_camera_mode(raw_mode: JsonValue) -> CameraMode:
    """Parse one externally reported or configured mode into a typed candidate."""
    return _parse_mode(raw_mode)


def _parse_profile(raw_profile: JsonValue) -> CameraProfile:
    if not isinstance(raw_profile, dict):
        raise MalformedProfileError("profile must be an object")
    raw_name = raw_profile.get("name")
    raw_role = raw_profile.get("role")
    raw_candidates = raw_profile.get("candidates")
    if not isinstance(raw_name, str):
        raise MalformedProfileError("profile name must be text")
    if not isinstance(raw_role, str):
        raise MalformedProfileError("profile role must be text")
    if not isinstance(raw_candidates, list) or not raw_candidates:
        raise MalformedProfileError("profile candidates must be a non-empty list")
    try:
        name = ProfileName(raw_name)
    except ValueError as error:
        raise MalformedProfileError(str(error)) from error
    match raw_role:
        case "both":
            role = None
        case raw_camera_role:
            try:
                role = CameraRole(raw_camera_role)
            except ValueError as error:
                raise MalformedProfileError(str(error)) from error
    candidates = tuple(_parse_mode(raw_candidate) for raw_candidate in raw_candidates)
    return CameraProfile(name, role, candidates)


def _parse_mode(raw_mode: JsonValue) -> CameraMode:
    if not isinstance(raw_mode, dict):
        raise MalformedProfileError("mode must be an object")
    fourcc = raw_mode.get("fourcc")
    width = raw_mode.get("width")
    height = raw_mode.get("height")
    frames_per_second = raw_mode.get("frames_per_second")
    raw_compression = raw_mode.get("compression")
    bits_per_pixel = raw_mode.get("bits_per_pixel")
    declared_peak_mbit_s = raw_mode.get("declared_peak_mbit_s")
    if not isinstance(fourcc, str) or len(fourcc) != 4:
        raise MalformedProfileError("fourcc must be exactly four characters")
    if not isinstance(width, int) or width <= 0:
        raise MalformedProfileError("width must be a positive integer")
    if not isinstance(height, int) or height <= 0:
        raise MalformedProfileError("height must be a positive integer")
    if not isinstance(frames_per_second, int) or frames_per_second <= 0:
        raise MalformedProfileError("frames_per_second must be a positive integer")
    if not isinstance(raw_compression, str):
        raise MalformedProfileError("compression must be text")
    if bits_per_pixel is not None and (not isinstance(bits_per_pixel, int) or bits_per_pixel <= 0):
        raise MalformedProfileError("bits_per_pixel must be a positive integer")
    if declared_peak_mbit_s is not None and (
        not isinstance(declared_peak_mbit_s, (int, float)) or declared_peak_mbit_s <= 0
    ):
        raise MalformedProfileError("declared_peak_mbit_s must be positive")
    try:
        compression = Compression(raw_compression)
    except ValueError as error:
        raise MalformedProfileError(f"compression {raw_compression!r} is unsupported") from error
    return CameraMode(
        fourcc,
        width,
        height,
        frames_per_second,
        compression,
        bits_per_pixel,
        float(declared_peak_mbit_s) if declared_peak_mbit_s is not None else None,
    )
