"""Preflight a capability-probed dual-camera plan before launching V4L2 drivers."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing_extensions import assert_never

from .calibration import CalibrationDescriptor, validate_calibration
from .identity import CameraBinding, ObservedCamera, bind_observed_cameras
from .model import CameraRole
from .profiles import (
    CameraMode,
    CameraProfile,
    ControllerAssignment,
    JsonObject,
    JsonValue,
    MalformedProfileError,
    ProfileName,
    evaluate_controller_budget,
    load_profile_catalog,
    parse_camera_mode,
    select_supported_mode,
)


@dataclass(frozen=True, slots=True)
class RuntimePlanError(Exception):
    """Raised when a launch plan cannot safely create the contracted camera graph."""

    detail: str

    def __str__(self) -> str:
        return f"runtime camera plan: {self.detail}"


@dataclass(frozen=True, slots=True)
class RuntimeCamera:
    """Fully validated one-camera launch input."""

    binding: CameraBinding
    controller_id: str
    profile: ProfileName
    mode: CameraMode
    calibration: CalibrationDescriptor
    frame_id: str

    @property
    def image_topic(self) -> str:
        return f"/camera/{self.binding.role.value}/image_raw"

    @property
    def camera_info_topic(self) -> str:
        return f"/camera/{self.binding.role.value}/camera_info"


@dataclass(frozen=True, slots=True)
class RuntimePlan:
    """Validated pair of independent camera launch inputs on declared controllers."""

    controller_budget_mbit_s: float
    cameras: tuple[RuntimeCamera, ...]


def load_runtime_plan(plan_path: Path, catalog_path: Path, now_ns: int) -> RuntimePlan:
    """Load and validate external capability/calibration evidence before driver startup."""
    catalog = load_profile_catalog(catalog_path)
    raw_plan = _load_json_object(plan_path)
    return parse_runtime_plan(raw_plan, catalog, now_ns)


def parse_runtime_plan(
    raw_plan: JsonObject, catalog: tuple[CameraProfile, ...], now_ns: int
) -> RuntimePlan:
    """Convert an external runtime plan into the fixed P03 namespace graph."""
    budget = _positive_number(raw_plan.get("controller_budget_mbit_s"), "controller_budget_mbit_s")
    raw_cameras = raw_plan.get("cameras")
    if not isinstance(raw_cameras, list):
        raise RuntimePlanError("cameras must be a list")

    cameras = tuple(_parse_runtime_camera(raw_camera, catalog) for raw_camera in raw_cameras)
    roles = {camera.binding.role for camera in cameras}
    if roles != set(CameraRole):
        raise RuntimePlanError("plan must contain exactly narrow and wide cameras")

    bind_observed_cameras(
        tuple(camera.binding for camera in cameras),
        tuple(ObservedCamera(camera.binding.serial, camera.binding.by_id) for camera in cameras),
    )
    for camera in cameras:
        validate_calibration(camera.binding, camera.calibration, camera.mode, now_ns)
    evaluate_controller_budget(
        tuple(
            ControllerAssignment(camera.controller_id, camera.binding.role, camera.mode)
            for camera in cameras
        ),
        budget,
    )
    return RuntimePlan(budget, cameras)


def _parse_runtime_camera(raw_camera: JsonValue, catalog: tuple[CameraProfile, ...]) -> RuntimeCamera:
    if not isinstance(raw_camera, dict):
        raise RuntimePlanError("camera entry must be an object")
    role = _camera_role(raw_camera.get("role"))
    serial = _text(raw_camera.get("serial"), "serial")
    by_id = _text(raw_camera.get("by_id"), "by_id")
    observed_serial = _text(raw_camera.get("observed_serial"), "observed_serial")
    controller_id = _text(raw_camera.get("controller_id"), "controller_id")
    profile_name = _profile_name(raw_camera.get("profile"))
    frame_id = _text(raw_camera.get("frame_id"), "frame_id")
    mode = parse_camera_mode(raw_camera.get("mode"))
    calibration = _parse_calibration(raw_camera.get("calibration"))
    binding = CameraBinding(role, serial, by_id)
    _validate_observed_serial(binding, observed_serial)
    _validate_frame(binding.role, frame_id)
    profile = _find_profile(catalog, profile_name)
    if profile.role is not None and profile.role is not role:
        raise RuntimePlanError(f"profile {profile.name.value} cannot serve {role.value}")
    select_supported_mode(profile, (mode,))
    return RuntimeCamera(binding, controller_id, profile_name, mode, calibration, frame_id)


def _load_json_object(path: Path) -> JsonObject:
    try:
        raw_value: JsonValue = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimePlanError(str(error)) from error
    if not isinstance(raw_value, dict):
        raise RuntimePlanError("plan root must be an object")
    return raw_value


def _parse_calibration(raw_calibration: JsonValue) -> CalibrationDescriptor:
    if not isinstance(raw_calibration, dict):
        raise RuntimePlanError("calibration must be an object")
    return CalibrationDescriptor(
        _text(raw_calibration.get("serial"), "calibration serial"),
        _positive_integer(raw_calibration.get("width"), "calibration width"),
        _positive_integer(raw_calibration.get("height"), "calibration height"),
        _nonnegative_integer(raw_calibration.get("captured_at_ns"), "calibration captured_at_ns"),
        _positive_integer(raw_calibration.get("valid_for_ns"), "calibration valid_for_ns"),
        _text(raw_calibration.get("camera_info_url"), "camera_info_url"),
    )


def _validate_observed_serial(binding: CameraBinding, observed_serial: str) -> None:
    bind_observed_cameras((binding,), (ObservedCamera(observed_serial, binding.by_id),))


def _find_profile(catalog: tuple[CameraProfile, ...], profile_name: ProfileName) -> CameraProfile:
    for profile in catalog:
        if profile.name is profile_name:
            return profile
    raise RuntimePlanError(f"profile {profile_name.value} is absent from catalog")


def _validate_frame(role: CameraRole, frame_id: str) -> None:
    match role:
        case CameraRole.NARROW:
            expected_frame = "camera_narrow_optical_frame"
        case CameraRole.WIDE:
            expected_frame = "camera_wide_optical_frame"
        case unreachable:
            assert_never(unreachable)
    if frame_id != expected_frame:
        raise RuntimePlanError(f"{role.value} frame must be {expected_frame}")


def _camera_role(raw_role: JsonValue) -> CameraRole:
    if not isinstance(raw_role, str):
        raise RuntimePlanError("role must be text")
    try:
        return CameraRole(raw_role)
    except ValueError as error:
        raise RuntimePlanError(f"unsupported role {raw_role!r}") from error


def _profile_name(raw_profile: JsonValue) -> ProfileName:
    if not isinstance(raw_profile, str):
        raise RuntimePlanError("profile must be text")
    try:
        return ProfileName(raw_profile)
    except ValueError as error:
        raise RuntimePlanError(f"unsupported profile {raw_profile!r}") from error


def _text(raw_value: JsonValue, field: str) -> str:
    if not isinstance(raw_value, str) or not raw_value:
        raise RuntimePlanError(f"{field} must be non-empty text")
    return raw_value


def _positive_integer(raw_value: JsonValue, field: str) -> int:
    if not isinstance(raw_value, int) or raw_value <= 0:
        raise RuntimePlanError(f"{field} must be a positive integer")
    return raw_value


def _nonnegative_integer(raw_value: JsonValue, field: str) -> int:
    if not isinstance(raw_value, int) or raw_value < 0:
        raise RuntimePlanError(f"{field} must be a non-negative integer")
    return raw_value


def _positive_number(raw_value: JsonValue, field: str) -> float:
    if not isinstance(raw_value, (int, float)) or raw_value <= 0:
        raise RuntimePlanError(f"{field} must be positive")
    return float(raw_value)
