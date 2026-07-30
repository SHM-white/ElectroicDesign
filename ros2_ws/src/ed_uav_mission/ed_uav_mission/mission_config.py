"""Load and validate mission configuration bundles at the file boundary."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from ed_uav_description.calibration import CalibrationError, load_calibration
from ed_uav_description.yaml_boundary import load_strict_yaml
from ed_uav_localization.field_profile.loader import load_profile
from ed_uav_localization.field_profile.model import KnownFieldProfile
from ed_uav_mission.mission_model import (
    MISSION_SCHEMA,
    MissionConfig,
    validate_mission_against_field,
)


@dataclass(frozen=True, slots=True)
class MissionBundle:
    """Validated field and mission configuration used by one executor."""

    profile: KnownFieldProfile
    mission: MissionConfig


def calibration_file_is_valid(
    calibration_path: Path,
    *,
    simulation_only: bool = False,
) -> bool:
    """Return whether calibration parses, hashes, and fits mission mode."""
    try:
        calibration = load_calibration(calibration_path)
    except CalibrationError:
        return False
    expected_status = "SYNTHETIC" if simulation_only else "CALIBRATED"
    return calibration.calibration_status == expected_status


def parse_mission_config_text(yaml_text: str) -> MissionConfig:
    """Parse YAML mission text into the validated mission model."""
    return MISSION_SCHEMA.validate_python(load_strict_yaml(yaml_text, "<inline mission>"))


def load_mission_bundle(
    profile_path: Path,
    mission_path: Path,
    *,
    allow_blocked_profile: bool = False,
) -> MissionBundle:
    """Load a known field profile and mission, then validate their relationship."""
    profile = load_profile(profile_path)
    if not isinstance(profile, KnownFieldProfile):
        raise ValueError("mission requires a known field profile")
    if profile.provenance.activation == "blocked":
        if not allow_blocked_profile:
            raise ValueError("blocked field profiles require simulation_only=true")
        if profile.provenance.classification != "synthetic_simulation":
            raise ValueError("only synthetic simulation profiles may run while blocked")

    try:
        mission_source = mission_path.read_text(encoding="utf-8")
    except OSError as error:
        raise ValueError(f"cannot read mission config {mission_path}: {error}") from error
    mission_document = load_strict_yaml(mission_source, str(mission_path))
    mission = MISSION_SCHEMA.validate_python(mission_document)
    if mission.field_profile_id != profile.profile_id:
        raise ValueError(
            f"mission field_profile_id {mission.field_profile_id} does not match "
            f"profile {profile.profile_id}"
        )
    _validate_altitudes(mission, profile)
    validate_mission_against_field(mission, profile)
    return MissionBundle(profile=profile, mission=mission)


def _validate_altitudes(mission: MissionConfig, profile: KnownFieldProfile) -> None:
    """Ensure configured takeoff and generated patrol altitudes fit the field."""
    limits = profile.altitude
    altitudes = [mission.takeoff_altitude_m]
    if mission.patrol is not None:
        altitudes.append(mission.patrol.altitude_m)
    if mission.target_visit is not None:
        altitudes.append(mission.target_visit.target.altitude_m)
    if mission.competition is not None:
        altitudes.append(mission.competition.altitude_m)
    if mission.stability_params is not None:
        altitudes.append(mission.stability_params.altitude_m)
    if any(not limits.minimum_m <= altitude <= limits.maximum_m for altitude in altitudes):
        raise ValueError("mission altitude is outside field altitude bounds")
