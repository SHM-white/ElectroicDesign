"""Pydantic v2 models for typed YAML mission profiles.

References validated field profiles from ``ed_uav_localization`` and defines
mission-specific types: waypoints, no-fly overlays, and typed parameter sets
for every built-in mission plugin.
"""

from __future__ import annotations

from enum import Enum
from typing import Annotated, Any, Literal

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    FiniteFloat,
    StringConstraints,
    TypeAdapter,
    model_validator,
)

from ed_uav_localization.field_profile.model import (
    Identifier,
    KnownFieldProfile,
    Point2D,
)


class MissionType(str, Enum):
    COVERAGE = "coverage"
    PATROL = "patrol"
    TARGET_VISIT = "target_visit"
    PAYLOAD = "payload"


class Waypoint(BaseModel):
    """A planar SI/ENU waypoint the flight controller can reach."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    x_m: FiniteFloat
    y_m: FiniteFloat
    altitude_m: FiniteFloat = Field(default=2.0, gt=0.0)
    heading_rad: FiniteFloat = Field(default=0.0, ge=-3.15, le=3.15)
    hover_sec: FiniteFloat = Field(default=0.0, ge=0.0)
    label: str = Field(default="", max_length=64)


class NoFlyZone(BaseModel):
    """A no-fly region for mission path planning, defined as a convex polygon."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    vertices: tuple[Point2D, ...] = Field(min_length=3, max_length=32)
    zone_id: Identifier


class CoverageParams(BaseModel):
    """Parameters for the grid-coverage mission plugin."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    cell_size_m: FiniteFloat = Field(default=2.0, gt=0.1, le=50.0)
    altitude_m: FiniteFloat = Field(default=3.0, gt=0.0)
    speed_m_s: FiniteFloat = Field(default=2.0, gt=0.0)
    overlap_m: FiniteFloat = Field(default=0.5, ge=0.0)


class PatrolParams(BaseModel):
    """Parameters for the waypoint-patrol mission plugin."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    waypoints: tuple[Waypoint, ...] = Field(min_length=2, max_length=256)
    altitude_m: FiniteFloat = Field(default=3.0, gt=0.0)
    speed_m_s: FiniteFloat = Field(default=2.0, gt=0.0)
    loiter_sec: FiniteFloat = Field(default=0.0, ge=0.0)
    loop_count: int = Field(default=1, ge=1, le=100)


class TargetVisitParams(BaseModel):
    """Parameters for the target-visit mission plugin."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    target: Waypoint
    approach_offset_m: FiniteFloat = Field(default=5.0, ge=1.0)
    standoff_sec: FiniteFloat = Field(default=3.0, ge=1.0)


class PayloadParams(BaseModel):
    """Parameters for the payload trigger plugin."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    action: Literal["laser_on", "laser_off", "led_on", "led_off"]
    duration_sec: FiniteFloat = Field(default=1.0, ge=0.1, le=60.0)


class TerminalLandingParams(BaseModel):
    """Parameters for the terminal-landing sequence."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    land_altitude_m: FiniteFloat = Field(default=0.0, ge=0.0)
    descent_speed_m_s: FiniteFloat = Field(default=0.5, gt=0.0)
    disarm_after_sec: FiniteFloat = Field(default=2.0, ge=0.0)


class MissionConfig(BaseModel):
    """A validated mission profile that drives one executor run."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    version: Literal[1]
    mission_id: Identifier
    mission_type: MissionType
    field_profile_id: Identifier
    timeout_sec: FiniteFloat = Field(default=0.0, ge=0.0)
    takeoff_altitude_m: FiniteFloat = Field(default=3.0, gt=0.0)
    coverage: CoverageParams | None = None
    patrol: PatrolParams | None = None
    target_visit: TargetVisitParams | None = None
    payload: PayloadParams | None = None
    terminal_landing: TerminalLandingParams | None = None

    @model_validator(mode="after")
    def require_matching_params(self) -> MissionConfig:
        match self.mission_type:
            case MissionType.COVERAGE:
                if self.coverage is None:
                    raise ValueError("coverage mission requires coverage params")
            case MissionType.PATROL:
                if self.patrol is None:
                    raise ValueError("patrol mission requires patrol params")
            case MissionType.TARGET_VISIT:
                if self.target_visit is None:
                    raise ValueError("target_visit mission requires target_visit params")
            case MissionType.PAYLOAD:
                if self.payload is None:
                    raise ValueError("payload mission requires payload params")
        return self


MISSION_SCHEMA = TypeAdapter(MissionConfig)


def validate_mission_against_field(
    mission: MissionConfig, field_profile: KnownFieldProfile
) -> None:
    """Raise ``ValueError`` when mission waypoints or zones fall outside the field.

    Delegates geometry containment checks to the field-profile polygon operators.
    """
    from ed_uav_localization.field_profile.geometry import (
        polygon_strictly_contains,
    )

    allowed = field_profile.allowed_zone.vertices
    no_fly_list = field_profile.no_fly_zones

    def _point_inside(p: Point2D) -> bool:
        from ed_uav_localization.field_profile.geometry import point_in_polygon

        return point_in_polygon(p, allowed) and all(
            not _in_no_fly_zone(p, nfz.vertices) for nfz in no_fly_list
        )

    def _in_no_fly_zone(point: Point2D, vertices: tuple[Point2D, ...]) -> bool:
        from ed_uav_localization.field_profile.geometry import (
            point_in_polygon,
            point_on_polygon_boundary,
        )

        return point_in_polygon(point, vertices) or point_on_polygon_boundary(point, vertices)

    waypoints: list[Waypoint] = []
    if mission.patrol is not None:
        waypoints.extend(mission.patrol.waypoints)
    if mission.target_visit is not None:
        waypoints.append(mission.target_visit.target)
    if mission.coverage is not None:
        pass

    for wp in waypoints:
        if not _point_inside(Point2D(x_m=wp.x_m, y_m=wp.y_m)):
            raise ValueError(
                f"waypoint {wp.label or (wp.x_m, wp.y_m)} outside allowed zone or inside no-fly zone"
            )
