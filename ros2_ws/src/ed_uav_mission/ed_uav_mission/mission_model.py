"""Pydantic v2 models for typed YAML mission profiles.

References validated field profiles from ``ed_uav_localization`` and defines
mission-specific types: waypoints, no-fly overlays, and typed parameter sets
for every built-in mission plugin.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Literal

from ed_uav_localization.field_profile.model import (
    Identifier,
    KnownFieldProfile,
    Point2D,
)
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    FiniteFloat,
    TypeAdapter,
    model_validator,
)


class MissionType(str, Enum):
    COVERAGE = "coverage"
    PATROL = "patrol"
    TARGET_VISIT = "target_visit"
    PAYLOAD = "payload"
    COMPETITION = "competition"
    STABILITY_TEST = "stability_test"


class Waypoint(BaseModel):
    """A planar SI/ENU waypoint the flight controller can reach."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    x_m: FiniteFloat
    y_m: FiniteFloat
    altitude_m: FiniteFloat = Field(default=2.0, ge=0.0)
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


class CompetitionParams(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    mission_profile_id: Identifier
    deployment_preset_id: Identifier
    target_revision: Literal["d2026-apriltag-v1", "d2026-circle-cross-v1"]
    mission_variant: Literal["competition", "stability"] = "competition"
    altitude_m: FiniteFloat = Field(default=1.5, gt=0.0)
    stability_params: StabilityParams | None = None
    forward_distance_m: FiniteFloat = Field(default=2.0, gt=0.0, le=50.0)
    right_offset_m: FiniteFloat = Field(default=0.75, gt=0.0, le=5.0)
    search_distance_m: FiniteFloat = Field(default=2.0, gt=0.0, le=10.0)
    stable_sec: FiniteFloat = Field(default=3.0, ge=3.0, le=3.0)
    start_deadline_s: FiniteFloat = Field(default=15.0, gt=0.0, le=15.0)
    b_deadline_s: FiniteFloat = Field(default=45.0, gt=15.0, lt=75.0)
    d_deadline_s: FiniteFloat = Field(default=75.0, gt=45.0, lt=90.0)
    vehicle_freshness_s: FiniteFloat = Field(default=0.5, gt=0.0, le=0.5)
    target_freshness_s: FiniteFloat = Field(default=0.2, gt=0.0, le=0.2)
    maximum_relative_error_m: FiniteFloat = Field(default=2.0, gt=0.0, le=5.0)
    planner_timeout_sec: FiniteFloat = Field(default=5.0, gt=0.0, le=30.0)


class StabilityParams(BaseModel):
    """Geometry and timing for the stability-test mission."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    # 飞行高度
    altitude_m: FiniteFloat = Field(default=1.5, gt=0.0, le=10.0)
    # 起飞后悬停
    pre_hover_sec: FiniteFloat = Field(default=5.0, gt=0.0, le=20.0)
    # 降落前悬停
    post_hover_sec: FiniteFloat = Field(default=5.0, gt=0.0, le=20.0)
    # 正方形边长
    square_side_m: FiniteFloat = Field(default=2.0, gt=0.0, le=20.0)
    # 正方形细分距离
    square_segment_m: FiniteFloat = Field(default=0.5, gt=0.05, le=5.0)
    # 圆形直径
    circle_diameter_m: FiniteFloat = Field(default=2.0, gt=0.0, le=20.0)
    # 圆形细分距离
    circle_segment_m: FiniteFloat = Field(default=0.5, gt=0.05, le=5.0)
    # 航向锁定公差
    heading_hold_tolerance_rad: FiniteFloat = Field(default=0.05, gt=0.0, le=0.5)


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
    takeoff_altitude_m: FiniteFloat = Field(default=0.5, gt=0.0)
    coverage: CoverageParams | None = None
    patrol: PatrolParams | None = None
    target_visit: TargetVisitParams | None = None
    payload: PayloadParams | None = None
    competition: CompetitionParams | None = None
    stability_params: StabilityParams | None = None
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
            case MissionType.COMPETITION:
                if self.competition is None:
                    raise ValueError("competition mission requires competition params")
                # if self.takeoff_altitude_m != 1.5 or self.timeout_sec != 90.0:
                #     raise ValueError(
                #         "2026 competition missions require 1.5 m takeoff altitude and 90 s deadline"
                #     )
            case MissionType.STABILITY_TEST:
                if self.stability_params is None:
                    raise ValueError("stability_test mission requires stability_params")
        return self


MISSION_SCHEMA = TypeAdapter(MissionConfig)


def validate_mission_against_field(
    mission: MissionConfig, field_profile: KnownFieldProfile
) -> None:
    """Raise ``ValueError`` when mission waypoints or zones fall outside the field.

    Delegates geometry containment checks to the field-profile polygon operators.
    """

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
    if mission.stability_params is not None:
        from ed_uav_mission.competition_tree import MapPose

        start = MapPose(
            x_m=field_profile.takeoff.origin.x_m,
            y_m=field_profile.takeoff.origin.y_m,
            yaw_rad=field_profile.takeoff.commanded_heading_rad,
        )
        waypoints.append(
            Waypoint(
                x_m=start.x_m,
                y_m=start.y_m,
                altitude_m=mission.stability_params.altitude_m,
                heading_rad=start.yaw_rad,
                label="stability_takeoff",
            )
        )

    if mission.competition is not None:
        from ed_uav_mission.competition_tree import MapPose, forward_goal

        start = MapPose(
            x_m=field_profile.takeoff.origin.x_m,
            y_m=field_profile.takeoff.origin.y_m,
            yaw_rad=field_profile.takeoff.commanded_heading_rad,
        )
        forward = forward_goal(start, mission.competition.forward_distance_m)
        waypoints.append(
            Waypoint(
                x_m=forward.x_m,
                y_m=forward.y_m,
                altitude_m=mission.competition.altitude_m,
                heading_rad=forward.yaw_rad,
                label="competition_forward",
            )
        )

    for wp in waypoints:
        if not _point_inside(Point2D(x_m=wp.x_m, y_m=wp.y_m)):
            raise ValueError(
                f"waypoint {wp.label or (wp.x_m, wp.y_m)} outside allowed zone or inside no-fly zone"
            )
