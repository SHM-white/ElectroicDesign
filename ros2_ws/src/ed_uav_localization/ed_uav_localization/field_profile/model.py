"""Pydantic models for strict, SI/ENU field profiles."""

from __future__ import annotations

from typing import Annotated, Literal, TypeAlias

from typing_extensions import Self, assert_never

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    FiniteFloat,
    StringConstraints,
    TypeAdapter,
    model_validator,
)

from ed_uav_localization.field_profile.geometry import (
    has_nonparallel_segments,
    polygon_has_area,
    polygon_self_intersects,
    polygon_strictly_contains,
    polygons_touch_or_overlap,
    segments_intersect,
)


Identifier = Annotated[
    str,
    StringConstraints(pattern=r"^[a-z][a-z0-9-]{0,63}$", min_length=1, max_length=64),
]
NonEmptyText = Annotated[str, StringConstraints(min_length=1, max_length=256, strip_whitespace=True)]


class StrictProfileModel(BaseModel):
    """Forbid undeclared profile fields at every YAML nesting level."""

    model_config = ConfigDict(extra="forbid", frozen=True)


class Units(StrictProfileModel):
    length: Literal["m"]
    angle: Literal["rad"]


class Frame(StrictProfileModel):
    id: Literal["map"]
    convention: Literal["ENU"]


class Point2D(StrictProfileModel):
    x_m: FiniteFloat
    y_m: FiniteFloat


class Takeoff(StrictProfileModel):
    origin: Point2D
    commanded_heading_rad: FiniteFloat


class Color(StrictProfileModel):
    id: Identifier
    label: NonEmptyText


class BoundarySegment(StrictProfileModel):
    id: Identifier
    start: Point2D
    end: Point2D
    color_id: Identifier

    @model_validator(mode="after")
    def require_nonzero_length(self) -> Self:
        if self.start == self.end:
            raise ValueError("boundary segment must have nonzero length")
        return self


class Polygon(StrictProfileModel):
    id: Identifier
    vertices: Annotated[tuple[Point2D, ...], Field(min_length=3, max_length=128)]

    @model_validator(mode="after")
    def require_non_degenerate_shape(self) -> Self:
        if self.vertices[0] == self.vertices[-1]:
            raise ValueError("polygon must not repeat its closing vertex")
        if polygon_self_intersects(self.vertices):
            raise ValueError("polygon self-intersects")
        if not polygon_has_area(self.vertices):
            raise ValueError("polygon must have nonzero area")
        return self


class Altitude(StrictProfileModel):
    minimum_m: FiniteFloat
    maximum_m: FiniteFloat
    takeoff_m: FiniteFloat

    @model_validator(mode="after")
    def require_ordered_limits(self) -> Self:
        if not self.minimum_m <= self.takeoff_m <= self.maximum_m:
            raise ValueError("altitude requires minimum_m <= takeoff_m <= maximum_m")
        return self


class Landmark(StrictProfileModel):
    id: Identifier
    kind: Identifier
    position: Point2D


class Provenance(StrictProfileModel):
    classification: Literal["current_field", "historical_example"]
    activation: Literal["eligible", "blocked"]

    @model_validator(mode="after")
    def block_historical_examples(self) -> Self:
        match (self.classification, self.activation):
            case ("current_field", _):
                return self
            case ("historical_example", "blocked"):
                return self
            case ("historical_example", "eligible"):
                raise ValueError("historical_example profiles must have blocked activation")
            case unreachable:
                assert_never(unreachable)


class KnownFieldProfile(StrictProfileModel):
    version: Literal[1]
    profile_type: Literal["field"]
    profile_id: Identifier
    units: Units
    frame: Frame
    provenance: Provenance
    takeoff: Takeoff
    colors: Annotated[tuple[Color, ...], Field(min_length=1, max_length=32)]
    boundary_segments: Annotated[tuple[BoundarySegment, ...], Field(min_length=1, max_length=256)]
    allowed_zone: Polygon
    no_fly_zones: tuple[Polygon, ...]
    altitude: Altitude
    landmarks: tuple[Landmark, ...]

    @model_validator(mode="after")
    def require_observable_non_overlapping_geometry(self) -> Self:
        _require_unique_ids(self)
        color_ids = {color.id for color in self.colors}
        for segment in self.boundary_segments:
            if segment.color_id not in color_ids:
                raise ValueError(f"boundary segment {segment.id} references unknown color {segment.color_id}")
        for zone in self.no_fly_zones:
            if not polygon_strictly_contains(self.allowed_zone.vertices, zone.vertices):
                raise ValueError(f"no_fly_zone {zone.id} must be strictly inside allowed_zone")
        for index, first_zone in enumerate(self.no_fly_zones):
            for second_zone in self.no_fly_zones[index + 1 :]:
                if polygons_touch_or_overlap(first_zone.vertices, second_zone.vertices):
                    raise ValueError("no_fly_zones overlap")
        for index, first_segment in enumerate(self.boundary_segments):
            for second_segment in self.boundary_segments[index + 1 :]:
                if _segments_cross_without_shared_endpoint(first_segment, second_segment):
                    raise ValueError("boundary_segments self-intersect")
        segment_pairs = tuple((segment.start, segment.end) for segment in self.boundary_segments)
        if not has_nonparallel_segments(segment_pairs):
            raise ValueError("boundary_segments require at least two non-parallel segments")
        return self


class UnknownArenaProfile(StrictProfileModel):
    version: Literal[1]
    profile_type: Literal["unknown"]
    profile_id: Identifier
    units: Units
    frame: Frame
    activation: Literal["blocked"]
    reason: NonEmptyText


FieldProfile: TypeAlias = KnownFieldProfile | UnknownArenaProfile
PROFILE_SCHEMA = TypeAdapter(Annotated[FieldProfile, Field(discriminator="profile_type")])


def _require_unique_ids(profile: KnownFieldProfile) -> None:
    identifiers = (
        *(color.id for color in profile.colors),
        *(segment.id for segment in profile.boundary_segments),
        profile.allowed_zone.id,
        *(zone.id for zone in profile.no_fly_zones),
        *(landmark.id for landmark in profile.landmarks),
    )
    seen: set[str] = set()
    for identifier in identifiers:
        if identifier in seen:
            raise ValueError(f"duplicate ID: {identifier}")
        seen.add(identifier)


def _segments_cross_without_shared_endpoint(
    first: BoundarySegment, second: BoundarySegment
) -> bool:
    shared_endpoints = {first.start, first.end}.intersection({second.start, second.end})
    return not shared_endpoints and segments_intersect(first.start, first.end, second.start, second.end)
