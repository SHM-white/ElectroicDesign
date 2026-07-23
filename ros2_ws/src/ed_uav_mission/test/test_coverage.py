"""Tests for the grid-coverage path generator."""

from __future__ import annotations

import pytest

from ed_uav_localization.field_profile.model import (
    Altitude,
    Color,
    Frame,
    KnownFieldProfile,
    Point2D,
    Polygon,
    Provenance,
    Takeoff,
    Units,
)
from ed_uav_mission.mission_model import CoverageParams
from ed_uav_mission.plugins.coverage import GridCoveragePlugin
from ed_uav_localization.field_profile.geometry import (
    point_in_polygon,
    point_on_polygon_boundary,
)


def _make_rectangle_profile(
    x_min: float, x_max: float, y_min: float, y_max: float,
    no_fly_zones: tuple[Polygon, ...] = (),
) -> KnownFieldProfile:
    return KnownFieldProfile(
        version=1,
        profile_type="field",
        profile_id="test-field",
        units=Units(length="m", angle="rad"),
        frame=Frame(id="map", convention="ENU"),
        provenance=Provenance(classification="current_field", activation="eligible"),
        takeoff=Takeoff(
            origin=Point2D(x_m=x_min + 2.0, y_m=y_min + 2.0),
            commanded_heading_rad=0.0,
        ),
        colors=(Color(id="red", label="Red"),),
        boundary_segments=(
            _segment("seg-e", x_min, y_min, x_max, y_min, "red"),
            _segment("seg-n", x_max, y_min, x_max, y_max, "red"),
            _segment("seg-w", x_max, y_max, x_min, y_max, "red"),
            _segment("seg-s", x_min, y_max, x_min, y_min, "red"),
        ),
        allowed_zone=Polygon(
            id="field-zone",
            vertices=(
                Point2D(x_m=x_min, y_m=y_min),
                Point2D(x_m=x_max, y_m=y_min),
                Point2D(x_m=x_max, y_m=y_max),
                Point2D(x_m=x_min, y_m=y_max),
            ),
        ),
        no_fly_zones=no_fly_zones,
        altitude=Altitude(minimum_m=0.5, takeoff_m=3.0, maximum_m=10.0),
        landmarks=(),
    )


def _segment(
    sid: str, sx: float, sy: float, ex: float, ey: float, color: str
):
    from ed_uav_localization.field_profile.model import BoundarySegment

    return BoundarySegment(
        id=sid,
        start=Point2D(x_m=sx, y_m=sy),
        end=Point2D(x_m=ex, y_m=ey),
        color_id=color,
    )


def test_coverage_generates_valid_path() -> None:
    """Grid path covers all cells in the allowed zone when there are no obstacles."""
    profile = _make_rectangle_profile(0.0, 10.0, 0.0, 10.0)
    plugin = GridCoveragePlugin()
    params = CoverageParams(cell_size_m=2.0, altitude_m=3.0, speed_m_s=2.0)

    waypoints = plugin.generate(profile, params)

    assert len(waypoints) > 0, "grid must produce at least one waypoint"

    allowed = profile.allowed_zone.vertices
    for wp in waypoints:
        pt = Point2D(x_m=wp.x_m, y_m=wp.y_m)
        assert point_in_polygon(pt, allowed), (
            f"waypoint ({wp.x_m:.2f}, {wp.y_m:.2f}) outside allowed zone"
        )
        assert not point_on_polygon_boundary(pt, allowed), (
            f"waypoint ({wp.x_m:.2f}, {wp.y_m:.2f}) on boundary"
        )


def test_coverage_avoids_no_fly_zones() -> None:
    """Path avoids cells that intersect a no-fly zone."""
    no_fly = Polygon(
        id="obstacle",
        vertices=(
            Point2D(x_m=4.0, y_m=4.0),
            Point2D(x_m=6.0, y_m=4.0),
            Point2D(x_m=6.0, y_m=6.0),
            Point2D(x_m=4.0, y_m=6.0),
        ),
    )
    profile = _make_rectangle_profile(0.0, 10.0, 0.0, 10.0, no_fly_zones=(no_fly,))
    plugin = GridCoveragePlugin()
    params = CoverageParams(cell_size_m=1.0, altitude_m=3.0, speed_m_s=2.0)

    waypoints = plugin.generate(profile, params)

    for wp in waypoints:
        pt = Point2D(x_m=wp.x_m, y_m=wp.y_m)
        assert not point_in_polygon(pt, no_fly.vertices), (
            f"waypoint ({wp.x_m:.2f}, {wp.y_m:.2f}) inside no-fly zone"
        )
        assert not point_on_polygon_boundary(pt, no_fly.vertices), (
            f"waypoint ({wp.x_m:.2f}, {wp.y_m:.2f}) on no-fly boundary"
        )
