"""Tests for the waypoint-patrol plugin."""

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
from ed_uav_mission.mission_model import PatrolParams, Waypoint
from ed_uav_mission.plugins.patrol import WaypointPatrolPlugin


def _make_square_profile(side: float = 10.0) -> KnownFieldProfile:
    return KnownFieldProfile(
        version=1,
        profile_type="field",
        profile_id="test-patrol",
        units=Units(length="m", angle="rad"),
        frame=Frame(id="map", convention="ENU"),
        provenance=Provenance(classification="current_field", activation="eligible"),
        takeoff=Takeoff(
            origin=Point2D(x_m=1.0, y_m=1.0),
            commanded_heading_rad=0.0,
        ),
        colors=(Color(id="blue", label="Blue"),),
        boundary_segments=(
            _seg("b1", 0, 0, side, 0, "blue"),
            _seg("b2", side, 0, side, side, "blue"),
            _seg("b3", side, side, 0, side, "blue"),
            _seg("b4", 0, side, 0, 0, "blue"),
        ),
        allowed_zone=Polygon(
            id="zone",
            vertices=(
                Point2D(x_m=0.0, y_m=0.0),
                Point2D(x_m=side, y_m=0.0),
                Point2D(x_m=side, y_m=side),
                Point2D(x_m=0.0, y_m=side),
            ),
        ),
        no_fly_zones=(),
        altitude=Altitude(minimum_m=0.5, takeoff_m=3.0, maximum_m=10.0),
        landmarks=(),
    )


def _seg(sid: str, sx: float, sy: float, ex: float, ey: float, cid: str):
    from ed_uav_localization.field_profile.model import BoundarySegment

    return BoundarySegment(
        id=sid,
        start=Point2D(x_m=sx, y_m=sy),
        end=Point2D(x_m=ex, y_m=ey),
        color_id=cid,
    )


def test_patrol_follows_waypoints() -> None:
    """Patrol returns all waypoints in the correct order."""
    profile = _make_square_profile()
    params = PatrolParams(
        waypoints=(
            Waypoint(x_m=2.0, y_m=2.0, label="wp-a"),
            Waypoint(x_m=8.0, y_m=2.0, label="wp-b"),
            Waypoint(x_m=8.0, y_m=8.0, label="wp-c"),
            Waypoint(x_m=2.0, y_m=8.0, label="wp-d"),
        ),
        loop_count=2,
    )
    plugin = WaypointPatrolPlugin()
    result = plugin.generate(profile, params)

    assert len(result) == 8
    labels = [wp.label for wp in result]
    assert labels == ["wp-a", "wp-b", "wp-c", "wp-d", "wp-a", "wp-b", "wp-c", "wp-d"]


def test_patrol_rejects_waypoint_outside_zone() -> None:
    """A waypoint outside the allowed zone raises ValueError."""
    profile = _make_square_profile(10.0)
    params = PatrolParams(
        waypoints=(
            Waypoint(x_m=5.0, y_m=5.0, label="ok"),
            Waypoint(x_m=15.0, y_m=5.0, label="outside"),
        ),
    )
    plugin = WaypointPatrolPlugin()
    with pytest.raises(ValueError, match="outside allowed zone"):
        plugin.generate(profile, params)
