"""Waypoint patrol plugin — visits waypoints in order.

The plugin is stateless; cancel and timeout are enforced by the executor
that calls it.  It delegates no-fly-zone avoidance to the field-profile
validation done during mission-model construction.
"""

from __future__ import annotations

from ed_uav_localization.field_profile.geometry import (
    point_in_polygon,
    point_on_polygon_boundary,
)
from ed_uav_localization.field_profile.model import KnownFieldProfile
from ed_uav_mission.mission_model import PatrolParams, Waypoint


class WaypointPatrolPlugin:
    """Verify and return the ordered waypoint list for one patrol mission."""

    def generate(
        self,
        field_profile: KnownFieldProfile,
        params: PatrolParams,
    ) -> list[Waypoint]:
        allowed = field_profile.allowed_zone.vertices
        no_fly_zones = field_profile.no_fly_zones

        for wp in params.waypoints:
            self._validate_waypoint(wp, allowed, no_fly_zones)

        waypoints: list[Waypoint] = []
        for _ in range(params.loop_count):
            for wp in params.waypoints:
                waypoints.append(
                    Waypoint(
                        x_m=wp.x_m,
                        y_m=wp.y_m,
                        altitude_m=params.altitude_m,
                        heading_rad=wp.heading_rad,
                        hover_sec=params.loiter_sec,
                        label=wp.label,
                    )
                )
        return waypoints

    @staticmethod
    def _validate_waypoint(
        wp: Waypoint,
        allowed: tuple,
        no_fly_zones: tuple,
    ) -> None:
        from ed_uav_localization.field_profile.model import Point2D

        pt = Point2D(x_m=wp.x_m, y_m=wp.y_m)
        if not point_in_polygon(pt, allowed):
            raise ValueError(f"patrol waypoint ({wp.x_m:.2f}, {wp.y_m:.2f}) outside allowed zone")
        if point_on_polygon_boundary(pt, allowed):
            raise ValueError(f"patrol waypoint ({wp.x_m:.2f}, {wp.y_m:.2f}) on boundary")
        for nfz in no_fly_zones:
            if point_in_polygon(pt, nfz.vertices) or point_on_polygon_boundary(
                pt, nfz.vertices
            ):
                raise ValueError(
                    f"patrol waypoint ({wp.x_m:.2f}, {wp.y_m:.2f}) inside no-fly zone {nfz.id}"
                )
