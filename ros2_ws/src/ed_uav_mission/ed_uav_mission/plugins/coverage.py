"""Grid-coverage path generator that avoids no-fly zones.

Produces a boustrophedon (lawnmower) sweep over every cell whose centre
falls inside the allowed zone and outside every no-fly zone.
"""

from __future__ import annotations

from ed_uav_localization.field_profile.geometry import (
    point_in_polygon,
    point_on_polygon_boundary,
)
from ed_uav_localization.field_profile.model import KnownFieldProfile, Point2D
from ed_uav_mission.mission_model import CoverageParams, NoFlyZone, Waypoint


class GridCoveragePlugin:
    """Stateless generator: call ``generate`` once per mission config."""

    def generate(
        self,
        field_profile: KnownFieldProfile,
        params: CoverageParams,
    ) -> list[Waypoint]:
        allowed = field_profile.allowed_zone.vertices
        no_fly_zones = self._build_no_fly_zones(field_profile)
        cell = params.cell_size_m
        bounds = self._axis_aligned_bounds(allowed)
        rows = self._rows(bounds, cell)

        waypoints: list[Waypoint] = []
        for row_index, (y_centre, reverse) in enumerate(rows):
            x_centres = self._x_centres_for_row(bounds, cell, reverse)
            for x_centre in x_centres:
                centre = Point2D(x_m=x_centre, y_m=y_centre)
                if self._cell_eligible(centre, cell, allowed, no_fly_zones):
                    waypoints.append(
                        Waypoint(
                            x_m=x_centre,
                            y_m=y_centre,
                            altitude_m=params.altitude_m,
                            heading_rad=0.0 if not reverse else 3.1416,
                        )
                    )
        return waypoints

    @staticmethod
    def _build_no_fly_zones(profile: KnownFieldProfile) -> list[NoFlyZone]:
        return [
            NoFlyZone(vertices=zone.vertices, zone_id=zone.id)
            for zone in profile.no_fly_zones
        ]

    @staticmethod
    def _axis_aligned_bounds(
        vertices: tuple[Point2D, ...],
    ) -> tuple[float, float, float, float]:
        xs = [v.x_m for v in vertices]
        ys = [v.y_m for v in vertices]
        return min(xs), max(xs), min(ys), max(ys)

    def _rows(
        self, bounds: tuple[float, float, float, float], cell: float
    ) -> list[tuple[float, bool]]:
        _, _, y_min, y_max = bounds
        rows: list[tuple[float, bool]] = []
        y = y_min + cell / 2.0
        row_index = 0
        while y < y_max:
            rows.append((y, row_index % 2 == 1))
            y += cell
            row_index += 1
        return rows

    def _x_centres_for_row(
        self,
        bounds: tuple[float, float, float, float],
        cell: float,
        reverse: bool,
    ) -> list[float]:
        x_min, x_max, _, _ = bounds
        centres: list[float] = []
        x = x_min + cell / 2.0
        while x < x_max:
            centres.append(x)
            x += cell
        if reverse:
            centres.reverse()
        return centres

    def _cell_eligible(
        self,
        centre: Point2D,
        cell: float,
        allowed: tuple[Point2D, ...],
        no_fly: list[NoFlyZone],
    ) -> bool:
        half = cell / 2.0
        corners = [
            Point2D(x_m=centre.x_m - half, y_m=centre.y_m - half),
            Point2D(x_m=centre.x_m + half, y_m=centre.y_m - half),
            Point2D(x_m=centre.x_m + half, y_m=centre.y_m + half),
            Point2D(x_m=centre.x_m - half, y_m=centre.y_m + half),
        ]
        for corner in corners:
            if not point_in_polygon(corner, allowed) or point_on_polygon_boundary(
                corner, allowed
            ):
                return False
        for nfz in no_fly:
            for corner in corners:
                if point_in_polygon(corner, nfz.vertices) or point_on_polygon_boundary(
                    corner, nfz.vertices
                ):
                    return False
        return True
