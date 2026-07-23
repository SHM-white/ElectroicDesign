"""Target-visit plugin — approach then hover at a target location."""

from __future__ import annotations

import math

from ed_uav_mission.mission_model import TargetVisitParams, Waypoint


class TargetVisitPlugin:
    """Generate a two-step approach: offset loiter, then close-in visit."""

    def generate(self, params: TargetVisitParams) -> list[Waypoint]:
        approach = self._compute_approach(params)
        return [
            Waypoint(
                x_m=approach.x_m,
                y_m=approach.y_m,
                altitude_m=params.target.altitude_m,
                heading_rad=approach.heading_rad,
                hover_sec=params.standoff_sec,
                label="approach",
            ),
            Waypoint(
                x_m=params.target.x_m,
                y_m=params.target.y_m,
                altitude_m=params.target.altitude_m,
                heading_rad=params.target.heading_rad,
                hover_sec=params.standoff_sec,
                label="visit",
            ),
        ]

    @staticmethod
    def _compute_approach(params: TargetVisitParams) -> Waypoint:
        offset = params.approach_offset_m
        heading = params.target.heading_rad
        return Waypoint(
            x_m=params.target.x_m - offset * math.cos(heading),
            y_m=params.target.y_m - offset * math.sin(heading),
            altitude_m=params.target.altitude_m,
            heading_rad=heading,
        )
