"""Terminal landing sequence — descend, land, disarm."""

from __future__ import annotations

from enum import Enum, auto

from ed_uav_mission.mission_model import TerminalLandingParams, Waypoint


class LandingStep(Enum):
    DESCEND = auto()
    LAND = auto()
    DISARM = auto()


class TerminalLandingPlugin:
    """Produce the ordered landing-step list consumed by the executor."""

    def generate(
        self,
        current_x_m: float,
        current_y_m: float,
        params: TerminalLandingParams | None = None,
    ) -> list[tuple[LandingStep, Waypoint | None]]:
        if params is None:
            params = TerminalLandingParams()

        plan: list[tuple[LandingStep, Waypoint | None]] = []

        plan.append(
            (
                LandingStep.DESCEND,
                Waypoint(
                    x_m=current_x_m,
                    y_m=current_y_m,
                    altitude_m=params.land_altitude_m,
                    label="descend_to_land",
                ),
            )
        )
        plan.append((LandingStep.LAND, None))
        plan.append((LandingStep.DISARM, None))
        return plan
