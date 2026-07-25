from ed_uav_mission.mission_model import TerminalLandingParams
from ed_uav_mission.plugins.terminal_landing import LandingStep, TerminalLandingPlugin


def test_terminal_landing_accepts_ground_altitude() -> None:
    # Given: a terminal plan that descends to the physical ground boundary.
    params = TerminalLandingParams(land_altitude_m=0.0)

    # When: the typed landing steps are generated.
    plan = TerminalLandingPlugin().generate(0.0, 0.0, params)

    # Then: zero altitude remains a valid target before LAND and DISARM.
    assert [step for step, _ in plan] == [
        LandingStep.DESCEND,
        LandingStep.LAND,
        LandingStep.DISARM,
    ]
    assert plan[0][1] is not None
    assert plan[0][1].altitude_m == 0.0
