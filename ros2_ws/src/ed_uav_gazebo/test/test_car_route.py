import math

import pytest

from ed_uav_gazebo.car_route import (
    A,
    B,
    C,
    D,
    TOTAL_LENGTH_M,
    CapsuleRouteFollower,
    build_capsule_route,
)


def test_capsule_route_matches_d_task_landmarks_and_closes() -> None:
    points = build_capsule_route(samples_per_section=8)

    assert (points[0].x_m, points[0].y_m) == A
    assert (points[8].x_m, points[8].y_m) == pytest.approx(B)
    assert (points[16].x_m, points[16].y_m) == pytest.approx(C)
    assert (points[24].x_m, points[24].y_m) == pytest.approx(D)
    assert (points[-1].x_m, points[-1].y_m) == pytest.approx(A)
    assert points[-1].displacement_m == pytest.approx(TOTAL_LENGTH_M, rel=0.01)


def test_route_follower_reports_monotonic_b_d_a_complete_events() -> None:
    follower = CapsuleRouteFollower(speed_m_s=0.15)
    stages: list[int] = []
    result = None
    for point in follower.points[1:]:
        next_index = min(follower.index + 1, len(follower.points) - 1)
        ahead = follower.points[next_index]
        yaw = math.atan2(ahead.y_m - point.y_m, ahead.x_m - point.x_m)
        result = follower.command(point.x_m, point.y_m, yaw)
        stages.append(result.stage)

    assert result is not None and result.complete
    assert stages == sorted(stages)
    assert {0, 1, 2, 3, 4}.issubset(stages)
