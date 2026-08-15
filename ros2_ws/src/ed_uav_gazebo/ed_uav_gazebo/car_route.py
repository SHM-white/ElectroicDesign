"""Pure geometry and control helpers for the 2026 D-task capsule route."""

from __future__ import annotations

from dataclasses import dataclass
import math


A = (1.5, 2.0)
B = (1.5, 3.5)
C = (3.0, 3.5)
D = (3.0, 2.0)
RADIUS_M = 0.75
STRAIGHT_M = 1.5
ARC_M = math.pi * RADIUS_M
TOTAL_LENGTH_M = 2.0 * STRAIGHT_M + 2.0 * ARC_M


@dataclass(frozen=True, slots=True)
class RoutePoint:
    x_m: float
    y_m: float
    displacement_m: float


@dataclass(frozen=True, slots=True)
class RouteCommand:
    speed_m_s: float
    yaw_rate_rad_s: float
    displacement_m: float
    stage: int
    complete: bool


def wrap_angle(angle_rad: float) -> float:
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def build_capsule_route(samples_per_section: int = 32) -> tuple[RoutePoint, ...]:
    """Sample A→B→C→D→A with semicircular ends and cumulative distance."""
    if samples_per_section < 4:
        raise ValueError("samples_per_section must be at least 4")
    coordinates: list[tuple[float, float]] = []
    for index in range(samples_per_section + 1):
        ratio = index / samples_per_section
        coordinates.append((A[0], A[1] + STRAIGHT_M * ratio))
    for index in range(1, samples_per_section + 1):
        theta = math.pi - math.pi * index / samples_per_section
        coordinates.append((2.25 + RADIUS_M * math.cos(theta), 3.5 + RADIUS_M * math.sin(theta)))
    for index in range(1, samples_per_section + 1):
        ratio = index / samples_per_section
        coordinates.append((C[0], C[1] - STRAIGHT_M * ratio))
    for index in range(1, samples_per_section + 1):
        theta = -math.pi * index / samples_per_section
        coordinates.append((2.25 + RADIUS_M * math.cos(theta), 2.0 + RADIUS_M * math.sin(theta)))

    points: list[RoutePoint] = []
    displacement = 0.0
    previous = coordinates[0]
    for coordinate in coordinates:
        displacement += math.hypot(coordinate[0] - previous[0], coordinate[1] - previous[1])
        points.append(RoutePoint(coordinate[0], coordinate[1], displacement))
        previous = coordinate
    return tuple(points)


class CapsuleRouteFollower:
    """Monotonic waypoint follower; restart reset is intentionally sufficient."""

    def __init__(self, speed_m_s: float = 0.15) -> None:
        if not 0.0 < speed_m_s <= 1.0:
            raise ValueError("speed_m_s must be in (0, 1]")
        self.points = build_capsule_route()
        self.speed_m_s = speed_m_s
        self.index = 1

    def command(self, x_m: float, y_m: float, yaw_rad: float) -> RouteCommand:
        """Advance only forward through the path and return a body-frame command."""
        while self.index < len(self.points) - 1:
            target = self.points[self.index]
            if math.hypot(target.x_m - x_m, target.y_m - y_m) > 0.10:
                break
            self.index += 1
        target = self.points[self.index]
        distance = math.hypot(target.x_m - x_m, target.y_m - y_m)
        complete = self.index == len(self.points) - 1 and distance <= 0.12
        displacement = min(target.displacement_m, TOTAL_LENGTH_M)
        if complete:
            return RouteCommand(0.0, 0.0, TOTAL_LENGTH_M, 4, True)

        desired_yaw = math.atan2(target.y_m - y_m, target.x_m - x_m)
        error = wrap_angle(desired_yaw - yaw_rad)
        yaw_rate = max(-1.2, min(1.2, 2.4 * error))
        speed = self.speed_m_s * max(0.15, math.cos(error))
        if displacement < STRAIGHT_M - 0.08:
            stage = 0
        elif displacement < STRAIGHT_M + ARC_M + STRAIGHT_M - 0.08:
            stage = 1
        elif displacement < TOTAL_LENGTH_M - 0.12:
            stage = 2
        else:
            stage = 3
        return RouteCommand(speed, yaw_rate, displacement, stage, False)
