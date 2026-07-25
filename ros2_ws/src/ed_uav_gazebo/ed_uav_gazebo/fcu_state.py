"""Pure FCU kinematic predicates used by the simulator action adapter."""

from dataclasses import dataclass
from math import sqrt


@dataclass(frozen=True, slots=True)
class Position3D:
    """A position in the simulator's ENU frame, measured in metres."""

    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class Velocity3D:
    """A linear velocity in the simulator's ENU frame, measured in m/s."""

    x: float
    y: float
    z: float


def reached_position(current: Position3D, target: Position3D, tolerance: float) -> bool:
    """Return whether two positions are within a Euclidean tolerance."""
    distance = sqrt(
        (current.x - target.x) ** 2
        + (current.y - target.y) ** 2
        + (current.z - target.z) ** 2
    )
    return distance <= tolerance


def reached_altitude(current: Position3D, target_z: float, tolerance: float) -> bool:
    """Return whether the vehicle has reached a requested altitude."""
    return abs(current.z - target_z) <= tolerance


def touched_down(current: Position3D, threshold: float) -> bool:
    """Return whether the vehicle is at or below the touchdown threshold."""
    return current.z <= threshold


def command_velocity(current: Position3D, target: Position3D, maximum: float) -> Velocity3D:
    """Create a magnitude-bounded proportional velocity command."""
    raw = Velocity3D(target.x - current.x, target.y - current.y, target.z - current.z)
    magnitude = sqrt(raw.x**2 + raw.y**2 + raw.z**2)
    scale = max(magnitude, maximum) / maximum
    return Velocity3D(raw.x / scale, raw.y / scale, raw.z / scale)
