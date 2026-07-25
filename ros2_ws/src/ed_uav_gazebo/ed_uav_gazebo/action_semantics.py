"""Closed FCU command vocabulary and completion policies for simulation."""

from enum import IntEnum
from typing_extensions import assert_never


class CommandKind(IntEnum):
    """Commands accepted by the simulator FCU action server."""

    ARM = 1
    DISARM = 2
    SET_MODE = 3
    TAKEOFF = 4
    MOVE = 5
    HOVER = 6
    LAND = 7


def command_from_value(value: int) -> CommandKind | None:
    """Parse an action command at the ROS boundary."""
    try:
        return CommandKind(value)
    except ValueError:
        return None


def requires_armed_vehicle(command: CommandKind) -> bool:
    """Return whether a command requires the simulated motors to be armed."""
    match command:
        case CommandKind.ARM | CommandKind.DISARM | CommandKind.SET_MODE:
            return False
        case CommandKind.TAKEOFF | CommandKind.MOVE | CommandKind.HOVER | CommandKind.LAND:
            return True
        case unreachable:
            assert_never(unreachable)


def bounded_timeout(requested: float, default: float = 30.0) -> float:
    """Return a finite action timeout with a bounded upper limit."""
    if requested <= 0.0:
        return default
    return min(requested, 120.0)
