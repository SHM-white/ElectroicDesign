"""Pure motion completion and command generation for the simulator FCU."""

from typing import Final

from typing_extensions import assert_never

from ed_uav_interfaces.action import FlightCommand
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from .action_semantics import CommandKind
from .fcu_state import (
    Position3D,
    Velocity3D,
    command_velocity,
    reached_altitude,
    reached_position,
    touched_down,
)


SIMULATOR_MOVE_SPEED_LIMIT_M_S: Final = 0.6


def motion_complete(command: CommandKind, current: Position3D, odometry: Odometry, goal: FlightCommand.Goal) -> bool:
    """Evaluate a motion goal using the latest actual odometry."""
    match command:
        case CommandKind.TAKEOFF:
            target_z = goal.target_pose.pose.position.z or 1.0
            return reached_altitude(current, target_z, 0.08)
        case CommandKind.MOVE:
            position = goal.target_pose.pose.position
            return reached_position(current, Position3D(position.x, position.y, position.z), 0.12)
        case CommandKind.HOVER:
            linear = odometry.twist.twist.linear
            return max(abs(linear.x), abs(linear.y), abs(linear.z)) < 0.12
        case CommandKind.LAND:
            return touched_down(current, 0.14)
        case unreachable:
            assert_never(unreachable)


def motion_command(
    command: CommandKind,
    current: Position3D,
    goal: FlightCommand.Goal,
    odometry: Odometry,
) -> Twist:
    """Create the body-frame velocity command required by Fortress."""
    match command:
        case CommandKind.TAKEOFF:
            target_z = goal.target_pose.pose.position.z or 1.0
            world_velocity = command_velocity(
                current,
                Position3D(current.x, current.y, target_z),
                1.0,
            )
        case CommandKind.MOVE:
            position = goal.target_pose.pose.position
            world_velocity = command_velocity(
                current,
                Position3D(position.x, position.y, position.z),
                SIMULATOR_MOVE_SPEED_LIMIT_M_S,
            )
        case CommandKind.HOVER:
            world_velocity = Velocity3D(0.0, 0.0, 0.0)
        case CommandKind.LAND:
            world_velocity = Velocity3D(0.0, 0.0, -0.45)
        case unreachable:
            assert_never(unreachable)

    body_velocity = _world_to_body(world_velocity, odometry)
    twist = Twist()
    twist.linear.x = body_velocity.x
    twist.linear.y = body_velocity.y
    twist.linear.z = body_velocity.z
    return twist


def _world_to_body(velocity: Velocity3D, odometry: Odometry) -> Velocity3D:
    """Rotate an ENU velocity through the inverse odometry orientation."""
    orientation = odometry.pose.pose.orientation
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return Velocity3D(
        (1.0 - 2.0 * (y * y + z * z)) * velocity.x
        + 2.0 * (x * y + z * w) * velocity.y
        + 2.0 * (x * z - y * w) * velocity.z,
        2.0 * (x * y - z * w) * velocity.x
        + (1.0 - 2.0 * (x * x + z * z)) * velocity.y
        + 2.0 * (y * z + x * w) * velocity.z,
        2.0 * (x * z + y * w) * velocity.x
        + 2.0 * (y * z - x * w) * velocity.y
        + (1.0 - 2.0 * (x * x + y * y)) * velocity.z,
    )
