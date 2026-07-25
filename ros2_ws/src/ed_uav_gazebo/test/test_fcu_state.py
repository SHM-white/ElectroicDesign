from ed_uav_gazebo.fcu_state import (
    Position3D,
    command_velocity,
    reached_altitude,
    reached_position,
    touched_down,
)
from ed_uav_gazebo.action_semantics import CommandKind, bounded_timeout, requires_armed_vehicle
from ed_uav_gazebo.motion_policy import motion_command
from ed_uav_interfaces.action import FlightCommand
from nav_msgs.msg import Odometry


def test_reached_position_when_pose_is_inside_tolerance() -> None:
    # Given: the vehicle is close to the requested position.
    current = Position3D(1.0, 2.0, 3.0)
    target = Position3D(1.03, 2.02, 3.02)

    # When: the action evaluates the position condition.
    result = reached_position(current, target, 0.05)

    # Then: the position goal is complete.
    assert result


def test_reached_altitude_when_height_is_above_target() -> None:
    # Given: the vehicle is at the requested takeoff height.
    current = Position3D(0.0, 0.0, 1.01)

    # When: the action evaluates the altitude condition.
    result = reached_altitude(current, 1.0, 0.03)

    # Then: altitude completion is reported.
    assert result


def test_command_velocity_when_position_goal_is_not_complete() -> None:
    # Given: the vehicle is below and behind a requested pose.
    current = Position3D(0.0, 0.0, 0.2)
    target = Position3D(2.0, -1.0, 1.2)

    # When: the adapter creates a bounded velocity command.
    velocity = command_velocity(current, target, 2.0)

    # Then: each component points toward the target and the vector respects the limit.
    assert velocity.x > 0.0
    assert velocity.y < 0.0
    assert velocity.z > 0.0
    assert sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) <= 2.0


def test_touched_down_when_vehicle_is_near_ground() -> None:
    # Given: the vehicle has reached the configured touchdown height.
    current = Position3D(0.0, 0.0, 0.08)

    # When: the landing condition is evaluated.
    result = touched_down(current, 0.12)

    # Then: landing can complete.
    assert result


def test_motion_commands_require_arming() -> None:
    # Given: the closed command vocabulary.
    command = CommandKind.TAKEOFF

    # When: the action policy is queried.
    result = requires_armed_vehicle(command)

    # Then: motion is rejected until the FCU is armed.
    assert result


def test_action_timeout_is_bounded() -> None:
    # Given: a caller requests an unboundedly long action.
    requested = 900.0

    # When: the action server normalizes the timeout.
    result = bounded_timeout(requested)

    # Then: the simulator retains a finite upper bound.
    assert result == 120.0


def test_move_command_uses_stable_simulator_speed_limit() -> None:
    # Given: a long move that would otherwise request the old aggressive limit.
    goal = FlightCommand.Goal()
    goal.target_pose.pose.position.x = 8.0
    goal.target_pose.pose.position.y = 8.0
    goal.target_pose.pose.position.z = 1.5
    current = Position3D(0.0, 0.0, 1.5)
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0

    # When: the simulator creates the native velocity command.
    command = motion_command(CommandKind.MOVE, current, goal, odometry)

    # Then: horizontal reversal remains within the validated simulation limit.
    assert sqrt(command.linear.x**2 + command.linear.y**2 + command.linear.z**2) <= 0.6


def test_move_command_rotates_world_target_into_body_frame() -> None:
    # Given: the vehicle is yawed 90 degrees while the world target is east.
    goal = FlightCommand.Goal()
    goal.target_pose.pose.position.x = 8.0
    goal.target_pose.pose.position.z = 1.5
    current = Position3D(0.0, 0.0, 1.5)
    odometry = Odometry()
    odometry.pose.pose.orientation.z = 2**-0.5
    odometry.pose.pose.orientation.w = 2**-0.5

    # When: the adapter creates the body-frame command required by Fortress.
    command = motion_command(CommandKind.MOVE, current, goal, odometry)

    # Then: world east is body south at this orientation.
    assert abs(command.linear.x) < 1e-9
    assert abs(command.linear.y + 0.6) < 1e-9
from math import sqrt
