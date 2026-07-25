from __future__ import annotations

import rclpy
from rclpy import _rclpy_pybind11
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.action import GoalResponse

from ed_uav_gazebo import sim_fcu, sim_localization


class _TransformCapture:
    def __init__(self) -> None:
        self.message = None

    def sendTransform(self, message) -> None:
        self.message = message


class _FakeNode:
    def __init__(self) -> None:
        self.destroyed = False

    def destroy_node(self) -> None:
        self.destroyed = True


class _ShutdownExecutor:
    def __init__(self, *, num_threads: int) -> None:
        self.num_threads = num_threads
        self.node = None
        self.shutdown_called = False

    def add_node(self, node) -> None:
        self.node = node

    def spin(self) -> None:
        raise ExternalShutdownException()

    def shutdown(self) -> None:
        self.shutdown_called = True


class _InvalidContextExecutor(_ShutdownExecutor):
    def spin(self) -> None:
        raise _rclpy_pybind11.RCLError("context is not valid")


def test_sim_fcu_odometry_callback_builds_vector3_translation() -> None:
    # Given: a live simulator adapter and one Gazebo odometry sample.
    rclpy.init()
    node = sim_fcu.SimulatorFcuNode()
    capture = _TransformCapture()
    node._tf_broadcaster = capture
    odometry = Odometry()
    odometry.pose.pose.position.x = 1.25
    odometry.pose.pose.position.y = -0.5
    odometry.pose.pose.position.z = 0.75

    try:
        # When: the real odometry callback publishes dynamic TF.
        node._on_odometry(odometry)

        # Then: Transform.translation remains the generated Vector3 type.
        assert capture.message is not None
        assert isinstance(capture.message.transform.translation, Vector3)
        assert capture.message.transform.translation.x == 1.25
        assert capture.message.transform.translation.y == -0.5
        assert capture.message.transform.translation.z == 0.75
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_sim_fcu_rejects_non_finite_target_position() -> None:
    rclpy.init()
    node = sim_fcu.SimulatorFcuNode()
    try:
        goal = sim_fcu.FlightCommand.Goal()
        goal.command = sim_fcu.FlightCommand.Goal.COMMAND_MOVE
        goal.timeout_sec = 5.0
        goal.target_pose.pose.position.x = float("nan")

        assert node._goal_callback(goal) == GoalResponse.REJECT
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_sim_localization_main_cleans_up_external_shutdown(monkeypatch) -> None:
    # Given: signal-driven executor shutdown after node creation.
    node = _FakeNode()
    shutdown_calls: list[bool] = []
    monkeypatch.setattr(sim_localization, "SimulatorLocalizationNode", lambda: node)
    monkeypatch.setattr(sim_localization.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(
        sim_localization.rclpy,
        "spin",
        lambda _node: (_ for _ in ()).throw(ExternalShutdownException()),
    )
    monkeypatch.setattr(sim_localization.rclpy, "try_shutdown", lambda: shutdown_calls.append(True))

    # When: the executable boundary observes external shutdown.
    sim_localization.main()

    # Then: it exits normally and releases the node and context.
    assert node.destroyed
    assert shutdown_calls == [True]


def test_sim_localization_main_cleans_up_invalid_context_shutdown(monkeypatch) -> None:
    # Given: Humble invalidates the context while the default executor is waiting.
    node = _FakeNode()
    shutdown_calls: list[bool] = []
    monkeypatch.setattr(sim_localization, "SimulatorLocalizationNode", lambda: node)
    monkeypatch.setattr(sim_localization.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(sim_localization.rclpy, "ok", lambda: False)
    monkeypatch.setattr(
        sim_localization.rclpy,
        "spin",
        lambda _node: (_ for _ in ()).throw(
            _rclpy_pybind11.RCLError("context is not valid")
        ),
    )
    monkeypatch.setattr(sim_localization.rclpy, "try_shutdown", lambda: shutdown_calls.append(True))

    # When: signal shutdown invalidates the context during spin.
    sim_localization.main()

    # Then: the process exits normally and releases its node and context.
    assert node.destroyed
    assert shutdown_calls == [True]


def test_sim_fcu_main_cleans_up_external_shutdown(monkeypatch) -> None:
    # Given: signal-driven shutdown while the FCU executor is spinning.
    node = _FakeNode()
    executor = _ShutdownExecutor(num_threads=4)
    shutdown_calls: list[bool] = []
    monkeypatch.setattr(sim_fcu, "SimulatorFcuNode", lambda: node)
    monkeypatch.setattr(sim_fcu, "MultiThreadedExecutor", lambda num_threads: executor)
    monkeypatch.setattr(sim_fcu.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(sim_fcu.rclpy, "try_shutdown", lambda: shutdown_calls.append(True))

    # When: the executable boundary observes external shutdown.
    sim_fcu.main()

    # Then: executor, node, and context are released exactly once.
    assert executor.node is node
    assert executor.shutdown_called
    assert node.destroyed
    assert shutdown_calls == [True]


def test_sim_fcu_main_cleans_up_invalid_context_shutdown(monkeypatch) -> None:
    # Given: Humble invalidates the context while MultiThreadedExecutor is waiting.
    node = _FakeNode()
    executor = _InvalidContextExecutor(num_threads=4)
    shutdown_calls: list[bool] = []
    monkeypatch.setattr(sim_fcu, "SimulatorFcuNode", lambda: node)
    monkeypatch.setattr(sim_fcu, "MultiThreadedExecutor", lambda num_threads: executor)
    monkeypatch.setattr(sim_fcu.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(sim_fcu.rclpy, "ok", lambda: False)
    monkeypatch.setattr(sim_fcu.rclpy, "try_shutdown", lambda: shutdown_calls.append(True))

    # When: the executable boundary observes the invalid-context RCLError.
    sim_fcu.main()

    # Then: normal signal shutdown releases every owned resource.
    assert executor.shutdown_called
    assert node.destroyed
    assert shutdown_calls == [True]
