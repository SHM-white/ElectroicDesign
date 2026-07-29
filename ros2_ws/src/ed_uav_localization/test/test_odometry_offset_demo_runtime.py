from __future__ import annotations

import sys
from collections.abc import Callable
from dataclasses import dataclass
from types import ModuleType
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization.odometry_offset_demo import main


@dataclass(frozen=True, slots=True)
class RuntimeScenario:
    context_active_after_spin: bool
    expected_shutdown: bool


@dataclass(frozen=True, slots=True)
class FakeStamp:
    sec: int = 1
    nanosec: int = 0


@dataclass(frozen=True, slots=True)
class FakeHeader:
    stamp: FakeStamp = FakeStamp()
    frame_id: str = "odom"


@dataclass(frozen=True, slots=True)
class FakePosition:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass(frozen=True, slots=True)
class FakeOrientation:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass(frozen=True, slots=True)
class FakePose:
    position: FakePosition = FakePosition()
    orientation: FakeOrientation = FakeOrientation()


@dataclass(frozen=True, slots=True)
class FakePoseWithCovariance:
    pose: FakePose = FakePose()


@dataclass(frozen=True, slots=True)
class FakeOdometry:
    header: FakeHeader = FakeHeader()
    pose: FakePoseWithCovariance = FakePoseWithCovariance()


@pytest.mark.parametrize(
    "scenario",
    (
        RuntimeScenario(context_active_after_spin=True, expected_shutdown=True),
        RuntimeScenario(context_active_after_spin=False, expected_shutdown=False),
    ),
)
def test_main_uses_queue_ten_prints_origin_and_cleans_up_keyboard_interrupt(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    scenario: RuntimeScenario,
) -> None:
    # Given: fake ROS modules that deliver one valid pose then interrupt the foreground spin.
    events: list[str] = []
    context_active = True
    rclpy_module = ModuleType("rclpy")
    rclpy_node_module = ModuleType("rclpy.node")
    nav_msgs_module = ModuleType("nav_msgs")
    nav_msgs_msg_module = ModuleType("nav_msgs.msg")

    class FakeNode:
        callback: Callable[[FakeOdometry], None]

        def __init__(self, node_name: str) -> None:
            events.append(f"node:{node_name}")

        def create_subscription(
            self,
            message_type: type[FakeOdometry],
            topic: str,
            callback: Callable[[FakeOdometry], None],
            depth: int,
        ) -> None:
            events.append(f"subscription:{topic}:{depth}")
            self.callback = callback

        def destroy_node(self) -> None:
            events.append("destroy_node")

    def initialize(*, args: list[str]) -> None:
        events.append(f"init:{args}")

    def is_ok() -> bool:
        return context_active

    def spin_once(node: FakeNode, *, timeout_sec: float) -> None:
        nonlocal context_active
        events.append(f"spin:{timeout_sec}")
        node.callback(FakeOdometry())
        context_active = scenario.context_active_after_spin
        raise KeyboardInterrupt

    def shutdown() -> None:
        nonlocal context_active
        events.append("shutdown")
        context_active = False

    rclpy_module.init = initialize
    rclpy_module.ok = is_ok
    rclpy_module.spin_once = spin_once
    rclpy_module.shutdown = shutdown
    rclpy_module.node = rclpy_node_module
    rclpy_node_module.Node = FakeNode
    nav_msgs_module.msg = nav_msgs_msg_module
    nav_msgs_msg_module.Odometry = FakeOdometry
    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node_module)
    monkeypatch.setitem(sys.modules, "nav_msgs", nav_msgs_module)
    monkeypatch.setitem(sys.modules, "nav_msgs.msg", nav_msgs_msg_module)

    # When: the executable boundary runs with the default topic.
    exit_code = main([])

    # Then: it owns only its subscription and releases the context cleanly on interruption.
    assert exit_code == 0
    assert "LIDAR_ODOMETRY_OFFSET" in capsys.readouterr().out
    assert events[:4] == [
        "init:[]",
        "node:lidar_odometry_offset_demo",
        "subscription:/localization/odom:10",
        "spin:0.1",
    ]
    assert events[-1] == ("shutdown" if scenario.expected_shutdown else "destroy_node")
    assert "destroy_node" in events
