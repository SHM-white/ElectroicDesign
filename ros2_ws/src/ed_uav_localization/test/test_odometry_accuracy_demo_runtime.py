from __future__ import annotations

import sys
import json
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization import odometry_accuracy_report
from ed_uav_localization.odometry_accuracy_demo import (
    INTERRUPTED,
    NO_SAMPLE_TIMEOUT,
    main,
)


@dataclass(frozen=True, slots=True)
class RuntimeScenario:
    raise_keyboard_interrupt: bool
    context_active_after_spin: bool
    expected_status: str
    expected_events: tuple[str, ...]


@pytest.mark.parametrize(
    "scenario",
    (
        RuntimeScenario(
            raise_keyboard_interrupt=False,
            context_active_after_spin=False,
            expected_status=NO_SAMPLE_TIMEOUT,
            expected_events=(
                "init:[]",
                "node:odometry_accuracy_demo",
                "subscription:/localization/odom:10",
                "spin:0.05",
                "destroy_node",
            ),
        ),
        RuntimeScenario(
            raise_keyboard_interrupt=True,
            context_active_after_spin=True,
            expected_status="INTERRUPTED",
            expected_events=(
                "init:[]",
                "node:odometry_accuracy_demo",
                "subscription:/localization/odom:10",
                "spin:0.05",
                "destroy_node",
                "shutdown",
            ),
        ),
        RuntimeScenario(
            raise_keyboard_interrupt=True,
            context_active_after_spin=False,
            expected_status="INTERRUPTED",
            expected_events=(
                "init:[]",
                "node:odometry_accuracy_demo",
                "subscription:/localization/odom:10",
                "spin:0.05",
                "destroy_node",
            ),
        ),
    ),
)
def test_main_finalizes_after_context_shutdown_or_keyboard_interrupt(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    scenario: RuntimeScenario,
) -> None:
    # Given: a ROS boundary that stops its context or raises KeyboardInterrupt while spinning.
    events: list[str] = []
    context_active = True
    rclpy_module = ModuleType("rclpy")
    rclpy_node_module = ModuleType("rclpy.node")
    nav_msgs_module = ModuleType("nav_msgs")
    nav_msgs_msg_module = ModuleType("nav_msgs.msg")

    class FakeOdometry:
        pass

    class FakeNode:
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

        def destroy_node(self) -> None:
            events.append("destroy_node")

    def initialize(*, args: list[str]) -> None:
        events.append(f"init:{args}")

    def is_ok() -> bool:
        return context_active

    def spin_once(node: FakeNode, *, timeout_sec: float) -> None:
        nonlocal context_active
        events.append(f"spin:{timeout_sec}")
        context_active = scenario.context_active_after_spin
        if scenario.raise_keyboard_interrupt:
            raise KeyboardInterrupt

    def shutdown() -> None:
        nonlocal context_active
        if not context_active:
            raise AssertionError("rclpy.shutdown must not run after external shutdown")
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

    # When: the runtime exits through the affected lifecycle boundary.
    try:
        exit_code = main(["--mode", "loop"])
    except KeyboardInterrupt:
        pytest.fail("KeyboardInterrupt escaped the runtime boundary")

    # Then: it retains a structured terminal result and performs only valid cleanup.
    lines = capsys.readouterr().out.splitlines()
    assert exit_code != 0
    assert len(lines) == 1
    result = json.loads(lines[0].removeprefix("ODOMETRY_ACCURACY_RESULT="))
    assert result["status"] == scenario.expected_status
    assert tuple(events) == scenario.expected_events


def test_interrupted_status_is_a_stable_pure_report_constant() -> None:
    # Given: the report contract imported without a ROS runtime.
    # When: its interrupt status is inspected.
    status = odometry_accuracy_report.INTERRUPTED

    # Then: CLI callers can consume the stable status value.
    assert status == INTERRUPTED == "INTERRUPTED"
