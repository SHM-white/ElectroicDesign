"""Focused launch and RViz contract tests for landing-marker display."""

from __future__ import annotations

import importlib.util
import shutil
from pathlib import Path
from types import ModuleType

import pytest

pytest.importorskip("launch")
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
BRINGUP_ROOT = REPOSITORY_ROOT / "ros2_ws/src/ed_uav_bringup"
LAUNCH_PATH = BRINGUP_ROOT / "launch/landing_marker_recognition.launch.py"
RVIZ_PATH = BRINGUP_ROOT / "rviz/landing_marker_recognition.rviz"


def _load_launch() -> ModuleType:
    spec = importlib.util.spec_from_file_location(LAUNCH_PATH.stem, LAUNCH_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(use_rviz: str) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "use_rviz": use_rviz,
            "rviz_config": str(RVIZ_PATH),
            "camera_plan": "/secure/p25-runtime-plan.json",
            "profile_catalog": "/secure/profile-catalog.yaml",
            "vehicle_topic": "/external/vehicle/telemetry",
            "target_revision": "d2026-circle-cross-v1",
            "max_reprojection_rms_px": "1.5",
        }
    )
    return context


def _launch_arguments(include: IncludeLaunchDescription) -> dict[str, str]:
    return {
        name: value.name
        for name, value in include.launch_arguments
    }


def test_launch_surface_composes_camera_observer_without_rviz() -> None:
    # Given: the composed landing-marker launch description.
    module = _load_launch()
    description = module.generate_launch_description()

    # When: launch declarations and deferred actions are inspected.
    argument_names = {
        action.name
        for action in description.entities
        if hasattr(action, "name") and action.__class__.__name__ == "DeclareLaunchArgument"
    }
    context = _context("false")
    actions = module._build_actions(context)
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    nodes = [action for action in actions if isinstance(action, Node)]

    # Then: the command owns only the two sensor/perception includes when RViz is disabled.
    assert {
        "camera_plan",
        "profile_catalog",
        "vehicle_topic",
        "target_revision",
        "max_reprojection_rms_px",
        "use_rviz",
        "rviz_config",
    } <= argument_names
    assert len(includes) == 2
    include_text = [
        include.launch_description_source.location.perform(context)
        for include in includes
    ]
    assert any("dual_uvc.launch.py" in value for value in include_text)
    assert any("target_observation.launch.py" in value for value in include_text)
    assert nodes == []
    camera_include = next(
        include
        for include, location in zip(includes, include_text)
        if "dual_uvc.launch.py" in location
    )
    observation_include = next(
        include
        for include, location in zip(includes, include_text)
        if "target_observation.launch.py" in location
    )
    assert _launch_arguments(camera_include) == {
        "camera_plan": "camera_plan",
        "profile_catalog": "profile_catalog",
    }
    assert _launch_arguments(observation_include) == {
        "vehicle_topic": "vehicle_topic",
        "target_revision": "target_revision",
        "max_reprojection_rms_px": "max_reprojection_rms_px",
    }


def test_launch_surface_starts_one_rviz_when_requested(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Given: RViz is requested and its executable is available.
    module = _load_launch()
    monkeypatch.setattr(shutil, "which", lambda _: "/usr/bin/rviz2")

    # When: the deferred actions are evaluated.
    actions = module._build_actions(_context("true"))
    nodes = [action for action in actions if isinstance(action, Node)]

    # Then: exactly one RViz node is added with the selected config substitution.
    assert len(nodes) == 1
    assert nodes[0].node_package == "rviz2"
    node_arguments = nodes[0]._Node__arguments
    assert node_arguments[0] == "-d"
    assert node_arguments[1].name == "rviz_config"


def test_rviz_config_displays_annotated_image_only() -> None:
    # Given: the restrained landing-marker RViz layout.
    rviz_text = RVIZ_PATH.read_text(encoding="utf-8")

    # When: display topics are read from the saved configuration.
    # Then: the annotated image is the only camera image and no telemetry owner is added.
    assert "Value: /d_task/target_observation/annotated_image" in rviz_text
    assert rviz_text.count("Class: rviz_default_plugins/Image") == 1
    assert "/d_task/vehicle/telemetry" not in rviz_text
    assert "/camera/narrow/image_raw" not in rviz_text
    assert "/camera/wide/image_raw" not in rviz_text
    assert "rviz_default_plugins/TF" not in rviz_text
    assert "base_link" not in rviz_text


def test_rviz_requested_without_binary_fails_clearly(monkeypatch: pytest.MonkeyPatch) -> None:
    # Given: RViz is requested but the executable is unavailable.
    module = _load_launch()
    monkeypatch.setattr(shutil, "which", lambda _: None)

    # When: the deferred launch actions are evaluated.
    # Then: launch fails before constructing any child actions with an actionable message.
    with pytest.raises(RuntimeError, match="use_rviz=true requested but rviz2 is unavailable"):
        module._build_actions(_context("true"))


def test_launch_does_not_own_vehicle_or_unrelated_systems() -> None:
    # Given: the source of the composed launch.
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    # When: all package and executable references are inspected.
    # Then: external telemetry, mission, FCU, lidar, and unrelated bringup remain external.
    for forbidden in (
        "ed_uav_vehicle_bridge",
        "ed_uav_mission",
        "ed_uav_fcu_bridge",
        "ed_uav_lidar",
        "bringup.launch.py",
        "mission",
        "fcu",
        "lidar",
    ):
        assert forbidden not in source
