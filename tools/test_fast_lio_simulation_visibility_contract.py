from __future__ import annotations

import json
from pathlib import Path

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
GAZEBO_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_gazebo"
LOCALIZATION_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_localization"
MANIFEST_PATH = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_interfaces" / "contracts" / "ros2_contract_manifest.json"


def test_fast_lio_simulation_visibility_contract() -> None:
    config = yaml.safe_load((GAZEBO_ROOT / "config" / "fast_lio_gazebo.yaml").read_text(encoding="utf-8"))
    launch = (GAZEBO_ROOT / "launch" / "fast_lio_simulation.launch.py").read_text(encoding="utf-8")
    adapter = (LOCALIZATION_ROOT / "ed_uav_localization" / "lio_adapter.py").read_text(encoding="utf-8")
    rviz = (GAZEBO_ROOT / "rviz" / "sim.rviz").read_text(encoding="utf-8")
    runner = (REPOSITORY_ROOT / "tools" / "run_gazebo_slam_nav.sh").read_text(encoding="utf-8")
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    documentation = (REPOSITORY_ROOT / "docs" / "architecture" / "ROS2_CONTRACTS.md").read_text(encoding="utf-8")

    assert config["/**"]["ros__parameters"]["publish"]["map_en"] is True
    for remapping in (
        '("/Odometry", "/fast_lio/odometry")',
        '("/cloud_registered", "/fast_lio/cloud_registered")',
        '("/Laser_map", "/fast_lio/laser_map")',
        '("/path", "/fast_lio/path")',
        '("/tf", "/fast_lio/tf")',
    ):
        assert remapping in launch
    for topic in (
        "/fast_lio/odometry",
        "/fast_lio/cloud_registered",
        "/fast_lio/laser_map",
        "/fast_lio/path",
        "/localization/lio/cloud_registered",
        "/localization/lio/map",
        "/localization/lio/path",
    ):
        assert topic in adapter
    assert "TransformBroadcaster" not in adapter
    assert "StaticTransformBroadcaster" not in adapter
    assert 'Value: /localization/lio/cloud_registered' in rviz
    assert 'Value: /localization/lio/map' in rviz
    assert 'Value: /localization/lio/path' in rviz
    assert "/fast_lio/" not in rviz
    assert "/lidar/points" not in rviz
    for topic in (
        "/localization/lio/cloud_registered",
        "/localization/lio/map",
        "/localization/lio/path",
    ):
        assert topic in runner
    topic_names = {topic["name"] for topic in manifest["topics"]}
    assert {
        "/fast_lio/odometry",
        "/fast_lio/cloud_registered",
        "/fast_lio/laser_map",
        "/fast_lio/path",
        "/fast_lio/tf",
        "/localization/lio/cloud_registered",
        "/localization/lio/map",
        "/localization/lio/path",
        "/map",
    } <= topic_names
    assert "/compute_path_to_pose" in {action["name"] for action in manifest["actions"]}
    assert manifest["tf_edges"] == [
        {"parent": "map", "child": "odom", "publisher": "ed_uav_localization.field_anchor"},
        {"parent": "odom", "child": "base_link", "publisher": "ed_uav_localization.source_supervisor"},
    ]
    assert "simulation-only" in documentation
    assert "/fast_lio/tf" in documentation
