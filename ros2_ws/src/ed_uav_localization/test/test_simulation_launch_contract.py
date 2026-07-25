from __future__ import annotations

from pathlib import Path
import sys

import rclpy
from rclpy.node import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


def test_localization_setup_exposes_both_nodes() -> None:
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert "source_supervisor = ed_uav_localization.source_supervisor:main" in setup_text
    assert "field_anchor = ed_uav_localization.field_anchor:main" in setup_text


def test_localization_launch_has_single_topic_and_tf_owners() -> None:
    launch_text = (
        PACKAGE_ROOT / "launch" / "localization_simulation.launch.py"
    ).read_text(encoding="utf-8")

    assert 'executable="source_supervisor"' in launch_text
    assert 'executable="field_anchor"' in launch_text
    assert '"use_sim_time": use_sim_time' in launch_text
    assert '"profile_path": profile_path' in launch_text
    source_text = (
        PACKAGE_ROOT / "ed_uav_localization" / "source_supervisor.py"
    ).read_text(encoding="utf-8")
    anchor_text = (PACKAGE_ROOT / "ed_uav_localization" / "field_anchor.py").read_text(
        encoding="utf-8"
    )
    assert source_text.count('"/localization/status"') == 1
    assert source_text.count('"/localization/odom"') == 1
    assert "StaticTransformBroadcaster" in anchor_text
    assert '"odom"' in anchor_text


def test_fresh_finite_lio_is_active() -> None:
    from ed_uav_localization.source_supervisor import (
        LocalizationSource,
        SourceState,
        SupervisorThresholds,
        decide_source_switch,
        evaluate_source_state,
    )

    thresholds = SupervisorThresholds()
    state = evaluate_source_state(
        age_sec=0.01,
        no_msg_duration_sec=0.01,
        time_regression=False,
        covariance_finite=True,
        covariance_exceeds=False,
        max_age_active=thresholds.lio_max_age_active,
        max_age_degraded=thresholds.lio_max_age_degraded,
        lost_timeout=thresholds.lost_timeout,
        covariance_blowup=thresholds.covariance_blowup,
    )

    assert state == SourceState.ACTIVE
    assert decide_source_switch(
        current_primary=LocalizationSource.NONE,
        lio_state=state,
        visual_state=SourceState.LOST,
        visual_stable=False,
        primary_duration_sec=0.0,
        thresholds=thresholds,
    ) == LocalizationSource.LIO


def test_source_supervisor_is_a_real_ros_node_subclass() -> None:
    from ed_uav_localization.source_supervisor import SourceSupervisor

    assert issubclass(SourceSupervisor, Node)


def test_source_supervisor_accepts_sim_time_override() -> None:
    from ed_uav_localization.source_supervisor import SourceSupervisor

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = None
    try:
        node = SourceSupervisor()
        assert node.get_parameter("use_sim_time").value is True
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def test_field_anchor_accepts_sim_time_override() -> None:
    from ed_uav_localization.field_anchor import FieldAnchor

    profile_path = PACKAGE_ROOT / "config" / "fields" / "simulation_arena.yaml"
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            f"profile_path:={profile_path}",
        ]
    )
    node = None
    try:
        node = FieldAnchor()
        assert node.get_parameter("use_sim_time").value is True
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
