from __future__ import annotations

import ast
import re
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
BRINGUP_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_bringup"
VERIFICATION_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_verification"
OFFLINE_LAUNCH = BRINGUP_ROOT / "launch" / "offline_integration.launch.py"
BRINGUP_LAUNCH = BRINGUP_ROOT / "launch" / "bringup.launch.py"
VERIFICATION_LAUNCH = VERIFICATION_ROOT / "launch" / "verification_harness.launch.py"
RVIZ_CONFIG = BRINGUP_ROOT / "rviz" / "offline_integration.rviz"
PUBLISHER_SOURCE = VERIFICATION_ROOT / "ed_uav_verification" / "ros_node.py"
SETUP_SOURCE = BRINGUP_ROOT / "setup.py"

EXPECTED_TOPICS = frozenset(
    {
        "/camera/narrow/image_raw",
        "/camera/wide/image_raw",
        "/fcu/optical_flow/odom",
        "/lidar/imu",
        "/lidar/points",
        "/localization/lio/odom",
    }
)
RVIZ_TOPICS = frozenset(
    {
        "/camera/narrow/image_raw",
        "/camera/wide/image_raw",
        "/lidar/points",
    }
)


def _required_source(path: Path) -> str:
    assert path.is_file(), f"missing planned offline asset: {path.relative_to(REPOSITORY_ROOT)}"
    return path.read_text(encoding="utf-8")


def _string_constants(path: Path) -> frozenset[str]:
    tree = ast.parse(_required_source(path), filename=str(path))
    return frozenset(node.value for node in ast.walk(tree) if isinstance(node, ast.Constant) and isinstance(node.value, str))


def test_offline_integration_declares_seed_duration_rate_sim_time_and_rviz() -> None:
    # Given: the planned standalone offline integration launch contract.
    expected_arguments = {"seed", "duration_seconds", "rate_hz", "use_sim_time", "use_rviz", "rviz_config"}

    # When: its literal launch surface is inspected without requiring ROS on the host.
    launch_constants = _string_constants(OFFLINE_LAUNCH)

    # Then: every reproducibility, clock, and visualization control is declared.
    missing = expected_arguments - launch_constants
    assert not missing, f"offline integration launch is missing arguments: {sorted(missing)}"


def test_offline_integration_publishes_six_topics_and_static_tf() -> None:
    # Given: the existing synthetic publisher and the planned integration launch.
    publisher_constants = _string_constants(PUBLISHER_SOURCE)
    launch_constants = _string_constants(OFFLINE_LAUNCH)
    bringup_constants = _string_constants(BRINGUP_LAUNCH)
    verification_constants = _string_constants(VERIFICATION_LAUNCH)

    # When: their observable ROS surfaces are compared.
    published_topics = EXPECTED_TOPICS & publisher_constants

    # Then: all six existing topics remain visible and the existing static model supplies TF for RViz.
    assert published_topics == EXPECTED_TOPICS
    assert "bringup.launch.py" in launch_constants
    assert "verification_harness.launch.py" in launch_constants
    assert "ed-uav-verify-ros" in verification_constants
    assert "robot_state_publisher" in bringup_constants
    assert "map -> odom" not in launch_constants
    assert "odom -> base_link" not in launch_constants


def test_rviz_config_is_packaged_and_uses_existing_topics() -> None:
    # Given: the planned RViz asset and the verification package manifest.
    rviz_source = _required_source(RVIZ_CONFIG)
    setup_source = _required_source(SETUP_SOURCE)

    # When: packaging and display topic references are inspected.
    missing_topics = {topic for topic in RVIZ_TOPICS if topic not in rviz_source}
    missing_publisher_topics = {topic for topic in RVIZ_TOPICS if topic not in _string_constants(PUBLISHER_SOURCE)}

    # Then: installation includes the asset and every display consumes an existing topic.
    assert 'glob("rviz/*.rviz")' in setup_source, "setup.py does not install RViz configurations"
    assert not missing_topics, f"RViz config is missing existing visualization topics: {sorted(missing_topics)}"
    assert not missing_publisher_topics, (
        "RViz topic is absent from the deterministic publisher: "
        f"{sorted(missing_publisher_topics)}"
    )


def test_rviz_uses_base_link_and_excludes_disconnected_odometry_displays() -> None:
    # Given: the offline RViz configuration and its fixed-frame TF contract.
    rviz_source = _required_source(RVIZ_CONFIG)

    # When: display classes and frame references are inspected.
    odometry_topics = {"/localization/lio/odom", "/fcu/optical_flow/odom"}

    # Then: only displays with a valid path from base_link remain configured.
    assert "Fixed Frame: base_link" in rviz_source
    assert "Class: rviz_default_plugins/TF" in rviz_source
    assert "Class: rviz_default_plugins/RobotModel" in rviz_source
    assert rviz_source.count("Class: rviz_default_plugins/Image") == 2
    assert "Class: rviz_default_plugins/PointCloud2" in rviz_source
    assert "Class: rviz_default_plugins/Odometry" not in rviz_source
    assert not odometry_topics.intersection(rviz_source.splitlines())


def test_rviz_uses_topic_robot_description_and_salient_inspection_defaults() -> None:
    # Given: the Humble RViz configuration used by the offline integration.
    rviz_source = _required_source(RVIZ_CONFIG)
    robot_model_block = rviz_source.split(
        "Class: rviz_default_plugins/RobotModel", maxsplit=1
    )[1].split("Class: rviz_default_plugins/PointCloud2", maxsplit=1)[0]

    # When: the RobotModel, TF tree, and lidar display properties are inspected.
    point_size_match = re.search(r"Size \(Pixels\): (\d+)", rviz_source)
    marker_scale_match = re.search(r"Marker Scale: ([0-9.]+)", rviz_source)
    assert point_size_match is not None
    assert marker_scale_match is not None

    # Then: Humble receives the latched robot description and leaves the model inspectable.
    assert "Description Source: Topic" in robot_model_block
    assert "Description Topic:" in robot_model_block
    assert "Durability Policy: Transient Local" in robot_model_block
    assert "Reliability Policy: Reliable" in robot_model_block
    assert "Value: /robot_description" in robot_model_block
    assert float(marker_scale_match.group(1)) < 1
    assert int(point_size_match.group(1)) > 3
    assert "Property Tree Widget:" in rviz_source
    assert "- /TF1" in rviz_source
    assert "- /TF1/Frames1" in rviz_source
    assert "- /TF1/Tree1" in rviz_source
    assert "Show Names: true" in rviz_source
    assert "Frames:" in rviz_source
    for frame in (
        "base_link",
        "fcu_link",
        "lidar_link",
        "camera_narrow_optical_frame",
        "camera_wide_optical_frame",
        "rangefinder_link",
        "illustrative_forward_link",
        "illustrative_up_link",
    ):
        assert f"        {frame}:" in rviz_source
    assert "Tree:" in rviz_source
    assert "          fcu_link: {}" in rviz_source
    assert "          lidar_link: {}" in rviz_source
    assert "          illustrative_forward_link: {}" in rviz_source
    assert "          illustrative_up_link: {}" in rviz_source
