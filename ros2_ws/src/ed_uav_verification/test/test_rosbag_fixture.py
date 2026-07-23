from __future__ import annotations

import pytest

from ed_uav_verification.model import ScenarioConfig
from ed_uav_verification.rosbag import RosbagFixtureBuilder
from ed_uav_verification.scenario import DeterministicScenario


def test_rosbag_fixture_builder_writes_real_sqlite_bag_when_ros_is_available(tmp_path) -> None:
    """Given a ROS runtime, when a report is converted, then a rosbag2 fixture exists."""
    pytest.importorskip("rosbag2_py")

    root = tmp_path / "fixture_bag"
    report = DeterministicScenario(ScenarioConfig(seed=14, duration_seconds=1, rate_hz=20)).run()

    written = RosbagFixtureBuilder(root).write(report)

    assert written == root
    assert (root / "metadata.yaml").is_file()
    assert list(root.glob("*.db3"))
