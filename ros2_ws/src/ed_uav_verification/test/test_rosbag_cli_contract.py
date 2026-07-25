from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _environment() -> dict[str, str]:
    return os.environ | {"PYTHONPATH": str(PACKAGE_ROOT)}


def test_cli_writes_real_rosbag_fixture(tmp_path: Path) -> None:
    # Given: a Humble runtime with rosbag2 and empty event/bag destinations.
    event_path = tmp_path / "events.json"
    bag_path = tmp_path / "fixture_bag"

    # When: the verification CLI is asked to persist a real rosbag2 fixture.
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "ed_uav_verification.cli",
            "--seed",
            "37",
            "--duration-seconds",
            "1",
            "--rate-hz",
            "20",
            "--event-json",
            str(event_path),
            "--rosbag-dir",
            str(bag_path),
        ],
        capture_output=True,
        check=False,
        env=_environment(),
        text=True,
    )

    # Then: success is reported only after standard rosbag2 output exists.
    assert result.returncode == 0, result.stderr
    assert "SCENARIO: GREEN" in result.stdout
    assert (bag_path / "metadata.yaml").is_file()
    assert list(bag_path.glob("*.db3")), "CLI did not write a SQLite rosbag2 storage file"


def test_cli_rejects_overlapping_faults_without_artifacts(tmp_path: Path) -> None:
    # Given: two overlapping faults targeting the same stream.
    event_path = tmp_path / "events.json"

    # When: the untrusted CLI request is parsed.
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "ed_uav_verification.cli",
            "--duration-seconds",
            "2",
            "--rate-hz",
            "20",
            "--fault",
            "drop:lidar_points:3:5",
            "--fault",
            "latency:lidar_points:6:5",
            "--event-json",
            str(event_path),
        ],
        capture_output=True,
        check=False,
        env=_environment(),
        text=True,
    )

    # Then: existing overlap validation remains a pre-write boundary.
    assert result.returncode != 0
    assert "fault windows cannot overlap on one stream" in result.stderr
    assert "SCENARIO: GREEN" not in result.stdout
    assert not event_path.exists()
