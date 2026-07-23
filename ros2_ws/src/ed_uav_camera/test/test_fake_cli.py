"""End-to-end fake image-device command surface tests."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_simulates_ten_minutes_with_wide_unplug_and_narrow_survival() -> None:
    # Given: a deterministic ten-minute fake dual-camera scenario.
    environment = {**os.environ, "PYTHONPATH": str(PACKAGE_ROOT)}

    # When: an operator runs the fake image-device CLI surface.
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "ed_uav_camera.fake_cli",
            "--duration-seconds",
            "600",
            "--wide-unplug-at-seconds",
            "120",
            "--wide-reconnect-at-seconds",
            "180",
            "--restart-wide",
        ],
        capture_output=True,
        check=False,
        text=True,
        env=environment,
    )

    # Then: the output records namespace, acquisition provenance, and isolation.
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report["simulated_duration_seconds"] == 600
    assert report["narrow"]["topic"] == "/camera/narrow/image_raw"
    assert report["wide"]["camera_info_topic"] == "/camera/wide/camera_info"
    assert report["narrow_healthy_during_wide_unplug"] is True
    assert report["wide"]["restart_count"] == 1
    assert report["timestamp_provenance"] == "camera_acquisition_ros_time"


def test_rejects_nonmonotonic_fake_stamp_without_misleading_green_outcome() -> None:
    # Given: a scenario that deliberately regresses the wide source timestamp.
    result = run_fake_cli(("--inject-nonmonotonic-wide",))

    # When: the operator executes the fake surface.
    report = json.loads(result.stdout)

    # Then: its process status and report both expose the rejected provenance.
    assert result.returncode == 1
    assert report["outcome"] == "rejected"
    assert report["wide"]["health"] == "nonmonotonic_stamp"
    assert report["wide"]["rejected_nonmonotonic_frames"] == 1


def test_fake_surface_is_repeatable_without_wall_clock_timing_flakes() -> None:
    # Given: one fixed fake dual-camera scenario.
    arguments = ("--restart-wide",)

    # When: it is replayed twice without real-time sleeps.
    first = run_fake_cli(arguments)
    second = run_fake_cli(arguments)

    # Then: the surface emits byte-identical diagnostics both times.
    assert first.returncode == second.returncode == 0
    assert first.stdout == second.stdout


def test_rejects_invalid_reconnect_timeline_before_fake_devices_start() -> None:
    # Given: a reconnect instant preceding its simulated disconnect.
    result = run_fake_cli(
        ("--wide-unplug-at-seconds", "180", "--wide-reconnect-at-seconds", "120")
    )

    # When: the fake command parses its scenario boundary.
    # Then: it refuses the malformed input with no success report.
    assert result.returncode == 64
    assert "reconnect time must be after unplug" in result.stderr
    assert result.stdout == ""


def run_fake_cli(extra_arguments: tuple[str, ...]) -> subprocess.CompletedProcess[str]:
    """Run the fixed fake CLI baseline with one focused scenario mutation."""
    environment = {**os.environ, "PYTHONPATH": str(PACKAGE_ROOT)}
    return subprocess.run(
        [
            sys.executable,
            "-m",
            "ed_uav_camera.fake_cli",
            "--duration-seconds",
            "600",
            "--wide-unplug-at-seconds",
            "120",
            "--wide-reconnect-at-seconds",
            "180",
            *extra_arguments,
        ],
        capture_output=True,
        check=False,
        text=True,
        env=environment,
    )
