"""Configuration, pin, and CLI replay acceptance tests."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_lidar.config import ConfigurationError, normalize_config
from ed_uav_lidar.launch_plan import build_launch_plan
from ed_uav_lidar.pin import PinDriftError, validate_livox_pin


def test_accepts_disabled_lidar_camera_only_configuration_without_livox() -> None:
    # Given: a camera-only launch request with lidar explicitly disabled.
    raw_config = {"lidar_enabled": False, "transport": "disabled"}

    # When: the package normalizes its launch configuration.
    config = normalize_config(raw_config)

    # Then: no Livox package or endpoint is required.
    assert config.enabled is False
    assert config.requires_livox is False
    assert config.monitoring_topic == "/lidar/points"


def test_rejects_malformed_transport_configuration() -> None:
    # Given: an unknown transport selection from launch configuration.
    raw_config = {"lidar_enabled": True, "transport": "serial"}

    # When: the transport normalizes the configuration.
    with pytest.raises(ConfigurationError, match="transport"):
        normalize_config(raw_config)

    # Then: malformed configuration is rejected before driver startup.


def test_field_placeholders_hold_mid360_driver_at_a_safe_preflight_gate() -> None:
    # Given: an enabled Mid-360 request retaining all field placeholders.
    config = normalize_config(
        {
            "lidar_enabled": True,
            "transport": "mid360",
            "serial_number": "UNSET",
            "sensor_ip": "0.0.0.0",
            "firmware_version": "UNSET",
        }
    )

    # When: launch actions are planned from the normalized configuration.
    plan = build_launch_plan(config)

    # Then: no external driver starts before serial, IP, and firmware field checks pass.
    assert plan.code == "LIDAR_FIELD_CONFIGURATION_INCOMPLETE"
    assert plan.nodes == ()


def test_default_driver_json_cannot_bypass_field_configuration_gate() -> None:
    # Given: declared hardware identifiers but the checked-in placeholder driver JSON.
    config = normalize_config(
        {
            "lidar_enabled": True,
            "transport": "mid360",
            "serial_number": "MID360-EXAMPLE",
            "sensor_ip": "192.168.1.12",
            "firmware_version": "FIELD-VERIFY",
        }
    )

    # When: launch planning uses the default placeholder JSON path.
    plan = build_launch_plan(config)

    # Then: it refuses to start the vendor driver until a field-verified JSON is supplied.
    assert plan.code == "LIDAR_FIELD_CONFIGURATION_INCOMPLETE"
    assert plan.nodes == ()


def test_disabled_launch_plan_has_no_livox_process() -> None:
    # Given: a camera-only lidar-disabled configuration.
    config = normalize_config({"lidar_enabled": False, "transport": "disabled"})

    # When: the launch plan is built without ROS or vendor packages installed.
    plan = build_launch_plan(config)

    # Then: it succeeds and does not reference the external driver.
    assert plan.code == "LIDAR_DISABLED"
    assert plan.nodes == ()


def test_rejects_livox_pin_drift_from_authoritative_repos_file(tmp_path: Path) -> None:
    # Given: a copied dependency manifest with an altered Livox revision.
    repos = tmp_path / "dependencies.repos"
    repos.write_text(
        json.dumps(
            {"repositories": {"livox_ros_driver2": {"version": "not-the-reviewed-sha"}}}
        ),
        encoding="utf-8",
    )

    # When: the package validates the source-of-truth pin.
    with pytest.raises(PinDriftError, match="revision drift"):
        validate_livox_pin(repos)

    # Then: stale cached pin data cannot enable a Mid-360 driver.


def test_manual_mid360_and_generic_replays_have_truthful_outcomes(tmp_path: Path) -> None:
    # Given: deterministic synthetic Mid-360 and generic PointCloud2 inputs.
    mid360 = tmp_path / "mid360.json"
    generic = tmp_path / "generic.json"
    mid360.write_text(
        json.dumps({"kind": "mid360", "offset_times_ns": [10, 20], "imu_stamp_ns": 40}),
        encoding="utf-8",
    )
    generic.write_text(
        json.dumps({"kind": "generic", "fields": ["x", "y", "z", "intensity"]}),
        encoding="utf-8",
    )
    environment = {**os.environ, "PYTHONPATH": str(PACKAGE_ROOT)}

    # When: users run the replay surface without ROS hardware dependencies.
    mid360_result = subprocess.run(
        [sys.executable, "-m", "ed_uav_lidar.replay", str(mid360)],
        capture_output=True,
        check=False,
        text=True,
        env=environment,
    )
    generic_result = subprocess.run(
        [sys.executable, "-m", "ed_uav_lidar.replay", str(generic)],
        capture_output=True,
        check=False,
        text=True,
        env=environment,
    )

    # Then: Mid-360 is LIO eligible while untimed generic monitoring is honestly not.
    assert mid360_result.returncode == 0, mid360_result.stderr
    assert "REPLAY: GREEN: MID360_LIO_ELIGIBLE" in mid360_result.stdout
    assert generic_result.returncode == 0, generic_result.stderr
    assert "REPLAY: GREEN: GENERIC_MONITORING_ONLY" in generic_result.stdout


def test_manual_replay_reports_stale_imu_as_a_health_failure() -> None:
    # Given: the checked-in deterministic stale-IMU replay fixture.
    fixture = PACKAGE_ROOT / "test" / "fixtures" / "stale-imu.json"
    environment = {**os.environ, "PYTHONPATH": str(PACKAGE_ROOT)}

    # When: the hardware-free replay command consumes the failed health state.
    result = subprocess.run(
        [sys.executable, "-m", "ed_uav_lidar.replay", str(fixture)],
        capture_output=True,
        check=False,
        text=True,
        env=environment,
    )

    # Then: it has a nonzero health result, not a misleading malformed-input label.
    assert result.returncode == 1
    assert result.stderr.startswith("REPLAY: RED: LIDAR_IMU_STALE")


def test_manual_replay_rejects_missing_generic_spatial_fields() -> None:
    # Given: a generic PointCloud2 fixture without y and z fields.
    fixture = PACKAGE_ROOT / "test" / "fixtures" / "generic-malformed-fields.json"
    environment = {**os.environ, "PYTHONPATH": str(PACKAGE_ROOT)}

    # When: the monitoring contract receives the malformed standard cloud.
    result = subprocess.run(
        [sys.executable, "-m", "ed_uav_lidar.replay", str(fixture)],
        capture_output=True,
        check=False,
        text=True,
        env=environment,
    )

    # Then: it refuses field corruption without emitting a GREEN result.
    assert result.returncode == 1
    assert result.stdout == ""
    assert result.stderr.startswith("REPLAY: RED: LIDAR_POINT_FIELDS_INVALID")


def test_manual_replay_is_repeatable_without_clock_flakiness() -> None:
    # Given: one fixed timestamped Mid-360 fixture.
    fixture = PACKAGE_ROOT / "test" / "fixtures" / "mid360-valid.json"
    environment = {**os.environ, "PYTHONPATH": str(PACKAGE_ROOT)}
    command = [sys.executable, "-m", "ed_uav_lidar.replay", str(fixture)]

    # When: the same replay is executed twice.
    first = subprocess.run(command, capture_output=True, check=False, text=True, env=environment)
    second = subprocess.run(command, capture_output=True, check=False, text=True, env=environment)

    # Then: both status and output are exact because the input owns all time values.
    assert first.returncode == second.returncode == 0
    assert first.stdout == second.stdout == "REPLAY: GREEN: MID360_LIO_ELIGIBLE\n"
    assert first.stderr == second.stderr == ""
