from __future__ import annotations

from pathlib import Path

import pytest

from tools.flight_readiness_test_support import (
    assert_rejected,
    create_bom,
    create_measurements,
    load_manifest,
    run_checker,
    write_json,
)


def test_cli_rejects_bom_path_that_is_directory(tmp_path: Path) -> None:
    # Given: --bom points at a directory instead of a JSON file.
    bom = tmp_path / "bom-dir"
    bom.mkdir()
    measurements = create_measurements(tmp_path)

    # When: the CLI reads the BOM boundary.
    result = run_checker(bom, measurements)

    # Then: it reports a bounded validation error.
    assert_rejected(result, "cannot read BOM JSON")


def test_cli_rejects_measurements_path_that_is_file(tmp_path: Path) -> None:
    # Given: --measurements points at a file, so manifest resolution is invalid.
    bom = create_bom(tmp_path)
    measurements = tmp_path / "2026-07-24-flight-readiness"
    measurements.write_text("not a directory", encoding="utf-8")

    # When: the CLI reads the manifest boundary.
    result = run_checker(bom, measurements)

    # Then: it reports a bounded validation error.
    assert_rejected(result, "cannot read readiness manifest JSON")


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("missing_limit", "missing config vibration_peak_limit_g"),
        ("invalid_limit", "vibration_peak_limit_g must be positive"),
        ("above_limit", "vibration peak exceeds config limit"),
    ],
)
def test_cli_rejects_vibration_config_limit_regressions(tmp_path: Path, mutation: str, message: str) -> None:
    # Given: vibration evidence whose pass/fail bound must come from the hashed config artifact.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    manifest = measurements / "flight-readiness.json"
    data = load_manifest(measurements)
    config_path = measurements / "flight-config.json"
    match mutation:
        case "missing_limit":
            config_path.write_text('{"config_id":"uav-flight-config-a"}\n', encoding="utf-8")
        case "invalid_limit":
            config_path.write_text('{"config_id":"uav-flight-config-a","vibration_peak_limit_g":0}\n', encoding="utf-8")
        case "above_limit":
            data["values"]["vibration_peak_g"]["value"] = 999
        case _ as unreachable:
            raise AssertionError(unreachable)
    if mutation != "above_limit":
        data["config_sha256"] = "0" * 64
    write_json(manifest, data)

    # When: the CLI validates vibration documentation.
    result = run_checker(bom, measurements)

    # Then: it rejects without a hardcoded threshold or traceback.
    assert_rejected(result, message)
