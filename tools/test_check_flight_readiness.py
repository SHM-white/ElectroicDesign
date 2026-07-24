from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.flight_readiness_test_support import (
    assert_rejected,
    create_bom,
    create_measurements,
    load_manifest,
    output,
    run_checker,
    write_json,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_cli_accepts_complete_passing_measurements(tmp_path: Path) -> None:
    # Given: a complete measured readiness manifest with hashed local artifacts.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)

    # When: the strict readiness CLI validates the bundle.
    result = run_checker(bom, measurements)

    # Then: every flight gate passes and the CLI prints the success banner.
    assert result.returncode == 0, output(result)
    assert result.stdout == "PASS: flight readiness evidence satisfies offline gates\n"


def test_cli_rejects_planning_only_bom(tmp_path: Path) -> None:
    # Given: the repository planning BOM and an incomplete measurements directory.
    measurements = tmp_path / "2026-07-24-incomplete"
    measurements.mkdir()

    # When: the readiness CLI checks the current planning-only BOM shape.
    result = run_checker(PROJECT_ROOT / "docs/hardware/BOM.json", measurements)

    # Then: unknown values remain unknown and cannot pass as measured zero.
    assert_rejected(result, "BOM required active item has unknown mass")


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("bad_json", "invalid JSON"),
        ("root_list", "JSON root must be an object"),
        ("stale_artifact_hash", "artifact hash mismatch"),
        ("path_traversal", "artifact path escapes measurement directory"),
        ("bad_config_hash", "invalid config_sha256"),
        ("config_mismatch", "config_sha256 mismatch"),
        ("stale_config_hash", "artifact hash mismatch: config"),
    ],
)
def test_cli_rejects_malformed_stale_or_traversal_input(tmp_path: Path, mutation: str, message: str) -> None:
    # Given: a complete bundle with one malformed or stale input class.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    manifest = measurements / "flight-readiness.json"
    data = load_manifest(measurements)
    match mutation:
        case "bad_json":
            manifest.write_text("{not-json", encoding="utf-8")
        case "root_list":
            write_json(manifest, [])
        case "stale_artifact_hash":
            (measurements / "thermal.log").write_text("changed", encoding="utf-8")
        case "path_traversal":
            data["artifacts"]["thermal_log"]["path"] = "../thermal.log"
            write_json(manifest, data)
        case "bad_config_hash":
            data["config_sha256"] = "not-a-hash"
            write_json(manifest, data)
        case "config_mismatch":
            data["config_sha256"] = "b" * 64
            write_json(manifest, data)
        case "stale_config_hash":
            (measurements / "flight-config.json").write_text("changed\n", encoding="utf-8")
        case _ as unreachable:
            raise AssertionError(unreachable)

    # When: the CLI parses and verifies the bundle.
    result = run_checker(bom, measurements)

    # Then: the failure is clear, bounded, and has no success banner.
    assert_rejected(result, message)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("empty_items", "BOM items must be nonempty"),
        ("duplicate_id", "duplicate BOM item id"),
        ("missing_id", "missing BOM item id"),
        ("catalogue_status", "invalid BOM status"),
        ("missing_totals", "BOM totals must be an object"),
    ],
)
def test_cli_rejects_invalid_bom_structure(tmp_path: Path, mutation: str, message: str) -> None:
    # Given: a BOM that omits or corrupts required quantity-aware structure.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    data = json.loads(bom.read_text(encoding="utf-8"))
    match mutation:
        case "empty_items":
            data["items"] = []
            data["totals"] = {"known_mass_g": 0, "known_peak_w": 0}
        case "duplicate_id":
            data["items"][1]["id"] = "mid-360"
        case "missing_id":
            del data["items"][0]["id"]
        case "catalogue_status":
            data["items"][0]["mass_status"] = "catalogue"
        case "missing_totals":
            del data["totals"]
        case _ as unreachable:
            raise AssertionError(unreachable)
    write_json(bom, data)

    # When: the CLI validates readiness against the malformed BOM.
    result = run_checker(bom, measurements)

    # Then: the structural problem blocks readiness.
    assert_rejected(result, message)
