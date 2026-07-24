from __future__ import annotations

import json
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


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("empty_regulators", "regulators must be nonempty"),
        ("empty_rails", "rails must be nonempty"),
        ("duplicate_regulator", "duplicate regulator id"),
        ("duplicate_rail", "duplicate rail id"),
        ("missing_regulator_traceability", "missing regulator method"),
        ("missing_rail_traceability", "missing rail measured_at"),
    ],
)
def test_cli_rejects_missing_power_traceability(tmp_path: Path, mutation: str, message: str) -> None:
    # Given: regulator or rail evidence without required measured traceability.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    manifest = measurements / "flight-readiness.json"
    data = load_manifest(measurements)
    match mutation:
        case "empty_regulators":
            data["regulators"] = []
        case "empty_rails":
            data["rails"] = []
        case "duplicate_regulator":
            data["regulators"].append(data["regulators"][0].copy())
        case "duplicate_rail":
            data["rails"].append(data["rails"][0].copy())
        case "missing_regulator_traceability":
            del data["regulators"][0]["method"]
        case "missing_rail_traceability":
            del data["rails"][0]["measured_at"]
        case _ as unreachable:
            raise AssertionError(unreachable)
    write_json(manifest, data)

    # When: the CLI checks measured rail/regulator evidence.
    result = run_checker(bom, measurements)

    # Then: missing traceability blocks readiness.
    assert_rejected(result, message)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("float_prop_count", "count must be an integer"),
        ("zero_mass", "all_up_mass_g must be positive"),
        ("directory_date", "measurement directory date mismatch"),
        ("config_date", "config_date must equal measured_at"),
        ("value_date", "measured_at must equal manifest measured_at"),
        ("instrument_future", "instrument calibrated after measurement"),
    ],
)
def test_cli_rejects_count_mass_and_date_regressions(tmp_path: Path, mutation: str, message: str) -> None:
    # Given: a complete bundle with one stale date, count, or mass defect.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    manifest = measurements / "flight-readiness.json"
    data = load_manifest(measurements)
    match mutation:
        case "float_prop_count":
            data["values"]["installed_prop_count"]["value"] = 4.5
            data["values"]["expected_prop_count"]["value"] = 4.5
        case "zero_mass":
            data["values"]["all_up_mass_g"]["value"] = 0
        case "directory_date":
            stale = tmp_path / "2026-07-25-flight-readiness"
            measurements.rename(stale)
            measurements = stale
            manifest = measurements / "flight-readiness.json"
        case "config_date":
            data["config_date"] = "2026-07-23"
        case "value_date":
            data["values"]["static_thrust_g"]["measured_at"] = "2026-07-23"
        case "instrument_future":
            data["instruments"][0]["calibrated_at"] = "2026-07-25"
        case _ as unreachable:
            raise AssertionError(unreachable)
    write_json(manifest, data)

    # When: the CLI validates the bundle.
    result = run_checker(bom, measurements)

    # Then: it rejects without crashing or printing PASS.
    assert_rejected(result, message)


def test_cli_rejects_yaml_bom_without_pyyaml_dependency(tmp_path: Path) -> None:
    # Given: a YAML BOM path, matching the stale plan example but unsupported here.
    bom = tmp_path / "BOM.yaml"
    bom.write_text("schema_version: 1\nitems: []\n", encoding="utf-8")
    measurements = create_measurements(tmp_path)

    # When: the dependency-free CLI is asked to read YAML.
    result = run_checker(bom, measurements)

    # Then: it clearly rejects YAML rather than pretending to parse it.
    assert_rejected(result, "YAML BOM is unsupported")


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("config_id_mismatch", "config_id mismatch"),
        ("malformed_config", "invalid config artifact JSON"),
        ("config_root_list", "config artifact must be an object"),
    ],
)
def test_cli_rejects_unbound_or_malformed_config_artifact(tmp_path: Path, mutation: str, message: str) -> None:
    # Given: a complete measured fixture whose config artifact is stale or malformed.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    manifest = measurements / "flight-readiness.json"
    data = load_manifest(measurements)
    match mutation:
        case "config_id_mismatch":
            data["config_id"] = "other-config"
            write_json(manifest, data)
        case "malformed_config":
            (measurements / "flight-config.json").write_text("{not-json", encoding="utf-8")
            data["config_sha256"] = "0" * 64
            write_json(manifest, data)
        case "config_root_list":
            (measurements / "flight-config.json").write_text("[]\n", encoding="utf-8")
            data["config_sha256"] = "0" * 64
            write_json(manifest, data)
        case _ as unreachable:
            raise AssertionError(unreachable)

    # When: the CLI validates the config identity binding.
    result = run_checker(bom, measurements)

    # Then: it rejects without a traceback or PASS banner.
    assert_rejected(result, message)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("zero_active_mass", "active BOM mass must be positive"),
        ("illegal_zero_power", "powered BOM steady power must be positive"),
        ("missing_required", "missing required BOM item"),
        ("steady_total_mismatch", "BOM total mismatch: known_steady_w"),
    ],
)
def test_cli_rejects_required_bom_component_regressions(tmp_path: Path, mutation: str, message: str) -> None:
    # Given: a BOM that violates task-26 required active component evidence.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    data = json.loads(bom.read_text(encoding="utf-8"))
    match mutation:
        case "zero_active_mass":
            data["items"][0]["mass_g"] = 0
        case "illegal_zero_power":
            data["items"][0]["steady_w"] = 0
        case "missing_required":
            data["items"] = [item for item in data["items"] if item["id"] != "propulsion-set"]
        case "steady_total_mismatch":
            data["totals"]["known_steady_w"] = 1
        case _ as unreachable:
            raise AssertionError(unreachable)
    write_json(bom, data)

    # When: the CLI checks the BOM.
    result = run_checker(bom, measurements)

    # Then: readiness is rejected without PASS.
    assert_rejected(result, message)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("directory_artifact", "artifact must be a regular file"),
        ("symlink_artifact", "artifact must be a regular file"),
        ("missing_vibration_artifact", "artifact vibration_spectrum must be an object"),
        ("missing_vibration_record", "vibration_peak_g must be an object"),
    ],
)
def test_cli_rejects_artifact_and_vibration_regressions(tmp_path: Path, mutation: str, message: str) -> None:
    # Given: artifact evidence that is missing, not regular, or lacks vibration traceability.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    manifest = measurements / "flight-readiness.json"
    data = load_manifest(measurements)
    match mutation:
        case "directory_artifact":
            (measurements / "thermal-dir").mkdir()
            data["artifacts"]["thermal_log"]["path"] = "thermal-dir"
        case "symlink_artifact":
            (measurements / "thermal-link").symlink_to(measurements / "thermal.log")
            data["artifacts"]["thermal_log"]["path"] = "thermal-link"
        case "missing_vibration_artifact":
            del data["artifacts"]["vibration_spectrum"]
        case "missing_vibration_record":
            del data["values"]["vibration_peak_g"]
        case _ as unreachable:
            raise AssertionError(unreachable)
    write_json(manifest, data)

    # When: the CLI validates artifact and vibration evidence.
    result = run_checker(bom, measurements)

    # Then: it rejects without traceback.
    assert_rejected(result, message)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("value_missing_instrument", "missing instrument_id"),
        ("regulator_missing_instrument", "missing regulator instrument_id"),
        ("rail_missing_instrument", "missing rail instrument_id"),
        ("sentinel_instrument", "invalid instrument id"),
    ],
)
def test_cli_rejects_sentinel_or_missing_instrument_binding(tmp_path: Path, mutation: str, message: str) -> None:
    # Given: evidence that tries to pass by coercing missing instrument_id to "None".
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    manifest = measurements / "flight-readiness.json"
    data = load_manifest(measurements)
    data["instruments"].append({"id": "None", "kind": "fake", "serial": "S", "calibrated_at": "2026-07-01", "calibration_ref": "CAL"})
    match mutation:
        case "value_missing_instrument":
            del data["values"]["static_thrust_g"]["instrument_id"]
        case "regulator_missing_instrument":
            del data["regulators"][0]["instrument_id"]
        case "rail_missing_instrument":
            del data["rails"][0]["instrument_id"]
        case "sentinel_instrument":
            pass
        case _ as unreachable:
            raise AssertionError(unreachable)
    write_json(manifest, data)

    # When: the CLI validates instrument bindings.
    result = run_checker(bom, measurements)

    # Then: sentinel or missing bindings fail without PASS.
    assert_rejected(result, message)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("manifest_steady_total", "BOM reconciliation mismatch: known_steady_w"),
        ("missing_steady_power", "steady_power_w must be an object"),
        ("measured_steady_below_bom", "measured steady power below BOM known steady power"),
    ],
)
def test_cli_rejects_steady_power_reconciliation_regressions(tmp_path: Path, mutation: str, message: str) -> None:
    # Given: manifest steady-power evidence that does not reconcile to the BOM.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    manifest = measurements / "flight-readiness.json"
    data = load_manifest(measurements)
    match mutation:
        case "manifest_steady_total":
            data["bom_totals"]["known_steady_w"] = 1
        case "missing_steady_power":
            del data["values"]["steady_power_w"]
        case "measured_steady_below_bom":
            data["values"]["steady_power_w"]["value"] = 1
        case _ as unreachable:
            raise AssertionError(unreachable)
    write_json(manifest, data)

    # When: the CLI validates power reconciliation.
    result = run_checker(bom, measurements)

    # Then: it rejects without PASS.
    assert_rejected(result, message)


def test_cli_accepts_negative_cg_coordinates_inside_declared_envelope(tmp_path: Path) -> None:
    # Given: signed CG coordinates inside the declared manufacturer envelope.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    manifest = measurements / "flight-readiness.json"
    data = load_manifest(measurements)
    data["values"]["cg_x_mm"]["value"] = -10
    data["values"]["cg_y_mm"]["value"] = -5
    write_json(manifest, data)

    # When: the CLI validates the signed CG evidence.
    result = run_checker(bom, measurements)

    # Then: the CG gate accepts signed coordinates inside the envelope.
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout == "PASS: flight readiness evidence satisfies offline gates\n"


def test_cli_rejects_negative_cg_coordinate_outside_declared_envelope(tmp_path: Path) -> None:
    # Given: a signed CG coordinate outside the declared manufacturer envelope.
    bom = create_bom(tmp_path)
    measurements = create_measurements(tmp_path)
    manifest = measurements / "flight-readiness.json"
    data = load_manifest(measurements)
    data["values"]["cg_x_mm"]["value"] = -25
    write_json(manifest, data)

    # When: the CLI validates the signed CG evidence.
    result = run_checker(bom, measurements)

    # Then: the envelope gate still rejects out-of-range signed values.
    assert_rejected(result, "CG outside envelope")
