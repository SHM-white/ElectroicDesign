from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.test_check_competition_docs import assert_rejected, create_valid_workspace, output, run_checker, write_json


HISTORICAL_HEADERS = ("Year", "Objective", "Arena", "Autonomy", "Sensing constraints", "Scoring", "Failure mode", "Reusable capability", "Evidence")


def replace_historical_header(path: Path, old: str, new: str | None) -> None:
    lines = path.read_text(encoding="utf-8").splitlines()
    cells = [cell.strip() for cell in lines[0].strip("|").split("|")]
    index = cells.index(old)
    if new is None:
        del cells[index]
    else:
        cells[index] = new
    lines[0] = "| " + " | ".join(cells) + " |"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


@pytest.mark.parametrize("header", HISTORICAL_HEADERS)
def test_checker_rejects_renamed_historical_dimension(tmp_path: Path, header: str) -> None:
    # Given: one required matrix dimension is renamed, including Arena -> Site.
    workspace = create_valid_workspace(tmp_path)
    historical = workspace / "docs/competition/HISTORICAL_UAV_TASKS.md"
    replace_historical_header(historical, header, "Site" if header == "Arena" else f"Renamed {header}")

    # When: the strict CLI validates the historical matrix.
    result = run_checker(workspace)

    # Then: it rejects the renamed dimension without a PASS banner.
    assert_rejected(result, f"historical matrix missing dimension: {header}")


@pytest.mark.parametrize("header", HISTORICAL_HEADERS)
def test_checker_rejects_removed_historical_dimension(tmp_path: Path, header: str) -> None:
    # Given: one required matrix dimension is removed from the header.
    workspace = create_valid_workspace(tmp_path)
    historical = workspace / "docs/competition/HISTORICAL_UAV_TASKS.md"
    replace_historical_header(historical, header, None)

    # When: the strict CLI validates the historical matrix.
    result = run_checker(workspace)

    # Then: it rejects the missing dimension without a PASS banner.
    assert_rejected(result, f"historical matrix missing dimension: {header}")


def test_checker_rejects_reordered_historical_dimensions(tmp_path: Path) -> None:
    # Given: all dimensions exist but Objective and Arena are reordered.
    workspace = create_valid_workspace(tmp_path)
    historical = workspace / "docs/competition/HISTORICAL_UAV_TASKS.md"
    text = historical.read_text(encoding="utf-8")
    historical.write_text(text.replace("Objective | Arena", "Arena | Objective", 1), encoding="utf-8")

    # When: the strict CLI validates the historical matrix.
    result = run_checker(workspace)

    # Then: it rejects the schema order without a PASS banner.
    assert_rejected(result, "historical matrix dimension order mismatch")


def test_checker_rejects_document_confirmed_allowed_lidar_without_rule_source(tmp_path: Path) -> None:
    # Given: a scenario table that promotes an unknown lidar claim to confirmed allowed.
    workspace = create_valid_workspace(tmp_path)
    scenario = workspace / "docs/competition/2026_SCENARIOS.md"
    scenario.write_text(
        "| Topic | Status | Evidence |\n| --- | --- | --- |\n"
        "| Camera | confirmed | [[C-SCENARIO-CAMERA]] |\n"
        "| Lidar | confirmed allowed | [[C-SCENARIO-LIDAR]] |\n",
        encoding="utf-8",
    )

    # When: the strict CLI validates document text and evidence together.
    result = run_checker(workspace)

    # Then: it rejects the unsupported document claim without a PASS banner.
    assert result.returncode != 0
    assert "document confirmed lidar" in output(result)
    assert "PASS:" not in output(result)


def test_checker_rejects_stale_local_claim_hash(tmp_path: Path) -> None:
    # Given: a local source claim with a syntactically valid but stale digest.
    workspace = create_valid_workspace(tmp_path)
    evidence_path = workspace / "docs/competition/evidence.json"
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    evidence["claims"][3]["source_hash"] = "b" * 64
    write_json(evidence_path, evidence)

    # When: the strict CLI recomputes the local source hash.
    result = run_checker(workspace)

    # Then: it rejects the stale local claim.
    assert result.returncode != 0
    assert "local source hash mismatch" in output(result)


def test_checker_rejects_invalid_calendar_date_and_confidence(tmp_path: Path) -> None:
    # Given: otherwise complete evidence with impossible date and unknown confidence vocabulary.
    workspace = create_valid_workspace(tmp_path)
    evidence_path = workspace / "docs/competition/evidence.json"
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    evidence["claims"][0]["retrieved_at"] = "2026-99-99"
    evidence["claims"][0]["confidence"] = "certain"
    write_json(evidence_path, evidence)

    # When: the strict CLI parses evidence metadata.
    result = run_checker(workspace)

    # Then: it rejects both invalid values.
    assert result.returncode != 0
    assert "invalid retrieval date" in output(result)
    assert "invalid confidence" in output(result)


def test_checker_rejects_invalid_bom_statuses_and_missing_spare_status(tmp_path: Path) -> None:
    # Given: an item with invented ownership/procurement statuses and no spare classification.
    workspace = create_valid_workspace(tmp_path)
    bom_path = workspace / "docs/hardware/BOM.json"
    bom = json.loads(bom_path.read_text(encoding="utf-8"))
    bom["items"][0]["ownership"] = "nonsense"
    bom["items"][0]["procurement_status"] = "nonsense"
    del bom["items"][0]["spare_status"]
    write_json(bom_path, bom)

    # When: the strict CLI validates the BOM status contract.
    result = run_checker(workspace)

    # Then: it reports all unsupported or missing status fields.
    assert result.returncode != 0
    assert "invalid ownership" in output(result)
    assert "invalid procurement_status" in output(result)
    assert "spare_status" in output(result)


def test_checker_multiplies_known_totals_by_quantity(tmp_path: Path) -> None:
    # Given: two identical known units while totals incorrectly record one unit's values.
    workspace = create_valid_workspace(tmp_path)
    bom_path = workspace / "docs/hardware/BOM.json"
    bom = json.loads(bom_path.read_text(encoding="utf-8"))
    item = bom["items"][0]
    item.update(
        {
            "quantity": 2,
            "mass_g": 10,
            "mass_status": "known",
            "steady_w": 2,
            "steady_power_status": "known",
            "peak_w": 3,
            "peak_power_status": "known",
        }
    )
    bom["totals"].update({"known_mass_g": 10, "known_steady_w": 2, "known_peak_w": 3})
    bom["totals"].update({"unknown_mass_items": 0, "unknown_steady_power_items": 0, "unknown_peak_power_items": 0})
    write_json(bom_path, bom)

    # When: the strict CLI verifies per-unit known values against aggregate totals.
    result = run_checker(workspace)

    # Then: it rejects totals that do not include quantity.
    assert result.returncode != 0
    assert "BOM total mismatch: known_mass_g" in output(result)
    assert "BOM total mismatch: known_steady_w" in output(result)
    assert "BOM total mismatch: known_peak_w" in output(result)


def test_checker_rejects_historical_matrix_missing_required_dimension(tmp_path: Path) -> None:
    # Given: a historical matrix without its explicit arena dimension.
    workspace = create_valid_workspace(tmp_path)
    historical = workspace / "docs/competition/HISTORICAL_UAV_TASKS.md"
    historical.write_text(historical.read_text(encoding="utf-8").replace("Arena", "Site"), encoding="utf-8")

    # When: the strict CLI validates the historical matrix schema.
    result = run_checker(workspace)

    # Then: it rejects the incomplete matrix header.
    assert result.returncode != 0
    assert "historical matrix missing dimension: Arena" in output(result)
