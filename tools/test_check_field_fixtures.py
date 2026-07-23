"""Behavior tests for the field-image fixture gate."""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "tools" / "check_field_fixtures.py"


def run_checker(manifest: Path) -> subprocess.CompletedProcess[str]:
    """Run the real checker CLI against one manifest."""
    return subprocess.run(
        [
            sys.executable,
            str(CHECKER),
            "--manifest",
            str(manifest),
            "--expect-current-state",
        ],
        cwd=REPOSITORY_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def test_checker_reports_declared_absence_when_manifest_matches_current_state(
    tmp_path: Path,
) -> None:
    # Given
    manifest = tmp_path / "field-images.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "fixture_root": "fixtures",
                "fixtures": [
                    {
                        "path": "mission_vision_missing.png",
                        "state": "absent",
                        "sha256": None,
                        "expectations": ["OCR label 21"],
                    },
                ],
            },
        ),
        encoding="utf-8",
    )

    # When
    result = run_checker(manifest)

    # Then
    assert result.returncode == 0, result.stderr
    assert "ABSENT mission_vision_missing.png" in result.stdout
    assert "CURRENT STATE MATCHES MANIFEST" in result.stdout


def test_checker_reports_stale_hash_without_a_success_message(tmp_path: Path) -> None:
    # Given
    fixture_root = tmp_path / "fixtures"
    fixture_root.mkdir()
    fixture = fixture_root / "mission_vision_sample.png"
    fixture.write_bytes(b"original-image-bytes")
    expected_hash = hashlib.sha256(fixture.read_bytes()).hexdigest()
    manifest = tmp_path / "field-images.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "fixture_root": "fixtures",
                "fixtures": [
                    {
                        "path": fixture.name,
                        "state": "present",
                        "sha256": expected_hash,
                        "expectations": ["OCR label 21"],
                    },
                ],
            },
        ),
        encoding="utf-8",
    )
    fixture.write_bytes(b"changed-image-bytes")

    # When
    result = run_checker(manifest)

    # Then
    assert result.returncode == 1
    assert "STALE mission_vision_sample.png" in result.stdout
    assert "CURRENT STATE MATCHES MANIFEST" not in result.stdout


def test_checker_rejects_invalid_json_manifest(tmp_path: Path) -> None:
    # Given
    manifest = tmp_path / "field-images.json"
    manifest.write_text("{not-json", encoding="utf-8")

    # When
    result = run_checker(manifest)

    # Then
    assert result.returncode == 2
    assert "INVALID MANIFEST" in result.stderr


def test_checker_rejects_malformed_hash(tmp_path: Path) -> None:
    # Given
    manifest = tmp_path / "field-images.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "fixture_root": "fixtures",
                "fixtures": [
                    {
                        "path": "mission_vision_sample.png",
                        "state": "present",
                        "sha256": "not-a-sha256",
                        "expectations": ["OCR label 21"],
                    },
                ],
            },
        ),
        encoding="utf-8",
    )

    # When
    result = run_checker(manifest)

    # Then
    assert result.returncode == 2
    assert "INVALID MANIFEST" in result.stderr
    assert "sha256" in result.stderr


def test_checker_rejects_path_outside_fixture_root(tmp_path: Path) -> None:
    # Given
    manifest = tmp_path / "field-images.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "fixture_root": "fixtures",
                "fixtures": [
                    {
                        "path": "../mission_vision_sample.png",
                        "state": "absent",
                        "sha256": None,
                        "expectations": ["OCR label 21"],
                    },
                ],
            },
        ),
        encoding="utf-8",
    )

    # When
    result = run_checker(manifest)

    # Then
    assert result.returncode == 2
    assert "INVALID MANIFEST" in result.stderr
    assert "relative path" in result.stderr


def test_pytest_collection_excludes_aggregate_but_keeps_vision_regression() -> None:
    # Given
    command = [sys.executable, "-m", "pytest", "--collect-only", "-q"]

    # When
    result = subprocess.run(
        command,
        cwd=REPOSITORY_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    # Then
    assert result.returncode == 0, result.stderr
    assert "drone/test/test_all.py::" not in result.stdout
    assert "drone/test/test_vision_regression.py::" in result.stdout
