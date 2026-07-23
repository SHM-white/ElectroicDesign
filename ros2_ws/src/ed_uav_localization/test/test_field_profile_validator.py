from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
VALIDATOR = WORKSPACE_ROOT / "tools" / "validate_field_profile.py"

sys.path.insert(0, str(PACKAGE_ROOT))

from test_field_profile import VALID_PROFILE


def run_validator(*arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(VALIDATOR), *arguments],
        capture_output=True,
        text=True,
        check=False,
    )


def test_validates_a_profile_through_the_real_cli(tmp_path: Path) -> None:
    # Given: a valid field YAML on disk.
    profile_path = tmp_path / "valid.yaml"
    profile_path.write_text(VALID_PROFILE, encoding="utf-8")

    # When: the operator-facing validator runs.
    result = run_validator(str(profile_path))

    # Then: it reports an activation-eligible profile and a content hash.
    assert result.returncode == 0, result.stderr
    assert "FIELD PROFILE: PASS" in result.stdout
    assert "hash=" in result.stdout


def test_rejects_an_invalid_temp_profile_through_the_real_cli(tmp_path: Path) -> None:
    # Given: malformed YAML on disk.
    profile_path = tmp_path / "invalid.yaml"
    profile_path.write_text("version: [\n", encoding="utf-8")

    # When: the validator receives it.
    result = run_validator(str(profile_path))

    # Then: it exits nonzero and never emits a pass result.
    assert result.returncode != 0
    assert "FIELD PROFILE: RED" in result.stderr
    assert "PASS" not in result.stdout


def test_reloads_same_path_and_reports_a_fresh_profile_hash(tmp_path: Path) -> None:
    # Given: a valid profile path whose content is replaced between validation runs.
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(VALID_PROFILE, encoding="utf-8")
    first = run_validator(str(profile_path))

    # When: a changed profile is validated at the same filesystem path.
    profile_path.write_text(VALID_PROFILE.replace("y_m: -1.0", "y_m: 0.0", 1), encoding="utf-8")
    second = run_validator(str(profile_path))

    # Then: no stale cached profile is reused.
    first_hash = re.search(r"hash=([0-9a-f]{64})", first.stdout)
    second_hash = re.search(r"hash=([0-9a-f]{64})", second.stdout)
    assert first.returncode == second.returncode == 0
    assert first_hash is not None
    assert second_hash is not None
    assert first_hash.group(1) != second_hash.group(1)
