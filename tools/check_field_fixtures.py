"""Check the recorded current state of external mission-image fixtures."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from pathlib import Path, PurePosixPath
from typing import Final

from typing_extensions import assert_never


SUPPORTED_SCHEMA_VERSION: Final = 1
SHA256_PATTERN: Final = re.compile(r"[0-9a-f]{64}")
FIELD_IMAGE_PATTERN: Final = re.compile(r"mission_vision_[A-Za-z0-9_-]+\.png")


class FixtureState(str, Enum):
    """The state a manifest declares for one external image."""

    ABSENT = "absent"
    PRESENT = "present"


@dataclass(frozen=True, slots=True)
class ManifestError(Exception):
    """The fixture manifest did not satisfy its input contract."""

    reason: str

    def __str__(self) -> str:
        return self.reason


@dataclass(frozen=True, slots=True)
class FieldFixture:
    """One fixture's expected availability, identity, and legacy expectations."""

    path: PurePosixPath
    state: FixtureState
    sha256: str | None
    expectations: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class FieldImageManifest:
    """Validated field-image manifest with its resolved fixture root."""

    fixture_root: Path
    fixtures: tuple[FieldFixture, ...]


@dataclass(frozen=True, slots=True)
class FixtureReport:
    """One machine-checkable comparison between manifest and filesystem."""

    line: str
    current: bool


def _require_string(value, field: str) -> str:
    if isinstance(value, str):
        return value
    raise ManifestError(f"{field} must be a string")


def _parse_fixture(raw) -> FieldFixture:
    if not isinstance(raw, dict):
        raise ManifestError("each fixtures entry must be an object")

    relative_path = _require_string(raw.get("path"), "fixture.path")
    path = PurePosixPath(relative_path)
    if (
        path.is_absolute()
        or len(path.parts) != 1
        or path.name != relative_path
        or not FIELD_IMAGE_PATTERN.fullmatch(path.name)
    ):
        raise ManifestError("fixture.path must be one relative path named mission_vision_*.png")

    state_value = _require_string(raw.get("state"), "fixture.state")
    try:
        state = FixtureState(state_value)
    except ValueError as error:
        raise ManifestError("fixture.state must be 'absent' or 'present'") from error

    sha256 = raw.get("sha256")
    match state:
        case FixtureState.ABSENT:
            if sha256 is not None:
                raise ManifestError("an absent fixture must use sha256: null")
        case FixtureState.PRESENT:
            if not isinstance(sha256, str) or not SHA256_PATTERN.fullmatch(sha256):
                raise ManifestError("a present fixture sha256 must be 64 lowercase hex characters")
        case unreachable:
            assert_never(unreachable)

    raw_expectations = raw.get("expectations")
    if (
        not isinstance(raw_expectations, list)
        or not raw_expectations
        or not all(isinstance(expectation, str) and expectation for expectation in raw_expectations)
    ):
        raise ManifestError("fixture.expectations must be a nonempty list of strings")

    return FieldFixture(
        path=path,
        state=state,
        sha256=sha256,
        expectations=tuple(raw_expectations),
    )


def load_manifest(manifest_path: Path) -> FieldImageManifest:
    """Parse and validate an external-image manifest at the input boundary."""
    try:
        raw = json.loads(manifest_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ManifestError(f"manifest does not exist: {manifest_path}") from error
    except UnicodeDecodeError as error:
        raise ManifestError(f"manifest is not UTF-8: {manifest_path}") from error
    except json.JSONDecodeError as error:
        raise ManifestError(f"manifest is not valid JSON: {error.msg}") from error

    if not isinstance(raw, dict):
        raise ManifestError("manifest root must be an object")
    if raw.get("schema_version") != SUPPORTED_SCHEMA_VERSION:
        raise ManifestError(f"schema_version must be {SUPPORTED_SCHEMA_VERSION}")

    fixture_root_value = _require_string(raw.get("fixture_root"), "fixture_root")
    raw_fixtures = raw.get("fixtures")
    if not isinstance(raw_fixtures, list) or not raw_fixtures:
        raise ManifestError("fixtures must be a nonempty list")

    fixtures = tuple(_parse_fixture(raw_fixture) for raw_fixture in raw_fixtures)
    paths = tuple(fixture.path for fixture in fixtures)
    if len(paths) != len(set(paths)):
        raise ManifestError("fixtures must not repeat a path")

    return FieldImageManifest(
        fixture_root=(manifest_path.parent / fixture_root_value).resolve(),
        fixtures=fixtures,
    )


def _sha256(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


def _report_fixture(fixture_root: Path, fixture: FieldFixture) -> FixtureReport:
    path = fixture_root / fixture.path
    if not path.exists():
        match fixture.state:
            case FixtureState.ABSENT:
                return FixtureReport(f"ABSENT {fixture.path}", True)
            case FixtureState.PRESENT:
                return FixtureReport(f"MISSING {fixture.path}", False)
            case unreachable:
                assert_never(unreachable)

    if not path.is_file():
        return FixtureReport(f"UNREADABLE {fixture.path}", False)

    actual_hash = _sha256(path)
    match fixture.state:
        case FixtureState.ABSENT:
            return FixtureReport(
                f"UNEXPECTED_PRESENT {fixture.path} sha256={actual_hash}",
                False,
            )
        case FixtureState.PRESENT:
            if actual_hash == fixture.sha256:
                return FixtureReport(f"PRESENT {fixture.path} sha256={actual_hash}", True)
            return FixtureReport(
                f"STALE {fixture.path} expected={fixture.sha256} actual={actual_hash}",
                False,
            )
        case unreachable:
            assert_never(unreachable)


def check_current_state(manifest: FieldImageManifest) -> int:
    """Print every field-image state and return an honest process status."""
    reports = tuple(
        _report_fixture(manifest.fixture_root, fixture) for fixture in manifest.fixtures
    )
    for report in reports:
        print(report.line)

    if all(report.current for report in reports):
        print("CURRENT STATE MATCHES MANIFEST")
        return 0
    print("FIELD FIXTURE STATE MISMATCH", file=sys.stderr)
    return 1


def parse_args(argv: Sequence[str] | None = None) -> Path:
    """Require an explicit state comparison for this truth-reporting CLI."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--expect-current-state", action="store_true")
    arguments = parser.parse_args(argv)
    if not arguments.expect_current_state:
        parser.error("--expect-current-state is required")
    return arguments.manifest


def main(argv: Sequence[str] | None = None) -> int:
    """Run the fixture gate without converting mismatches into success."""
    try:
        manifest = load_manifest(parse_args(argv))
        return check_current_state(manifest)
    except ManifestError as error:
        print(f"INVALID MANIFEST: {error}", file=sys.stderr)
        return 2
    except OSError as error:
        print(f"CHECK ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
