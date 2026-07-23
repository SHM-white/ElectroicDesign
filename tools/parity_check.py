#!/usr/bin/env python3
"""parity_check.py — Protected-file integrity verifier.

Reads the SHA-256 hashes of the three protected dirty launch scripts and
compares them against the Task-1 baseline recorded in
``docs/testing/LEGACY_BASELINE.md``. Exits 0 when all hashes match, 1 on
mismatch or unreadable file, 2 on CLI usage error.

Usage::

    python3 tools/parity_check.py
    python3 tools/parity_check.py --json          # machine-readable output
    python3 tools/parity_check.py --baseline PATH  # custom baseline doc
"""

from __future__ import annotations

import hashlib
import json
import re
import sys
from argparse import ArgumentParser
from pathlib import Path
from typing import NamedTuple

# ── Constants ─────────────────────────────────────────────────────────

_PROTECTED_FILES = (
    "drone/start.sh",
    "drone/debug_start.sh",
    "drone/field_test.sh",
)

_DEFAULT_BASELINE = "docs/testing/LEGACY_BASELINE.md"

# Extracts "| `drone/start.sh` | `abcd...1234` |" rows
_BASELINE_ROW_RE = re.compile(
    r"\|\s*`(?P<path>[^`]+)`\s*\|\s*`(?P<hash>[0-9a-f]{64})`"
)


class FileStatus(NamedTuple):
    path: str
    expected: str | None  # hex digest from baseline, None if missing from doc
    actual: str | None    # hex digest from filesystem, None if unreadable
    match: bool


# ── Parsing ───────────────────────────────────────────────────────────


def _parse_expected_hashes(baseline_path: Path) -> dict[str, str]:
    """Return ``{relative_path: sha256_hex}`` from the baseline document."""
    if not baseline_path.is_file():
        print(f"ERROR: baseline document not found: {baseline_path}", file=sys.stderr)
        sys.exit(2)

    text = baseline_path.read_text(encoding="utf-8")
    expected: dict[str, str] = {}
    for match in _BASELINE_ROW_RE.finditer(text):
        expected[match["path"]] = match["hash"].lower()
    return expected


def _compute_actual_hash(file_path: Path) -> str | None:
    """Return the lowercase SHA-256 hex digest of *file_path*, or None."""
    try:
        return hashlib.sha256(file_path.read_bytes()).hexdigest()
    except (OSError, PermissionError) as exc:
        print(f"ERROR: cannot read {file_path}: {exc}", file=sys.stderr)
        return None


# ── Checker ───────────────────────────────────────────────────────────


def _check_protected_files(
    project_root: Path,
    expected: dict[str, str],
) -> tuple[list[FileStatus], bool]:
    """Return statuses per protected file and an overall pass/fail boolean."""
    statuses: list[FileStatus] = []
    all_match = True

    for rel in _PROTECTED_FILES:
        exp = expected.get(rel)
        if exp is None:
            print(
                f"WARNING: {rel} not found in baseline document", file=sys.stderr,
            )
        actual = _compute_actual_hash(project_root / rel)
        ok = (exp is not None and actual is not None and exp == actual)
        if not ok:
            all_match = False
        statuses.append(FileStatus(rel, exp, actual, ok))

    return statuses, all_match


def _report_text(statuses: list[FileStatus], all_match: bool) -> None:
    """Print a human-readable report to stdout."""
    width = max(len(s.path) for s in statuses) + 2
    header = f"{'File':<{width}}  EXPECTED                                                          ACTUAL"
    sep = "-" * len(header)

    print(sep)
    print(header)
    print(sep)
    for st in statuses:
        expected_str = st.expected or "(missing from baseline)"
        actual_str = st.actual or "(unreadable)"
        marker = "PASS" if st.match else "FAIL"
        print(
            f"{st.path:<{width}}  {expected_str}  {actual_str}  {marker}"
        )
    print(sep)

    if all_match:
        print("PASS  all protected-file hashes match Task-1 baseline")
    else:
        mismatches = [s for s in statuses if not s.match]
        print(f"FAIL  {len(mismatches)} file(s) do not match baseline")
        for st in mismatches:
            if st.expected is None:
                print(f"      {st.path}: missing from baseline document")
            elif st.actual is None:
                print(f"      {st.path}: unreadable on filesystem")
            else:
                print(f"      {st.path}: expected={st.expected} actual={st.actual}")


def _report_json(statuses: list[FileStatus], all_match: bool) -> None:
    """Print a JSON report to stdout."""
    output = {
        "pass": all_match,
        "protected_files": [
            {
                "path": s.path,
                "expected": s.expected,
                "actual": s.actual,
                "match": s.match,
            }
            for s in statuses
        ],
    }
    json.dump(output, sys.stdout, indent=2)
    print()  # trailing newline


# ── CLI ───────────────────────────────────────────────────────────────


def _parse_args(argv: list[str] | None = None) -> tuple[Path, Path, bool]:
    parser = ArgumentParser(
        description="Verify protected dirty-file hashes against Task-1 baseline.",
    )
    parser.add_argument(
        "--baseline", type=Path, default=None,
        help=f"Path to baseline document (default: {_DEFAULT_BASELINE})",
    )
    parser.add_argument(
        "--project-root", type=Path, default=None,
        help="Project root directory (default: detected from script location)",
    )
    parser.add_argument(
        "--json", action="store_true",
        help="Output machine-readable JSON instead of text",
    )
    args = parser.parse_args(argv)

    if args.project_root is None:
        # infer from this script: tools/parity_check.py → project root
        script_dir = Path(__file__).resolve().parent
        project_root = script_dir.parent
    else:
        project_root = args.project_root.resolve()

    baseline = args.baseline or (project_root / _DEFAULT_BASELINE)
    return project_root, baseline.resolve(), args.json


def main(argv: list[str] | None = None) -> int:
    project_root, baseline_path, use_json = _parse_args(argv)

    expected = _parse_expected_hashes(baseline_path)
    statuses, all_match = _check_protected_files(project_root, expected)

    if use_json:
        _report_json(statuses, all_match)
    else:
        _report_text(statuses, all_match)

    return 0 if all_match else 1


if __name__ == "__main__":
    sys.exit(main())
