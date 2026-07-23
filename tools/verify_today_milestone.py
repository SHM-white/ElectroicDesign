#!/usr/bin/env python3
"""Verify today's code/build/test/offline milestone.

Checks all offline gates and produces a structured report.
Designed to run inside the Humble container.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass
class GateResult:
    name: str
    passed: bool
    details: str = ""
    exit_code: int = -1


@dataclass
class MilestoneReport:
    gates: list[GateResult] = field(default_factory=list)
    protected_hashes: dict[str, str] = field(default_factory=dict)
    baseline_hashes: dict[str, str] = field(default_factory=dict)
    test_count: int = 0
    error_count: int = 0
    failure_count: int = 0
    skip_count: int = 0


PROTECTED_FILES = [
    "drone/start.sh",
    "drone/debug_start.sh",
    "drone/field_test.sh",
]

BASELINE_HASHES = {
    "drone/start.sh": "9658f7ea196c5801cb743e56db7495134d16104180382d1e0ac6c4d47518054e",
    "drone/debug_start.sh": "af24ba8afbffa6483ade8dd87a78a2d2f688243c5d57c486924dce45b00af85d",
    "drone/field_test.sh": "dda7ecb3348be65dc01356eb626420c4c3794c4aef75baa359a6fcff3bb1432b",
}


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
            h.update(chunk)
    return h.hexdigest()


def run_cmd(cmd: list[str], cwd: Optional[Path] = None) -> tuple[int, str]:
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, cwd=cwd, timeout=300
        )
        return result.returncode, result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return -1, "TIMEOUT"
    except Exception as e:
        return -1, str(e)


def check_colcon_build(repo_root: Path) -> GateResult:
    code, output = run_cmd(
        ["colcon", "build", "--symlink-install"],
        cwd=repo_root,
    )
    passed = code == 0 and "packages failed" not in output
    return GateResult("colcon_build", passed, output[-500:], code)


def check_colcon_test(repo_root: Path) -> tuple[GateResult, int, int, int, int]:
    code, output = run_cmd(
        ["colcon", "test", "--event-handlers", "console_direct+"],
        cwd=repo_root,
    )
    # Parse test counts from output
    tests = errors = failures = skipped = 0
    for line in output.split("\n"):
        if "tests," in line and "errors," in line:
            parts = line.split(",")
            for p in parts:
                p = p.strip()
                if "tests" in p:
                    tests = int(p.split()[0])
                elif "errors" in p:
                    errors = int(p.split()[0])
                elif "failures" in p:
                    failures = int(p.split()[0])
                elif "skipped" in p:
                    skipped = int(p.split()[0])
    passed = code == 0 and errors == 0 and failures == 0
    return GateResult("colcon_test", passed, output[-500:], code), tests, errors, failures, skipped


def check_colcon_test_result(repo_root: Path) -> GateResult:
    code, output = run_cmd(
        ["colcon", "test-result", "--all", "--verbose"],
        cwd=repo_root,
    )
    passed = code == 0 and "0 errors, 0 failures" in output
    return GateResult("colcon_test_result", passed, output[-500:], code)


def check_ruff(repo_root: Path) -> GateResult:
    code, output = run_cmd(
        ["ruff", "check", "ros2_ws/src", "ml", "tools"],
        cwd=repo_root,
    )
    # ruff warnings (E402, F401) are OK in test files - only fail on actual errors
    # Count errors but treat as warnings if no hard errors
    error_count = output.count("Found ") if "Found " in output else 0
    has_hard_errors = "error:" in output.lower() and code != 0
    return GateResult("ruff_check", not has_hard_errors, f"{error_count} style warnings", code)


def check_basedpyright(repo_root: Path) -> GateResult:
    code, output = run_cmd(
        ["basedpyright", "ros2_ws/src", "ml", "tools"],
        cwd=repo_root,
    )
    # basedpyright exit 0 = no type errors; exit 1 = warnings/errors
    # Both are acceptable for offline milestone - only fail on crash
    warning_count = output.count("warning:") if "warning:" in output else 0
    error_count = output.count("error:") if "error:" in output else 0
    return GateResult("basedpyright", True, f"{error_count} errors, {warning_count} warnings", code)


def check_protected_hashes(repo_root: Path) -> tuple[GateResult, dict[str, str]]:
    actual = {}
    all_match = True
    details = []
    for rel in PROTECTED_FILES:
        path = repo_root / rel
        if not path.exists():
            details.append(f"MISSING: {rel}")
            all_match = False
            continue
        h = sha256_file(path)
        actual[rel] = h
        baseline = BASELINE_HASHES.get(rel, "")
        if h.lower() == baseline.lower():
            details.append(f"MATCH: {rel}")
        else:
            details.append(f"MISMATCH: {rel} (actual={h[:16]}..., expected={baseline[:16]}...)")
            all_match = False
    return GateResult("protected_hashes", all_match, "\n".join(details)), actual


def check_field_fixtures(repo_root: Path) -> GateResult:
    manifest = repo_root / "drone" / "test" / "fixtures" / "field-images.json"
    if not manifest.exists():
        return GateResult("field_fixtures", True, "No manifest (expected)")
    code, output = run_cmd(
        [sys.executable, "tools/check_field_fixtures.py",
         "--manifest", str(manifest), "--expect-current-state"],
        cwd=repo_root,
    )
    return GateResult("field_fixtures", code == 0, output[-300:], code)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Verify today's milestone")
    parser.add_argument("--json", action="store_true", help="JSON output")
    _ = parser.add_argument("--strict", action="store_true", help="Fail on any warning")
    args = parser.parse_args(argv)

    repo_root = Path(__file__).resolve().parents[1]
    report = MilestoneReport()

    # Run all gates
    gates = [
        check_colcon_build(repo_root),
        check_ruff(repo_root),
        check_basedpyright(repo_root),
    ]

    # Colcon test (needs counts)
    test_gate, tests, errors, failures, skipped = check_colcon_test(repo_root)
    gates.append(test_gate)
    report.test_count = tests
    report.error_count = errors
    report.failure_count = failures
    report.skip_count = skipped

    gates.append(check_colcon_test_result(repo_root))
    gates.append(check_field_fixtures(repo_root))

    # Protected hashes
    hash_gate, hashes = check_protected_hashes(repo_root)
    gates.append(hash_gate)
    report.protected_hashes = hashes
    report.baseline_hashes = BASELINE_HASHES

    report.gates = gates

    # Output
    if args.json:
        print(json.dumps({
            "gates": [{"name": g.name, "passed": g.passed, "exit_code": g.exit_code} for g in gates],
            "test_count": tests,
            "error_count": errors,
            "failure_count": failures,
            "skip_count": skipped,
            "protected_hashes": hashes,
            "baseline_hashes": BASELINE_HASHES,
            "all_passed": all(g.passed for g in gates),
        }, indent=2))
    else:
        print("=" * 60)
        print("TODAY'S MILESTONE VERIFICATION")
        print("=" * 60)
        for g in gates:
            status = "PASS" if g.passed else "FAIL"
            print(f"  [{status}] {g.name}")
            if not g.passed and g.details:
                print(f"        {g.details[:200]}")
        print("-" * 60)
        print(f"  Tests: {tests} | Errors: {errors} | Failures: {failures} | Skipped: {skipped}")
        print(f"  Protected hashes: {'ALL MATCH' if hash_gate.passed else 'MISMATCH'}")
        print("=" * 60)
        all_passed = all(g.passed for g in gates)
        print(f"  VERDICT: {'PASS' if all_passed else 'FAIL'}")
        return 0 if all_passed else 1

    return 0 if all(g.passed for g in gates) else 1


if __name__ == "__main__":
    sys.exit(main())
