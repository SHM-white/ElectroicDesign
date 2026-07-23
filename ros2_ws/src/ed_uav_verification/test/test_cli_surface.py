from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_cli_writes_exact_sixty_second_event_artifact(tmp_path: Path) -> None:
    """Given the CLI, when a seeded 60-second replay runs, then it writes JSON."""
    event_path = tmp_path / "events.json"
    environment = os.environ | {"PYTHONPATH": str(PACKAGE_ROOT)}

    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "ed_uav_verification.cli",
            "--seed",
            "23",
            "--duration-seconds",
            "60",
            "--rate-hz",
            "20",
            "--event-json",
            str(event_path),
        ],
        capture_output=True,
        check=False,
        env=environment,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "SCENARIO: GREEN" in result.stdout
    assert event_path.read_bytes()


def test_cli_rejects_malformed_fault_without_success_log(tmp_path: Path) -> None:
    """Given malformed fault input, when the CLI parses it, then it cannot claim green."""
    environment = os.environ | {"PYTHONPATH": str(PACKAGE_ROOT)}

    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "ed_uav_verification.cli",
            "--fault",
            "not-a-fault",
            "--event-json",
            str(tmp_path / "events.json"),
        ],
        capture_output=True,
        check=False,
        env=environment,
        text=True,
    )

    assert result.returncode != 0
    assert "SCENARIO: RED" in result.stderr
    assert "SCENARIO: GREEN" not in result.stdout
