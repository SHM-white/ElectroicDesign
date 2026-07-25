from __future__ import annotations

import hashlib
import re
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
OFFLINE_SCRIPTS = {
    "run_offline_static.sh": "parity_check.py",
    "run_offline_sim.sh": "offline_integration.launch.py",
    "run_offline_rviz.sh": "offline_integration.launch.py",
    "run_offline_fcu_dry_run.sh": "fcu_dry_run.launch.py",
    "run_offline_full_replay.sh": "offline_replay.launch.py",
}
PROTECTED_HASHES = {
    "drone/start.sh": "9658f7ea196c5801cb743e56db7495134d16104180382d1e0ac6c4d47518054e",
    "drone/debug_start.sh": "af24ba8afbffa6483ade8dd87a78a2d2f688243c5d57c486924dce45b00af85d",
    "drone/field_test.sh": "dda7ecb3348be65dc01356eb626420c4c3794c4aef75baa359a6fcff3bb1432b",
}


def test_offline_scripts_call_run_humble_and_preserve_protected_paths() -> None:
    # Given: five staged operator entry points and the frozen legacy launch scripts.
    for script_name, required_surface in OFFLINE_SCRIPTS.items():
        script_path = REPOSITORY_ROOT / "tools" / script_name
        assert script_path.is_file(), f"missing planned offline script: {script_path.relative_to(REPOSITORY_ROOT)}"

        # When: each offline entry point is inspected.
        source = script_path.read_text(encoding="utf-8")

        # Then: every ROS stage uses the pinned runner through bash and its real offline surface.
        assert "set -euo pipefail" in source, f"{script_path.name} does not fail closed"
        assert 'bash "$repo_root/tools/run_humble.sh"' in source, f"{script_path.name} bypasses bash tools/run_humble.sh"
        assert required_surface in source, f"{script_path.name} does not invoke {required_surface}"
        assert ".omo/evidence/offline-integration/scripts" in source, f"{script_path.name} does not write script evidence"
        for protected_path in PROTECTED_HASHES:
            assert protected_path not in source, f"{script_path.name} invokes protected path {protected_path}"

    # Then: adding the staged scripts has not changed any protected legacy entry point.
    actual_hashes = {
        relative_path: hashlib.sha256((REPOSITORY_ROOT / relative_path).read_bytes()).hexdigest()
        for relative_path in PROTECTED_HASHES
    }
    assert actual_hashes == PROTECTED_HASHES


def test_offline_scripts_are_executable_and_have_bash_shebang() -> None:
    # Given: all offline entry points are required to be executable tools.
    for script_name in OFFLINE_SCRIPTS:
        script_path = REPOSITORY_ROOT / "tools" / script_name

        # When: a direct user-execution check is performed.
        file_mode = script_path.stat().st_mode

        # Then: user execute bit is set and the shebang is a valid bash invocation.
        assert file_mode & 0o100, f"{script_path.name} is not user-executable"
        source = script_path.read_text(encoding="utf-8")
        first_line = source.splitlines()[0] if source else ""
        assert first_line.startswith("#!/usr/bin/env bash"), f"{script_path.name} does not start with a bash shebang"


def test_run_offline_static_preserves_inherited_pythonpath() -> None:
    script_path = REPOSITORY_ROOT / "tools" / "run_offline_static.sh"
    source = script_path.read_text(encoding="utf-8")

    # Then: static mode must not rebind PYTHONPATH in command scope.
    # A hard-coded prefix here would shadow inherited values and break ROS launch.
    pattern = r"(?m)^\s*PYTHONPATH="
    assert not re.search(pattern, source), (
        "run_offline_static.sh uses command-scoped PYTHONPATH assignment instead of inheriting environment"
    )

    # And: pytest must still be invoked from the wrapped heredoc command.
    assert "python3 -m pytest -q" in source, "run_offline_static.sh must still invoke pytest in offline mode"
