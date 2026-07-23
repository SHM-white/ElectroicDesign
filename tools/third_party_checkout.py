from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path


def validate_checkout(checkout: Path, source: dict, errors: list[str]) -> None:
    """Verify an imported third-party checkout without contacting the network."""
    try:
        head = subprocess.run(
            ["git", "-C", str(checkout), "rev-parse", "HEAD"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
        status = subprocess.run(
            ["git", "-C", str(checkout), "status", "--porcelain"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except subprocess.TimeoutExpired:
        errors.append(f"checkout verification timed out: {checkout}")
        return
    if head.returncode != 0:
        errors.append(f"third-party checkout is not a git repository: {checkout}")
        return
    expected_revision = source.get("revision")
    if not isinstance(expected_revision, str) or head.stdout.strip() != expected_revision:
        errors.append(f"checkout revision mismatch: {checkout}")
    if status.returncode != 0 or status.stdout:
        errors.append(f"dirty third-party checkout: {checkout}")

    license = source.get("license")
    if not isinstance(license, dict):
        return
    repository_path = license.get("repository_path")
    expected_hash = license.get("sha256")
    if not isinstance(repository_path, str) or not isinstance(expected_hash, str):
        return
    license_path = checkout / repository_path
    if not license_path.is_file():
        errors.append(f"checkout license file missing: {license_path}")
        return
    observed_hash = hashlib.sha256(license_path.read_bytes()).hexdigest()
    if observed_hash != expected_hash:
        errors.append(f"license hash mismatch: {license_path}")
