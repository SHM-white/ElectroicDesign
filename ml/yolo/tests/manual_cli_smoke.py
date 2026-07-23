"""Drive the dry-run and mock commands against a cleanup-safe tiny fixture."""

from __future__ import annotations

import hashlib
import os
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

from test_schema import valid_dataset, valid_model, write_json


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def run_cli(environment: dict[str, str], *arguments: str) -> subprocess.CompletedProcess[str]:
    """Run one real CLI command with a hard five-second process bound."""
    return subprocess.run(
        [sys.executable, "-m", "yolo_contract", *arguments],
        check=False,
        capture_output=True,
        env=environment,
        text=True,
        timeout=5,
    )


def main() -> int:
    """Exercise every permitted user-facing command without retaining test data."""
    environment = os.environ | {"PYTHONPATH": str(PROJECT_ROOT / "src")}
    with TemporaryDirectory(prefix="p12-yolo-") as temporary_directory:
        temporary_root = Path(temporary_directory)
        dataset_path = temporary_root / "dataset.json"
        artifact_path = temporary_root / "model.onnx"
        artifact_path.write_bytes(b"synthetic contract artifact, not model weights")
        write_json(dataset_path, valid_dataset())
        model_path = temporary_root / "model.json"
        write_json(model_path, valid_model(dataset_path, artifact_path))
        commands = (
            ("train", "--dataset", str(dataset_path), "--model", str(model_path), "--dry-run"),
            ("validate", "--dataset", str(dataset_path), "--model", str(model_path), "--dry-run"),
            ("export", "--model", str(model_path), "--format", "onnx", "--output", str(temporary_root / "export.onnx"), "--dry-run"),
        )
        for command in commands:
            result = run_cli(environment, *command)
            if result.returncode != 0:
                print(result.stderr, file=sys.stderr)
                return result.returncode
            print(result.stdout, end="")
        detection_arguments = (
            "detect-mock",
            "--dataset",
            str(dataset_path),
            "--model",
            str(model_path),
            "--image-id",
            "camera-frame-0001",
            "--image-sha256",
            hashlib.sha256(b"synthetic-image").hexdigest(),
            "--frame-id",
            "camera_narrow_optical_frame",
        )
        first = run_cli(environment, *detection_arguments)
        second = run_cli(environment, *detection_arguments)
        if first.returncode != 0 or second.returncode != 0 or first.stdout != second.stdout:
            print(first.stderr + second.stderr, file=sys.stderr)
            return 2
        print(first.stdout, end="")
        interrupted = run_cli(
            environment,
            *detection_arguments,
            "--failure-reason",
            "synthetic provider interruption",
        )
        if interrupted.returncode != 2 or interrupted.stdout or "ERROR:" not in interrupted.stderr:
            print(interrupted.stderr, file=sys.stderr)
            return 2
        print("INTERRUPTION: provider failure returned bounded error without detection")
    print("CLEANUP: temporary fixture removed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
