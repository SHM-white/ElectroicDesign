from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

from test_schema import valid_dataset, valid_model, write_json


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PROJECT_ROOT / "src"


def run_cli(*arguments: str) -> subprocess.CompletedProcess[str]:
    environment = os.environ | {"PYTHONPATH": str(SOURCE_ROOT)}
    return subprocess.run(
        [sys.executable, "-m", "yolo_contract", *arguments],
        check=False,
        capture_output=True,
        env=environment,
        text=True,
    )


def create_contracts(tmp_path: Path) -> tuple[Path, Path]:
    dataset_path = tmp_path / "dataset.json"
    artifact_path = tmp_path / "model.onnx"
    artifact_path.write_bytes(b"synthetic contract artifact, not model weights")
    write_json(dataset_path, valid_dataset())
    model_path = tmp_path / "model.json"
    write_json(model_path, valid_model(dataset_path, artifact_path))
    return dataset_path, model_path


def test_train_validate_and_export_support_dry_run_without_side_effects(tmp_path: Path) -> None:
    # Given: a valid tiny synthetic dataset and a bound model contract.
    dataset_path, model_path = create_contracts(tmp_path)
    export_path = tmp_path / "exported.onnx"

    # When: each deliberately non-training CLI is invoked with --dry-run.
    train = run_cli("train", "--dataset", str(dataset_path), "--model", str(model_path), "--dry-run")
    validate = run_cli(
        "validate", "--dataset", str(dataset_path), "--model", str(model_path), "--dry-run"
    )
    export = run_cli(
        "export",
        "--model",
        str(model_path),
        "--format",
        "onnx",
        "--output",
        str(export_path),
        "--dry-run",
    )

    # Then: each plans work only and export never writes an artifact.
    assert train.returncode == validate.returncode == export.returncode == 0
    assert "DRY-RUN" in train.stdout
    assert "DRY-RUN" in validate.stdout
    assert "DRY-RUN" in export.stdout
    assert not export_path.exists()


def test_mock_detection_cli_round_trips_a_deterministic_contract(tmp_path: Path) -> None:
    # Given: a valid mock-runtime contract.
    dataset_path, model_path = create_contracts(tmp_path)
    image_hash = hashlib.sha256(b"synthetic-image").hexdigest()

    # When: the mock adapter is driven through its CLI surface twice.
    first = run_cli(
        "detect-mock",
        "--dataset",
        str(dataset_path),
        "--model",
        str(model_path),
        "--image-id",
        "camera-frame-0001",
        "--image-sha256",
        image_hash,
        "--frame-id",
        "camera_narrow_optical_frame",
    )
    second = run_cli(
        "detect-mock",
        "--dataset",
        str(dataset_path),
        "--model",
        str(model_path),
        "--image-id",
        "camera-frame-0001",
        "--image-sha256",
        image_hash,
        "--frame-id",
        "camera_narrow_optical_frame",
    )

    # Then: it emits one stable, provider-neutral ROS-compatible payload.
    assert first.returncode == second.returncode == 0
    assert first.stdout == second.stdout
    payload = json.loads(first.stdout)
    assert payload["contract"] == "vision_msgs/Detection2DArray-compatible/v1"
    assert payload["detections"][0]["class_name"] == "marker"


def test_cli_rejects_malformed_untrusted_metadata_without_success_output(tmp_path: Path) -> None:
    # Given: malformed metadata from an untrusted input file.
    malformed = tmp_path / "malformed.json"
    malformed.write_text("{", encoding="utf-8")

    # When: the validator CLI attempts to parse it.
    result = run_cli("validate", "--dataset", str(malformed), "--dry-run")

    # Then: it returns a bounded error and never prints a green result.
    assert result.returncode == 2
    assert "ERROR:" in result.stderr
    assert "GREEN" not in result.stdout
