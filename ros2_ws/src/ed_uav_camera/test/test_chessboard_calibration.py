"""Selected-camera chessboard calibration bootstrap acceptance tests."""

from __future__ import annotations

import json
import hashlib
import os
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
import pytest
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
TOOL = REPOSITORY_ROOT / "tools" / "calibration" / "calibrate_chessboard.py"
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_camera.calibration import (
    CaptureProvenance,
    CalibrationDescriptor,
    NonProductionCalibrationError,
    validate_calibration,
)
from ed_uav_camera.calibration_artifacts import canonical_descriptor_bytes
from ed_uav_camera.identity import CameraBinding
from ed_uav_camera.model import CameraRole
from ed_uav_camera.profiles import CameraMode, Compression


def test_cli_calibrates_deterministic_video_as_nonproduction_artifact(tmp_path: Path) -> None:
    # Given: a deterministic 1280x720 video of diverse views of the physical board.
    video = tmp_path / "chessboard.avi"
    _write_chessboard_video(video)
    output = tmp_path / "output"

    # When: the selected narrow camera is calibrated through the real CLI surface.
    completed = _run_cli(video, output)

    # Then: standard camera_info is emitted but hardware runtime rejects fixture provenance.
    assert completed.returncode == 0, completed.stderr
    descriptor = json.loads((output / "descriptor.json").read_text(encoding="utf-8"))
    camera_info = yaml.safe_load((output / "camera_info.yaml").read_text(encoding="utf-8"))
    assert descriptor["calibration"]["serial"] == "FIXTURE-NARROW-001"
    assert descriptor["calibration"]["width"] == 1280
    assert descriptor["calibration"]["height"] == 720
    assert descriptor["device"]["by_id"] == "/dev/v4l/by-id/fixture-narrow"
    assert descriptor["board"]["inner_corners"] == [10, 7]
    assert descriptor["board"]["square_size_mm"] == 15.0
    assert descriptor["metrics"]["holdout_mean_px"] <= 0.5
    assert descriptor["metrics"]["holdout_max_px"] <= 1.0
    assert descriptor["provenance"]["kind"] == "recorded_video_fixture"
    assert descriptor["provenance"]["production_eligible"] is False
    assert descriptor["calibration"]["capture_provenance"] == "recorded_video_fixture"
    assert descriptor["descriptor_hash"]["algorithm"] == "ed-canonical-json-v1"
    canonical_hash = hashlib.sha256(canonical_descriptor_bytes(descriptor)).hexdigest()
    assert descriptor["descriptor_hash"]["sha256"] == canonical_hash
    descriptor_bytes = (output / "descriptor.json").read_bytes()
    file_hash = hashlib.sha256(descriptor_bytes).hexdigest()
    assert (output / "descriptor.json.sha256").read_text(encoding="ascii") == (
        f"{file_hash}  descriptor.json\n"
    )
    assert file_hash != canonical_hash
    assert camera_info["image_width"] == 1280
    assert camera_info["image_height"] == 720
    assert camera_info["camera_name"] == "narrow_FIXTURE-NARROW-001"
    assert len(tuple((output / "overlays").glob("*.png"))) >= 3
    calibration = descriptor["calibration"]
    with pytest.raises(NonProductionCalibrationError, match="non-production"):
        validate_calibration(
            CameraBinding(CameraRole.NARROW, "FIXTURE-NARROW-001", descriptor["device"]["by_id"]),
            CalibrationDescriptor(
                calibration["serial"],
                calibration["width"],
                calibration["height"],
                calibration["captured_at_ns"],
                calibration["valid_for_ns"],
                calibration["camera_info_url"],
                CaptureProvenance(calibration["capture_provenance"]),
                calibration["observed_serial"],
                calibration["observed_by_id"],
            ),
            CameraMode("MJPG", 1280, 720, 15, Compression.MJPEG, None, 48.0),
            now_ns=calibration["captured_at_ns"],
        )


def test_cli_atomically_replaces_partial_output_on_retry(tmp_path: Path) -> None:
    # Given: an interrupted prior attempt left a visible partial output directory.
    video = tmp_path / "chessboard.avi"
    _write_chessboard_video(video)
    output = tmp_path / "output"
    output.mkdir()
    (output / "partial.tmp").write_text("interrupted", encoding="utf-8")
    stale_staging = tmp_path / ".output.stage-interrupted"
    stale_staging.mkdir()
    (stale_staging / "partial.tmp").write_text("interrupted", encoding="utf-8")

    # When: the operator retries the identical calibration command.
    completed = _run_cli(video, output)

    # Then: a complete artifact atomically replaces partial state.
    assert completed.returncode == 0, completed.stderr
    assert not (output / "partial.tmp").exists()
    assert (output / "camera_info.yaml").is_file()
    assert (output / "descriptor.json").is_file()
    assert not tuple(tmp_path.glob(".output.*"))


def test_canonical_descriptor_hash_algorithm_is_pinned() -> None:
    # Given: insertion-order-dependent JSON-compatible descriptor data.
    # When: the public descriptor payload canonicalizer runs.
    canonical = canonical_descriptor_bytes({"z": 2, "a": 1})

    # Then: exact bytes and their SHA-256 remain stable and explicitly non-file-byte semantics.
    assert canonical == b'{"a":1,"z":2}'
    assert hashlib.sha256(canonical).hexdigest() == (
        "99168216144c7fed5d4c54916cf98d9c66096280c04a499822a99b6658bd177a"
    )


@pytest.mark.parametrize(
    ("extra_arguments", "expected_error"),
    [
        (("--inner-corners", "8x11"), "inner-corner pattern must be 10x7"),
        (("--confirm-square-mm", "14.9"), "measured square confirmation must be 15.0 mm"),
        (("--observed-serial", "FIXTURE-WIDE-999"), "selected serial mismatch"),
        (("--width", "640", "--height", "480"), "capture raster"),
    ],
)
def test_cli_rejects_wrong_board_identity_or_raster(
    tmp_path: Path, extra_arguments: tuple[str, ...], expected_error: str
) -> None:
    # Given: a valid fixture paired with one adversarial bootstrap declaration.
    video = tmp_path / "chessboard.avi"
    _write_chessboard_video(video)

    # When: calibration parses the declaration at its input boundary.
    completed = _run_cli(video, tmp_path / "output", extra_arguments)

    # Then: the bootstrap fails closed before publishing reusable calibration.
    assert completed.returncode != 0
    assert expected_error in completed.stderr
    assert "Traceback" not in completed.stderr
    assert not (tmp_path / "output" / "descriptor.json").exists()


def test_cli_rejects_blurry_insufficient_capture(tmp_path: Path) -> None:
    # Given: a video with no detectable, sharp chessboard observations.
    video = tmp_path / "blank.avi"
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 5.0, (1280, 720))
    assert writer.isOpened()
    for _ in range(16):
        writer.write(np.full((720, 1280, 3), 127, dtype=np.uint8))
    writer.release()

    # When: the real capture/filter path processes it.
    completed = _run_cli(video, tmp_path / "output")

    # Then: insufficient observations fail without an artifact.
    assert completed.returncode != 0
    assert "minimum diverse observations" in completed.stderr
    assert not (tmp_path / "output" / "descriptor.json").exists()


def _run_cli(
    video: Path, output: Path, extra_arguments: tuple[str, ...] = ()
) -> subprocess.CompletedProcess[str]:
    arguments = [
        sys.executable,
        str(TOOL),
        "--input-video",
        str(video),
        "--role",
        "narrow",
        "--serial",
        "FIXTURE-NARROW-001",
        "--observed-serial",
        "FIXTURE-NARROW-001",
        "--by-id",
        "/dev/v4l/by-id/fixture-narrow",
        "--width",
        "1280",
        "--height",
        "720",
        "--confirm-square-mm",
        "15.0",
        "--output-dir",
        str(output),
        *extra_arguments,
    ]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(PACKAGE_ROOT)
    return subprocess.run(arguments, capture_output=True, text=True, env=environment, check=False)


def _write_chessboard_video(path: Path) -> None:
    board = np.zeros((800, 1100), dtype=np.uint8)
    for row in range(8):
        for column in range(11):
            if (row + column) % 2 == 0:
                board[row * 100 : (row + 1) * 100, column * 100 : (column + 1) * 100] = 255
    board_bgr = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)
    source = np.array([[0, 0], [1099, 0], [1099, 799], [0, 799]], dtype=np.float32)
    camera_matrix = np.array([[900.0, 0.0, 640.0], [0.0, 910.0, 360.0], [0.0, 0.0, 1.0]])
    outer = np.array([[0.0, 0.0, 0.0], [0.165, 0.0, 0.0], [0.165, 0.12, 0.0], [0.0, 0.12, 0.0]])
    poses = (
        (-0.12, -0.10, 0.02, -0.11, -0.07, 0.48),
        (0.10, -0.08, -0.03, 0.01, -0.07, 0.50),
        (-0.08, 0.12, 0.04, 0.10, -0.07, 0.52),
        (0.14, 0.08, -0.04, -0.10, -0.01, 0.54),
        (-0.16, 0.05, 0.05, 0.00, -0.01, 0.56),
        (0.06, -0.15, -0.02, 0.10, -0.01, 0.58),
        (0.18, 0.10, 0.03, -0.10, 0.05, 0.60),
        (-0.10, 0.16, -0.05, 0.00, 0.05, 0.62),
        (0.08, 0.14, 0.04, 0.10, 0.05, 0.64),
        (-0.18, -0.04, -0.03, -0.09, -0.06, 0.66),
        (0.12, -0.14, 0.05, 0.01, -0.06, 0.68),
        (-0.04, 0.18, -0.04, 0.09, -0.06, 0.70),
        (0.20, -0.02, 0.02, -0.08, 0.00, 0.72),
        (-0.14, 0.14, -0.02, 0.01, 0.00, 0.74),
        (0.04, -0.18, 0.03, 0.08, 0.00, 0.76),
        (-0.20, 0.02, -0.04, -0.07, 0.05, 0.78),
        (0.16, 0.12, 0.04, 0.01, 0.05, 0.80),
        (-0.02, -0.16, -0.03, 0.07, 0.05, 0.82),
    )
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 5.0, (1280, 720))
    assert writer.isOpened()
    for rx, ry, rz, tx, ty, tz in poses:
        projected, _ = cv2.projectPoints(
            outer, np.array([rx, ry, rz]), np.array([tx, ty, tz]), camera_matrix, np.zeros(5)
        )
        transform = cv2.getPerspectiveTransform(source, projected.reshape(4, 2).astype(np.float32))
        frame = cv2.warpPerspective(board_bgr, transform, (1280, 720), borderValue=(127, 127, 127))
        writer.write(frame)
    writer.release()
