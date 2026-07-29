"""Atomic ROS calibration artifacts and explicitly identified descriptor hashes."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
import shutil
import tempfile
from typing import TypeAlias
from uuid import uuid4

import cv2
import numpy as np
import yaml

from .calibration import CaptureProvenance
from .calibration_models import (
    ArtifactContext,
    ArtifactPaths,
    BoardSpec,
    CalibrationBootstrapError,
    CalibrationSelection,
    CalibrationSolution,
    PHYSICAL_SQUARES,
)

JsonValue: TypeAlias = str | int | float | bool | None | list["JsonValue"] | dict[str, "JsonValue"]
JsonObject: TypeAlias = dict[str, JsonValue]


@dataclass(frozen=True, slots=True)
class _ArtifactTransaction:
    staging: Path
    final_dir: Path
    solution: CalibrationSolution
    context: ArtifactContext


@dataclass(frozen=True, slots=True)
class _CameraInfoArtifact:
    path: Path
    sha256: str


def canonical_descriptor_bytes(descriptor: JsonObject) -> bytes:
    """Return ED canonical JSON v1: UTF-8, sorted keys, compact, excluding descriptor_hash."""
    payload = {key: value for key, value in descriptor.items() if key != "descriptor_hash"}
    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def write_artifacts(
    output_dir: Path, solution: CalibrationSolution, context: ArtifactContext
) -> ArtifactPaths:
    """Stage a complete artifact tree and atomically publish it over partial state."""
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    _remove_stale_staging(output_dir)
    staging = Path(tempfile.mkdtemp(prefix=f".{output_dir.name}.stage-", dir=output_dir.parent))
    try:
        _write_staged_artifacts(_ArtifactTransaction(staging, output_dir, solution, context))
        _publish_staging(staging, output_dir)
    except (OSError, ValueError, cv2.error, CalibrationBootstrapError):
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return ArtifactPaths(
        output_dir / "camera_info.yaml",
        output_dir / "descriptor.json",
        output_dir / "descriptor.json.sha256",
        output_dir / "overlays",
    )


def _write_staged_artifacts(transaction: _ArtifactTransaction) -> None:
    staging = transaction.staging
    context = transaction.context
    solution = transaction.solution
    overlay_dir = staging / "overlays"
    overlay_dir.mkdir()
    camera_info_text = yaml.safe_dump(_camera_info(context.selection, solution), sort_keys=False)
    (staging / "camera_info.yaml").write_text(camera_info_text, encoding="utf-8")
    camera_info_hash = hashlib.sha256(camera_info_text.encode("utf-8")).hexdigest()
    _write_overlays(overlay_dir, context.board, solution)
    descriptor = _descriptor(
        context,
        solution,
        _CameraInfoArtifact(transaction.final_dir / "camera_info.yaml", camera_info_hash),
    )
    descriptor["descriptor_hash"] = {
        "algorithm": "ed-canonical-json-v1",
        "sha256": hashlib.sha256(canonical_descriptor_bytes(descriptor)).hexdigest(),
    }
    descriptor_text = json.dumps(descriptor, indent=2, sort_keys=True, allow_nan=False) + "\n"
    (staging / "descriptor.json").write_text(descriptor_text, encoding="utf-8")
    file_hash = hashlib.sha256(descriptor_text.encode("utf-8")).hexdigest()
    (staging / "descriptor.json.sha256").write_text(
        f"{file_hash}  descriptor.json\n", encoding="ascii"
    )


def _publish_staging(staging: Path, output_dir: Path) -> None:
    backup: Path | None = None
    if output_dir.exists():
        if _is_complete_artifact(output_dir):
            raise CalibrationBootstrapError(f"complete artifact directory already exists: {output_dir}")
        backup = output_dir.with_name(f".{output_dir.name}.partial-{uuid4().hex}")
        output_dir.rename(backup)
    try:
        staging.rename(output_dir)
    except OSError:
        if backup is not None:
            backup.rename(output_dir)
        raise
    if backup is not None:
        shutil.rmtree(backup, ignore_errors=True)
    for stale_backup in output_dir.parent.glob(f".{output_dir.name}.partial-*"):
        shutil.rmtree(stale_backup, ignore_errors=True)


def _remove_stale_staging(output_dir: Path) -> None:
    for stale in output_dir.parent.glob(f".{output_dir.name}.stage-*"):
        shutil.rmtree(stale, ignore_errors=True)


def _is_complete_artifact(output_dir: Path) -> bool:
    required = (
        output_dir / "camera_info.yaml",
        output_dir / "descriptor.json",
        output_dir / "descriptor.json.sha256",
        output_dir / "overlays",
    )
    return all(path.exists() for path in required)


def _camera_info(selection: CalibrationSelection, solution: CalibrationSolution) -> JsonObject:
    matrix = solution.camera_matrix.reshape(-1).tolist()
    distortion = solution.distortion.reshape(-1).tolist()
    projection = [matrix[0], matrix[1], matrix[2], 0.0, matrix[3], matrix[4], matrix[5], 0.0, 0.0, 0.0, 1.0, 0.0]
    return {
        "image_width": selection.width,
        "image_height": selection.height,
        "camera_name": f"{selection.role.value}_{selection.serial}",
        "camera_matrix": {"rows": 3, "cols": 3, "data": matrix},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": len(distortion), "data": distortion},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
        "projection_matrix": {"rows": 3, "cols": 4, "data": projection},
    }


def _descriptor(
    context: ArtifactContext,
    solution: CalibrationSolution,
    camera_info: _CameraInfoArtifact,
) -> JsonObject:
    selection = context.selection
    production_eligible = context.capture_provenance is CaptureProvenance.DIRECT_V4L2
    provenance: JsonObject = {
        "kind": context.capture_provenance.value,
        "production_eligible": production_eligible,
        "source": context.source,
        "observed_serial": selection.observed_serial,
        "observed_by_id": selection.by_id,
        "accepted_frame_indices": sorted(
            item.frame_index for item in (*solution.train, *solution.holdout)
        ),
        "camera_info_sha256": camera_info.sha256,
    }
    return {
        "schema_version": 2,
        "role": selection.role.value,
        "device": {"serial": selection.serial, "by_id": selection.by_id},
        "raster": {"width": selection.width, "height": selection.height},
        "board": {
            "physical_squares": list(PHYSICAL_SQUARES),
            "inner_corners": list(context.board.inner_corners),
            "square_size_mm": context.board.square_size_mm,
        },
        "provenance": provenance,
        "metrics": {
            "train_observations": len(solution.train),
            "holdout_observations": len(solution.holdout),
            "train_mean_px": solution.metrics.train_mean_px,
            "holdout_mean_px": solution.metrics.holdout_mean_px,
            "holdout_max_px": solution.metrics.holdout_max_px,
        },
        "calibration": {
            "serial": selection.serial,
            "width": selection.width,
            "height": selection.height,
            "captured_at_ns": context.captured_at_ns,
            "valid_for_ns": context.valid_for_ns,
            "camera_info_url": camera_info.path.resolve().as_uri(),
            "capture_provenance": context.capture_provenance.value,
            "observed_serial": selection.observed_serial,
            "observed_by_id": selection.by_id,
        },
    }


def _write_overlays(
    overlay_dir: Path, board: BoardSpec, solution: CalibrationSolution
) -> None:
    object_points = board.object_points()
    for observation in solution.holdout:
        solved, rotation, translation = cv2.solvePnP(
            object_points,
            observation.corners,
            solution.camera_matrix,
            solution.distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not solved:
            raise CalibrationBootstrapError("cannot render holdout overlay because pose solve failed")
        projected, _ = cv2.projectPoints(
            object_points, rotation, translation, solution.camera_matrix, solution.distortion
        )
        overlay = observation.frame.copy()
        for detected, reprojection in zip(
            observation.corners.reshape(-1, 2), projected.reshape(-1, 2), strict=True
        ):
            detected_point = tuple(np.rint(detected).astype(int))
            projected_point = tuple(np.rint(reprojection).astype(int))
            cv2.line(overlay, detected_point, projected_point, (0, 255, 255), 1)
            cv2.circle(overlay, detected_point, 3, (0, 255, 0), -1)
            cv2.drawMarker(overlay, projected_point, (0, 0, 255), cv2.MARKER_CROSS, 7, 1)
        destination = overlay_dir / f"holdout_{observation.frame_index:04d}.png"
        if not cv2.imwrite(str(destination), overlay):
            raise CalibrationBootstrapError(f"cannot write overlay {destination}")
