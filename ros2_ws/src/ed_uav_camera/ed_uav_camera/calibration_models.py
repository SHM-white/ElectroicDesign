"""Immutable contracts for chessboard capture and calibration artifacts."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Final

import numpy as np
from numpy.typing import NDArray

from .calibration import CaptureProvenance
from .model import CameraRole

INNER_CORNERS: Final = (10, 7)
PHYSICAL_SQUARES: Final = (11, 8)
SQUARE_SIZE_MM: Final = 15.0


@dataclass(frozen=True, slots=True)
class CalibrationBootstrapError(Exception):
    """Raised when capture provenance or quality cannot produce a safe artifact."""

    detail: str

    def __str__(self) -> str:
        return f"camera calibration bootstrap: {self.detail}"


@dataclass(frozen=True, slots=True)
class BoardSpec:
    """Fixed physical board contract confirmed by the operator."""

    inner_corners: tuple[int, int]
    square_size_mm: float

    @classmethod
    def parse(cls, inner_corners: str, measured_square_mm: float) -> BoardSpec:
        if inner_corners != "10x7":
            raise CalibrationBootstrapError("inner-corner pattern must be 10x7 for 11x8 squares")
        if measured_square_mm != SQUARE_SIZE_MM:
            raise CalibrationBootstrapError("measured square confirmation must be 15.0 mm")
        return cls(INNER_CORNERS, SQUARE_SIZE_MM)

    def object_points(self) -> NDArray[np.float32]:
        columns, rows = self.inner_corners
        points = np.zeros((columns * rows, 3), dtype=np.float32)
        points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
        points[:, :2] *= self.square_size_mm / 1_000.0
        return points


@dataclass(frozen=True, slots=True)
class CalibrationSelection:
    """Selected camera identity and exact capture raster."""

    role: CameraRole
    serial: str
    observed_serial: str
    by_id: str
    width: int
    height: int

    def validate(self) -> None:
        if not self.serial or self.serial != self.observed_serial:
            raise CalibrationBootstrapError(
                f"selected serial mismatch: expected {self.serial!r}, observed {self.observed_serial!r}"
            )
        if not self.by_id.startswith("/dev/v4l/by-id/"):
            raise CalibrationBootstrapError("selected camera must use a stable /dev/v4l/by-id path")
        if self.width <= 0 or self.height <= 0:
            raise CalibrationBootstrapError("capture raster dimensions must be positive")


@dataclass(frozen=True, slots=True)
class CapturePolicy:
    """Deterministic image-quality and diversity thresholds."""

    minimum_observations: int = 15
    target_observations: int = 30
    maximum_frames: int = 0
    """Maximum capture frames. 0 = unlimited (time-based only)."""
    maximum_seconds: float = 9000
    """Maximum capture wall-clock seconds. 0 = unlimited (frame-based only)."""
    autofocus_settle_seconds: float = 20.0
    """Seconds to drain frames while camera autofocus converges before real capture."""
    minimum_blur_variance: float = 80.0
    duplicate_rms_fraction: float = 0.015
    minimum_coverage_cells: int = 4
    minimum_area_span: float = 0.025


@dataclass(frozen=True, slots=True)
class CaptureSession:
    """Source, camera, board, and filters for one bounded capture run."""

    source: str
    selection: CalibrationSelection
    board: BoardSpec
    policy: CapturePolicy
    direct_v4l2: bool


@dataclass(frozen=True, slots=True)
class Observation:
    """One accepted subpixel chessboard observation and its source frame."""

    frame_index: int
    frame: NDArray[np.uint8]
    corners: NDArray[np.float32]
    blur_variance: float
    center: tuple[float, float]
    area_fraction: float


@dataclass(frozen=True, slots=True)
class ReprojectionMetrics:
    """Train and holdout reprojection statistics in pixels."""

    train_mean_px: float
    holdout_mean_px: float
    holdout_max_px: float


@dataclass(frozen=True, slots=True)
class CalibrationSolution:
    """Solved intrinsic parameters plus deterministic split evidence."""

    camera_matrix: NDArray[np.float64]
    distortion: NDArray[np.float64]
    train: tuple[Observation, ...]
    holdout: tuple[Observation, ...]
    metrics: ReprojectionMetrics


@dataclass(frozen=True, slots=True)
class ArtifactPaths:
    """Files emitted by one accepted calibration run."""

    camera_info: Path
    descriptor: Path
    descriptor_file_sha256: Path
    overlays: Path


@dataclass(frozen=True, slots=True)
class ArtifactContext:
    """Identity, board, source, and lifetime provenance for artifact output."""

    selection: CalibrationSelection
    board: BoardSpec
    source: str
    capture_provenance: CaptureProvenance
    captured_at_ns: int
    valid_for_ns: int
