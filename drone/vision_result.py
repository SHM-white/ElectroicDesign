"""Shared result type for all vision backends."""

from dataclasses import dataclass
from typing import Any, Optional


@dataclass(frozen=True)
class VisionResult:
    """One processed vision observation.

    ``frame`` is populated by the industrial-camera backend for debugging and is
    intentionally ``None`` for OpenMV, because OpenMV only sends recognition
    results to the host.
    """

    green_ratio: float
    digit: Optional[int] = None
    frame: Any = None
    sequence: Optional[int] = None
    home_cross_center: Optional[tuple[float, float]] = None
    home_cross_confidence: float = 0.0
    start_marker_center: Optional[tuple[int, int]] = None
    gray_marker_center: Optional[tuple[float, float]] = None
    gray_marker_box: Optional[tuple[int, int, int, int]] = None
    gray_marker_confidence: float = 0.0
    gray_marker_char_count: int = 0
    gray_marker_dark_scene: bool = False
    gray_marker_sequence: Optional[int] = None

