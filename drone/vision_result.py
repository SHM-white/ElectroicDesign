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

