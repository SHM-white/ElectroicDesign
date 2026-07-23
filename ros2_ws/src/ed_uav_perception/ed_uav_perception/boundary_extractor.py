"""HSV- and edge-based boundary line detection with deterministic geometry.

Detects field-boundary line segments from rectified wide-camera images using
an HSV colour mask → Canny edges → probabilistic Hough transform pipeline.
Every operation is deterministic (no random seeds or probabilistic sampling).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore[import-untyped]

    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class ImageLine:
    """A detected line segment in image coordinates (pixels).

    The line is stored as two endpoints.  Derived geometric properties are
    available through cached properties.
    """

    x1: float
    y1: float
    x2: float
    y2: float

    # --- cached geometric properties ---

    @property
    def direction(self) -> tuple[float, float]:
        """Unit direction vector from (x1,y1) toward (x2,y2)."""
        dx = self.x2 - self.x1
        dy = self.y2 - self.y1
        length = self.length
        if length < 1e-9:
            return (0.0, 0.0)
        return (dx / length, dy / length)

    @property
    def normal(self) -> tuple[float, float]:
        """Unit normal pointing to the *right* of the direction vector."""
        dx, dy = self.direction
        return (-dy, dx)

    @property
    def length(self) -> float:
        """Euclidean length in pixels."""
        return float(np.hypot(self.x2 - self.x1, self.y2 - self.y1))

    @property
    def midpoint(self) -> tuple[float, float]:
        """Midpoint of the segment."""
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    @property
    def rho(self) -> float:
        """Distance from origin in Hough normal form (pixels)."""
        nx, ny = self.normal
        mx, my = self.midpoint
        return float(mx * nx + my * ny)

    @property
    def theta(self) -> float:
        """Angle of the normal vector in radians [0, π)."""
        nx, ny = self.normal
        return float(np.arctan2(ny, nx)) % np.pi


@dataclass(frozen=True, slots=True)
class HSVRange:
    """A closed HSV colour interval for thresholding."""

    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


# ---------------------------------------------------------------------------
# Presets
# ---------------------------------------------------------------------------

#: Dark boundary bands (black/dark-grey) — catches most tape/paint on grass.
DARK_BOUNDARY = HSVRange(lower=(0, 0, 0), upper=(180, 255, 80))

#: White/bright boundary bands (white tape, chalk lines).
BRIGHT_BOUNDARY = HSVRange(lower=(0, 0, 180), upper=(180, 40, 255))

#: High-contrast edge-only detection (no colour filter applied).
NO_COLOUR_FILTER = HSVRange(lower=(0, 0, 0), upper=(180, 255, 255))


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def extract_lines(
    image: np.ndarray,
    hsv_range: HSVRange = DARK_BOUNDARY,
    *,
    canny_low: int = 50,
    canny_high: int = 150,
    hough_threshold: int = 60,
    min_line_length: float = 50.0,
    max_line_gap: float = 25.0,
) -> list[ImageLine]:
    """Detect line segments through HSV thresholding → Canny → Hough.

    Every stage uses fixed thresholds — the pipeline is fully deterministic.

    Args:
        image: Input BGR image as ``(H, W, 3)`` uint8.
        hsv_range: HSV lower/upper bounds for the colour mask.
        canny_low: Low threshold for Canny edge detection.
        canny_high: High threshold for Canny edge detection.
        hough_threshold: Accumulator threshold for ``HoughLinesP``.
        min_line_length: Minimum line segment length in pixels.
        max_line_gap: Maximum gap between segments to be joined.

    Returns:
        Detected line segments, or an empty list if none found.
    """
    if not _HAS_CV2:
        raise RuntimeError("OpenCV (cv2) is required for line extraction")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(hsv_range.lower, dtype=np.uint8),
        np.array(hsv_range.upper, dtype=np.uint8),
    )
    edges = cv2.Canny(mask, canny_low, canny_high, L2gradient=True)

    segments = cv2.HoughLinesP(
        edges,
        rho=1.0,
        theta=np.pi / 180.0,
        threshold=hough_threshold,
        minLineLength=min_line_length,
        maxLineGap=max_line_gap,
    )

    if segments is None:
        return []

    return [
        ImageLine(
            x1=float(seg[0][0]),
            y1=float(seg[0][1]),
            x2=float(seg[0][2]),
            y2=float(seg[0][3]),
        )
        for seg in segments
    ]


def extract_lines_from_edges(
    edges: np.ndarray,
    *,
    hough_threshold: int = 60,
    min_line_length: float = 50.0,
    max_line_gap: float = 25.0,
) -> list[ImageLine]:
    """Detect line segments directly from a pre-computed edge map.

    Useful when the caller already has a binary edge image.
    """
    if not _HAS_CV2:
        raise RuntimeError("OpenCV (cv2) is required for line extraction")

    segments = cv2.HoughLinesP(
        edges,
        rho=1.0,
        theta=np.pi / 180.0,
        threshold=hough_threshold,
        minLineLength=min_line_length,
        maxLineGap=max_line_gap,
    )
    if segments is None:
        return []

    return [
        ImageLine(
            x1=float(seg[0][0]),
            y1=float(seg[0][1]),
            x2=float(seg[0][2]),
            y2=float(seg[0][3]),
        )
        for seg in segments
    ]


# ---------------------------------------------------------------------------
# Intersection and filtering
# ---------------------------------------------------------------------------


def compute_intersection(
    line_a: ImageLine,
    line_b: ImageLine,
) -> Optional[tuple[float, float]]:
    """Compute the intersection point of two line segments.

    Returns ``None`` when the segments are parallel or do not cross inside
    both segment extents.
    """
    x1, y1, x2, y2 = line_a.x1, line_a.y1, line_a.x2, line_a.y2
    x3, y3, x4, y4 = line_b.x1, line_b.y1, line_b.x2, line_b.y2

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-10:
        return None

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return (float(x1 + t * (x2 - x1)), float(y1 + t * (y2 - y1)))
    return None


def angle_between(image_line_a: ImageLine, image_line_b: ImageLine) -> float:
    """Return the acute angle (radians) between two line directions."""
    dx1, dy1 = image_line_a.direction
    dx2, dy2 = image_line_b.direction
    dot = dx1 * dx2 + dy1 * dy2
    dot = max(-1.0, min(1.0, dot))
    return float(np.arccos(abs(dot)))


def filter_nonparallel(
    lines: list[ImageLine],
    *,
    min_angle_deg: float = 30.0,
) -> list[tuple[ImageLine, ImageLine]]:
    """Return pairs of segments whose acute angle ≥ *min_angle_deg*."""
    min_angle_rad = np.deg2rad(min_angle_deg)
    max_angle_rad = np.pi - min_angle_rad
    pairs: list[tuple[ImageLine, ImageLine]] = []

    for i, a in enumerate(lines):
        for b in lines[i + 1 :]:
            ang = angle_between(a, b)
            if min_angle_rad <= ang <= max_angle_rad:
                pairs.append((a, b))
    return pairs


def merge_collinear(
    lines: list[ImageLine],
    *,
    angle_tol_deg: float = 5.0,
    gap_tol_px: float = 30.0,
) -> list[ImageLine]:
    """Merge nearly-collinear, nearby segments into longer lines.

    Merging is deterministic: segments are sorted by angle, then endpoints
    are clustered within the gap tolerance.
    """
    if len(lines) < 2:
        return list(lines)

    angle_tol_rad = np.deg2rad(angle_tol_deg)
    merged: list[ImageLine] = []
    used = [False] * len(lines)

    for i, base in enumerate(lines):
        if used[i]:
            continue
        cluster = [base]
        used[i] = True
        base_theta = base.theta

        for j, other in enumerate(lines):
            if used[j]:
                continue
            theta_diff = abs(base_theta - other.theta)
            if theta_diff > np.pi / 2:
                theta_diff = np.pi - theta_diff
            if theta_diff <= angle_tol_rad:
                # Check endpoint proximity.
                pts = [
                    (base.x1, base.y1),
                    (base.x2, base.y2),
                    (other.x1, other.y1),
                    (other.x2, other.y2),
                ]
                # Project all points onto base direction line and check gaps.
                dx, dy = base.direction
                projections = [dx * px + dy * py for px, py in pts]
                proj_min, proj_max = min(projections), max(projections)
                # If the gap between clusters is small enough.
                if proj_max - proj_min <= base.length + other.length + gap_tol_px:
                    cluster.append(other)
                    used[j] = True

        # Merge the cluster: take extreme endpoints.
        if len(cluster) == 1:
            merged.append(cluster[0])
        else:
            all_pts = []
            for seg in cluster:
                all_pts.extend([(seg.x1, seg.y1), (seg.x2, seg.y2)])
            # Fit a line through the points and take extremes.
            xs = [p[0] for p in all_pts]
            ys = [p[1] for p in all_pts]
            # Use PCA to find main direction.
            if len(all_pts) >= 2:
                mx = np.mean(xs)
                my = np.mean(ys)
                cov = np.cov(xs, ys)
                if cov.shape == (2, 2):
                    _, vecs = np.linalg.eigh(cov)
                    principal = vecs[:, -1]
                    projs = [
                        principal[0] * (x - mx) + principal[1] * (y - my)
                        for x, y in zip(xs, ys)
                    ]
                    idx_min = int(np.argmin(projs))
                    idx_max = int(np.argmax(projs))
                    merged.append(
                        ImageLine(
                            x1=xs[idx_min],
                            y1=ys[idx_min],
                            x2=xs[idx_max],
                            y2=ys[idx_max],
                        )
                    )
                    continue
            merged.append(cluster[0])

    return merged
