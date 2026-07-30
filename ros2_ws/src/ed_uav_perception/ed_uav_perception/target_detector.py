"""Landing marker detector with feature-point based PnP correspondence.

Marker geometry:
  - Outer circle: 50cm diameter (25cm radius)
  - Inner circle: 30cm diameter (15cm radius)
  - Cross: 2cm wide lines extending from center to outer circle

Feature points for PnP:
  1. Center point (cross intersection) - MUST detect
  2. 4 cross-circle intersections (where cross meets inner circle) - need 3+
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Sequence

import cv2
import numpy as np

from ed_uav_perception.target_types import CorrespondenceSet, RejectReason

TARGET_REVISION = "d2026-circle-cross-v1"
OUTER_RADIUS_M = 0.25
INNER_RADIUS_M = 0.15

# 3D model points for PnP (in marker coordinate frame, z=0)
MODEL_CENTER = np.array([0.0, 0.0, 0.0])
MODEL_CROSS_CIRCLE = np.array([
    [INNER_RADIUS_M, 0.0, 0.0],   # right
    [0.0, INNER_RADIUS_M, 0.0],   # top
    [-INNER_RADIUS_M, 0.0, 0.0],  # left
    [0.0, -INNER_RADIUS_M, 0.0],  # bottom
])


@dataclass(frozen=True, slots=True)
class DetectionFailure:
    reason: RejectReason


def _find_marker_region(gray: np.ndarray) -> tuple[np.ndarray, tuple[int, int, int, int]] | None:
    """Find the marker region using Otsu threshold and connected components."""
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        return None
    
    # Find largest component (should be the marker)
    best_idx = 1
    best_area = stats[1, cv2.CC_STAT_AREA]
    for i in range(2, count):
        area = stats[i, cv2.CC_STAT_AREA]
        if area > best_area:
            best_area = area
            best_idx = i
    
    if best_area < 500:
        return None
    
    x, y, w, h, _ = stats[best_idx]
    aspect = max(w, h) / max(1, min(w, h))
    if aspect > 2.0:
        return None
    
    marker_mask = (labels == best_idx).astype(np.uint8) * 255
    return marker_mask, (x, y, w, h)


def _find_inner_circle_hough(gray: np.ndarray, marker_mask: np.ndarray, bbox: tuple[int, int, int, int]) -> tuple[tuple[int, int], int] | None:
    """Find the inner circle using Hough Circle Transform on the original grayscale image."""
    x, y, w, h = bbox
    # Add margin around bbox for better circle detection
    margin = 20
    x1 = max(0, x - margin)
    y1 = max(0, y - margin)
    x2 = min(gray.shape[1], x + w + margin)
    y2 = min(gray.shape[0], y + h + margin)
    
    roi = gray[y1:y2, x1:x2]
    roi_mask = marker_mask[y1:y2, x1:x2]
    
    # Apply CLAHE for better contrast
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    roi_enhanced = clahe.apply(roi)
    
    # Estimate expected radius range from bbox
    expected_radius = min(w, h) // 4
    min_radius = max(15, int(expected_radius * 0.6))
    max_radius = int(expected_radius * 1.4)
    
    # Hough circles with tuned parameters
    circles = cv2.HoughCircles(
        roi_enhanced,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=min(w, h) // 3,
        param1=100,
        param2=25,
        minRadius=min_radius,
        maxRadius=max_radius,
    )
    
    if circles is None:
        return None
    
    # Find the circle closest to center of bbox
    cx_roi = (x + w // 2) - x1
    cy_roi = (y + h // 2) - y1
    best_circle = None
    best_dist = float('inf')
    
    for circle in circles[0]:
        cx, cy, r = circle
        dist = np.sqrt((cx - cx_roi)**2 + (cy - cy_roi)**2)
        if dist < best_dist:
            best_dist = dist
            best_circle = (int(cx) + x1, int(cy) + y1, int(r))
    
    if best_circle is None:
        return None
    
    return (best_circle[0], best_circle[1]), best_circle[2]


def _find_cross_center(marker_mask: np.ndarray, inner_center: tuple[int, int], inner_radius: int) -> tuple[int, int]:
    """Find the center point as the intersection of the cross arms.
    
    Uses the binary mask to find the center of horizontal and vertical
    white regions (cross arms) through the inner circle center.
    This is stable because it uses the thresholded mask, not raw grayscale.
    """
    h, w = marker_mask.shape
    cx, cy = inner_center
    
    # Scan horizontal line through center
    h_line = marker_mask[cy, :]
    h_white = np.where(h_line > 0)[0]
    
    # Scan vertical line through center
    v_line = marker_mask[:, cx]
    v_white = np.where(v_line > 0)[0]
    
    if len(h_white) > 0 and len(v_white) > 0:
        # The center of the cross is the center of the white regions
        # This is stable because we're using the binary mask
        h_center = int(np.mean(h_white))
        v_center = int(np.mean(v_white))
        return (h_center, v_center)
    
    return inner_center


def _find_cross_circle_intersections(
    gray: np.ndarray,
    marker_mask: np.ndarray,
    inner_center: tuple[int, int],
    inner_radius: int,
) -> list[tuple[int, int]]:
    """Find where the cross lines intersect the inner circle edge.
    
    Strategy: 
    1. Use the grayscale image to find the inner circle edge (dark ring)
    2. Scan along the 4 cardinal directions from center
    3. Find the transition from dark cross arm to lighter gap to dark circle edge
    """
    h, w = gray.shape
    cx, cy = inner_center
    
    intersections = []
    
    for angle_deg in [0, 90, 180, 270]:
        angle_rad = np.radians(angle_deg)
        dx, dy = np.cos(angle_rad), np.sin(angle_rad)
        
        # Scan outward from center
        # We're looking for: dark cross arm -> lighter gap -> dark inner circle edge
        profile = []
        scan_start = max(0, inner_radius - 20)
        scan_end = min(min(cx, w-cx, cy, h-cy), inner_radius + 20)
        
        for dist in range(scan_start, scan_end):
            px = int(cx + dist * dx)
            py = int(cy + dist * dy)
            if 0 <= px < w and 0 <= py < h:
                profile.append((dist, gray[py, px]))
        
        if len(profile) < 10:
            continue
        
        # Find the inner circle edge: look for a dip in values (dark ring)
        # The profile should show: lighter area -> dark circle edge -> lighter gap
        values = [v for _, v in profile]
        dists = [d for d, _ in profile]
        
        # Smooth the profile
        kernel_size = 3
        smoothed = np.convolve(values, np.ones(kernel_size)/kernel_size, mode='valid')
        smoothed_dists = dists[kernel_size//2:-kernel_size//2+1] if kernel_size % 2 == 1 else dists[kernel_size//2:-kernel_size//2]
        
        if len(smoothed) < 5:
            continue
        
        # Find the darkest point near the expected inner radius
        expected_idx = len(smoothed) // 2
        search_range = min(10, len(smoothed) // 4)
        search_start = max(0, expected_idx - search_range)
        search_end = min(len(smoothed), expected_idx + search_range)
        
        search_region = smoothed[search_start:search_end]
        if len(search_region) == 0:
            continue
        
        min_idx = np.argmin(search_region) + search_start
        best_dist = int(smoothed_dists[min_idx])
        
        px = int(cx + best_dist * dx)
        py = int(cy + best_dist * dy)
        intersections.append((px, py))
    
    return intersections


def detect_target(image: np.ndarray, revision: str) -> CorrespondenceSet | DetectionFailure:
    """Detect marker and return feature points for PnP.
    
    Returns CorrespondenceSet with:
      - 1 center point (cross intersection)
      - 4 cross-circle intersection points (where cross meets inner circle)
    
    Minimum 4 points required for PnP.
    """
    if revision != TARGET_REVISION:
        return DetectionFailure(RejectReason.WRONG_REVISION)
    if image.ndim not in (2, 3) or min(image.shape[:2]) < 64:
        return DetectionFailure(RejectReason.INVALID_INPUT)
    
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Step 1: Find marker region
    result = _find_marker_region(gray)
    if result is None:
        return DetectionFailure(RejectReason.PARTIAL_GEOMETRY)
    marker_mask, bbox = result
    
    # Step 2: Find inner circle using Hough transform
    circle_result = _find_inner_circle_hough(gray, marker_mask, bbox)
    if circle_result is None:
        return DetectionFailure(RejectReason.PARTIAL_GEOMETRY)
    inner_center, inner_radius = circle_result
    
    # Step 3: Find center point from cross intersection
    center_point = _find_cross_center(marker_mask, inner_center, inner_radius)
    
    # Step 4: Find cross-circle intersections
    cross_circle_points = _find_cross_circle_intersections(gray, marker_mask, inner_center, inner_radius)
    
    # Need at least center + 3 cross-circle intersections
    if len(cross_circle_points) < 3:
        return DetectionFailure(RejectReason.PARTIAL_GEOMETRY)
    
    # Build correspondence set
    image_points = [center_point] + cross_circle_points
    object_points = [MODEL_CENTER] + list(MODEL_CROSS_CIRCLE[:len(cross_circle_points)])
    
    return CorrespondenceSet(
        object_points=np.array(object_points, dtype=np.float64),
        image_points=np.array(image_points, dtype=np.float64),
        symmetry_order=4,
    )
