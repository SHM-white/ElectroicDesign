"""AprilTag detector using OpenCV ArUco.

Detects AprilTag 36h11 markers and returns CorrespondenceSet for PnP pose estimation.
This replaces the custom circle-cross detector with a standard fiducial marker approach.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ed_uav_perception.target_types import CorrespondenceSet, RejectReason

# AprilTag configuration
TAG_FAMILY = "tag36h11"
TAG_SIZE_M = 0.153  # 15.3cm - update to your actual measured tag size

# 3D object points for a square tag (in tag coordinate frame, z=0)
def _tag_object_points(size_m: float) -> np.ndarray:
    """Return 4 corner points of a square tag in 3D (counter-clockwise from top-left)."""
    half = size_m / 2.0
    return np.array([
        [-half,  half, 0],  # top-left
        [ half,  half, 0],  # top-right
        [ half, -half, 0],  # bottom-right
        [-half, -half, 0],  # bottom-left
    ], dtype=np.float64)


@dataclass(frozen=True, slots=True)
class DetectionFailure:
    reason: RejectReason


class AprilTagDetector:
    """AprilTag detector using OpenCV ArUco."""
    
    def __init__(self, tag_size_m: float = TAG_SIZE_M, tag_family: str = TAG_FAMILY):
        self._tag_size_m = tag_size_m
        self._object_points = _tag_object_points(tag_size_m)
        
        # Get ArUco dictionary for AprilTag
        if tag_family == "tag36h11":
            self._dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        elif tag_family == "tag36h10":
            self._dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h10)
        elif tag_family == "tag25h9":
            self._dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
        elif tag_family == "tag16h5":
            self._dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
        else:
            raise ValueError(f"Unsupported tag family: {tag_family}")
        
        # Detector parameters (tuned for reliability)
        self._params = cv2.aruco.DetectorParameters_create()
        self._params.adaptiveThreshWinSizeMin = 3
        self._params.adaptiveThreshWinSizeMax = 23
        self._params.adaptiveThreshWinSizeStep = 10
        self._params.adaptiveThreshConstant = 7
        self._params.minMarkerPerimeterRate = 0.03
        self._params.maxMarkerPerimeterRate = 4.0
        self._params.polygonalApproxAccuracyRate = 0.05
        self._params.minCornerDistanceRate = 0.05
        self._params.minDistanceToBorder = 3
        self._params.minMarkerDistanceRate = 0.05
        self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._params.cornerRefinementWinSize = 5
        self._params.cornerRefinementMaxIterations = 30
        self._params.cornerRefinementMinAccuracy = 0.1
    
    def detect(self, image: np.ndarray, target_tag_id: int | None = None) -> CorrespondenceSet | DetectionFailure:
        """Detect AprilTag and return CorrespondenceSet for PnP.
        
        Args:
            image: Input image (BGR or grayscale)
            target_tag_id: If specified, only detect this specific tag ID
            
        Returns:
            CorrespondenceSet with 4 corner points, or DetectionFailure
        """
        if image.ndim not in (2, 3) or min(image.shape[:2]) < 64:
            return DetectionFailure(RejectReason.INVALID_INPUT)
        
        # Convert to grayscale if needed
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        
        if ids is None or len(ids) == 0:
            return DetectionFailure(RejectReason.PARTIAL_GEOMETRY)
        
        # Find target tag
        if target_tag_id is not None:
            # Look for specific tag ID
            idx = np.where(ids.flatten() == target_tag_id)[0]
            if len(idx) == 0:
                return DetectionFailure(RejectReason.PARTIAL_GEOMETRY)
            idx = idx[0]
        else:
            # Use the first detected tag (or largest)
            if len(ids) > 1:
                # Pick the tag with largest area
                areas = []
                for corner in corners:
                    pts = corner.reshape(4, 2)
                    area = cv2.contourArea(pts)
                    areas.append(area)
                idx = int(np.argmax(areas))
            else:
                idx = 0
        
        # Get corners (4 points, counter-clockwise from top-left)
        tag_corners = corners[idx].reshape(4, 2)
        
        return CorrespondenceSet(
            object_points=self._object_points.copy(),
            image_points=tag_corners.astype(np.float64),
            symmetry_order=1,  # AprilTag has no symmetry (each corner is unique)
        )


# Global detector instance (initialized once)
_detector: AprilTagDetector | None = None


def detect_apriltag(
    image: np.ndarray,
    tag_size_m: float = TAG_SIZE_M,
    tag_family: str = TAG_FAMILY,
    target_tag_id: int | None = None,
) -> CorrespondenceSet | DetectionFailure:
    """Detect AprilTag and return CorrespondenceSet for PnP.
    
    This is the main entry point for AprilTag detection, compatible with
    the existing target_pipeline.py interface.
    """
    global _detector
    if _detector is None or _detector._tag_size_m != tag_size_m:
        _detector = AprilTagDetector(tag_size_m, tag_family)
    return _detector.detect(image, target_tag_id)
