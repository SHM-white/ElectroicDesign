"""Small shared value types for camera transport contracts."""

from __future__ import annotations

from enum import unique

from .string_enum import StrEnum


@unique
class CameraRole(StrEnum):
    """Fixed monocular camera namespaces approved by the ROS graph contract."""

    NARROW = "narrow"
    WIDE = "wide"
