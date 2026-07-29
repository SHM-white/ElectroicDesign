"""Python 3.10-compatible string enum base for ROS Humble."""

from __future__ import annotations

from enum import Enum


class StrEnum(str, Enum):
    """Backport the value-preserving behavior required from enum.StrEnum."""

    def __str__(self) -> str:
        return self.value
