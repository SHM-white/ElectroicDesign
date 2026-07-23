"""Read the P04 Livox revision from its authoritative dependency manifest."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


LIVOX_REVISION = "13eb05e4e6dd7a765b934d0c5fd6236676a57b49"


@dataclass(frozen=True, slots=True)
class PinDriftError(Exception):
    actual_revision: str

    def __str__(self) -> str:
        return f"livox_ros_driver2 revision drift: {self.actual_revision}"


@dataclass(frozen=True, slots=True)
class PinFormatError(Exception):
    detail: str

    def __str__(self) -> str:
        return f"invalid dependencies.repos: {self.detail}"


def validate_livox_pin(repos_path: Path) -> str:
    """Return the reviewed P04 revision or reject malformed and stale pin data."""
    try:
        payload = json.loads(repos_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise PinFormatError(detail=str(error)) from error
    except json.JSONDecodeError as error:
        raise PinFormatError(detail=str(error)) from error
    if not isinstance(payload, dict):
        raise PinFormatError(detail="root is not an object")
    repositories = payload.get("repositories")
    if not isinstance(repositories, dict):
        raise PinFormatError(detail="repositories is not an object")
    livox = repositories.get("livox_ros_driver2")
    if not isinstance(livox, dict):
        raise PinFormatError(detail="livox_ros_driver2 is missing")
    revision = livox.get("version")
    if not isinstance(revision, str):
        raise PinFormatError(detail="livox_ros_driver2.version is missing")
    if revision != LIVOX_REVISION:
        raise PinDriftError(actual_revision=revision)
    return revision
