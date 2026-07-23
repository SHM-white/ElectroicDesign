"""Small JSON boundary helpers shared by immutable contract parsers."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import TypeAlias

from .errors import ManifestError, ModelIntegrityError


JsonScalar: TypeAlias = str | int | float | bool | None
JsonValue: TypeAlias = JsonScalar | list["JsonValue"] | dict[str, "JsonValue"]


def load_json_mapping(path: Path) -> dict[str, JsonValue]:
    """Load an untrusted JSON object or raise one typed boundary failure."""
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ManifestError(f"missing manifest: {path}") from error
    except OSError as error:
        raise ManifestError(f"cannot read manifest: {path}") from error
    except json.JSONDecodeError as error:
        raise ManifestError(f"malformed manifest JSON: {error.msg}") from error
    if not isinstance(raw, dict):
        raise ManifestError("manifest root must be an object")
    return raw


def sha256_bytes(contents: bytes) -> str:
    """Return a lowercase SHA-256 digest."""
    return hashlib.sha256(contents).hexdigest()


def sha256_file(path: Path) -> str:
    """Hash one existing artifact without retaining its contents."""
    try:
        return sha256_bytes(path.read_bytes())
    except OSError as error:
        raise ModelIntegrityError(f"cannot read model artifact: {path}") from error

