"""Typed failures raised at the immutable contract boundary."""

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ContractError(Exception):
    """Base failure for untrusted dataset, model, or provider input."""

    message: str

    def __str__(self) -> str:
        return self.message


class ManifestError(ContractError):
    """The manifest structure or declared metadata is invalid."""


class MissingMetadataError(ManifestError):
    """A required provenance or runtime field is missing."""


class DuplicateHashError(ManifestError):
    """Distinct records reuse a content hash within one split."""


class SplitOverlapError(ManifestError):
    """One content hash occurs in more than one immutable split."""


class ClassMapDriftError(ManifestError):
    """Model and dataset class maps are not identical."""


class ModelIntegrityError(ContractError):
    """A model artifact is missing, escapes its root, or has a bad hash."""


class ProviderFailureError(ContractError):
    """A provider failed before it could produce a detection result."""
