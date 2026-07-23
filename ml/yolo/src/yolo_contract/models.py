"""Frozen internal values for dataset and model contracts."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ClassDefinition:
    """One immutable numeric class identity."""

    class_id: int
    name: str


@dataclass(frozen=True, slots=True)
class DatasetSource:
    """Immutable upstream dataset attribution."""

    url: str
    revision: str
    license_id: str


@dataclass(frozen=True, slots=True)
class DatasetSample:
    """One content-addressed image and its permanently assigned split."""

    sample_id: str
    split: str
    sha256: str
    source_url: str
    license_id: str
    class_ids: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class DatasetManifest:
    """A parsed dataset manifest whose samples have immutable split ownership."""

    dataset_id: str
    source: DatasetSource
    class_map: tuple[ClassDefinition, ...]
    samples: tuple[DatasetSample, ...]


@dataclass(frozen=True, slots=True)
class ResizeSpec:
    """The spatial input transform required by a runtime artifact."""

    width: int
    height: int
    strategy: str


@dataclass(frozen=True, slots=True)
class Preprocessing:
    """The complete image preprocessing contract."""

    color_space: str
    layout: str
    resize: ResizeSpec
    scale: float


@dataclass(frozen=True, slots=True)
class RuntimeSpec:
    """Portable inference runtime tensor names and format."""

    runtime_format: str
    input_tensor: str
    output_tensor: str


@dataclass(frozen=True, slots=True)
class ModelArtifact:
    """A path-relative content-addressed runtime artifact."""

    relative_path: str
    sha256: str


@dataclass(frozen=True, slots=True)
class TrainingProvider:
    """Pinned source metadata for the isolated training/export environment."""

    repository_url: str
    revision: str
    license_id: str


@dataclass(frozen=True, slots=True)
class ModelManifest:
    """A runtime artifact bound to one exact dataset manifest."""

    model_id: str
    dataset_manifest_sha256: str
    class_map: tuple[ClassDefinition, ...]
    preprocessing: Preprocessing
    runtime: RuntimeSpec
    artifact: ModelArtifact
    training_provider: TrainingProvider

    @property
    def runtime_format(self) -> str:
        """Expose the portable runtime format without leaking nested metadata."""
        return self.runtime.runtime_format
