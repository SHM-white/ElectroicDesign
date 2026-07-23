# Model Release Checklist

> **Status**: Contract layer implemented (Task 12)
> **Owner**: `ml/yolo`
> **License**: AGPL-3.0-only (Ultralytics)

---

## 1. Overview

This document defines the release checklist for YOLO model artifacts. The
system enforces strict isolation between the AGPL-3.0 licensed Ultralytics
training environment and the ROS runtime, with full provenance tracking and
hash verification at every boundary.

### Isolation Architecture

```
ml/yolo/                          ros2_ws/src/ed_uav_perception/
├── pyproject.toml                ├── provider_interface.py
├── src/yolo_contract/            │   ├── ONNXDetectorProvider (stub)
│   ├── schema.py                 │   └── OpenVINODetectorProvider (stub)
│   ├── runtime.py                ├── detector_node.py
│   └── cli.py                    └── localizer.py
└── tests/
```

**Critical rule**: `ros2_ws/` never imports `ultralytics`. The ROS runtime
consumes only provider-neutral ONNX/OpenVINO outputs via the `DetectionProvider`
protocol.

### Current State

| Component | File | Status |
|---|---|---|
| Schema validation | `ml/yolo/src/yolo_contract/schema.py` | **Implemented** |
| Hash verification | `ml/yolo/src/yolo_contract/jsonio.py` | **Implemented** |
| MockDetectionProvider | `ml/yolo/src/yolo_contract/runtime.py` | **Implemented** |
| CLI (dry-run only) | `ml/yolo/src/yolo_contract/cli.py` | **Implemented** |
| ONNXDetectorProvider | `ros2_ws/src/ed_uav_perception/provider_interface.py` | **Stub** |
| OpenVINODetectorProvider | `ros2_ws/src/ed_uav_perception/provider_interface.py` | **Stub** |
| Actual training | `ml/yolo/` | **Intentionally blocked** |
| Actual export | `ml/yolo/` | **Intentionally blocked** |
| Model weights | — | **None approved or downloaded** |

---

## 2. AGPL-3.0 Compliance

Ultralytics YOLOv8 is licensed under AGPL-3.0. This creates specific
obligations for any distribution or network use.

### 2.1 License Obligations

| Obligation | Requirement | Mitigation |
|---|---|---|
| **Source disclosure** | Complete source must be available to recipients | Pin exact upstream commit |
| **Same license** | Combined works must be under AGPL-3.0 | Keep YOLO isolated in `ml/yolo/` |
| **Corresponding source** | Must provide source for all linked components | Separate process boundary |
| **Network use** | If used over network, source must be available | Internal use only (competition) |

### 2.2 Internal Use Exception

Internal use (competition preparation, testing) does **not** trigger
distribution obligations. However, post-event publication of the complete
system would require AGPL compliance for the YOLO component.

### 2.3 Isolation Strategy

The project maintains AGPL isolation through:

1. **Separate process**: YOLO inference runs as a separate ROS node
2. **Standard interfaces**: Communication via `vision_msgs/Detection2DArray`
3. **No import**: ROS process never imports `ultralytics`
4. **Separate weights**: Trained weights published separately under AGPL-3.0

From `docs/legal/OPEN_SOURCE.md`:

> Ultralytics is recorded as AGPL-3.0-only. In addition to distribution
> questions, AGPL can require an offer of corresponding source to users who
> interact with a modified covered program over a network. The project
> therefore keeps its use isolated, pins the upstream source, and blocks
> model-weight downloads until task-specific provenance exists.

### 2.4 Verification

```bash
# Verify no ultralytics import in ROS workspace
grep -r "import ultralytics" ros2_ws/src/  # Should return nothing
grep -r "from ultralytics" ros2_ws/src/    # Should return nothing
```

---

## 3. Version Pinning

Ultralytics is pinned at a single commit across three locations that are
cross-validated.

### 3.1 Triple-Locked Pin

| Location | File | Line |
|---|---|---|
| pip install source | `ml/yolo/pyproject.toml` | Line 8 |
| VCS import | `ros2_ws/dependencies.repos` | Line 16 |
| Runtime constants | `ml/yolo/src/yolo_contract/schema.py` | Lines 25-27 |

### 3.2 Pinned Values

From `ml/yolo/src/yolo_contract/schema.py`:

```python
SCHEMA_VERSION = 1
ULTRALYTICS_REPOSITORY = "https://github.com/ultralytics/ultralytics.git"
ULTRALYTICS_REVISION = "7a159ea24ec94c47cf25c75785e0a56e47ba4e7b"
ULTRALYTICS_LICENSE = "AGPL-3.0-only"
```

From `ml/yolo/pyproject.toml`:

```toml
[project]
name = "ed-yolo-contract"
version = "0.1.0"
dependencies = [
  "ultralytics @ git+https://github.com/ultralytics/ultralytics.git@7a159ea24ec94c47cf25c75785e0a56e47ba4e7b",
]
```

### 3.3 Enforcement

The schema parser enforces the pin at load time:

```python
# From schema.py (lines 170-172)
if training_provider != TrainingProvider(
    ULTRALYTICS_REPOSITORY, ULTRALYTICS_REVISION, ULTRALYTICS_LICENSE
):
    raise ManifestError(
        "training_provider must match the reviewed P04 Ultralytics source and AGPL license"
    )
```

Any model manifest with a different `training_provider` is rejected.

---

## 4. Hash Verification Chain

### 4.1 Dataset Manifest Hashes

Every sample in the dataset manifest has a SHA-256 hash:

```json
{
  "id": "sample_001",
  "split": "train",
  "sha256": "abc123...",
  "source_url": "https://...",
  "license": "CC-BY-4.0",
  "class_ids": [0, 1]
}
```

The schema parser validates:
- No duplicate hashes within a split (`DuplicateHashError`)
- No hash overlap across splits (`SplitOverlapError`)
- All hashes are lowercase SHA-256 (regex: `[0-9a-f]{64}`)

### 4.2 Model Manifest Hashes

The model manifest binds to the dataset via `dataset_manifest_sha256`:

```json
{
  "model_id": "terminal_v1",
  "dataset_manifest_sha256": "def456...",
  "artifact": {
    "path": "best.onnx",
    "sha256": "ghi789..."
  }
}
```

Validation:
- `dataset_manifest_sha256` must match the actual dataset file hash
- `artifact.sha256` must match the actual model file hash
- `artifact.path` must be relative (no `..` escape)

### 4.3 Hash Functions

From `ml/yolo/src/yolo_contract/jsonio.py`:

```python
def sha256_bytes(contents: bytes) -> str:
    """Return a lowercase SHA-256 digest."""
    return hashlib.sha256(contents).hexdigest()

def sha256_file(path: Path) -> str:
    """Hash one existing artifact without retaining its contents."""
    try:
        return sha256_bytes(path.read_bytes())
    except OSError as error:
        raise ModelIntegrityError(f"cannot read model artifact: {path}") from error
```

### 4.4 Third-Party License Verification

From `tools/third_party_validation.py`:
- SHA-256 verification of license files
- Cached licenses in `docs/provenance/licenses/`
- Reference archives with exact SHAs
- Checkout integrity verification

---

## 5. Export Procedures

### 5.1 ONNX Export

```bash
# Dry-run (validates plan only, no actual export)
PYTHONPATH=ml/yolo/src ./.venv/bin/python -m yolo_contract.cli export \
  --model ml/yolo/models/v1/model-manifest.json \
  --format onnx \
  --output ml/yolo/models/v1/best.onnx \
  --dry-run
```

Export metadata requirements:
- Input shape: `[640, 640, 3]` (from preprocessing)
- Color space: `rgb` (from preprocessing)
- Layout: `NCHW` (from preprocessing)
- Scale: `0.00392156862745098` (1/255, from preprocessing)

### 5.2 OpenVINO Export

```bash
# Dry-run (validates plan only, no actual export)
PYTHONPATH=ml/yolo/src ./.venv/bin/python -m yolo_contract.cli export \
  --model ml/yolo/models/v1/model-manifest.json \
  --format openvino \
  --output ml/yolo/models/v1/openvino/ \
  --dry-run
```

OpenVINO optimization for Intel i5:
- Target: `CPU` (Intel i5-1240P)
- Precision: `FP16` (half-precision)
- Batching: `1` (single image inference)

### 5.3 Export CLI

From `ml/yolo/src/yolo_contract/cli.py`:

```python
exporter = commands.add_parser("export")
exporter.add_argument("--model", required=True, type=Path)
exporter.add_argument("--format", required=True, choices=("onnx", "openvino"))
exporter.add_argument("--output", required=True, type=Path)
exporter.add_argument("--dry-run", action="store_true")
```

**Note**: All export commands require `--dry-run`. Actual export is intentionally
disabled until training is approved.

### 5.4 Validation

After export, verify:

```bash
# Validate model against dataset
PYTHONPATH=ml/yolo/src ./.venv/bin/python -m yolo_contract.cli validate \
  --dataset ml/yolo/datasets/v1/dataset-manifest.json \
  --model ml/yolo/models/v1/model-manifest.json \
  --dry-run
```

---

## 6. Pre-Release Testing

### 6.1 Dataset Verification

- [ ] Immutable train/val/test splits verified (no overlap)
- [ ] All image hashes recorded in manifest
- [ ] Class map is consistent across all splits
- [ ] Source/license recorded for every image
- [ ] No duplicate images across splits

### 6.2 Model Training

- [ ] Training script uses pinned Ultralytics commit
- [ ] Hyperparameters documented
- [ ] Training logs preserved
- [ ] Validation metrics recorded (mAP50, mAP50-95)
- [ ] No overfitting detected (train/val gap < 10%)

### 6.3 Export Verification

- [ ] ONNX export successful
- [ ] ONNX inference matches PyTorch within tolerance
- [ ] OpenVINO export successful (Intel target)
- [ ] Export metadata includes input shape, class names
- [ ] Model hash recorded in manifest

### 6.4 Integration Testing

- [ ] Mock provider returns deterministic detections
- [ ] ONNX provider loads and infers correctly
- [ ] Terminal geometry produces valid poses
- [ ] No `ultralytics` import in ROS process
- [ ] All tests pass without model weights

### 6.5 Test Suite

From `ml/yolo/tests/`:

| Test File | Tests | Coverage |
|---|---|---|
| `test_schema.py` | 225 lines | Manifest parsing, split overlap, duplicate hash, class drift |
| `test_runtime.py` | 102 lines | Hash rejection, provider failure, determinism |
| `test_cli.py` | 119 lines | Dry-run, mock detection, malformed input |
| `manual_cli_smoke.py` | 86 lines | Cleanup-safe acceptance driver |

---

## 7. Model Manifest Format

### 7.1 Schema

From `ml/yolo/schemas/model-manifest.schema.json`:

```json
{
  "schema_version": 1,
  "model_id": "terminal_v1",
  "dataset_manifest_sha256": "abc123...",
  "class_map": [
    {"id": 0, "name": "terminal_target"},
    {"id": 1, "name": "obstacle"},
    {"id": 2, "name": "marker"}
  ],
  "preprocessing": {
    "color_space": "rgb",
    "layout": "NCHW",
    "resize": {"width": 640, "height": 640, "strategy": "letterbox"},
    "scale": 0.00392156862745098
  },
  "runtime": {
    "format": "onnx",
    "input_tensor": "images",
    "output_tensor": "output0"
  },
  "artifact": {
    "path": "best.onnx",
    "sha256": "ghi789..."
  },
  "training_provider": {
    "repository_url": "https://github.com/ultralytics/ultralytics.git",
    "revision": "7a159ea24ec94c47cf25c75785e0a56e47ba4e7b",
    "license": "AGPL-3.0-only"
  }
}
```

### 7.2 Runtime Contract

From `ml/yolo/src/yolo_contract/runtime.py`:

```python
ROS_DETECTION_CONTRACT = "vision_msgs/Detection2DArray-compatible/v1"

class DetectionProvider(Protocol):
    def detect(self, request: ImageRequest) -> Detection2DArray:
        """Return detected pixel-normalized boxes or raise a typed provider error."""
```

Provider-neutral output format:
- `contract`: Version string
- `image_id`: Content-addressed image identifier
- `frame_id`: Camera frame
- `detections`: Tuple of `Detection2D` (class_id, class_name, score, bbox)

---

## 8. ROS Integration

### 8.1 Provider Interface

From `ros2_ws/src/ed_uav_perception/ed_uav_perception/provider_interface.py`:

```python
class DetectorProvider(ABC):
    @abstractmethod
    def detect(self, request: ImageRequest) -> Detection2DArray:
        ...

class ONNXDetectorProvider(DetectorProvider):
    """Stub — not yet implemented."""
    def detect(self, request: ImageRequest) -> Detection2DArray:
        raise NotImplementedError("ONNX provider not yet implemented")

class OpenVINODetectorProvider(DetectorProvider):
    """Stub — not yet implemented."""
    def detect(self, request: ImageRequest) -> Detection2DArray:
        raise NotImplementedError("OpenVINO provider not yet implemented")
```

### 8.2 Model Validation

From `ros2_ws/src/ed_uav_perception/ed_uav_perception/model_validator.py`:

Pydantic-based manifest validation for ROS-side model loading. Validates:
- Schema version
- Class map consistency
- Preprocessing parameters
- Runtime format
- Artifact path safety

### 8.3 Localizer

From `ros2_ws/src/ed_uav_perception/ed_uav_perception/localizer.py`:

> YOLO detections are never used as a pose source. Camera calibration and
> terminal geometry remain P15 work.

---

## 9. Version Pinning Summary

| Component | Pin Location | Pin Value |
|---|---|---|
| Ultralytics commit | `ml/yolo/pyproject.toml` | `7a159ea24ec94c47cf25c75785e0a56e47ba4e7b` |
| Ultralytics commit | `ros2_ws/dependencies.repos` | `7a159ea24ec94c47cf25c75785e0a56e47ba4e7b` |
| Ultralytics commit | `ml/yolo/src/yolo_contract/schema.py` | `7a159ea24ec94c47cf25c75785e0a56e47ba4e7b` |
| Livox driver | `ros2_ws/dependencies.repos` | `13eb05e` |
| FAST-LIO | `ros2_ws/dependencies.repos` | Pinned |
| ONNX opset | `ml/yolo/pyproject.toml` | 17 |
| Input shape | `ml/yolo/src/yolo_contract/schema.py` | `[640, 640, 3]` |

---

## 10. Acceptance Criteria

From Task 12:

| Criterion | Verification |
|---|---|
| Schema rejects unknown metadata | `test_schema.py` — unknown field tests |
| Schema rejects split overlap | `test_schema.py` — `SplitOverlapError` tests |
| Schema rejects duplicate hashes | `test_schema.py` — `DuplicateHashError` tests |
| Schema rejects class drift | `test_schema.py` — `ClassMapDriftError` tests |
| Schema rejects unpinned provider | `test_schema.py` — provider mismatch tests |
| Mock provider verifies hash | `test_runtime.py` — hash rejection tests |
| Mock provider surfaces failures | `test_runtime.py` — provider failure tests |
| CLI dry-run validates plans | `test_cli.py` — dry-run tests |
| No ultralytics in ROS workspace | `grep -r "import ultralytics" ros2_ws/src/` |
| All tests pass without weights | `pytest ml/yolo/tests/` |

---

## 11. References

- `ml/yolo/src/yolo_contract/schema.py` — Manifest parsers and validation
- `ml/yolo/src/yolo_contract/runtime.py` — Provider interface and mock adapter
- `ml/yolo/src/yolo_contract/cli.py` — Export CLI
- `ml/yolo/src/yolo_contract/jsonio.py` — Hash verification functions
- `ml/yolo/src/yolo_contract/errors.py` — Typed error hierarchy
- `ml/yolo/schemas/model-manifest.schema.json` — JSON Schema for model manifests
- `ml/yolo/schemas/dataset-manifest.schema.json` — JSON Schema for dataset manifests
- `ml/yolo/pyproject.toml` — Ultralytics version pin
- `ros2_ws/src/ed_uav_perception/ed_uav_perception/provider_interface.py` — ROS provider stubs
- `ros2_ws/src/ed_uav_perception/ed_uav_perception/model_validator.py` — Manifest validation
- `docs/legal/OPEN_SOURCE.md` — Open source use boundary
- `docs/provenance/third-party-sources.json` — Machine-readable provenance
