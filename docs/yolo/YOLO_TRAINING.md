# YOLO Training and Deployment Runbook

> Source: `ml/yolo/` (contract layer), `ros2_ws/src/ed_uav_perception/` (runtime),
> `THIRD_PARTY_NOTICES.md` (license pinning), `docs/legal/OPEN_SOURCE.md` (release gate).

---

## 1. Licensing — AGPL-3.0 Obligations

Ultralytics is pinned at:

| Field | Value |
|---|---|
| Repository | `https://github.com/ultralytics/ultralytics.git` |
| Revision | `7a159ea24ec94c47cf25c75785e0a56e47ba4e7b` |
| License | **AGPL-3.0-only** |
| SHA-256 | `0d96a4ff68ad6d4b6f1f30f713b18d5184912ba8dd389f86aa7710db079abcb0` |

### AGPL Implications

- **Distribution**: If you distribute model weights trained with Ultralytics,
  you must also distribute the corresponding Ultralytics source code.
- **Remote access**: If the model is used as a network service (e.g., inference
  API), the source must be available to users.
- **Model artifacts**: ONNX/OpenVINO exports are derivative works — they carry
  the AGPL obligation.
- **Our mitigation**: The ML environment (`ml/yolo/`) is **isolated** from the
  ROS process. `ed_uav_perception` does NOT import Ultralytics. Inference uses
  provider-neutral ONNX/OpenVINO runtimes only.

### Release Gate

Before any distribution of model artifacts, review:
`docs/legal/OPEN_SOURCE.md` — artifact composition, GPL/AGPL conditions,
modifications, source-offer mechanics, model/dataset licenses.

---

## 2. Project Layout

```
ml/yolo/
├── pyproject.toml                    # Package: ed-yolo-contract v0.1.0
├── schemas/
│   ├── dataset-manifest.schema.json  # JSON Schema for dataset manifest
│   └── model-manifest.schema.json    # JSON Schema for model manifest
├── src/yolo_contract/
│   ├── __init__.py                   # Public API surface
│   ├── models.py                     # Frozen dataclasses (DatasetManifest, ModelManifest, etc.)
│   ├── schema.py                     # Strict parsers, split validation
│   ├── runtime.py                    # Provider-neutral ONNX/OpenVINO protocol
│   ├── cli.py                        # Dry-run CLI (train, validate, export, detect-mock)
│   ├── errors.py                     # Typed error hierarchy
│   └── jsonio.py                     # JSON loading + SHA-256 hashing
└── tests/
    ├── test_schema.py                # Manifest parser tests
    ├── test_runtime.py               # Runtime determinism tests
    └── test_cli.py                   # CLI integration tests
```

**No trained weights, dataset files, or actual training scripts exist.** The
contract layer defines the schema and validation — training execution is future work.

---

## 3. Dataset Collection

### 3.1 Image Sources

| Source | Use Case | Resolution |
|---|---|---|
| `/camera/narrow/image_raw` | Narrow-field detections | Per runtime profile |
| `/camera/wide/image_raw` | Wide-field boundary/localization | Per runtime profile |
| Manual capture (USB camera) | Offline dataset building | Match target resolution |

### 3.2 Capture Guidelines

- Capture at the **same resolution** as the target runtime mode
- Include varied lighting (indoor, outdoor, shadow, direct sun)
- Include varied angles (0°–60° from fronto-parallel)
- Include partial occlusions (10%–50% of target)
- Include negative examples (no target visible)
- Minimum **200 images per class** for initial training

### 3.3 Storage

```
datasets/
└── marker-v1/
    ├── images/
    │   ├── train/    # 70%
    │   ├── val/      # 15%
    │   └── test/     # 15%
    └── labels/
        ├── train/    # YOLO format: class_id cx cy w h (normalized)
        ├── val/
        └── test/
```

---

## 4. Labeling Guidelines

### 4.1 Tool

Use [CVAT](https://cvat.ai/) or [Label Studio](https://labelstud.io/) for
annotation. Export in YOLO format.

### 4.2 Bounding Box Rules

| Rule | Description |
|---|---|
| Tight fit | Box should be 1–2 pixels inside the object boundary |
| Full object | Include the entire object, even if partially occluded |
| Occluded parts | Draw the box as if the object were fully visible |
| Truncated objects | If >30% visible, label it. If <30%, skip |
| Overlapping boxes | Allowed — each object gets its own box |

### 4.3 Class Definitions

Define classes in the dataset manifest `class_map`:

```json
{
  "class_map": [
    {"id": 0, "name": "marker"},
    {"id": 1, "name": "terminal_target"},
    {"id": 2, "name": "obstacle"}
  ]
}
```

**Constraint**: Class IDs must be contiguous starting from 0. Class names must
be lowercase with underscores.

---

## 5. Immutable Train/Val/Test Splits

### 5.1 Split Requirements

From `schema.py::load_dataset_manifest()`:

| Requirement | Enforcement |
|---|---|
| All three splits present | `MissingMetadataError` if any split empty |
| No cross-split hash overlap | `SplitOverlapError` if SHA-256 appears in multiple splits |
| No duplicate hashes within split | `DuplicateHashError` |
| Every sample has license | `MissingMetadataError` |
| Every sample has SHA-256 | Schema validation failure |

### 5.2 Dataset Manifest Format

```json
{
  "schema_version": 1,
  "dataset_id": "marker-v1-2026-07",
  "source": {
    "url": "https://github.com/org/datasets/marker-v1",
    "revision": "abc123...",
    "license": "CC-BY-4.0"
  },
  "class_map": [
    {"id": 0, "name": "marker"}
  ],
  "samples": [
    {
      "id": "img_001",
      "split": "train",
      "sha256": "a1b2c3...",
      "source_url": "https://...",
      "license": "CC-BY-4.0",
      "class_ids": [0]
    }
  ]
}
```

### 5.3 Validation

```bash
# Validate dataset manifest
cd ml/yolo
python -m yolo_contract validate --dataset datasets/marker-v1/dataset.json
```

The schema parser rejects:
- Floating revisions (must be SHA-1 pinned)
- Missing license attribution
- Split overlap (same image in train AND val)
- Path traversal in sample URLs

---

## 6. Training Procedure

### 6.1 Environment Setup

```bash
# Create isolated ML environment (separate from ROS)
cd ml/yolo
python -m venv .venv
source .venv/bin/activate
pip install -e ".[train]"
```

This installs Ultralytics (AGPL-3.0) in the isolated `ml/yolo` environment only.

### 6.2 Training Data Config (YOLO format)

Create `data.yaml`:

```yaml
path: /absolute/path/to/datasets/marker-v1
train: images/train
val: images/val
test: images/test
names:
  0: marker
```

### 6.3 Training Command

```bash
# YOLOv8n (nano) — recommended for Intel i5 inference
yolo detect train \
  model=yolov8n.pt \
  data=data.yaml \
  epochs=100 \
  imgsz=640 \
  batch=8 \
  device=cpu \
  project=runs/train \
  name=marker-v1
```

| Parameter | Value | Rationale |
|---|---|---|
| `model` | `yolov8n.pt` | Nano — fits Intel i5 CPU inference budget |
| `imgsz` | 640 | Standard YOLO input, matches preprocessing contract |
| `batch` | 8 | Conservative for 16 GB RAM |
| `device` | `cpu` | No GPU on target platform |
| `epochs` | 100 | Start here; increase if underfitting |

### 6.4 Monitoring

```bash
# TensorBoard
tensorboard --logdir runs/train

# Or check results
cat runs/train/marker-v1/results.csv
```

### 6.5 Acceptance Criteria

| Metric | Target |
|---|---|
| mAP@50 | ≥ 0.85 |
| mAP@50-95 | ≥ 0.60 |
| Inference latency (CPU, i5) | < 100 ms per frame |
| False positive rate | < 5% on test set |

---

## 7. Model Manifest

### 7.1 Manifest Format

After training, create `model-manifest.json`:

```json
{
  "schema_version": 1,
  "model_id": "marker-yolov8n-v1",
  "dataset_manifest_sha256": "<sha256 of dataset.json>",
  "class_map": [
    {"id": 0, "name": "marker"}
  ],
  "preprocessing": {
    "color_space": "RGB",
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
    "path": "models/marker-yolov8n-v1.onnx",
    "sha256": "<sha256 of .onnx file>"
  },
  "training_provider": {
    "repository_url": "https://github.com/ultralytics/ultralytics.git",
    "revision": "7a159ea24ec94c47cf25c75785e0a56e47ba4e7b",
    "license": "AGPL-3.0-only"
  }
}
```

### 7.2 Validation

```bash
python -m yolo_contract validate --model model-manifest.json
```

Rejects:
- `training_provider` mismatch with pinned Ultralytics revision
- Missing or mismatched `dataset_manifest_sha256`
- Artifact SHA-256 mismatch with actual file
- Path escape (artifact path outside manifest directory)

---

## 8. Export to ONNX / OpenVINO

### 8.1 ONNX Export

```bash
yolo export \
  model=runs/train/marker-v1/weights/best.pt \
  format=onnx \
  imgsz=640 \
  simplify=True
```

Output: `best.onnx`

### 8.2 OpenVINO Export

```bash
yolo export \
  model=runs/train/marker-v1/weights/best.pt \
  format=openvino \
  imgsz=640
```

Output: `best_openvino_model/` directory with `.xml` + `.bin` files.

### 8.3 Intel i5 Optimization

OpenVINO is preferred for Intel i5 targets:

```bash
# Install OpenVINO runtime
pip install openvino

# Benchmark
yolo benchmark model=best_openvino_model imgsz=640 device=cpu
```

### 8.4 Export Acceptance Criteria

| Gate | Criterion |
|---|---|
| ONNX loads | `onnxruntime.InferenceSession` succeeds |
| OpenVINO loads | `openvino.Core.compile_model` succeeds |
| Output shape matches | `[1, num_classes+4, num_anchors]` for YOLOv8 |
| Numerical parity | ONNX output within 1e-4 of PyTorch output on test image |
| Artifact SHA-256 | Matches `model-manifest.json` `artifact.sha256` |

---

## 9. ROS Runtime Integration

### 9.1 Provider Architecture

Source: `ed_uav_perception/provider_interface.py`

```python
class DetectorProvider(ABC):
    @abstractmethod
    def detect(self, image: np.ndarray) -> list[Detection2D]: ...
    version: str
    provider_type: str
```

| Provider | Status | Import |
|---|---|---|
| `MockDetectorProvider` | ✅ Implemented | None (deterministic mock) |
| `ONNXDetectorProvider` | ❌ Stub (`NotImplementedError`) | `onnxruntime` |
| `OpenVINODetectorProvider` | ❌ Stub (`NotImplementedError`) | `openvino` |

### 9.2 Detection Pipeline

```
/camera/narrow/image_raw
  → DetectorNode._image_callback()
    → stale check (age > 0.5 s → reject)
    → rate limit (10 Hz)
    → CvBridge.imgmsg_to_cv2() → RGB numpy
    → DetectorProvider.detect(image)
    → Publish Detection2DArray to /perception/detections
```

### 9.3 Model Loading (future)

```python
# ONNX provider (to implement)
class ONNXDetectorProvider(DetectorProvider):
    def __init__(self, model_path: str, manifest: ModelManifest):
        import onnxruntime as ort
        self.session = ort.InferenceSession(model_path)
        self.manifest = manifest

    def detect(self, image: np.ndarray) -> list[Detection2D]:
        # Preprocess: resize, normalize, transpose to NCHW
        # Run inference
        # Postprocess: NMS, scale bboxes to image coords
        ...
```

### 9.4 Key Constraint

**Providers must NOT import Ultralytics in the ROS process.** The ML
environment is isolated. The ROS process only uses ONNX Runtime or OpenVINO
Runtime.

---

## 10. CLI Reference

Entry point: `ed-yolo` (maps to `yolo_contract.cli:main`)

**All commands require `--dry-run` — no actual training/export executes.**

```bash
# Train (dry-run validation only)
ed-yolo train --dataset dataset.json --model model.json --dry-run

# Validate manifest consistency
ed-yolo validate --dataset dataset.json --model model.json --dry-run

# Export (dry-run validation only)
ed-yolo export --model model.json --format onnx --output out.onnx --dry-run

# Mock detection (deterministic, no model needed)
ed-yolo detect-mock --dataset dataset.json --model model.json \
  --image-id FRAME --image-sha256 SHA256 --frame-id camera_narrow_optical_frame
```

---

## 11. Acceptance Criteria Summary

| Gate | Criterion | Tool |
|---|---|---|
| Dataset immutability | No split overlap, unique SHA-256s | `schema.py` parser |
| Model manifest integrity | Artifact SHA-256 matches, provider pin matches | `schema.py` parser |
| Training quality | mAP@50 ≥ 0.85, mAP@50-95 ≥ 0.60 | `yolo benchmark` |
| Export correctness | ONNX/OpenVINO loads, output shape matches, numerical parity | Export benchmark |
| CPU latency | < 100 ms/frame on Intel i5 | `yolo benchmark` |
| License compliance | AGPL-3.0 source offer ready | `check_third_party.py --strict` |
| ROS isolation | No Ultralytics import in `ed_uav_perception` | Grep for `ultralytics` in `ros2_ws/src/` |
