# Isolated YOLO Contract

This directory is the only P12-owned YOLO training/export boundary. It pins the
Ultralytics Git source to `7a159ea24ec94c47cf25c75785e0a56e47ba4e7b`, matching
P04 in `docs/provenance/third-party-sources.json`. The pinned source is
`AGPL-3.0-only`; internal competition use does not remove review obligations for
distribution, remote access, model artifacts, or corresponding source.

The ROS product does not import this package or Ultralytics. P12 exports a
provider-neutral value contract shaped for later adaptation to
`vision_msgs/Detection2DArray`; P15 owns that ROS runtime integration.

## Contract Files

- `schemas/dataset-manifest.schema.json` describes immutable source, license,
  hash, class-map, and train/val/test sample records.
- `schemas/model-manifest.schema.json` describes a dataset-bound model,
  preprocessing, ONNX/OpenVINO runtime metadata, artifact digest, and pinned
  training-provider provenance.
- `src/yolo_contract/schema.py` is the executable strict parser. It rejects
  unknown metadata, split overlap, duplicate hashes, attribution omissions,
  class drift, unpinned providers, missing preprocessing, and path escape.

No approved dataset, model weights, trained model, ONNX graph, or OpenVINO IR
is included here. Tests use temporary text artifacts only to prove hash handling;
they are not model weights and are removed by pytest.

## Commands

The isolated environment is intentionally pinned through `pyproject.toml`:

```bash
uv sync --project ml/yolo --frozen
```

After a reviewed lockfile can be produced from the P04-pinned source, use:

```bash
cd ml/yolo
ed-yolo train --dataset DATASET.json --model MODEL.json --dry-run
ed-yolo validate --dataset DATASET.json --model MODEL.json --dry-run
ed-yolo export --model MODEL.json --format onnx --output OUT.onnx --dry-run
ed-yolo detect-mock --dataset DATASET.json --model MODEL.json \
  --image-id FRAME --image-sha256 SHA256 --frame-id camera_narrow_optical_frame
```

`train`, `validate`, and `export` require `--dry-run`; they validate plans only
and never download weights, train, compute mAP/latency, or write an export.
`detect-mock` verifies the model artifact digest and emits one deterministic
provider-neutral detection. A corrupt artifact or configured provider failure
returns an error and emits no misleading detection result.

The cleanup-safe acceptance driver executes these exact CLI commands against a
temporary synthetic manifest and removes all fixture data afterward:

```bash
PYTHONPATH=ml/yolo/src ./.venv/bin/python ml/yolo/tests/manual_cli_smoke.py
```

## Runtime Boundary

`runtime.format` is only `onnx` or `openvino`. Its preprocessing metadata is
mandatory: color space, tensor layout, resize width/height/strategy, and scale.
Any future ONNX or OpenVINO provider must implement `DetectionProvider` and may
not turn a raw bounding box into a vehicle pose. Camera calibration and terminal
geometry remain P15 work.
