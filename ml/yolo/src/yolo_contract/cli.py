"""Dry-run-only CLIs for the isolated dataset, model, and mock contracts."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence

from .errors import ContractError, ManifestError
from .runtime import Detection2DArray, ImageRequest, MockDetectionProvider, MockProviderConfig
from .schema import load_dataset_manifest, load_model_manifest, validate_model_against_dataset


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ed-yolo")
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("train", "validate"):
        subparser = commands.add_parser(command)
        subparser.add_argument("--dataset", required=True, type=Path)
        subparser.add_argument("--model", type=Path)
        subparser.add_argument("--dry-run", action="store_true")
    exporter = commands.add_parser("export")
    exporter.add_argument("--model", required=True, type=Path)
    exporter.add_argument("--format", required=True, choices=("onnx", "openvino"))
    exporter.add_argument("--output", required=True, type=Path)
    exporter.add_argument("--dry-run", action="store_true")
    detector = commands.add_parser("detect-mock")
    detector.add_argument("--dataset", required=True, type=Path)
    detector.add_argument("--model", required=True, type=Path)
    detector.add_argument("--image-id", required=True)
    detector.add_argument("--image-sha256", required=True)
    detector.add_argument("--frame-id", required=True)
    detector.add_argument("--failure-reason")
    return parser


def _require_dry_run(value: bool) -> None:
    if not value:
        raise ManifestError("only --dry-run is supported; training and export are intentionally disabled")


def _load_bound_contract(dataset_path: Path, model_path: Path):
    dataset = load_dataset_manifest(dataset_path)
    model = load_model_manifest(model_path)
    validate_model_against_dataset(model, dataset, dataset_path)
    return dataset, model


def _detection_json(result: Detection2DArray) -> str:
    payload = {
        "contract": result.contract,
        "detections": [
            {
                "bbox": {
                    "center_x": item.bbox.center_x,
                    "center_y": item.bbox.center_y,
                    "height": item.bbox.height,
                    "width": item.bbox.width,
                },
                "class_id": item.class_id,
                "class_name": item.class_name,
                "score": item.score,
            }
            for item in result.detections
        ],
        "frame_id": result.frame_id,
        "image_id": result.image_id,
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def _run(arguments: argparse.Namespace) -> str:
    match arguments.command:
        case "train":
            _require_dry_run(arguments.dry_run)
            if arguments.model is None:
                raise MissingModelArgumentError("train requires --model")
            _load_bound_contract(arguments.dataset, arguments.model)
            return "DRY-RUN: train contract validated; no weights, training, or metrics were produced"
        case "validate":
            _require_dry_run(arguments.dry_run)
            dataset = load_dataset_manifest(arguments.dataset)
            if arguments.model is not None:
                validate_model_against_dataset(load_model_manifest(arguments.model), dataset, arguments.dataset)
            return "DRY-RUN: validation contract validated; no metrics were produced"
        case "export":
            _require_dry_run(arguments.dry_run)
            model = load_model_manifest(arguments.model)
            if model.runtime_format != arguments.format:
                raise ManifestError("export format must match the immutable model runtime format")
            return "DRY-RUN: export contract validated; no runtime artifact was written"
        case "detect-mock":
            dataset, model = _load_bound_contract(arguments.dataset, arguments.model)
            result = MockDetectionProvider(
                MockProviderConfig(
                    model=model,
                    dataset=dataset,
                    model_root=arguments.model.parent,
                    failure_reason=arguments.failure_reason,
                )
            ).detect(
                ImageRequest(
                    image_id=arguments.image_id,
                    image_sha256=arguments.image_sha256,
                    frame_id=arguments.frame_id,
                )
            )
            return _detection_json(result)
        case _:
            raise ManifestError("unsupported command")


class MissingModelArgumentError(ContractError):
    """A dry-run command needs a model contract but none was supplied."""


def main(argv: Sequence[str] | None = None) -> int:
    """Execute one bounded CLI contract operation."""
    parser = _parser()
    arguments = parser.parse_args(argv)
    try:
        print(_run(arguments))
    except ContractError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    return 0
