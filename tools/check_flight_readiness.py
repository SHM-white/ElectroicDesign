#!/usr/bin/env python3
"""Offline flight-readiness evidence checker for task 26 preparation."""

from __future__ import annotations

import argparse
from pathlib import Path

from flight_readiness_validation import validate


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate offline flight-readiness evidence.")
    parser.add_argument("--bom", type=Path, required=True, help="BOM JSON path")
    parser.add_argument("--measurements", type=Path, required=True, help="dated measurement directory")
    parser.add_argument("--strict", action="store_true", help="required compatibility flag")
    args = parser.parse_args()
    if not args.strict:
        print("ERROR: --strict is required")
        return 1
    errors = validate(args.bom.resolve(), args.measurements.resolve())
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    print("PASS: flight readiness evidence satisfies offline gates")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
