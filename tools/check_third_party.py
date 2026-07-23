#!/usr/bin/env python3
# Run: python3 tools/check_third_party.py --strict
from __future__ import annotations

import argparse
from pathlib import Path

from third_party_validation import validate


def main() -> int:
    """Run the checker and print one outcome per provenance rule."""
    parser = argparse.ArgumentParser(description="Validate pinned third-party provenance.")
    parser.add_argument("--strict", action="store_true", help="also inspect local third-party checkouts")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    arguments = parser.parse_args()
    errors = validate(arguments.root.resolve(), arguments.strict)
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    print("PASS: immutable third-party source, license, and dataset provenance is valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
