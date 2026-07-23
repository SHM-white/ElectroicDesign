#!/usr/bin/env python3
"""Validate ED UAV field profile YAML files without a ROS runtime."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing_extensions import assert_never


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = WORKSPACE_ROOT / "ros2_ws" / "src" / "ed_uav_localization"
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization.field_profile.loader import FieldProfileError, load_profile, profile_hash
from ed_uav_localization.field_profile.model import KnownFieldProfile, UnknownArenaProfile


def parse_arguments(argv: list[str]) -> argparse.Namespace:
    """Parse one profile path or a directory of profiles."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path, nargs="?", help="single YAML profile")
    parser.add_argument("--all", dest="directory", type=Path, help="validate every YAML file in DIRECTORY")
    arguments = parser.parse_args(argv)
    if (arguments.profile is None) == (arguments.directory is None):
        parser.error("provide exactly one PROFILE or --all DIRECTORY")
    return arguments


def profile_paths(arguments: argparse.Namespace) -> tuple[Path, ...]:
    """Return deterministic profile paths selected by the CLI."""
    if arguments.profile is not None:
        return (arguments.profile,)
    paths = tuple(sorted((*arguments.directory.glob("*.yaml"), *arguments.directory.glob("*.yml"))))
    if paths:
        return paths
    raise FieldProfileError(str(arguments.directory), "no YAML field profiles found")


def validate_path(path: Path) -> bool:
    """Validate one file and print an explicit activation state."""
    try:
        profile = load_profile(path)
    except FieldProfileError as error:
        print(f"FIELD PROFILE: RED {error}", file=sys.stderr)
        return False
    digest = profile_hash(profile)
    match profile:
        case KnownFieldProfile(provenance=provenance) if provenance.activation == "eligible":
            print(f"FIELD PROFILE: PASS path={path} id={profile.profile_id} hash={digest}")
        case KnownFieldProfile():
            print(
                "FIELD PROFILE: VALID-BLOCKED "
                f"path={path} id={profile.profile_id} class={profile.provenance.classification} "
                f"hash={digest}"
            )
        case UnknownArenaProfile():
            print(
                "FIELD PROFILE: VALID-BLOCKED "
                f"path={path} id={profile.profile_id} kind=unknown hash={digest}"
            )
        case unreachable:
            assert_never(unreachable)
    return True


def main(argv: list[str]) -> int:
    """Run validation and report every selected profile rather than failing fast."""
    arguments = parse_arguments(argv)
    try:
        paths = profile_paths(arguments)
    except FieldProfileError as error:
        print(f"FIELD PROFILE: RED {error}", file=sys.stderr)
        return 2
    outcomes = tuple(validate_path(path) for path in paths)
    return 0 if all(outcomes) else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
