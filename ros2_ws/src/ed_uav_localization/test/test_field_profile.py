from __future__ import annotations

import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization.field_profile import loader


VALID_PROFILE = """\
version: 1
profile_type: field
profile_id: rotated-proxy
units:
  length: m
  angle: rad
frame:
  id: map
  convention: ENU
provenance:
  classification: current_field
  activation: eligible
takeoff:
  origin:
    x_m: 10.0
    y_m: -1.0
  commanded_heading_rad: 0.6
colors:
  - id: boundary-dark
    label: black
boundary_segments:
  - id: south
    start: {x_m: 10.0, y_m: -1.0}
    end: {x_m: 15.0, y_m: 2.0}
    color_id: boundary-dark
  - id: east
    start: {x_m: 15.0, y_m: 2.0}
    end: {x_m: 12.0, y_m: 7.0}
    color_id: boundary-dark
  - id: north
    start: {x_m: 12.0, y_m: 7.0}
    end: {x_m: 7.0, y_m: 4.0}
    color_id: boundary-dark
  - id: west
    start: {x_m: 7.0, y_m: 4.0}
    end: {x_m: 10.0, y_m: -1.0}
    color_id: boundary-dark
allowed_zone:
  id: flyable
  vertices:
    - {x_m: 10.0, y_m: -1.0}
    - {x_m: 15.0, y_m: 2.0}
    - {x_m: 12.0, y_m: 7.0}
    - {x_m: 7.0, y_m: 4.0}
no_fly_zones:
  - id: obstacle
    vertices:
      - {x_m: 10.3, y_m: 1.3}
      - {x_m: 11.3, y_m: 1.9}
      - {x_m: 10.7, y_m: 2.9}
      - {x_m: 9.7, y_m: 2.3}
altitude:
  minimum_m: 0.4
  maximum_m: 2.5
  takeoff_m: 1.2
landmarks:
  - id: inspection-a
    kind: inspection
    position: {x_m: 11.0, y_m: 3.0}
"""


def test_round_trip_preserves_si_units_and_ids() -> None:
    # Given: an unseen rotated and translated field profile.
    profile = loader.load_profile_text(VALID_PROFILE, "valid.yaml")

    # When: its canonical YAML is parsed again.
    round_tripped = loader.load_profile_text(loader.dump_profile(profile), "round-trip.yaml")

    # Then: semantic identifiers and SI declarations survive unchanged.
    assert round_tripped.units.length == "m"
    assert round_tripped.units.angle == "rad"
    assert [segment.id for segment in round_tripped.boundary_segments] == [
        "south",
        "east",
        "north",
        "west",
    ]


def test_rejects_profile_without_declared_si_units() -> None:
    # Given: an otherwise valid profile without its unit declaration.
    invalid = VALID_PROFILE.replace("units:\n  length: m\n  angle: rad\n", "")

    # When / Then: parsing fails before a profile reaches localization.
    with pytest.raises(ValueError, match="units"):
        loader.load_profile_text(invalid, "missing-units.yaml")


def test_rejects_non_si_unit_declaration() -> None:
    # Given: a profile declaring legacy centimeters instead of SI meters.
    invalid = VALID_PROFILE.replace("length: m", "length: cm")

    # When / Then: parsing rejects the incompatible unit at the boundary.
    with pytest.raises(ValueError, match="length"):
        loader.load_profile_text(invalid, "invalid-unit.yaml")


def test_rejects_unknown_schema_key() -> None:
    # Given: an otherwise valid profile with a misspelled root key.
    invalid = VALID_PROFILE + "unexpected_dimension: 999\n"

    # When / Then: strict parsing reports the unknown key.
    with pytest.raises(ValueError, match="unexpected_dimension"):
        loader.load_profile_text(invalid, "unknown-key.yaml")


def test_rejects_duplicate_identifier() -> None:
    # Given: two boundary segments with the same identifier.
    invalid = VALID_PROFILE.replace("id: east\n", "id: south\n", 1)

    # When / Then: the duplicate is rejected independently of geometry.
    with pytest.raises(ValueError, match="duplicate ID: south"):
        loader.load_profile_text(invalid, "duplicate-id.yaml")


def test_rejects_self_intersecting_allowed_zone() -> None:
    # Given: a bow-tie allowed zone.
    invalid = VALID_PROFILE.replace(
        "    - {x_m: 15.0, y_m: 2.0}\n    - {x_m: 12.0, y_m: 7.0}\n",
        "    - {x_m: 12.0, y_m: 7.0}\n    - {x_m: 15.0, y_m: 2.0}\n",
        1,
    )

    # When / Then: polygon topology is rejected before activation.
    with pytest.raises(ValueError, match=r"(?s)allowed_zone.*self-intersects"):
        loader.load_profile_text(invalid, "self-intersection.yaml")


def test_rejects_overlapping_no_fly_zones() -> None:
    # Given: two no-fly zones whose interiors overlap.
    invalid = VALID_PROFILE.replace(
        "altitude:\n",
        "  - id: obstacle-b\n"
        "    vertices:\n"
        "      - {x_m: 10.5, y_m: 1.5}\n"
        "      - {x_m: 11.5, y_m: 2.1}\n"
        "      - {x_m: 10.9, y_m: 3.1}\n"
        "      - {x_m: 9.9, y_m: 2.5}\n"
        "altitude:\n",
    )

    # When / Then: overlapping exclusions are rejected.
    with pytest.raises(ValueError, match="no_fly_zones.*overlap"):
        loader.load_profile_text(invalid, "overlapping-zones.yaml")


def test_rejects_one_line_absolute_pose_profile() -> None:
    # Given: a field that exposes only one line orientation.
    invalid = VALID_PROFILE.replace(
        "  - id: east\n    start: {x_m: 15.0, y_m: 2.0}\n    end: {x_m: 12.0, y_m: 7.0}\n    color_id: boundary-dark\n"
        "  - id: north\n    start: {x_m: 12.0, y_m: 7.0}\n    end: {x_m: 7.0, y_m: 4.0}\n    color_id: boundary-dark\n"
        "  - id: west\n    start: {x_m: 7.0, y_m: 4.0}\n    end: {x_m: 10.0, y_m: -1.0}\n    color_id: boundary-dark\n",
        "",
    )

    # When / Then: an unobservable full planar pose is not accepted.
    with pytest.raises(ValueError, match="two non-parallel"):
        loader.load_profile_text(invalid, "one-line.yaml")


def test_rejects_malformed_yaml() -> None:
    # Given: malformed YAML.
    invalid = "version: [\n"

    # When / Then: syntax errors are typed as profile input failures.
    with pytest.raises(ValueError, match="malformed YAML"):
        loader.load_profile_text(invalid, "malformed.yaml")
