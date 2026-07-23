# Field Profiles

Each YAML file is parsed by `ed_uav_localization` before a mission can use it.
Coordinates are meters in the REP-103 `map` ENU frame and headings are radians.
The schema rejects undeclared fields, legacy units, duplicate IDs, invalid polygon
topology, overlapping no-fly zones, and field geometry that cannot support a full
planar pose.

`historical_2021_example.yaml` is a blocked historical illustration derived from
the legacy material. It is not an arena measurement or an activation-ready setup.
`unknown_arena.yaml` is intentionally geometry-free and blocked until official
field data is entered.

Validate a directory with:

```bash
python3 tools/validate_field_profile.py --all config/fields
```
