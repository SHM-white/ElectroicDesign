#!/usr/bin/env python3
"""Validate the P05 competition evidence bundle and structured BOM."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from datetime import date
from pathlib import Path
from typing import Final


CLAIM_TOKEN = re.compile(r"\[\[(C-[A-Z0-9-]+)\]\]")
HASH = re.compile(r"(?:[0-9a-f]{40}|[0-9a-f]{64})\Z", re.IGNORECASE)
TABLE_SEPARATOR = re.compile(r"^\|[\s|:-]+\|\s*$")
DIRECTIVE = re.compile(r"ignore (?:all )?(?:previous|prior) instructions", re.IGNORECASE)
DOCUMENTS: Final = ("HISTORICAL_UAV_TASKS.md", "2026_SCENARIOS.md", "FIELD_ADAPTATION.md")
HISTORICAL_HEADERS: Final = ("Year", "Objective", "Arena", "Autonomy", "Sensing constraints", "Scoring", "Failure mode", "Reusable capability", "Evidence")
CLAIM_FIELDS: Final = ("id", "topic", "status", "source_url", "source_kind", "page", "retrieved_at", "source_hash", "confidence")
BOM_FIELDS: Final = ("id", "quantity", "ownership", "mass_g", "mass_status", "steady_w", "steady_power_status", "peak_w", "peak_power_status", "voltage", "connector", "mount", "thermal_path", "firmware_or_driver", "procurement_status", "spare_status", "evidence")
CLAIM_STATUSES: Final = frozenset({"archive-inventory", "archive-listed-only", "confirmed", "confirmed-context", "historical", "historical-supported", "inferred", "inventory-context", "measurement-pending", "owned", "scenario-gated", "unknown", "untrusted"})
SOURCE_KINDS: Final = frozenset({"inventory", "local-guess", "local-pdf", "official-rule", "pinned-archive", "plan", "research-summary"})
CONFIDENCE_LEVELS: Final = frozenset({"low", "medium", "high"})
OWNERSHIP_STATUSES: Final = frozenset({"missing", "owned", "replacement-required", "scenario-gated"})
PROCUREMENT_STATUSES: Final = frozenset({"owned", "procure", "replacement-required", "scenario-gated"})
SPARE_STATUSES: Final = frozenset({"available", "needed", "not-applicable", "not-required", "procure", "required", "replacement-required", "scenario-gated", "spare-needed", "stocked", "unknown"})


def load_json(path: Path, errors: list[str]) -> dict[str, object] | None:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        errors.append(f"missing JSON: {path}")
        return None
    except json.JSONDecodeError:
        errors.append(f"invalid JSON: {path}")
        return None

    match raw:
        case dict() as mapping: return mapping
        case _:
            errors.append(f"JSON root must be an object: {path}")
            return None


def table_rows(path: Path, errors: list[str]) -> list[str]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        errors.append(f"missing document: {path}")
        return []

    if path.name == DOCUMENTS[0]:
        header_line = next((line.strip() for line in lines if line.strip().startswith("|") and not TABLE_SEPARATOR.fullmatch(line.strip())), "")
        headers = tuple(cell.strip() for cell in header_line.strip("|").split("|")) if header_line else ()
        for dimension in (dimension for dimension in HISTORICAL_HEADERS if dimension not in headers):
            errors.append(f"historical matrix missing dimension: {dimension}")
        if headers and set(headers) == set(HISTORICAL_HEADERS) and headers != HISTORICAL_HEADERS: errors.append("historical matrix dimension order mismatch")

    rows: list[str] = []
    for line in lines:
        candidate = line.strip()
        if not candidate.startswith("|") or TABLE_SEPARATOR.fullmatch(candidate):
            continue
        if "Evidence" in candidate:
            continue
        if not CLAIM_TOKEN.search(candidate):
            errors.append(f"uncited table row: {path}: {candidate}")
        rows.append(candidate)
    return rows


def validate_claim_hash(competition: Path, record: dict[str, object], source_cache: tuple[str, str]) -> str | None:
    claim_id, expected_hash = str(record["id"]), str(record["source_hash"]).lower()
    source_url, (cache_text, cache_hash) = str(record["source_url"]), source_cache
    if not source_url.startswith("local://"):
        locator = source_url.rsplit("/", 1)[-1].lower()
        cached_artifact = any(locator in line and expected_hash in line for line in cache_text.splitlines())
        cache_inventory = "/tree/" in source_url and expected_hash == cache_hash
        if not cached_artifact and not cache_inventory:
            return f"claim source hash mismatch: {claim_id}"
        return None
    workspace = competition.parent.parent.resolve()
    target = (workspace / source_url.removeprefix("local://")).resolve()
    if not target.is_relative_to(workspace):
        return f"local source escapes workspace: {claim_id}"
    try:
        actual_hash = hashlib.sha256(target.read_bytes()).hexdigest()
    except FileNotFoundError:
        return f"missing local source: {claim_id}: {target}"
    return None if actual_hash == expected_hash else f"claim source hash mismatch: local source hash mismatch: {claim_id}"


def validate_claims(competition: Path, errors: list[str]) -> dict[str, bool]:
    evidence = load_json(competition / "evidence.json", errors)
    if evidence is None:
        return {}
    if evidence.get("schema_version") != 1:
        errors.append("evidence schema_version must be 1")

    cache_text, cache_hash = "", ""
    cache_raw = evidence.get("source_cache")
    match cache_raw:
        case {"path": str() as relative_path, "sha256": str() as expected_hash}:
            cache_path = competition / relative_path
            if cache_path.parent.resolve() != competition.resolve():
                errors.append("source cache must remain inside docs/competition")
            else:
                try:
                    cache_bytes = cache_path.read_bytes()
                    actual_hash = hashlib.sha256(cache_bytes).hexdigest()
                except FileNotFoundError:
                    errors.append(f"missing source cache: {cache_path}")
                else:
                    if actual_hash != expected_hash:
                        errors.append("source cache hash mismatch")
                    else:
                        cache_text = cache_bytes.decode("utf-8").lower()
                        cache_hash = expected_hash.lower()
        case _:
            errors.append("missing source_cache path or sha256")

    claims_raw = evidence.get("claims")
    match claims_raw:
        case list() as claims: pass
        case _:
            errors.append("claims must be a list")
            return {}

    lidar_support: dict[str, bool] = {}
    for claim in claims:
        match claim:
            case dict() as record: pass
            case _:
                errors.append("claim must be an object")
                continue
        missing = [field for field in CLAIM_FIELDS if not record.get(field)]
        if missing:
            errors.append(f"claim missing fields: {', '.join(missing)}")
            continue
        claim_id = str(record["id"])
        if claim_id in lidar_support:
            errors.append(f"duplicate claim id: {claim_id}")
        status = str(record["status"])
        source_kind = str(record["source_kind"])
        confidence = str(record["confidence"])
        source_hash = str(record["source_hash"])
        if not HASH.fullmatch(source_hash):
            errors.append(f"invalid source hash: {claim_id}")
        else:
            hash_error = validate_claim_hash(competition, record, (cache_text, cache_hash))
            if hash_error is not None: errors.append(hash_error)
        try:
            date.fromisoformat(str(record["retrieved_at"]))
        except ValueError:
            errors.append(f"invalid retrieval date: {claim_id}")
        for value, allowed, label in ((status, CLAIM_STATUSES, "claim status"), (source_kind, SOURCE_KINDS, "source kind"), (confidence, CONFIDENCE_LEVELS, "confidence")):
            if value not in allowed: errors.append(f"invalid {label}: {claim_id}")
        if DIRECTIVE.search(json.dumps(record, ensure_ascii=True)): errors.append(f"directive-like evidence: {claim_id}")
        rule_quote = str(record.get("rule_quote", "")).lower()
        supported = source_kind == "official-rule" and "lidar" in rule_quote
        lidar_support[claim_id] = supported
        if str(record["topic"]).lower() == "lidar" and status == "confirmed" and not supported:
            errors.append(f"unsupported confirmed lidar claim: {claim_id}")
    return lidar_support


def validate_bom(bom_markdown: Path, claim_ids: set[str], errors: list[str]) -> None:
    bom = load_json(bom_markdown.with_suffix(".json"), errors)
    if bom is None:
        return
    if bom.get("schema_version") != 1:
        errors.append("BOM schema_version must be 1")
    items_raw = bom.get("items")
    match items_raw:
        case list() as items: pass
        case _:
            errors.append("BOM items must be a list")
            return

    known = {"mass_g": 0.0, "steady_w": 0.0, "peak_w": 0.0}
    unknown = {"mass_g": 0, "steady_w": 0, "peak_w": 0}
    for item in items:
        match item:
            case dict() as record: pass
            case _:
                errors.append("BOM item must be an object")
                continue
        missing = [field for field in BOM_FIELDS if field not in record or record[field] == ""]
        if missing:
            errors.append(f"BOM item missing fields: {', '.join(missing)}")
        quantity = record.get("quantity")
        if not isinstance(quantity, int) or quantity < 1:
            errors.append(f"invalid quantity: {record['id']}")
            quantity = 0
        if "evidence" in record and str(record["evidence"]) not in claim_ids:
            errors.append(f"BOM item has unknown evidence: {record['id']}")
        for field, allowed in (
            ("ownership", OWNERSHIP_STATUSES),
            ("procurement_status", PROCUREMENT_STATUSES),
            ("spare_status", SPARE_STATUSES),
        ):
            if field in record and record[field] != "" and str(record[field]) not in allowed:
                errors.append(f"invalid {field}: invalid {field.replace('_', ' ')}: {record['id']}")
        for value_field, status_field, label in (
            ("mass_g", "mass_status", "mass"),
            ("steady_w", "steady_power_status", "steady power"),
            ("peak_w", "peak_power_status", "peak power"),
        ):
            if value_field not in record or status_field not in record:
                continue
            status = record[status_field]
            value = record[value_field]
            if status == "unknown" and value is not None:
                errors.append(f"unknown {label} recorded as a value: {record['id']}")
            if status == "known" and not isinstance(value, int | float):
                errors.append(f"known {label} missing numeric value: {record['id']}")
            match (status, value):
                case ("known", int() | float() as measured):
                    known[value_field] += float(measured) * quantity
                case ("unknown", _):
                    unknown[value_field] += 1
                case ("known", _):
                    pass
                case _:
                    errors.append(f"invalid {label} status: {record['id']}")

    totals_raw = bom.get("totals")
    match totals_raw:
        case dict() as totals: pass
        case _:
            errors.append("BOM totals must be an object")
            return
    expected = {"known_mass_g": known["mass_g"], "known_steady_w": known["steady_w"], "known_peak_w": known["peak_w"], "unknown_mass_items": unknown["mass_g"], "unknown_steady_power_items": unknown["steady_w"], "unknown_peak_power_items": unknown["peak_w"]}
    for field, value in expected.items():
        if totals.get(field) != value:
            errors.append(f"BOM total mismatch: {field}")


def validate(competition: Path, bom_markdown: Path, strict: bool) -> list[str]:
    errors: list[str] = []
    lidar_support = validate_claims(competition, errors)
    claim_ids = set(lidar_support)
    citations: set[str] = set()
    for name in DOCUMENTS:
        for row in table_rows(competition / name, errors):
            row_citations = set(CLAIM_TOKEN.findall(row))
            citations.update(row_citations)
            lowered = row.lower()
            if "lidar" in lowered and "confirmed" in lowered and not any(lidar_support.get(citation, False) for citation in row_citations):
                errors.append(f"unsupported confirmed lidar document: document confirmed lidar lacks official support: {competition / name}")
    for row in table_rows(bom_markdown, errors):
        citations.update(CLAIM_TOKEN.findall(row))
    for citation in sorted(citations - claim_ids):
        errors.append(f"unknown evidence citation: {citation}")
    if strict:
        validate_bom(bom_markdown, claim_ids, errors)
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate P05 competition evidence and BOM records.")
    parser.add_argument("--strict", action="store_true", help="validate structured BOM fields and totals")
    parser.add_argument("competition", type=Path, help="competition document directory")
    parser.add_argument("bom", type=Path, help="BOM Markdown path")
    arguments = parser.parse_args()
    errors = validate(arguments.competition.resolve(), arguments.bom.resolve(), arguments.strict)
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    print("PASS: cited competition evidence and BOM records are valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
