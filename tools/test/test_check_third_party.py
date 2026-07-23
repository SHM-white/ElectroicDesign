from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CHECKER_PATH = PROJECT_ROOT / "tools" / "check_third_party.py"
EXAMPLE_REVISION = "a" * 40
EXAMPLE_LICENSE = "Example upstream license text.\n"
SOURCE_IDS = ("livox_ros_driver2", "fast_lio_ros2", "ultralytics")


def write_json(path: Path, contents) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(contents, indent=2) + "\n", encoding="utf-8")


def create_valid_workspace(tmp_path: Path) -> Path:
    workspace = tmp_path / "workspace"
    license_hash = hashlib.sha256(EXAMPLE_LICENSE.encode()).hexdigest()
    repositories = {}
    sources = []

    for source_id in SOURCE_IDS:
        license_path = workspace / f"docs/provenance/licenses/{source_id}.txt"
        license_path.parent.mkdir(parents=True, exist_ok=True)
        license_path.write_text(EXAMPLE_LICENSE, encoding="utf-8")
        repository_url = f"https://example.invalid/vendor/{source_id}.git"
        repositories[source_id] = {
            "type": "git",
            "url": repository_url,
            "version": EXAMPLE_REVISION,
        }
        sources.append(
            {
                "id": source_id,
                "repository_url": repository_url,
                "checkout_directory": source_id,
                "revision": EXAMPLE_REVISION,
                "license": {
                    "spdx": "MIT",
                    "repository_path": "LICENSE",
                    "source_url": repository_url.removesuffix(".git")
                    + "/blob/"
                    + EXAMPLE_REVISION
                    + "/LICENSE",
                    "cache_path": f"docs/provenance/licenses/{source_id}.txt",
                    "sha256": license_hash,
                    "retrieved_at": "2026-07-22",
                },
                "corresponding_source": {
                    "repository_url": repository_url,
                    "revision": EXAMPLE_REVISION,
                    "availability": "upstream-pinned",
                    "archive_url": repository_url.removesuffix(".git")
                    + "/archive/"
                    + EXAMPLE_REVISION
                    + ".tar.gz",
                },
                "invocation_boundary": {
                    "kind": "separate-process",
                    "description": "The project invokes the upstream process without copying source into ed_* packages.",
                },
                "forbidden_copy_markers": [source_id],
            }
        )

    write_json(
        workspace / "ros2_ws/dependencies.repos",
        {"repositories": repositories},
    )
    write_json(
        workspace / "docs/provenance/third-party-sources.json",
        {
            "schema_version": 1,
            "sources": sources,
        },
    )
    write_json(
        workspace / "docs/provenance/dataset-manifest.json",
        {
            "schema_version": 1,
            "policy": {
                "approved_dataset_imports": "none",
                "model_weight_downloads": "prohibited",
            },
            "datasets": [],
            "model_weights": [],
            "reference_archives": [],
        },
    )
    return workspace


def run_checker(workspace: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(CHECKER_PATH), "--strict", "--root", str(workspace)],
        check=False,
        capture_output=True,
        text=True,
    )


def combined_output(result: subprocess.CompletedProcess[str]) -> str:
    return result.stdout + result.stderr


def test_checker_accepts_complete_immutable_provenance_when_records_match(tmp_path: Path) -> None:
    # Given: a complete workspace with exact source, license, and data records.
    workspace = create_valid_workspace(tmp_path)

    # When: the strict checker validates the workspace.
    result = run_checker(workspace)

    # Then: the immutable provenance records pass.
    assert result.returncode == 0, combined_output(result)


def test_checker_rejects_floating_revision_when_manifest_uses_main(tmp_path: Path) -> None:
    # Given: otherwise valid provenance with a floating VCS revision.
    workspace = create_valid_workspace(tmp_path)
    repos_path = workspace / "ros2_ws/dependencies.repos"
    repos = json.loads(repos_path.read_text(encoding="utf-8"))
    repos["repositories"]["livox_ros_driver2"]["version"] = "main"
    write_json(repos_path, repos)

    # When: the strict checker validates the altered manifest.
    result = run_checker(workspace)

    # Then: it explains that the floating revision is forbidden.
    assert result.returncode != 0
    assert "floating revision" in combined_output(result)


def test_checker_rejects_license_hash_drift_when_cache_changes(tmp_path: Path) -> None:
    # Given: a valid manifest with a stale cached license file.
    workspace = create_valid_workspace(tmp_path)
    (workspace / "docs/provenance/licenses/livox_ros_driver2.txt").write_text(
        "Changed upstream license text.\n", encoding="utf-8"
    )

    # When: the strict checker verifies the recorded license digest.
    result = run_checker(workspace)

    # Then: it rejects the hash drift instead of trusting the stale cache.
    assert result.returncode != 0
    assert "license hash mismatch" in combined_output(result)


def test_checker_rejects_missing_corresponding_source_when_metadata_is_incomplete(tmp_path: Path) -> None:
    # Given: a source entry that lacks its corresponding-source obligation.
    workspace = create_valid_workspace(tmp_path)
    source_path = workspace / "docs/provenance/third-party-sources.json"
    sources = json.loads(source_path.read_text(encoding="utf-8"))
    del sources["sources"][0]["corresponding_source"]
    write_json(source_path, sources)

    # When: the strict checker validates the incomplete record.
    result = run_checker(workspace)

    # Then: it rejects the source record before it can be used.
    assert result.returncode != 0
    assert "missing corresponding-source metadata" in combined_output(result)


def test_checker_rejects_copied_source_when_marker_appears_under_ed_package(tmp_path: Path) -> None:
    # Given: an otherwise valid workspace with an upstream marker copied below ed_*.
    workspace = create_valid_workspace(tmp_path)
    copied_path = workspace / "ros2_ws/src/ed_uav_lidar/livox_ros_driver2/CMakeLists.txt"
    copied_path.parent.mkdir(parents=True, exist_ok=True)
    copied_path.write_text("project(livox_ros_driver2)\n", encoding="utf-8")

    # When: the strict checker scans the project-owned package tree.
    result = run_checker(workspace)

    # Then: it rejects the vendored third-party source path.
    assert result.returncode != 0
    assert "copied third-party source" in combined_output(result)


def test_checker_rejects_reference_archive_drift_when_local_hash_changes(tmp_path: Path) -> None:
    # Given: a reference archive record whose local locator was changed after hashing.
    workspace = create_valid_workspace(tmp_path)
    reference_path = workspace / "data/reference-locator.txt"
    reference_path.parent.mkdir(parents=True, exist_ok=True)
    reference_path.write_text("known locator\n", encoding="utf-8")
    dataset_path = workspace / "docs/provenance/dataset-manifest.json"
    dataset_manifest = json.loads(dataset_path.read_text(encoding="utf-8"))
    dataset_manifest["reference_archives"].append(
        {
            "id": "reference-locator",
            "local_path": "data/reference-locator.txt",
            "local_sha256": hashlib.sha256(b"known locator\n").hexdigest(),
            "upstream_url": "https://example.invalid/reference.git",
            "reviewed_revision": EXAMPLE_REVISION,
            "license_status": "reference-only",
        }
    )
    write_json(dataset_path, dataset_manifest)
    reference_path.write_text("changed locator\n", encoding="utf-8")

    # When: the strict checker verifies local reference provenance.
    result = run_checker(workspace)

    # Then: it rejects the stale hash instead of accepting altered source metadata.
    assert result.returncode != 0
    assert "reference archive hash mismatch" in combined_output(result)
