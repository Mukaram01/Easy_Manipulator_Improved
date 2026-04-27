#!/usr/bin/env python3
"""Inspect exported workcell commissioning bundles (folder or zip)."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tempfile
from pathlib import Path
from zipfile import ZipFile

REQUIRED_FILES = (
    "README.md",
    "bundle_manifest.json",
    "operator_checklist.md",
    "scene_manifest.yaml",
    "execution_plan.md",
    "execution_plan.json",
    "validation_summary.md",
)

OPTIONAL_FILES = (
    "reports",
    "task_recipe_dry_run_summary.md",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            chunk = handle.read(65536)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _load_manifest(bundle_root: Path) -> dict:
    manifest_path = bundle_root / "bundle_manifest.json"
    with manifest_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def inspect_bundle(bundle_path: Path) -> tuple[str, list[str]]:
    notes: list[str] = []
    status = "PASS"

    if not bundle_path.exists():
        return "FAIL", [f"Bundle path does not exist: {bundle_path}"]

    extracted_root: Path | None = None
    temp_dir: tempfile.TemporaryDirectory[str] | None = None

    try:
        if bundle_path.is_file() and bundle_path.suffix.lower() == ".zip":
            temp_dir = tempfile.TemporaryDirectory()
            extracted_root = Path(temp_dir.name)
            with ZipFile(bundle_path, "r") as archive:
                archive.extractall(extracted_root)
            root = extracted_root
        else:
            root = bundle_path

        manifest_path = root / "bundle_manifest.json"
        if not manifest_path.is_file():
            return "FAIL", ["Missing required file: bundle_manifest.json"]

        manifest = _load_manifest(root)

        for name in REQUIRED_FILES:
            if not (root / name).exists():
                notes.append(f"Missing required file: {name}")
                status = "FAIL"

        files_block = manifest.get("files")
        if not isinstance(files_block, list):
            notes.append("bundle_manifest.json missing valid files[] checksum block")
            status = "FAIL"
            files_block = []

        for entry in files_block:
            if not isinstance(entry, dict):
                notes.append("Invalid file checksum entry in manifest")
                status = "FAIL"
                continue
            rel_path = entry.get("path")
            expected = entry.get("sha256")
            if not isinstance(rel_path, str) or not isinstance(expected, str):
                notes.append("Invalid path/sha256 in manifest file entry")
                status = "FAIL"
                continue

            candidate = root / rel_path
            if not candidate.is_file():
                notes.append(f"Checksum file missing: {rel_path}")
                status = "FAIL"
                continue

            actual = _sha256(candidate)
            if actual != expected:
                notes.append(f"Checksum mismatch: {rel_path}")
                status = "FAIL"

        for optional in OPTIONAL_FILES:
            if not (root / optional).exists():
                notes.append(f"Optional file missing: {optional}")

        return status, notes
    finally:
        if extracted_root and extracted_root.exists():
            shutil.rmtree(extracted_root, ignore_errors=True)
        if temp_dir is not None:
            temp_dir.cleanup()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", help="Bundle directory or .zip path")
    args = parser.parse_args()

    status, notes = inspect_bundle(Path(args.bundle).resolve())
    print(f"{status} bundle={args.bundle}")
    for note in notes:
        print(f"- {note}")

    return 1 if status == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
