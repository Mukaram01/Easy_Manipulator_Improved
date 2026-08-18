#!/usr/bin/env python3
"""Validate portable, hash-backed imported assets for one Workcell project."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
from typing import Any, Mapping

import yaml


REQUIRED_PROVENANCE = (
    "catalog_id", "original_filename", "source_sha256", "source_units",
    "unit_scale_to_m", "staged_relative_path", "import_contract_version", "imported_at_utc",
)


def _load_yaml(path: Path) -> Any:
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read {path}: {exc}") from exc


def _safe_relative(value: Any) -> bool:
    if not isinstance(value, str) or not value.strip() or "\\" in value:
        return False
    path = PurePosixPath(value)
    return not path.is_absolute() and ".." not in path.parts and "." not in path.parts


def validate_scene(scene: Path) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    manifest_path = scene / "assets" / "imported" / "asset_manifest.yaml"
    layout_path = scene / "layout" / "workcell_studio_layout.yaml"
    manifest = _load_yaml(manifest_path)
    layout = _load_yaml(layout_path)
    if not isinstance(manifest, Mapping):
        raise ValueError("asset manifest root must be a mapping")
    assets = manifest.get("assets")
    if not isinstance(assets, list):
        raise ValueError("asset manifest assets must be a list")
    seen: set[str] = set()
    verified: dict[str, dict[str, Any]] = {}
    for index, asset in enumerate(assets):
        if not isinstance(asset, Mapping):
            errors.append(f"assets[{index}] must be a mapping")
            continue
        asset_id = asset.get("id")
        if not isinstance(asset_id, str) or not asset_id:
            errors.append(f"assets[{index}].id is missing")
            continue
        if asset_id in seen:
            errors.append(f"duplicate catalog asset id: {asset_id}")
            continue
        seen.add(asset_id)
        missing = [field for field in REQUIRED_PROVENANCE if asset.get(field) in (None, "")]
        if missing:
            errors.append(f"{asset_id}: missing provenance fields: {', '.join(missing)}")
        if asset.get("catalog_id") != asset_id:
            errors.append(f"{asset_id}: catalog_id must equal id")
        relative = asset.get("staged_relative_path")
        if not _safe_relative(relative):
            errors.append(f"{asset_id}: staged_relative_path must be safe and relative")
            continue
        staged = manifest_path.parent / str(relative)
        if not staged.is_file():
            errors.append(f"{asset_id}: staged asset is missing: {relative}")
            continue
        actual_hash = hashlib.sha256(staged.read_bytes()).hexdigest()
        if actual_hash != asset.get("source_sha256"):
            errors.append(f"{asset_id}: SHA-256 mismatch for {relative}")
        if asset.get("path") != relative:
            errors.append(f"{asset_id}: path and staged_relative_path must match")
        verified[asset_id] = {"path": relative, "sha256": actual_hash, "bytes": staged.stat().st_size}

    instances = []
    if isinstance(layout, Mapping) and isinstance(layout.get("items"), list):
        instances = [item for item in layout["items"] if isinstance(item, Mapping) and item.get("catalog_asset_id")]
        if Path(str(layout.get("scene_path") or ".")).is_absolute():
            errors.append("layout.scene_path must be project-relative")
        for item in layout["items"]:
            if isinstance(item, Mapping) and Path(str(item.get("scene_path") or ".")).is_absolute():
                errors.append(f"{item.get('id', '<unknown>')}: scene_path must be project-relative")
    for item in instances:
        item_id = str(item.get("id") or "<unknown>")
        catalog_id = item.get("catalog_asset_id")
        if catalog_id not in seen:
            errors.append(f"{item_id}: unknown catalog_asset_id {catalog_id!r}")
        references = [item.get("source_path"), item.get("mesh_path")]
        mesh = item.get("mesh") if isinstance(item.get("mesh"), Mapping) else {}
        references.append(mesh.get("path"))
        for reference in [value for value in references if value not in (None, "")]:
            if not _safe_relative(reference) or not str(reference).startswith("assets/imported/"):
                errors.append(f"{item_id}: imported mesh reference must be scene-relative: {reference!r}")
        if str(item.get("role") or "") == "asset":
            warnings.append(f"{item_id}: semantic role remains generic asset until intentionally assigned")

    return {
        "schema": "workcell_asset_provenance_report/v1",
        "scene": scene.name,
        "status": "FAIL" if errors else ("WARN" if warnings else "PASS"),
        "summary": {
            "catalog_asset_count": len(assets),
            "verified_hash_count": len(verified),
            "placed_imported_instance_count": len(instances),
            "portable_reference_count": sum(1 for item in instances if str((item.get("mesh") or {}).get("path", "")).startswith("assets/imported/")),
        },
        "verified_assets": verified,
        "errors": errors,
        "warnings": warnings,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        report = validate_scene(args.scene.resolve())
    except ValueError as exc:
        report = {"schema": "workcell_asset_provenance_report/v1", "status": "FAIL", "errors": [str(exc)], "warnings": []}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True) if args.json else f"{report['status']}: {args.scene}")
    return 1 if report["status"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
