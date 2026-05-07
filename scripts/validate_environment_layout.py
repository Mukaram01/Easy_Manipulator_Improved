#!/usr/bin/env python3
"""Validate environment_layout/v1 YAML/JSON files."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import validate_cell_definition as yaml_support


@dataclass
class ValidationResult:
    path: Path
    parser: str
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    summary: dict[str, Any] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.errors


def _is_num3(value: Any) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(isinstance(v, (int, float)) for v in value)


def _load_manifests(repo_root: Path) -> dict[str, dict[str, Any]]:
    manifests: dict[str, dict[str, Any]] = {}
    for candidate in list(repo_root.rglob("asset_manifest.yaml")) + list(repo_root.rglob("assets_manifest.yaml")):
        try:
            loaded, _, _ = yaml_support.load_yaml(candidate)
        except Exception:
            continue
        if isinstance(loaded, dict):
            if "asset_id" in loaded:
                manifests[str(loaded.get("asset_id"))] = loaded
            if isinstance(loaded.get("assets"), list):
                for item in loaded["assets"]:
                    if isinstance(item, dict) and item.get("asset_id"):
                        manifests[str(item["asset_id"])] = item
    return manifests


def load_layout(path: Path) -> tuple[dict[str, Any], str, list[str]]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        data = json.loads(text)
        if not isinstance(data, dict):
            raise ValueError("Top-level layout must be an object")
        return data, "json", []
    loaded, parser, notes = yaml_support.load_yaml(path)
    return loaded, parser, notes


def validate_layout(defn: dict[str, Any], path: Path, parser: str, notes: list[str], strict: bool = False, repo_root: Path = REPO_ROOT) -> ValidationResult:
    result = ValidationResult(path=path, parser=parser, notes=list(notes))
    manifests = _load_manifests(repo_root)

    if defn.get("schema_version") != "environment_layout/v1":
        result.errors.append("schema_version must be exactly 'environment_layout/v1'.")

    assets = defn.get("assets") if isinstance(defn.get("assets"), list) else []
    zones = defn.get("zones") if isinstance(defn.get("zones"), list) else []
    safety = defn.get("safety") if isinstance(defn.get("safety"), dict) else {}
    safety_zones = safety.get("zones") if isinstance(safety.get("zones"), list) else []

    seen_asset_ids: set[str] = set()

    has_robot_base = False
    has_table = False
    has_pick_area = False
    has_camera = False
    perception_enabled = False
    conveyor_present = False
    for idx, asset in enumerate(assets):
        if not isinstance(asset, dict):
            result.errors.append(f"assets[{idx}] must be an object.")
            continue
        aid = asset.get("id")
        if not isinstance(aid, str) or not aid.strip():
            result.errors.append(f"assets[{idx}].id must be a non-empty string.")
        elif aid in seen_asset_ids:
            result.errors.append(f"Duplicate asset id '{aid}'.")
        else:
            seen_asset_ids.add(aid)

        pose = asset.get("pose") if isinstance(asset.get("pose"), dict) else None
        if pose is None:
            result.errors.append(f"assets[{idx}].pose is required.")
        else:
            if not _is_num3(pose.get("xyz")):
                result.errors.append(f"assets[{idx}].pose.xyz must be numeric list length 3.")
            if not _is_num3(pose.get("rpy")):
                result.errors.append(f"assets[{idx}].pose.rpy must be numeric list length 3.")

        src = asset.get("source") if isinstance(asset.get("source"), dict) else {}
        src_path = src.get("path")
        src_uri = src.get("uri")
        if isinstance(src_path, str) and src_path.strip():
            abs_path = (repo_root / src_path).resolve()
            if not abs_path.exists():
                message = f"assets[{idx}].source.path '{src_path}' does not exist relative to repo root."
                (result.errors if strict else result.warnings).append(message)
        if isinstance(src_uri, str) and src_uri.startswith("package://"):
            pass

        atype = str(asset.get("type", ""))
        has_robot_base = has_robot_base or atype == "robot_base"
        has_table = has_table or atype in {"table", "support_surface"}
        has_pick_area = has_pick_area or atype == "object_spawn_area"
        has_camera = has_camera or atype == "camera_mount"
        conveyor_present = conveyor_present or atype == "conveyor_placeholder"
        cam_meta = asset.get("camera") if isinstance(asset.get("camera"), dict) else {}
        perception_enabled = perception_enabled or bool(cam_meta.get("perception_enabled"))

        ref = asset.get("asset_ref")
        if isinstance(ref, str) and ref.strip() and ref not in manifests:
            message = f"assets[{idx}].asset_ref '{ref}' not found in optional asset manifests."
            (result.errors if strict else result.warnings).append(message)

    def _validate_zone(zone: Any, owner: str, seen: set[str]) -> None:
        if not isinstance(zone, dict):
            result.errors.append(f"{owner} must be an object.")
            return
        zid = zone.get("id")
        if not isinstance(zid, str) or not zid.strip():
            result.errors.append(f"{owner}.id must be a non-empty string.")
        elif zid in seen:
            result.errors.append(f"Duplicate zone id '{zid}'.")
        else:
            seen.add(zid)
        bounds = zone.get("bounds_xyz") if isinstance(zone.get("bounds_xyz"), dict) else {}
        if not _is_num3(bounds.get("min")):
            result.errors.append(f"{owner}.bounds_xyz.min must be numeric list length 3.")
        if not _is_num3(bounds.get("max")):
            result.errors.append(f"{owner}.bounds_xyz.max must be numeric list length 3.")

    seen_zone_ids: set[str] = set()
    for idx, zone in enumerate(zones):
        _validate_zone(zone, f"zones[{idx}]", seen_zone_ids)
    for idx, zone in enumerate(safety_zones):
        _validate_zone(zone, f"safety.zones[{idx}]", seen_zone_ids)

    zone_types = {str(z.get("type", "")) for z in zones if isinstance(z, dict)}
    has_pick_area = has_pick_area or ("pick_zone" in zone_types)

    if not has_robot_base:
        result.warnings.append("Missing robot base asset in environment layout.")
    if not has_table:
        result.warnings.append("Missing table/support surface asset in environment layout.")
    if not has_pick_area:
        result.warnings.append("Missing pick area (object_spawn_area or pick_zone).")
    if perception_enabled and not has_camera:
        result.warnings.append("Perception enabled but camera pose metadata missing.")
    if conveyor_present:
        result.warnings.append("Conveyor selected is placeholder/visual metadata only; physics backend unavailable.")

    result.summary = {
        "layout_id": defn.get("layout_id"),
        "asset_count": len(assets),
        "zone_count": len(zones),
        "safety_zone_count": len(safety_zones),
        "manifest_asset_count": len(manifests),
    }
    return result


def _to_payload(summary: ValidationResult) -> dict[str, Any]:
    result = "PASS" if summary.ok and not summary.warnings else "WARN" if summary.ok else "FAIL"
    return {
        "path": str(summary.path),
        "parser": summary.parser,
        "result": result,
        "errors": summary.errors,
        "warnings": summary.warnings,
        "notes": summary.notes,
        "summary": summary.summary,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("layout_path", type=Path)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    try:
        loaded, parser_name, notes = load_layout(args.layout_path)
    except FileNotFoundError:
        print(f"FAIL: File not found: {args.layout_path}")
        return 2
    except Exception as exc:
        print(f"FAIL: Could not parse layout: {exc}")
        return 1

    summary = validate_layout(loaded, args.layout_path, parser_name, notes, strict=args.strict)
    payload = _to_payload(summary)

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    elif not args.quiet:
        print(f"Layout: {summary.path}")
        for note in summary.notes:
            print(f"NOTE: {note}")
        for warning in summary.warnings:
            print(f"WARN: {warning}")
        for error in summary.errors:
            print(f"FAIL: {error}")
        print(f"RESULT: {payload['result']}")

    return 0 if summary.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
