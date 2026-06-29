#!/usr/bin/env python3
"""Validate preview-only Workcell Studio web scene edit patches.

This helper only reads web_scene.json and edit_patch.json. It never writes YAML,
layout, cell definition, manifest, generated scene files, or patch outputs.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

PATCH_SCHEMA_VERSION = "workcell_studio_web_scene_edit_patch/v1"
WEB_SCENE_SCHEMA_VERSION = "workcell_studio_web_scene/v1"
GENERATED_SOURCES = {"generated_preview", "locked_generated_urdf_visual", "generated_urdf_visual"}
GENERATED_GROUP_TOKENS = ("robot", "tool", "gripper", "suction", "urdf", "moveit", "generated")


def _load_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001 - CLI should report parse/open failures clearly.
        raise ValueError(f"{path}: unable to load JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return data


def _scene_id(web_scene: dict[str, Any]) -> str:
    scene = web_scene.get("scene")
    return str(scene.get("id", "")) if isinstance(scene, dict) else str(web_scene.get("scene_id", ""))


def _items_by_id(web_scene: dict[str, Any]) -> dict[str, dict[str, Any]]:
    items: dict[str, dict[str, Any]] = {}
    for bucket in ("robots", "tools", "assets", "sensors", "zones", "items", "objects"):
        for item in web_scene.get(bucket, []) if isinstance(web_scene.get(bucket, []), list) else []:
            if isinstance(item, dict) and item.get("id"):
                items[str(item["id"])] = item
    return items


def _identity(item: dict[str, Any]) -> str:
    parts = [item.get(k) for k in ("source_kind", "source_layer", "active_visual_source", "role", "category", "type", "id", "display_name", "label", "name")]
    return " ".join(str(p or "").lower().replace("_", "-") for p in parts)


def _is_generated_robot_or_tool(item: dict[str, Any]) -> bool:
    identity = _identity(item)
    if str(item.get("source_kind", "")).lower() in GENERATED_SOURCES:
        return any(token in identity for token in GENERATED_GROUP_TOKENS)
    if str(item.get("source_layer", "")).lower() in GENERATED_SOURCES:
        return any(token in identity for token in GENERATED_GROUP_TOKENS)
    return False


def _walk_numbers(value: Any, path: str, errors: list[str]) -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            _walk_numbers(child, f"{path}.{key}", errors)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _walk_numbers(child, f"{path}[{index}]", errors)
    elif not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
        errors.append(f"{path}: expected finite number, got {value!r}")


def validate(web_scene: dict[str, Any], patch: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if web_scene.get("schema_version") != WEB_SCENE_SCHEMA_VERSION:
        errors.append(f"web scene schema_version must be {WEB_SCENE_SCHEMA_VERSION}")
    if patch.get("schema_version") != PATCH_SCHEMA_VERSION:
        errors.append(f"patch schema_version must be {PATCH_SCHEMA_VERSION}")
    scene_id = _scene_id(web_scene)
    if patch.get("scene_id") != scene_id:
        errors.append(f"scene_id mismatch: patch={patch.get('scene_id')!r} web_scene={scene_id!r}")
    if patch.get("created_by") != "static_web_viewer":
        errors.append("created_by must be static_web_viewer")
    items = _items_by_id(web_scene)
    edits = patch.get("edits")
    if not isinstance(edits, list):
        errors.append("edits must be an array")
        return errors
    for index, edit in enumerate(edits):
        prefix = f"edits[{index}]"
        if not isinstance(edit, dict):
            errors.append(f"{prefix}: expected object")
            continue
        item_id = edit.get("item_id")
        if not item_id:
            errors.append(f"{prefix}.item_id: required")
            continue
        item = items.get(str(item_id))
        if item is None:
            errors.append(f"{prefix}.item_id={item_id!r}: item not found in web scene")
            continue
        if item.get("locked") is True:
            errors.append(f"{prefix}.item_id={item_id!r}: locked=true items cannot be edited")
        if item.get("editable") is not True:
            errors.append(f"{prefix}.item_id={item_id!r}: editable=false items cannot be edited")
        if _is_generated_robot_or_tool(item):
            errors.append(f"{prefix}.item_id={item_id!r}: generated robot/tool preview items cannot be edited")
        if edit.get("editable_required") is not True:
            errors.append(f"{prefix}.editable_required must be true")
        if edit.get("locked_required") is not False:
            errors.append(f"{prefix}.locked_required must be false")
        if edit.get("operation") != "update_transform":
            errors.append(f"{prefix}.operation must be update_transform")
        for key in ("old_transform", "new_transform"):
            transform = edit.get(key)
            if not isinstance(transform, dict):
                errors.append(f"{prefix}.{key}: required object")
                continue
            _walk_numbers(transform, f"{prefix}.{key}", errors)
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate a preview-only Workcell Studio web scene edit patch.")
    parser.add_argument("--web-scene", required=True, type=Path)
    parser.add_argument("--patch", required=True, type=Path)
    args = parser.parse_args()
    try:
        errors = validate(_load_json(args.web_scene), _load_json(args.patch))
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print("Edit patch is valid for the supplied web_scene.json. No source scene files were modified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
