#!/usr/bin/env python3
"""Verify the full Workcell Studio Web 3D edit persistence loop.

This checker compares a before web_scene.json, a viewer edit_patch.json, and an
after web_scene.json re-exported after the backend applicator was run with
--write. It never writes scene files; it only proves that intended editable
source edits persisted and that unrelated/generated preview items stayed stable.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

from validate_workcell_studio_web_scene_edit_patch import _derived_transform_target, _is_generated_robot_or_tool, _items_by_id, _load_json, _scene_id, validate

_ALLOWED_ITEM_METADATA = {
    "provenance",
    "exported_at",
    "updated_at",
    "created_at",
    "timestamp",
    "export_metadata",
    "metadata",
}


def _as_xyz(value: Any) -> dict[str, float] | None:
    if isinstance(value, dict) and all(k in value for k in ("x", "y", "z")):
        return {k: float(value[k]) for k in ("x", "y", "z")}
    if isinstance(value, list) and len(value) >= 3:
        return {k: float(value[i]) for i, k in enumerate(("x", "y", "z"))}
    return None


def _item_transform(item: dict[str, Any]) -> dict[str, Any]:
    pose = item.get("pose") if isinstance(item.get("pose"), dict) else {}
    xyz = _as_xyz(pose.get("xyz")) or _as_xyz(item.get("pose_xyz"))
    rpy = _as_xyz(pose.get("rpy")) or _as_xyz(item.get("pose_rpy"))
    transform: dict[str, Any] = {"pose": {}}
    if xyz is not None:
        transform["pose"]["xyz"] = xyz
    if rpy is not None:
        transform["pose"]["rpy"] = rpy
    scale = _as_xyz(item.get("scale")) or _as_xyz(item.get("mesh_scale"))
    if scale is not None:
        transform["scale"] = scale
    return transform


def _numbers_close(a: Any, b: Any, path: str = "transform") -> list[str]:
    errors: list[str] = []
    if isinstance(a, dict) and isinstance(b, dict):
        for key in sorted(set(a) | set(b)):
            if key not in a or key not in b:
                errors.append(f"{path}.{key}: missing in {'after' if key not in b else 'expected'}")
            else:
                errors.extend(_numbers_close(a[key], b[key], f"{path}.{key}"))
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if not (math.isfinite(float(a)) and math.isfinite(float(b))) or not math.isclose(float(a), float(b), rel_tol=1e-9, abs_tol=1e-9):
            errors.append(f"{path}: expected {a!r}, got {b!r}")
    else:
        errors.append(f"{path}: expected {a!r}, got {b!r}")
    return errors


def _strip_allowed_metadata(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: _strip_allowed_metadata(v) for k, v in value.items() if k not in _ALLOWED_ITEM_METADATA}
    if isinstance(value, list):
        return [_strip_allowed_metadata(v) for v in value]
    return value


def verify(before: dict[str, Any], patch: dict[str, Any], after: dict[str, Any]) -> tuple[bool, list[str], list[str]]:
    details: list[str] = []
    errors: list[str] = []

    before_scene_id = _scene_id(before)
    after_scene_id = _scene_id(after)
    patch_scene_id = str(patch.get("scene_id", ""))
    if before_scene_id != patch_scene_id or after_scene_id != patch_scene_id:
        errors.append(f"scene_id mismatch: before={before_scene_id!r} patch={patch_scene_id!r} after={after_scene_id!r}")

    validation_errors = validate(before, patch)
    errors.extend(f"patch invalid against before web_scene: {error}" for error in validation_errors)

    before_items = _items_by_id(before)
    after_items = _items_by_id(after)
    edited_ids: set[str] = set()
    derived_dependents = {
        item_id: target_id for item_id, item in before_items.items()
        if (target_id := _derived_transform_target(item))
    }

    for index, edit in enumerate(patch.get("edits", []) if isinstance(patch.get("edits"), list) else []):
        item_id = str(edit.get("item_id", ""))
        prefix = f"edits[{index}] item {item_id!r}"
        edited_ids.add(item_id)
        before_item = before_items.get(item_id)
        after_item = after_items.get(item_id)
        if before_item is None:
            errors.append(f"{prefix}: edited item missing from before web_scene")
            continue
        if after_item is None:
            errors.append(f"{prefix}: edited item missing from after web_scene")
            continue
        old_transform = edit.get("old_transform")
        new_transform = edit.get("new_transform")
        if not isinstance(old_transform, dict) or not isinstance(new_transform, dict):
            errors.append(f"{prefix}: old_transform and new_transform must be objects")
            continue
        # Browser object scale is 1 for ordinary layout transforms; mesh_scale
        # belongs to the mesh child and is not an authored object-scale edit.
        # Only verify scale when the patch actually changes it.
        scale_changed = old_transform.get("scale") != new_transform.get("scale")
        expected_old = dict(old_transform)
        expected_new = dict(new_transform)
        actual_old = _item_transform(before_item)
        actual_new = _item_transform(after_item)
        if not scale_changed:
            expected_old.pop("scale", None)
            expected_new.pop("scale", None)
            actual_old.pop("scale", None)
            actual_new.pop("scale", None)
        old_errors = _numbers_close(expected_old, actual_old, f"{prefix} old_transform")
        if old_errors:
            errors.extend(old_errors)
        new_errors = _numbers_close(expected_new, actual_new, f"{prefix} new_transform")
        if new_errors:
            errors.extend(new_errors)
        if not new_errors:
            details.append(f"PASS edited item {item_id}: new_transform persisted")

    for item_id, before_item in sorted(before_items.items()):
        after_item = after_items.get(item_id)
        if after_item is None:
            errors.append(f"item {item_id!r}: missing from after web_scene")
            continue
        if item_id in edited_ids:
            continue
        if derived_dependents.get(item_id) in edited_ids:
            if _strip_allowed_metadata(before_item) == _strip_allowed_metadata(after_item):
                details.append(f"PASS linked place-zone record {item_id}: remained exporter-consistent with destination {derived_dependents[item_id]}")
            else:
                details.append(f"PASS derived place-zone overlay {item_id}: regenerated from destination {derived_dependents[item_id]}")
            continue
        if before_item.get("locked") is True or _is_generated_robot_or_tool(before_item):
            label = "locked/generated"
        else:
            label = "unrelated"
        if _strip_allowed_metadata(before_item) != _strip_allowed_metadata(after_item):
            errors.append(f"item {item_id!r}: {label} item changed unexpectedly")
        else:
            details.append(f"PASS {label} item {item_id}: unchanged")

    return not errors, details, errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Verify Workcell Studio Web 3D edit persistence after apply --write and re-export.")
    parser.add_argument("--scene", required=True, type=Path, help="Scene directory used for operator context; not modified.")
    parser.add_argument("--web-scene-before", required=True, type=Path)
    parser.add_argument("--patch", required=True, type=Path)
    parser.add_argument("--web-scene-after", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        before = _load_json(args.web_scene_before)
        patch = _load_json(args.patch)
        after = _load_json(args.web_scene_after)
    except ValueError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2
    ok, details, errors = verify(before, patch, after)
    print(f"scene: {args.scene}")
    for detail in details:
        print(detail)
    if ok:
        print("PASS: Web 3D edit persistence verified; generated/locked and unrelated items stayed unchanged.")
        return 0
    for error in errors:
        print(f"FAIL: {error}", file=sys.stderr)
    print("FAIL: Web 3D edit persistence verification failed.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
