#!/usr/bin/env python3
"""Safely apply validated Workcell Studio Web 3D edit patches.

Dry-run is the default. Source YAML is mutated only with --write, and only for
clear editable layout/environment provenance. Generated files, manifests, and
cell definitions are intentionally out of scope for this first persistence step.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import shutil
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore

from validate_workcell_studio_web_scene_edit_patch import _items_by_id, _load_json, _scene_id, validate

ALLOWED_SOURCES = {
    "layout/workcell_studio_layout.yaml",
    "environment.yaml",
}
FORBIDDEN_TARGET_PARTS = {"generated"}
FORBIDDEN_TARGET_NAMES = {"cell_definition.yaml", "scene_manifest.yaml"}
ENVIRONMENT_LIST_KEYS = ("support_surfaces", "assets", "sensors", "zones")


@dataclass
class PlannedUpdate:
    item_id: str
    label: str
    source: str
    target_file: Path
    target_rel: str
    item: dict[str, Any]
    record: dict[str, Any]
    old_transform: dict[str, Any]
    new_transform: dict[str, Any]
    update_scale: bool


def _load_yaml(path: Path) -> Any:
    if yaml is None:
        raise ValueError("PyYAML is required to apply edit patches")
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception as exc:  # noqa: BLE001
        raise ValueError(f"{path}: unable to load YAML: {exc}") from exc


def _write_yaml(path: Path, data: Any) -> None:
    """Atomically replace one approved YAML file in its original directory."""
    assert yaml is not None
    payload = yaml.safe_dump(data, sort_keys=False, default_flow_style=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            os.chmod(temporary, path.stat().st_mode)
        os.replace(temporary, path)
    except Exception:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


def _as_list3(value: Any, field: str) -> list[float]:
    if not isinstance(value, dict):
        raise ValueError(f"new_transform.{field}: expected object with x/y/z")
    return [float(value[k]) for k in ("x", "y", "z")]


def _pose_lists(transform: dict[str, Any]) -> tuple[list[float], list[float]]:
    pose = transform.get("pose")
    if not isinstance(pose, dict):
        raise ValueError("new_transform.pose: expected object")
    return _as_list3(pose.get("xyz"), "pose.xyz"), _as_list3(pose.get("rpy"), "pose.rpy")


def _scale_list(transform: dict[str, Any]) -> list[float] | None:
    scale = transform.get("scale")
    if scale is None:
        return None
    return _as_list3(scale, "scale")


def _source_from_item(item: dict[str, Any]) -> str | None:
    candidates: set[str] = set()
    for key in ("source", "source_file", "source_path"):
        value = item.get(key)
        if isinstance(value, str) and value in ALLOWED_SOURCES:
            candidates.add(value)
    prov = item.get("provenance")
    if isinstance(prov, dict):
        for value in prov.values():
            if isinstance(value, str) and value in ALLOWED_SOURCES:
                candidates.add(value)
    if len(candidates) == 1:
        return next(iter(candidates))
    return None


def _safe_target(scene_dir: Path, rel: str) -> Path:
    if rel not in ALLOWED_SOURCES:
        raise ValueError(f"ambiguous or unsupported source mapping: {rel!r}")
    path = (scene_dir / rel).resolve()
    scene_root = scene_dir.resolve()
    try:
        relative_parts = path.relative_to(scene_root).parts
    except ValueError as exc:
        raise ValueError(f"target file escapes scene directory: {path}") from exc
    if any(part in FORBIDDEN_TARGET_PARTS for part in relative_parts) or path.name in FORBIDDEN_TARGET_NAMES:
        raise ValueError(f"refusing to update forbidden source file: {path}")
    return path


def _find_layout_record(data: Any, item_id: str) -> dict[str, Any] | None:
    if not isinstance(data, dict):
        return None
    for record in data.get("items", []) if isinstance(data.get("items"), list) else []:
        if isinstance(record, dict) and str(record.get("id")) == item_id:
            return record
    return None


def _find_environment_record(data: Any, item_id: str) -> dict[str, Any] | None:
    root = data.get("environment") if isinstance(data, dict) else None
    if not isinstance(root, dict):
        return None
    matches: list[dict[str, Any]] = []
    for key in ENVIRONMENT_LIST_KEYS:
        for record in root.get(key, []) if isinstance(root.get(key), list) else []:
            if isinstance(record, dict) and (str(record.get("id")) == item_id or str(record.get("layout_item_ref")) == item_id):
                matches.append(record)
    return matches[0] if len(matches) == 1 else None


def _has_obvious_scale(record: dict[str, Any]) -> bool:
    return isinstance(record.get("scale"), (list, dict)) or isinstance(record.get("mesh_scale"), (list, dict))


def _plan(scene_dir: Path, web_scene: dict[str, Any], patch: dict[str, Any]) -> list[PlannedUpdate]:
    errors = validate(web_scene, patch)
    if errors:
        raise ValueError("patch validation failed:\n" + "\n".join(f"- {e}" for e in errors))
    items = _items_by_id(web_scene)
    loaded_yaml: dict[str, Any] = {}
    planned: list[PlannedUpdate] = []
    for edit in patch.get("edits", []):
        item_id = str(edit["item_id"])
        item = items[item_id]
        rel = _source_from_item(item)
        if rel is None:
            raise ValueError(f"{item_id}: ambiguous source mapping; item provenance must clearly identify layout/workcell_studio_layout.yaml or environment.yaml")
        target = _safe_target(scene_dir, rel)
        if rel not in loaded_yaml:
            loaded_yaml[rel] = _load_yaml(target)
        record = _find_layout_record(loaded_yaml[rel], item_id) if rel == "layout/workcell_studio_layout.yaml" else _find_environment_record(loaded_yaml[rel], item_id)
        if record is None:
            raise ValueError(f"{item_id}: source item not found exactly once in {rel}")
        new_scale = _scale_list(edit["new_transform"])
        old_scale = _scale_list(edit["old_transform"])
        scale_changed = new_scale is not None and old_scale is not None and new_scale != old_scale
        if scale_changed and not _has_obvious_scale(record):
            raise ValueError(f"{item_id}: scale changed but {rel} has no obvious scale field; refusing to guess")
        planned.append(PlannedUpdate(item_id, str(edit.get("label") or item.get("label") or item.get("display_name") or item_id), rel, target, rel, item, record, edit["old_transform"], edit["new_transform"], scale_changed))
    return planned


def _format_transform(transform: dict[str, Any]) -> str:
    pose = transform.get("pose", {}) if isinstance(transform.get("pose"), dict) else {}
    return f"xyz={pose.get('xyz')} rpy={pose.get('rpy')} scale={transform.get('scale')}"


def _apply_update(update: PlannedUpdate) -> None:
    xyz, rpy = _pose_lists(update.new_transform)
    if update.target_rel == "layout/workcell_studio_layout.yaml":
        pose = update.record.setdefault("pose", {})
        if not isinstance(pose, dict):
            raise ValueError(f"{update.item_id}: layout pose is not an object")
        pose["xyz"] = xyz
        pose["rpy"] = rpy
    else:
        update.record["pose_xyz"] = xyz
        update.record["pose_rpy"] = rpy
    if update.update_scale:
        scale = _scale_list(update.new_transform)
        if "scale" in update.record:
            update.record["scale"] = scale
        elif "mesh_scale" in update.record:
            update.record["mesh_scale"] = scale


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Dry-run or apply a validated Workcell Studio web_scene_edit_patch/v1 patch.")
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--web-scene", required=True, type=Path)
    parser.add_argument("--patch", required=True, type=Path)
    parser.add_argument("--write", action="store_true", help="Mutate editable source YAML. Omitted means dry-run only.")
    parser.add_argument("--backup", action="store_true", help="In --write mode, create timestamped .bak files next to edited YAML.")
    args = parser.parse_args(argv)
    try:
        scene_dir = args.scene.resolve()
        web_scene = _load_json(args.web_scene)
        patch = _load_json(args.patch)
        planned = _plan(scene_dir, web_scene, patch)
        print(f"scene id: {_scene_id(web_scene)}")
        if not planned:
            print("No edits to apply.")
            return 0
        for update in planned:
            print(f"item id: {update.item_id}")
            print(f"  label: {update.label}")
            print(f"  source: {update.source}")
            print(f"  target file: {update.target_file}")
            print(f"  old transform: {_format_transform(update.old_transform)}")
            print(f"  new transform: {_format_transform(update.new_transform)}")
            print(f"  write would occur: {bool(args.write)}")
        if not args.write:
            print("Dry-run only; no files were modified. Pass --write to apply these safe editable updates.")
            return 0
        by_file: dict[Path, list[PlannedUpdate]] = {}
        for update in planned:
            by_file.setdefault(update.target_file, []).append(update)
        for path, updates in by_file.items():
            data = _load_yaml(path)
            for update in updates:
                update.record = _find_layout_record(data, update.item_id) if update.target_rel == "layout/workcell_studio_layout.yaml" else _find_environment_record(data, update.item_id)  # type: ignore[assignment]
                if update.record is None:
                    raise ValueError(f"{update.item_id}: source item disappeared from {update.target_rel}")
                _apply_update(update)
            if args.backup:
                stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
                shutil.copy2(path, path.with_name(f"{path.name}.{stamp}.bak"))
            _write_yaml(path, data)
            print(f"updated file path: {path}")
            print(f"updated item count: {len(updates)}")
        print("skipped/rejected edits: none")
        print(f"next suggested command: python3 scripts/export_workcell_studio_web_scene.py --scene {args.scene} --output {args.web_scene}")
        return 0
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
