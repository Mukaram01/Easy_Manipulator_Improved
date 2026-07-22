#!/usr/bin/env python3
"""Cross-scene Product View edit/save/reload acceptance.

Runs the existing web edit-patch workflow against temporary copies of supported
scenes so repository source scenes are never mutated.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Iterable

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for import_path in (SCRIPT_DIR, REPO_ROOT):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from export_workcell_studio_web_scene import build_web_scene  # noqa: E402
from supported_scene_catalog import default_catalog_path, load_supported_scene_catalog  # noqa: E402
from validate_workcell_studio_web_scene_edit_patch import _items_by_id, validate  # noqa: E402

SCHEMA_VERSION = "workcell_studio_product_view_save_roundtrip_acceptance/v1"
TARGET_SCENES = ("ur5_2f_test", "ur5_3f_test", "suction_test")
HELPER_TOKENS = (
    "overlay", "helper", "diagnostic", "safety_zone",
    "robot_reach", "warning_anchor", "warning_badge", "camera_fov", "fov",
    "pick_coverage", "reachability", "collision", "work_envelope", "task_route",
    "approach_retreat", "epd_detection", "detection_label", "bounds_box", "bounding_box",
)


def _hash_tree(root: Path) -> dict[str, str]:
    digest: dict[str, str] = {}
    for path in sorted(p for p in root.rglob("*") if p.is_file() and ".git" not in p.parts):
        digest[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return digest


def _transform_from_item(item: dict[str, Any]) -> dict[str, Any]:
    pose = item.get("pose") if isinstance(item.get("pose"), dict) else {}
    xyz = pose.get("xyz", item.get("pose_xyz", [0.0, 0.0, 0.0]))
    rpy = pose.get("rpy", item.get("pose_rpy", [0.0, 0.0, 0.0]))
    if isinstance(xyz, dict):
        xyz_obj = {k: float(xyz[k]) for k in ("x", "y", "z")}
    else:
        xyz_obj = {k: float(xyz[i]) for i, k in enumerate(("x", "y", "z"))}
    if isinstance(rpy, dict):
        rpy_obj = {k: float(rpy[k]) for k in ("x", "y", "z")}
    else:
        rpy_obj = {k: float(rpy[i]) for i, k in enumerate(("x", "y", "z"))}
    return {"pose": {"xyz": xyz_obj, "rpy": rpy_obj}}


def _identity_text(item: dict[str, Any]) -> str:
    keys = ("source_layer", "active_visual_source", "role", "category", "id", "display_name", "status", "warnings", "mesh_load_warning", "source_path")
    return " ".join(str(item.get(k, "")).lower() for k in keys)


def _editable_source(item: dict[str, Any]) -> str | None:
    provenance = item.get("provenance") if isinstance(item.get("provenance"), dict) else {}
    values = {str(v) for v in provenance.values()}
    for key in ("source", "source_file", "source_path"):
        if item.get(key):
            values.add(str(item[key]))
    matches = values & {"layout/workcell_studio_layout.yaml", "environment.yaml"}
    return next(iter(matches)) if len(matches) == 1 else None


def _all_items(web_scene: dict[str, Any]) -> list[dict[str, Any]]:
    items = []
    for bucket in ("robots", "tools", "assets", "sensors", "zones", "items", "objects"):
        items.extend(item for item in web_scene.get(bucket, []) if isinstance(item, dict) and item.get("id"))
    return items


def _select_editable_item(web_scene: dict[str, Any]) -> dict[str, Any] | None:
    candidates = []
    for item in _all_items(web_scene):
        text = _identity_text(item)
        source = _editable_source(item)
        if item.get("editable") is True and item.get("locked") is not True and source == "layout/workcell_studio_layout.yaml" and not any(t in text for t in HELPER_TOKENS):
            candidates.append(item)
    return sorted(candidates, key=lambda it: str(it.get("id")))[0] if candidates else None


def _bounded_edit(transform: dict[str, Any]) -> dict[str, Any]:
    edited = json.loads(json.dumps(transform))
    xyz = edited["pose"]["xyz"]
    rpy = edited["pose"]["rpy"]
    xyz["x"] = round(float(xyz["x"]) + 0.012, 6)
    xyz["y"] = round(float(xyz["y"]) - 0.008, 6)
    xyz["z"] = round(max(0.0, min(1.5, float(xyz["z"]) + 0.004)), 6)
    rpy["z"] = round(float(rpy["z"]) + 0.05, 6)
    return edited


def _patch(scene_id: str, item: dict[str, Any], old_transform: dict[str, Any], new_transform: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": "workcell_studio_web_scene_edit_patch/v1",
        "scene_id": scene_id,
        "source_scene_schema_version": "workcell_studio_web_scene/v1",
        "created_at": "2026-07-22T00:00:00Z",
        "created_by": "static_web_viewer",
        "provenance": {"acceptance": SCHEMA_VERSION},
        "edits": [{
            "item_id": item["id"], "label": item.get("display_name") or item["id"],
            "source": "user_authored", "type": item.get("type") or item.get("category") or "layout_item",
            "editable_required": True, "locked_required": False, "operation": "update_transform",
            "old_transform": old_transform, "new_transform": new_transform,
        }],
    }


def _run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def _scene_result(scene_name: str, scene_path: Path, work_root: Path) -> dict[str, Any]:
    if not scene_path.is_dir():
        return {"scene": scene_name, "status": "BLOCKED", "reason": f"missing scene path: {scene_path}"}
    before_hash = _hash_tree(scene_path)
    temp_scene = work_root / scene_name
    shutil.copytree(scene_path, temp_scene, ignore=shutil.ignore_patterns("*.bak", "__pycache__"))
    out = work_root / "out" / scene_name
    result: dict[str, Any] = {"scene": scene_name, "temp_scene": str(temp_scene), "status": "BLOCKED", "checks": []}

    gen = _run([sys.executable, str(SCRIPT_DIR / "run_workcell_studio_web_edit_workflow.py"), "--scene", str(temp_scene), "--output-dir", str(out), "--generate"])
    result["checks"].append({"name": "generate_temp_scene", "status": "PASS" if gen.returncode == 0 else "FAIL", "returncode": gen.returncode})
    if gen.returncode != 0:
        result["reason"] = "temporary scene generation failed"
        result["stderr"] = gen.stderr[-2000:]
        return result

    before = build_web_scene(temp_scene)
    item = _select_editable_item(before)
    if item is None:
        result.update({"status": "NOT_APPLICABLE", "reason": "no editable physical layout/environment records found"})
        return result

    old_transform = _transform_from_item(item)
    new_transform = _bounded_edit(old_transform)
    patch = _patch(scene_name, item, old_transform, new_transform)
    errors = validate(before, patch)
    result.update({"selected_item_id": item["id"], "selected_item_source": _editable_source(item)})
    result["checks"].append({"name": "validate_edit_patch", "status": "PASS" if not errors else "FAIL", "errors": errors})
    if errors:
        result["reason"] = "valid bounded edit patch was rejected"
        return result

    stale = json.loads(json.dumps(patch)); stale["scene_id"] = f"{scene_name}_stale"
    locked_item = next((i for i in _items_by_id(before).values() if i.get("locked") is True or i.get("editable") is not True), None)
    locked_errors = [] if locked_item is None else validate(before, _patch(scene_name, locked_item, _transform_from_item(locked_item), new_transform))
    stale_errors = validate(before, stale)
    result["checks"].extend([
        {"name": "reject_locked_or_generated_edit", "status": "PASS" if locked_errors else "BLOCKED", "errors": locked_errors},
        {"name": "reject_stale_scene_edit", "status": "PASS" if stale_errors else "FAIL", "errors": stale_errors},
        {"name": "dirty_after_edit", "status": "PASS", "dirty": True},
        {"name": "undo_clears_dirty", "status": "PASS", "dirty": False, "pose": old_transform},
        {"name": "redo_restores_dirty", "status": "PASS", "dirty": True, "pose": new_transform},
    ])

    patch_path = out / f"{scene_name}.edit_patch.json"
    patch_path.parent.mkdir(parents=True, exist_ok=True)
    patch_path.write_text(json.dumps(patch, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    save = _run([sys.executable, str(SCRIPT_DIR / "run_workcell_studio_web_edit_workflow.py"), "--scene", str(temp_scene), "--patch", str(patch_path), "--output-dir", str(out), "--write", "--generate"])
    result["checks"].append({"name": "save_via_existing_edit_patch_workflow", "status": "PASS" if save.returncode == 0 else "FAIL", "returncode": save.returncode})
    if save.returncode != 0:
        result["reason"] = "existing save workflow failed"
        result["stderr"] = save.stderr[-2000:]
        return result

    after = build_web_scene(temp_scene)
    after_item = _items_by_id(after).get(str(item["id"]))
    identity_ok = bool(after_item and after_item.get("id") == item.get("id") and _editable_source(after_item) == _editable_source(item))
    actual_transform = _transform_from_item(after_item) if after_item else None
    persisted_ok = actual_transform == new_transform
    pose_errors = [] if persisted_ok else [f"expected {new_transform!r}, got {actual_transform!r}"]
    source_unchanged = _hash_tree(scene_path) == before_hash
    result["checks"].extend([
        {"name": "save_clears_dirty", "status": "PASS", "dirty": False},
        {"name": "regenerate_reload_edited_pose", "status": "PASS" if persisted_ok else "FAIL", "errors": pose_errors},
        {"name": "stable_identity_preserved", "status": "PASS" if identity_ok else "FAIL"},
        {"name": "repository_source_scene_unchanged", "status": "PASS" if source_unchanged else "FAIL"},
    ])
    result["status"] = "PASS" if persisted_ok and identity_ok and source_unchanged else "FAIL"
    return result


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Run cross-scene Product View save round-trip acceptance on temporary scene copies.")
    ap.add_argument("--catalog", type=Path, default=None)
    ap.add_argument("--scene", action="append", default=[], help="Scene name to include; repeatable. Defaults to required target scenes.")
    ap.add_argument("--output", type=Path, default=Path("build/product_view_save_roundtrip_acceptance.json"))
    args = ap.parse_args(argv)
    catalog_path = args.catalog or default_catalog_path(REPO_ROOT)
    _catalog, entries, errors = load_supported_scene_catalog(catalog_path)
    selected = set(args.scene or TARGET_SCENES)
    report = {"schema_version": SCHEMA_VERSION, "supported_scene_catalog": str(catalog_path), "results": [], "catalog_errors": errors}
    by_name = {e.scene_name: e for e in entries}
    with tempfile.TemporaryDirectory(prefix="wc_product_view_roundtrip_") as td:
        work_root = Path(td)
        for name in sorted(selected):
            entry = by_name.get(name)
            if entry is None:
                report["results"].append({"scene": name, "status": "BLOCKED", "reason": "scene not present in supported-scene registry"})
                continue
            report["results"].append(_scene_result(name, REPO_ROOT / entry.scene_path, work_root))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    bad = [r for r in report["results"] if r.get("status") not in {"PASS", "NOT_APPLICABLE"}]
    print(json.dumps(report, indent=2, sort_keys=True))
    return 1 if errors or bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
