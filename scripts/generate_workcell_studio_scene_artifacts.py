#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import generate_scene_from_cell_definition as scene_yaml
import validate_cell_definition as cell_validator
from validate_all_workcell_studio_scenes import resolve_scenes_root


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    payload, _, _ = cell_validator.load_yaml(path)
    return payload if isinstance(payload, dict) else {}


def _write_yaml(path: Path, payload: dict[str, Any], dry_run: bool) -> None:
    if dry_run:
        print(f"DRY-RUN write: {path}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(scene_yaml._to_yaml_text(payload), encoding="utf-8")
    print(f"Wrote: {path}")


def _scene_names(scenes_root: Path) -> list[str]:
    return sorted(p.name for p in scenes_root.iterdir() if p.is_dir())


def _infer_environment_assets(scene_dir: Path) -> dict[str, Any]:
    env = _load_yaml(scene_dir / "environment.yaml")
    layout = _load_yaml(scene_dir / "layout" / "workcell_studio_layout.yaml")
    cell_def = _load_yaml(scene_dir / "cell_definition.yaml")
    table_ids, bin_ids, conveyor_ids, camera_ids, zone_ids = [], [], [], [], []
    for surf in env.get("support_surfaces", []) if isinstance(env.get("support_surfaces"), list) else []:
        if isinstance(surf, dict) and surf.get("id"): table_ids.append(str(surf["id"]))
    for zone in env.get("task_zones", []) if isinstance(env.get("task_zones"), list) else []:
        if isinstance(zone, dict) and zone.get("id"): zone_ids.append(str(zone["id"]))
    for key in ("bins", "targets", "objects", "assets"):
        for item in env.get(key, []) if isinstance(env.get(key), list) else []:
            if isinstance(item, dict) and item.get("id"):
                item_id = str(item.get("id")); lowered = item_id.lower()
                if "bin" in lowered: bin_ids.append(item_id)
                if "conveyor" in lowered: conveyor_ids.append(item_id)
    for item in layout.get("items", []) if isinstance(layout.get("items"), list) else []:
        if isinstance(item, dict):
            item_id = str(item.get("id") or ""); item_type = str(item.get("type") or "").lower()
            if "camera" in item_type or "camera" in item_id.lower(): camera_ids.append(item_id)
            if "conveyor" in item_type or "conveyor" in item_id.lower(): conveyor_ids.append(item_id)
    for sensor in cell_def.get("sensors", []) if isinstance(cell_def.get("sensors"), list) else []:
        if isinstance(sensor, dict) and sensor.get("id") and "camera" in str(sensor.get("type") or "").lower():
            camera_ids.append(str(sensor["id"]))
    return {
        "schema_version": "workcell_studio_environment_assets/v1",
        "status": "inferred" if env else "incomplete",
        "sections": {
            "tables": {"status": "inferred" if table_ids else "incomplete", "assets": sorted(set(table_ids))},
            "bins": {"status": "inferred" if bin_ids else "incomplete", "assets": sorted(set(bin_ids))},
            "conveyors_placeholders": {"status": "inferred" if conveyor_ids else "incomplete", "assets": sorted(set(conveyor_ids))},
            "cameras": {"status": "inferred" if camera_ids else "incomplete", "assets": sorted(set(camera_ids))},
            "zones": {"status": "inferred" if zone_ids else "incomplete", "assets": sorted(set(zone_ids))},
        },
        "provenance": {"mode": "inferred_from_existing_scene_files", "sources": ["environment.yaml", "layout/workcell_studio_layout.yaml", "cell_definition.yaml", "urdf/scene.urdf", "urdf/scene.urdf.xacro"]},
    }


def _generate_layout_preview(scene_dir: Path) -> dict[str, Any]:
    editable = _load_yaml(scene_dir / "layout" / "workcell_studio_layout.yaml")
    items = editable.get("items", []) if isinstance(editable.get("items"), list) else []
    preview_items, locked_entities = [], []
    for item in items:
        if not isinstance(item, dict):
            continue
        item_id = str(item.get("id") or "")
        item_type = str(item.get("type") or "unknown")
        preview_items.append({"id": item_id, "type": item_type, "status": "ready" if item_id else "incomplete"})
        if "urdf" in item_type.lower() or item.get("locked") is True:
            locked_entities.append({"id": item_id, "type": item_type, "editable": False, "source": "urdf_preview"})
    return {
        "schema_version": "workcell_studio_layout_preview/v1",
        "status": "inferred" if preview_items else "incomplete",
        "summary": {"editable_layout_source": "layout/workcell_studio_layout.yaml", "editable_items_count": len(preview_items), "preview_readiness": "ready" if preview_items else "missing_layout_items"},
        "preview_items": preview_items,
        "locked_urdf_preview_entities": locked_entities,
        "notes": ["Generated preview only.", "Locked URDF preview entities are intentionally not written into editable layout items."],
        "provenance": {"mode": "generated_readiness_metadata", "sources": ["layout/workcell_studio_layout.yaml"]},
    }


def _generate_task_recipe(scene_dir: Path) -> dict[str, Any]:
    cell_def = _load_yaml(scene_dir / "cell_definition.yaml")
    task = cell_def.get("task") if isinstance(cell_def.get("task"), dict) else {}
    recipe = {
        "schema_version": "task_recipe/v1",
        "task": {
            "id": str(task.get("id") or "generated_preview_task"),
            "type": str(task.get("type") or "pick_place"),
            "source_object": str(task.get("source_object") or "detected_object"),
            "destinations": task.get("destinations") if isinstance(task.get("destinations"), list) else [],
            "decision_rules": task.get("rules") if isinstance(task.get("rules"), list) else [{"id": "default_preview", "when": {"default": True}, "destination": "unknown"}],
            "notes": "Generated from existing scene metadata. Preview-safe.",
        },
        "provenance": {"mode": "generated_readiness_metadata", "sources": ["cell_definition.yaml"]},
    }
    return recipe


def _generate_task_intent(scene_dir: Path) -> dict[str, Any]:
    env = _load_yaml(scene_dir / "environment.yaml")
    zones = env.get("task_zones", []) if isinstance(env.get("task_zones"), list) else []
    pick_zone = next((str(z.get("id")) for z in zones if isinstance(z, dict) and z.get("type") == "pick" and z.get("id")), "pick_zone_main")
    place_zone = next((str(z.get("id")) for z in zones if isinstance(z, dict) and z.get("type") == "place" and z.get("id")), "place_zone_main")
    return {
        "schema": "workcell_builder_task_intent/v1",
        "scene_package": str(scene_dir),
        "task": {"id": f"{scene_dir.name}_preview_task", "type": "pick_place", "family": "pick_place", "mode": "simulation_preview"},
        "pick": {"source": {"type": "zone", "id": pick_zone}},
        "place": {"target": {"type": "zone", "id": place_zone}, "release_strategy": "tool_release"},
        "grasp": {"strategy": "tool_profile_default", "approach_axis": "z_down", "retreat_axis": "z_up"},
        "safety": {"preview_only": True, "use_fake_hardware": True, "execution_mode": "simulation_preview", "no_robot_motion": True},
        "provenance": {"mode": "generated_readiness_metadata", "sources": ["environment.yaml"]},
    }


def _generate_for_scene(scene_dir: Path, dry_run: bool, overwrite: bool) -> None:
    targets = [scene_dir / "generated" / "environment_assets.yaml", scene_dir / "layout" / "workcell_studio_layout.generated.yaml", scene_dir / "config" / "task_recipe.yaml", scene_dir / "config" / "workcell_builder_task_intent.yaml"]
    conflicts = [str(t) for t in targets if t.exists()]
    if conflicts and not overwrite and not dry_run:
        raise ValueError(f"overwrite required, existing artifacts: {', '.join(conflicts)}")
    _write_yaml(targets[0], _infer_environment_assets(scene_dir), dry_run)
    _write_yaml(targets[1], _generate_layout_preview(scene_dir), dry_run)
    _write_yaml(targets[2], _generate_task_recipe(scene_dir), dry_run)
    _write_yaml(targets[3], _generate_task_intent(scene_dir), dry_run)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate workcell studio scene artifacts.")
    parser.add_argument("--root", type=Path, default=resolve_scenes_root(REPO_ROOT), help="scene root path")
    parser.add_argument("--scene", type=str, help="single-scene mode")
    parser.add_argument("--all", action="store_true", dest="all_scenes", help="generate for all discovered scenes")
    parser.add_argument("--dry-run", action="store_true", help="print planned writes only")
    parser.add_argument("--overwrite", action="store_true", help="required to replace existing artifacts")
    parser.add_argument("--json", action="store_true", help="compatibility no-op")
    args = parser.parse_args()
    if args.scene and args.all_scenes:
        print("ERROR: --scene and --all are mutually exclusive", file=sys.stderr); return 2
    if not args.scene and not args.all_scenes:
        print("ERROR: either --scene or --all is required", file=sys.stderr); return 2
    scenes_root = args.root
    for name in ([args.scene] if args.scene else _scene_names(scenes_root)):
        scene_dir = scenes_root / name
        if not scene_dir.is_dir():
            print(f"ERROR: scene not found: {name}", file=sys.stderr); return 2
        try:
            _generate_for_scene(scene_dir, args.dry_run, args.overwrite)
        except ValueError as exc:
            print(f"ERROR: {exc}", file=sys.stderr); return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
