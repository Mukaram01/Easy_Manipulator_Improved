#!/usr/bin/env python3
"""Offline acceptance runner for Workcell Studio editable-layout bootstrap.

The runner copies requested scenes into a temporary workspace, runs the same
source-priority bootstrap/save/reload flow used by the Workcell Builder shared
model helper, and reports machine-readable acceptance entries.  It intentionally
stays offline: no ROS launch paths, no hardware commands, no robot motion, and
no EPD/perception mutation are used.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

import yaml


LAYOUT_REL = Path("layout") / "workcell_studio_layout.yaml"
ENV_LAYOUT_REL = Path("environment_layout.yaml")


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def write_yaml(path: Path, payload: dict[str, Any]) -> bool:
    before = path.read_text(encoding="utf-8") if path.is_file() else None
    path.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(payload, sort_keys=False)
    path.write_text(text, encoding="utf-8")
    return before != text


def as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def first_string(node: dict[str, Any], keys: list[str]) -> str:
    for key in keys:
        value = node.get(key)
        if value is not None:
            text = str(value).strip()
            if text:
                return text
    return ""


def read_bool(value: Any) -> tuple[bool, bool]:
    if isinstance(value, bool):
        return True, value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "yes", "1", "on"}:
            return True, True
        if lowered in {"false", "no", "0", "off"}:
            return True, False
    return False, False


def bool_true_or_absent(node: dict[str, Any], key: str) -> bool:
    if key not in node:
        return True
    parsed, value = read_bool(node.get(key))
    return parsed and value


def bool_false_or_absent(node: dict[str, Any], key: str) -> bool:
    if key not in node:
        return True
    parsed, value = read_bool(node.get(key))
    return parsed and not value


def is_fallback_preview_entry(node: dict[str, Any]) -> bool:
    for key in ("source_layer", "source_file", "source_path", "provenance", "preview_source"):
        value = str(node.get(key, "")).lower()
        if "fallback" in value or "static_preview" in value:
            return True
    return False


def is_effectively_editable(node: dict[str, Any]) -> bool:
    parsed_locked, locked = read_bool(node.get("locked"))
    if parsed_locked and locked:
        return False
    if "locked" in node and not parsed_locked:
        return False
    parsed_editable, editable = read_bool(node.get("editable"))
    if parsed_editable:
        return editable
    if "editable" in node and not parsed_editable:
        return False
    return not is_fallback_preview_entry(node)


def should_skip_source_node_as_locked(node: dict[str, Any]) -> bool:
    parsed_locked, locked = read_bool(node.get("locked"))
    if parsed_locked and locked:
        return True
    return not bool_false_or_absent(node, "generated_locked")


def sequence3(value: Any, fallback: list[float]) -> list[float]:
    if isinstance(value, list) and len(value) >= 3:
        try:
            return [float(value[0]), float(value[1]), float(value[2])]
        except (TypeError, ValueError):
            return fallback
    return fallback


def normalized_pose(node: dict[str, Any]) -> dict[str, list[float]]:
    pose = node.get("pose")
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if isinstance(pose, dict):
        xyz = sequence3(pose.get("xyz"), xyz)
        rpy = sequence3(pose.get("rpy"), rpy)
        if "xyz" not in pose:
            xyz = [float(pose.get("x", 0.0) or 0.0), float(pose.get("y", 0.0) or 0.0), float(pose.get("z", 0.0) or 0.0)]
        if "rpy" not in pose:
            rpy = [float(pose.get("roll", 0.0) or 0.0), float(pose.get("pitch", 0.0) or 0.0), float(pose.get("yaw", 0.0) or 0.0)]
    elif isinstance(pose, list) and len(pose) >= 6:
        xyz = sequence3(pose[:3], xyz)
        rpy = sequence3(pose[3:6], rpy)
    else:
        xyz = sequence3(node.get("xyz"), xyz)
        rpy = sequence3(node.get("rpy"), rpy)
    return {"xyz": xyz, "rpy": rpy}


def normalized_dimensions(node: dict[str, Any]) -> list[float]:
    fallback = [0.25, 0.25, 0.25]
    for key in ("dimensions", "size"):
        value = node.get(key)
        if isinstance(value, list):
            return sequence3(value, fallback)
        if isinstance(value, dict):
            return [float(value.get("width", 0.25) or 0.25), float(value.get("depth", 0.25) or 0.25), float(value.get("height", 0.25) or 0.25)]
    return fallback


def new_layout(scene_name: str) -> dict[str, Any]:
    return {"schema_version": "workcell_studio_layout/v1", "scene_name": scene_name, "items": []}


def normalize_source_node(node: dict[str, Any], *, id_hint: str = "", type_hint: str = "object", source: str) -> dict[str, Any] | None:
    item_id = id_hint or first_string(node, ["id", "name", "camera_id"])
    if not item_id:
        return None
    item_type = type_hint or first_string(node, ["type", "category", "class", "role"]) or "object"
    item: dict[str, Any] = {
        "id": item_id,
        "type": item_type,
        "category": item_type,
        "pose": normalized_pose(node),
        "dimensions": normalized_dimensions(node),
        "source": source,
        "editable": True,
        "locked": False,
    }
    role = first_string(node, ["role", "class", "type"])
    if role:
        item["role"] = role
    label = first_string(node, ["display_name", "label", "name"])
    if label:
        item["display_name"] = label
    mesh = node.get("mesh")
    if mesh:
        item["mesh"] = mesh
    else:
        mesh_path = first_string(node, ["mesh_path", "visual_mesh", "collision_mesh", "source_path"])
        if mesh_path:
            item["mesh"] = {"path": mesh_path}
    return item


def append_unique(items: list[dict[str, Any]], item: dict[str, Any] | None, ids: set[str]) -> None:
    if not item:
        return
    item_id = str(item.get("id", "")).strip()
    if not item_id or item_id in ids:
        return
    ids.add(item_id)
    items.append(item)


def append_sequence_items(
    seq: Any,
    *,
    type_hint: str,
    source: str,
    items: list[dict[str, Any]],
    ids: set[str],
) -> int:
    skipped_locked = 0
    for node in as_list(seq):
        if not isinstance(node, dict):
            continue
        if should_skip_source_node_as_locked(node) or not bool_true_or_absent(node, "editable") or not bool_true_or_absent(node, "placeable"):
            skipped_locked += 1
            continue
        append_unique(items, normalize_source_node(node, type_hint=type_hint, source=source), ids)
    return skipped_locked


def append_environment_object_map_items(map_node: Any, *, source: str, items: list[dict[str, Any]], ids: set[str]) -> int:
    skipped_locked = 0
    for item_id, node in as_dict(map_node).items():
        if not isinstance(node, dict):
            continue
        if should_skip_source_node_as_locked(node):
            skipped_locked += 1
            continue
        append_unique(items, normalize_source_node(node, id_hint=str(item_id), type_hint="object", source=source), ids)
    return skipped_locked


def finish_if_items(scene_name: str, source: str, items: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not items:
        return None
    layout = new_layout(scene_name)
    layout["items"] = items
    layout["empty_layout_marker"] = False
    return {"source_used": source, "layout": layout}


def bootstrap_layout(scene_dir: Path, scene_name: str) -> dict[str, Any]:
    result: dict[str, Any] = {
        "source_used": "",
        "layout": {**new_layout(scene_name), "empty_layout_marker": True},
        "editable_item_count": 0,
        "skipped_locked_count": 0,
        "skipped_fallback_count": 0,
        "skipped_unsafe_metadata_count": 0,
        "blockers": [],
    }

    existing = load_yaml(scene_dir / LAYOUT_REL)
    if existing:
        existing_items = as_list(existing.get("items"))
        schema = str(existing.get("schema_version", ""))
        if schema == "workcell_studio_layout/v1" or (not schema and isinstance(existing.get("items"), list)):
            items: list[dict[str, Any]] = []
            ids: set[str] = set()
            for node in existing_items:
                if not isinstance(node, dict):
                    result["skipped_unsafe_metadata_count"] += 1
                    continue
                if is_effectively_editable(node):
                    item = dict(node)
                    item["editable"] = True
                    item["locked"] = False
                    item.setdefault("source", "layout/workcell_studio_layout.yaml")
                    append_unique(items, item, ids)
                else:
                    parsed_locked, locked = read_bool(node.get("locked"))
                    if parsed_locked and locked:
                        result["skipped_locked_count"] += 1
                    elif is_fallback_preview_entry(node):
                        result["skipped_fallback_count"] += 1
                    else:
                        result["skipped_unsafe_metadata_count"] += 1
            finished = finish_if_items(scene_name, "layout/workcell_studio_layout.yaml", items)
            if finished:
                result.update(finished)
                result["editable_item_count"] = len(items)
                return result

    env_layout_path = scene_dir / ENV_LAYOUT_REL
    env_layout = load_yaml(env_layout_path)
    if not env_layout_path.exists():
        result["blockers"].append("no environment_layout.yaml")
    if env_layout:
        items = []
        ids = set()
        for key, type_hint in (
            ("items", "object"), ("assets", "object"), ("placed_assets", "object"),
            ("objects", "object"), ("zones", "zone"), ("targets", "place_target"),
        ):
            result["skipped_locked_count"] += append_sequence_items(env_layout.get(key), type_hint=type_hint, source="environment_layout.yaml", items=items, ids=ids)
        camera = env_layout.get("camera")
        if isinstance(camera, dict) and bool_true_or_absent(camera, "editable"):
            append_unique(items, normalize_source_node(camera, id_hint=first_string(camera, ["id", "camera_id", "name"]), type_hint="camera", source="environment_layout.yaml"), ids)
        finished = finish_if_items(scene_name, "environment_layout.yaml", items)
        if finished:
            result.update(finished)
            result["editable_item_count"] = len(items)
            return result

    environment_path = scene_dir / "environment.yaml"
    environment = load_yaml(environment_path)
    if not environment_path.exists():
        result["blockers"].append("no environment.yaml")
    if environment:
        items = []
        ids = set()
        result["skipped_locked_count"] += append_sequence_items(environment.get("placed_objects"), type_hint="object", source="environment.yaml", items=items, ids=ids)
        result["skipped_locked_count"] += append_environment_object_map_items(environment.get("objects"), source="environment.yaml", items=items, ids=ids)
        result["skipped_locked_count"] += append_sequence_items(environment.get("cameras"), type_hint="camera", source="environment.yaml", items=items, ids=ids)
        camera = environment.get("camera")
        if isinstance(camera, dict):
            parsed, enabled = read_bool(camera.get("enabled"))
            if not parsed or enabled:
                append_unique(items, normalize_source_node(camera, id_hint=first_string(camera, ["id", "camera_id", "name"]), type_hint="camera", source="environment.yaml"), ids)
        result["skipped_locked_count"] += append_sequence_items(environment.get("zones"), type_hint="zone", source="environment.yaml", items=items, ids=ids)
        result["skipped_locked_count"] += append_sequence_items(environment.get("task_zones"), type_hint="zone", source="environment.yaml", items=items, ids=ids)
        result["skipped_locked_count"] += append_sequence_items(as_dict(environment.get("workspace")).get("zones"), type_hint="zone", source="environment.yaml", items=items, ids=ids)
        finished = finish_if_items(scene_name, "environment.yaml", items)
        if finished:
            result.update(finished)
            result["editable_item_count"] = len(items)
            return result

    cell_path = scene_dir / "cell_definition.yaml"
    cell = load_yaml(cell_path)
    if not cell_path.exists():
        result["blockers"].append("no cell_definition.yaml")
    if cell:
        items = []
        ids = set()
        for seq in (
            cell.get("assets"), as_dict(cell.get("scene")).get("assets"), as_dict(cell.get("environment")).get("assets"),
        ):
            result["skipped_locked_count"] += append_sequence_items(seq, type_hint="object", source="cell_definition.yaml", items=items, ids=ids)
        for seq in (cell.get("zones"), as_dict(cell.get("environment")).get("zones")):
            result["skipped_locked_count"] += append_sequence_items(seq, type_hint="zone", source="cell_definition.yaml", items=items, ids=ids)
        result["skipped_locked_count"] += append_sequence_items(as_dict(cell.get("task")).get("destinations"), type_hint="place_target", source="cell_definition.yaml", items=items, ids=ids)
        finished = finish_if_items(scene_name, "cell_definition.yaml", items)
        if finished:
            result.update(finished)
            result["editable_item_count"] = len(items)
            return result

    result["blockers"].append("no editable bootstrap source with safe metadata")
    return result


def bootstrap_environment_layout(scene_dir: Path, scene_name: str, layout: dict[str, Any]) -> tuple[bool, bool, str]:
    layout_items = [item for item in as_list(layout.get("items")) if isinstance(item, dict) and item.get("id")]
    if not layout_items:
        return False, False, "editable layout has no bootstrappable editable/placeable items"

    env_path = scene_dir / ENV_LAYOUT_REL
    env = load_yaml(env_path) if env_path.exists() else {}
    before = yaml.safe_dump(env, sort_keys=False)
    env.setdefault("schema_version", "environment_layout/v1")
    env.setdefault("scene_name", scene_name)
    placed_assets = as_list(env.get("placed_assets"))
    by_id = {str(item["id"]): item for item in layout_items}
    emitted: set[str] = set()
    updated: list[Any] = []
    for existing in placed_assets:
        if isinstance(existing, dict) and str(existing.get("id", "")) in by_id:
            item = dict(existing)
            item.update(by_id[str(existing["id"])])
            updated.append(item)
            emitted.add(str(existing["id"]))
        else:
            updated.append(existing)
    for item_id, item in by_id.items():
        if item_id not in emitted:
            updated.append(dict(item))
    env["placed_assets"] = updated
    if yaml.safe_dump(env, sort_keys=False) == before:
        return True, False, ""
    write_yaml(env_path, env)
    return True, True, ""


def inspect_layout(path: Path) -> tuple[bool, int, str]:
    data = load_yaml(path)
    items = as_list(data.get("items"))
    if not data:
        return False, 0, "layout did not reload as a YAML map"
    if data.get("schema_version") != "workcell_studio_layout/v1":
        return False, 0, "layout reloaded with an unexpected schema_version"
    editable_count = sum(1 for item in items if isinstance(item, dict) and is_effectively_editable(item))
    return True, editable_count, ""


def run_scene(scene_name: str, scenes_root: Path, temp_root: Path) -> dict[str, Any]:
    entry: dict[str, Any] = {
        "scene_name": scene_name,
        "source_used_for_bootstrap": "",
        "editable_item_count": 0,
        "skipped_locked_count": 0,
        "skipped_fallback_count": 0,
        "skipped_unsafe_metadata_count": 0,
        "output_files_written": [],
        "status": "fail",
        "blocker_reason": "",
    }
    source_scene = scenes_root / scene_name
    if not source_scene.is_dir():
        entry["blocker_reason"] = f"source scene not found: {source_scene}"
        return entry

    copied_scene = temp_root / scene_name
    shutil.copytree(source_scene, copied_scene)
    try:
        bootstrap = bootstrap_layout(copied_scene, scene_name)
        entry["source_used_for_bootstrap"] = str(bootstrap["source_used"])
        entry["editable_item_count"] = int(bootstrap["editable_item_count"])
        entry["skipped_locked_count"] = int(bootstrap["skipped_locked_count"])
        entry["skipped_fallback_count"] = int(bootstrap["skipped_fallback_count"])
        entry["skipped_unsafe_metadata_count"] = int(bootstrap["skipped_unsafe_metadata_count"])
        blockers = list(bootstrap.get("blockers", []))
        if entry["editable_item_count"] <= 0:
            entry["blocker_reason"] = "; ".join(blockers) or "bootstrap created no editable items"
            return entry

        layout_path = copied_scene / LAYOUT_REL
        if write_yaml(layout_path, bootstrap["layout"]):
            entry["output_files_written"].append(str(LAYOUT_REL))
        ok, wrote_env, env_error = bootstrap_environment_layout(copied_scene, scene_name, bootstrap["layout"])
        if not ok:
            entry["blocker_reason"] = env_error
            return entry
        if wrote_env:
            entry["output_files_written"].append(str(ENV_LAYOUT_REL))

        reloaded_ok, reloaded_count, reload_error = inspect_layout(layout_path)
        if not reloaded_ok:
            entry["blocker_reason"] = reload_error
            return entry
        if reloaded_count != entry["editable_item_count"]:
            entry["blocker_reason"] = f"reload editable count mismatch: wrote {entry['editable_item_count']} but reloaded {reloaded_count}"
            return entry
        entry["status"] = "pass"
        entry["blocker_reason"] = ""
        return entry
    except Exception as exc:  # keep JSON machine-readable on unexpected acceptance failures
        entry["blocker_reason"] = f"unexpected acceptance error: {exc}"
        return entry


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run offline layout bootstrap/save/reload acceptance for copied scenes.")
    parser.add_argument("--scenes", nargs="+", required=True, help="Scene names under the scenes directory, e.g. ur5_2f_test suction_test.")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON.")
    parser.add_argument("--scenes-root", type=Path, default=repo_root() / "scenes", help="Directory containing source scenes.")
    parser.add_argument("--keep-temp", action="store_true", help="Keep the temporary copied-scene workspace for debugging.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    temp_root = Path(tempfile.mkdtemp(prefix="layout_bootstrap_acceptance_"))
    entries: list[dict[str, Any]] = []
    try:
        for scene in args.scenes:
            entries.append(run_scene(scene, args.scenes_root, temp_root))
        payload = {
            "acceptance": "layout_bootstrap_save_reload",
            "offline": True,
            "ros_hardware_paths_used": False,
            "robot_motion_used": False,
            "epd_changes_used": False,
            "temp_root": str(temp_root) if args.keep_temp else None,
            "results": entries,
        }
        if args.json:
            print(json.dumps(payload, indent=2, sort_keys=False))
        else:
            for entry in entries:
                reason = f" ({entry['blocker_reason']})" if entry["blocker_reason"] else ""
                print(f"{entry['scene_name']}: {entry['status']} - {entry['editable_item_count']} editable item(s){reason}")
        return 0 if all(entry["status"] == "pass" for entry in entries) else 2
    finally:
        if args.keep_temp:
            print(f"Kept temporary workspace: {temp_root}", file=sys.stderr)
        else:
            shutil.rmtree(temp_root, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
