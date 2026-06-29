#!/usr/bin/env python3
"""Export a dependency-light Workcell Studio scene description for web viewers.

This exporter is intentionally read-only with respect to scene inputs. It only
loads authoring/generated metadata files, normalizes the small subset needed by a
browser preview, and writes one deterministic JSON document to --output.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover - exercised only in minimal envs
    yaml = None  # type: ignore

SCHEMA_VERSION = "workcell_studio_web_scene/v1"
INPUTS = {
    "scene_manifest": "scene_manifest.yaml",
    "cell_definition": "cell_definition.yaml",
    "environment": "environment.yaml",
    "layout": "layout/workcell_studio_layout.yaml",
    "visual_mesh_index": "generated/scene_visual_mesh_index.json",
}
HELPER_TOKENS = (
    "overlay",
    "helper",
    "diagnostic",
    "safety_zone",
    "pick_zone",
    "place_zone",
    "robot_reach",
    "warning_anchor",
    "warning_badge",
    "camera_fov",
    "fov",
    "pick_coverage",
    "reachability",
    "collision",
    "work_envelope",
    "task_route",
    "approach_retreat",
    "epd_detection",
    "detection_label",
    "bounds_box",
    "bounding_box",
)

Json = Dict[str, Any]


def _warn(warnings: List[Json], code: str, message: str, source: Optional[str] = None) -> None:
    item: Json = {"code": code, "message": message}
    if source:
        item["source"] = source
    warnings.append(item)


def _load_yaml(path: Path, rel: str, warnings: List[Json]) -> Any:
    if yaml is None:
        _warn(warnings, "yaml_unavailable", "PyYAML is required to read YAML inputs; skipping file.", rel)
        return None
    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
        return loaded or {}
    except Exception as exc:  # noqa: BLE001 - exporter must keep producing diagnostic JSON
        _warn(warnings, "yaml_parse_failed", f"Failed to parse {rel}: {exc}", rel)
        return None


def _load_json(path: Path, rel: str, warnings: List[Json]) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        _warn(warnings, "json_parse_failed", f"Failed to parse {rel}: {exc}", rel)
        return None


def _load_inputs(scene_dir: Path, warnings: List[Json]) -> Dict[str, Any]:
    loaded: Dict[str, Any] = {}
    for key, rel in INPUTS.items():
        path = scene_dir / rel
        if not path.exists():
            _warn(warnings, "optional_file_missing", f"Optional input file is missing: {rel}", rel)
            loaded[key] = None
            continue
        loaded[key] = _load_json(path, rel, warnings) if rel.endswith(".json") else _load_yaml(path, rel, warnings)
    return loaded


def _provenance(fields: Iterable[str], source: str) -> Dict[str, str]:
    return {field: source for field in sorted(set(fields))}


def _as_map(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _as_list(value: Any) -> List[Any]:
    return value if isinstance(value, list) else []


def _first_present(*values: Any) -> Any:
    for value in values:
        if value not in (None, "", [], {}):
            return value
    return None


def _stable_id(prefix: str, index: int) -> str:
    return f"{prefix}_{index:04d}"


def _relative_uri(uri: Any, scene_dir: Path) -> Any:
    if not isinstance(uri, str) or not uri:
        return uri
    if uri.startswith("package://"):
        return uri
    p = Path(uri)
    if p.is_absolute():
        try:
            return os.path.relpath(p, scene_dir)
        except ValueError:
            return uri
    return uri


def _copy_fields(src: Mapping[str, Any], fields: Iterable[str], source: str, scene_dir: Path) -> Json:
    out: Json = {}
    prov_fields: List[str] = []
    for field in fields:
        if field in src:
            value = src[field]
            if field in {"mesh_uri", "package_uri", "source_path", "mesh_path", "resolved_source_path"}:
                value = _relative_uri(value, scene_dir)
            out[field] = value
            prov_fields.append(field)
    out["provenance"] = _provenance(prov_fields, source)
    return out


def _identity_text(item: Mapping[str, Any]) -> str:
    parts: List[str] = []
    for key in ("source_layer", "active_visual_source", "role", "category", "id", "display_name", "status", "mesh_load_warning", "source_path", "mesh_path"):
        value = item.get(key)
        if isinstance(value, list):
            value = " ".join(str(v) for v in value)
        if value is not None:
            parts.append(str(value).lower())
    warnings = item.get("warnings")
    if isinstance(warnings, list):
        parts.extend(str(v).lower() for v in warnings)
    return " ".join(parts)


def _is_helper(item: Mapping[str, Any]) -> bool:
    text = _identity_text(item)
    return any(token in text for token in HELPER_TOKENS)


def _section_from_item(item: Mapping[str, Any]) -> str:
    text = _identity_text(item)
    category = str(item.get("category", "")).lower()
    role = str(item.get("role", "")).lower()
    if category == "robot" or role == "robot":
        return "robots"
    if (
        category in {"tool", "gripper", "end_effector"}
        or role in {"tool", "gripper", "end_effector"}
        or any(token in text for token in ("tool", "gripper", "end_effector", "robotiq", "suction"))
    ):
        return "tools"
    if "camera" in text or "realsense" in text:
        return "sensors"
    if _is_helper(item):
        return "zones"
    return "assets"


def _generated_preview_items(index: Mapping[str, Any], scene_dir: Path, warnings: List[Json]) -> Dict[str, List[Json]]:
    sections = {"robots": [], "tools": [], "assets": [], "sensors": [], "zones": []}
    items = _as_list(index.get("visual_items") or index.get("items"))
    if not items:
        _warn(warnings, "visual_mesh_index_items_missing", "Visual mesh index has no visual_items/items list.", INPUTS["visual_mesh_index"])
        return sections
    fields = (
        "id", "type", "category", "role", "display_name", "link", "object_name", "visual", "pose", "world_pose",
        "baked_world_visual_pose", "visual_origin", "geometry_type", "primitive_geometry_type", "package_uri",
        "source_path", "mesh_path", "resolved_source_path", "scale", "mesh_scale", "source_layer", "active_visual_source",
        "render_expected", "mesh_available", "resolved", "warning",
    )
    for i, raw in enumerate(items):
        if not isinstance(raw, Mapping):
            _warn(warnings, "visual_mesh_index_item_invalid", f"Skipping non-object visual index item at offset {i}.", INPUTS["visual_mesh_index"])
            continue
        item = _copy_fields(raw, fields, INPUTS["visual_mesh_index"], scene_dir)
        item.setdefault("id", _stable_id("generated_preview", i))
        item["locked"] = True
        item["editable"] = False
        item["source_kind"] = "generated_preview"
        item["provenance"].update({"locked": INPUTS["visual_mesh_index"], "editable": INPUTS["visual_mesh_index"], "source_kind": INPUTS["visual_mesh_index"]})
        sections[_section_from_item(raw)].append(item)
    return sections


def _authored_item(raw: Mapping[str, Any], source: str, index: int, scene_dir: Path) -> Json:
    fields = (
        "id", "type", "role", "category", "display_name", "frame", "pose", "pose_xyz", "pose_rpy", "dimensions",
        "geometry_type", "primitive_geometry_type", "mesh_uri", "package_uri", "source_path", "mesh_path", "material",
        "layout_item_ref", "support_surface_ref", "task_zone_ref", "scale", "mesh_scale", "perception_mode", "runtime_enforced", "runtime_commanded",
    )
    item = _copy_fields(raw, fields, source, scene_dir)
    item.setdefault("id", _stable_id("authored", index))
    item["locked"] = False
    item["editable"] = True
    item["source_kind"] = "user_authored"
    item["provenance"].update({"locked": source, "editable": source, "source_kind": source})
    return item


def _authored_sections(data: Dict[str, Any], scene_dir: Path, warnings: List[Json]) -> Dict[str, List[Json]]:
    sections = {"assets": [], "sensors": [], "zones": []}
    counter = 0
    layout = _as_map(data.get("layout"))
    layout_items = _as_list(layout.get("items"))
    if data.get("layout") is not None and "items" not in layout:
        _warn(warnings, "layout_items_missing", "layout/workcell_studio_layout.yaml has no items list.", INPUTS["layout"])
    for raw in layout_items:
        if isinstance(raw, Mapping):
            item = _authored_item(raw, INPUTS["layout"], counter, scene_dir)
            counter += 1
            sections[_section_from_item(raw) if _section_from_item(raw) in sections else "assets"].append(item)

    env = _as_map(data.get("environment"))
    env_root = _as_map(env.get("environment"))
    for key in ("support_surfaces", "assets", "sensors", "zones"):
        for raw in _as_list(env_root.get(key)):
            if isinstance(raw, Mapping):
                item = _authored_item(raw, INPUTS["environment"], counter, scene_dir)
                counter += 1
                section = "sensors" if key == "sensors" or _section_from_item(raw) == "sensors" else ("zones" if key == "zones" or _is_helper(raw) else "assets")
                sections[section].append(item)
    return sections


def _entity(src: Mapping[str, Any], fields: Iterable[str], source: str, fallback_id: str) -> Json:
    item = {field: src[field] for field in fields if field in src}
    if not item.get("id"):
        item["id"] = str(_first_present(item.get("model"), item.get("profile"), item.get("type"), fallback_id))
    item["provenance"] = _provenance(item.keys(), source)
    return item


def _top_level_entities(data: Dict[str, Any], warnings: List[Json]) -> Tuple[List[Json], List[Json], List[Json]]:
    robots: List[Json] = []
    tools: List[Json] = []
    sensors: List[Json] = []
    for key, source in (("cell_definition", INPUTS["cell_definition"]), ("environment", INPUTS["environment"]), ("scene_manifest", INPUTS["scene_manifest"])):
        root = _as_map(data.get(key))
        robot = _as_map(root.get("robot"))
        if robot:
            robots.append(_entity(robot, ("id", "model", "profile", "planning_group", "world_frame", "base_frame", "tool_link", "tool_mount_link", "home_named_target"), source, "robot"))
        else:
            if data.get(key) is not None:
                _warn(warnings, "robot_field_missing", f"{source} has no robot object.", source)
        tool = _as_map(root.get("tool") or root.get("end_effector"))
        if tool:
            tools.append(_entity(tool, ("id", "type", "model", "profile", "mount_link", "grasp_frame", "allowed_touch_links"), source, "tool"))
        camera = _as_map(root.get("camera"))
        if camera:
            sensors.append(_entity(camera, ("id", "enabled", "camera_id", "frame_id", "pose", "rgb_topic", "depth_topic", "pointcloud_topic"), source, "camera"))
    return robots, tools, sensors


def _sort_items(items: List[Json]) -> List[Json]:
    return sorted(items, key=lambda x: (str(x.get("source_kind", "")), str(x.get("category", "")), str(x.get("role", "")), str(x.get("id", ""))))


def build_web_scene(scene_dir: Path) -> Json:
    scene_dir = scene_dir.resolve()
    warnings: List[Json] = []
    data = _load_inputs(scene_dir, warnings)

    manifest = _as_map(data.get("scene_manifest"))
    scene_meta = _as_map(manifest.get("scene"))
    cell_scene = _as_map(_as_map(data.get("cell_definition")).get("cell"))
    env_scene = _as_map(_as_map(data.get("environment")).get("scene"))
    scene_name = _first_present(scene_meta.get("name"), cell_scene.get("name"), cell_scene.get("id"), env_scene.get("name"), env_scene.get("id"), scene_dir.name)

    generated = _generated_preview_items(_as_map(data.get("visual_mesh_index")), scene_dir, warnings) if data.get("visual_mesh_index") is not None else {"robots": [], "tools": [], "assets": [], "sensors": [], "zones": []}
    authored = _authored_sections(data, scene_dir, warnings)
    top_robots, top_tools, top_sensors = _top_level_entities(data, warnings)

    robots = top_robots + generated["robots"]
    tools = top_tools + generated["tools"]
    sensors = top_sensors + authored["sensors"] + generated["sensors"]
    assets = authored["assets"] + generated["assets"]
    zones = authored["zones"] + generated["zones"]

    output: Json = {
        "schema_version": SCHEMA_VERSION,
        "scene": {
            "id": scene_name,
            "name": scene_name,
            "source_dir": os.path.relpath(scene_dir, Path.cwd()),
            "provenance": {
                "id": "scene_manifest.yaml|cell_definition.yaml|environment.yaml|directory_name",
                "name": "scene_manifest.yaml|cell_definition.yaml|environment.yaml|directory_name",
                "source_dir": "cli:--scene",
                "units": "contract",
                "coordinate_system": "contract",
            },
            "units": {"distance": "metre", "angle": "radian"},
            "coordinate_system": {
                "frame": "world",
                "up_axis": "z",
                "convention": "ros_world_z_up",
                "pose_reference": "item poses are relative to world unless their own frame field says otherwise",
            },
        },
        "inputs": {key: {"path": rel, "present": (scene_dir / rel).exists()} for key, rel in sorted(INPUTS.items())},
        "robots": _sort_items(robots),
        "tools": _sort_items(tools),
        "assets": _sort_items(assets),
        "sensors": _sort_items(sensors),
        "zones": _sort_items(zones),
        "warnings": sorted(warnings, key=lambda w: (str(w.get("source", "")), str(w.get("code", "")), str(w.get("message", "")))),
        "backend_actions": [
            {
                "id": "validate",
                "label": "Validate",
                "enabled": True,
                "request_kind": "backend_request",
                "description": "Ask the Workcell Studio backend to validate the selected scene inputs.",
                "safety_note": "Validation is offline metadata checking and does not command robot motion.",
            },
            {
                "id": "generate_scene_package",
                "label": "Generate Scene Package",
                "enabled": True,
                "request_kind": "backend_request",
                "description": "Ask the backend to regenerate ROS 2 scene package artifacts from source-of-truth inputs.",
                "safety_note": "Generation must preserve fake-hardware-first launch defaults.",
            },
            {
                "id": "plan_simulate",
                "label": "Plan / Simulate",
                "enabled": False,
                "request_kind": "backend_request",
                "description": "Request RViz/MoveIt fake-hardware simulation through a guarded backend workflow.",
                "safety_note": "Disabled in this exporter; real hardware execution is not exposed by the web scene contract.",
            },
        ],
    }
    return output


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Export a deterministic Workcell Studio web scene JSON file.")
    parser.add_argument("--scene", required=True, help="Scene directory, for example scenes/ur5_2f_test")
    parser.add_argument("--output", required=True, help="Output JSON path, typically under build/")
    args = parser.parse_args(argv)

    scene_dir = Path(args.scene)
    output_path = Path(args.output)
    if not scene_dir.exists() or not scene_dir.is_dir():
        print(f"error: --scene must be an existing directory: {scene_dir}", file=sys.stderr)
        return 2

    payload = build_web_scene(scene_dir)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
