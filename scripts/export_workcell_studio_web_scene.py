#!/usr/bin/env python3
"""Export a dependency-light Workcell Studio scene description for web viewers.

This exporter is intentionally read-only with respect to scene inputs. It only
loads authoring/generated metadata files, normalizes the small subset needed by a
browser preview, and writes one deterministic JSON document to --output.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
from pathlib import Path
from urllib.parse import unquote, urlparse
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

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
SUPPORTED_MESH_SUFFIXES = {".stl", ".dae", ".obj"}
MESH_URI_FIELDS = ("mesh_uri", "package_uri", "mesh_path", "source_path", "resolved_source_path")
ROBOTIQ_85_VISUAL_MESHES = {
    "gripper_base_link": "robotiq_85_base_link.dae",
    "gripper_finger1_knuckle_link": "robotiq_85_knuckle_link.dae",
    "gripper_finger2_knuckle_link": "robotiq_85_knuckle_link.dae",
    "gripper_finger1_finger_link": "robotiq_85_finger_link.dae",
    "gripper_finger2_finger_link": "robotiq_85_finger_link.dae",
    "gripper_finger1_inner_knuckle_link": "robotiq_85_inner_knuckle_link.dae",
    "gripper_finger2_inner_knuckle_link": "robotiq_85_inner_knuckle_link.dae",
    "gripper_finger1_finger_tip_link": "robotiq_85_finger_tip_link.dae",
    "gripper_finger2_finger_tip_link": "robotiq_85_finger_tip_link.dae",
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
        resolved = p.resolve()
        if _is_relative_to(resolved, scene_dir):
            try:
                return os.path.relpath(resolved, scene_dir)
            except ValueError:
                return uri
        return uri
    return uri


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _safe_relative_parts(rel: Path) -> Optional[Tuple[str, ...]]:
    if rel.is_absolute():
        return None
    parts = rel.parts
    if not parts or any(part in ("", ".", "..") for part in parts):
        return None
    return parts


def _package_share_roots(repo_root: Path) -> List[Path]:
    roots = [
        repo_root / "assets",
        repo_root / "assets" / "robots",
        repo_root / "assets" / "environment",
        repo_root / "assets" / "sensors",
    ]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            roots.append(Path(prefix) / "share")
    roots.append(Path("/opt/ros/humble/share"))
    out: List[Path] = []
    seen = set()
    for root in roots:
        resolved = root.resolve()
        if str(resolved) not in seen:
            seen.add(str(resolved))
            out.append(resolved)
    return out


def _resolve_package_uri(uri: str, repo_root: Path) -> Tuple[Optional[Path], str, Optional[Path], Optional[str]]:
    parsed = urlparse(uri)
    package = parsed.netloc
    rel = Path(unquote(parsed.path).lstrip("/"))
    rel_parts = _safe_relative_parts(rel)
    if parsed.scheme != "package" or not package or rel_parts is None:
        return None, package or "package", None, f"Invalid or unsafe package URI: {uri}"
    if rel.suffix.lower() not in SUPPORTED_MESH_SUFFIXES:
        return None, package, None, f"Unsupported mesh format for {uri}; supported formats are .stl, .dae, and .obj."
    stage_rel = Path(package, *rel_parts)
    for root in _package_share_roots(repo_root):
        direct = (root / package / rel).resolve()
        if _is_relative_to(direct, root) and direct.is_file():
            return direct, package, stage_rel, None
        if root.exists():
            for pkg_dir in root.rglob(package):
                if not pkg_dir.is_dir():
                    continue
                candidate = (pkg_dir / rel).resolve()
                if _is_relative_to(candidate, root) and candidate.is_file():
                    return candidate, package, stage_rel, None
    return None, package, None, f"Could not resolve package mesh URI: {uri}"


def _resolve_local_mesh_uri(uri: str, scene_dir: Path, repo_root: Path) -> Tuple[Optional[Path], str, Optional[Path], Optional[str]]:
    source_root = "local"
    raw_path: Optional[Path] = None
    if uri.startswith("file://"):
        parsed = urlparse(uri)
        if parsed.netloc:
            return None, source_root, None, f"Unsupported file URI host in {uri}"
        raw_path = Path(unquote(parsed.path))
    elif "://" in uri:
        return None, source_root, None, f"Unsupported mesh URI scheme in {uri}"
    else:
        raw_path = Path(uri)

    if raw_path is None:
        return None, source_root, None, f"Invalid mesh path: {uri}"
    if raw_path.suffix.lower() not in SUPPORTED_MESH_SUFFIXES:
        return None, source_root, None, f"Unsupported mesh format for {uri}; supported formats are .stl, .dae, and .obj."
    if raw_path.is_absolute():
        resolved = raw_path.resolve()
        if not (_is_relative_to(resolved, repo_root) or _is_relative_to(resolved, scene_dir)):
            return None, source_root, None, f"Absolute mesh path outside allowed roots rejected: {uri}"
    else:
        rel_parts = _safe_relative_parts(raw_path)
        if rel_parts is None:
            return None, source_root, None, f"Unsafe relative mesh path rejected: {uri}"
        repo_candidate = (repo_root / raw_path).resolve()
        scene_candidate = (scene_dir / raw_path).resolve()
        resolved = repo_candidate if repo_candidate.is_file() else scene_candidate
    if not resolved.is_file():
        return None, source_root, None, f"Mesh file does not exist: {uri}"
    if _is_relative_to(resolved, repo_root):
        first = resolved.relative_to(repo_root).parts[0]
        source_root = first if first else "repo"
        stage_rel = resolved.relative_to(repo_root)
    else:
        source_root = "external_" + hashlib.sha1(str(resolved.parent).encode("utf-8")).hexdigest()[:12]
        stage_rel = Path(source_root, resolved.name)
    return resolved, source_root, stage_rel, None


def _mesh_candidates(item: Mapping[str, Any]) -> List[Tuple[str, str]]:
    candidates: List[Tuple[str, str]] = []
    seen = set()
    for field in MESH_URI_FIELDS:
        value = item.get(field)
        if isinstance(value, str) and value and value not in seen:
            candidates.append((field, value))
            seen.add(value)
    return candidates


def _staging_failure_status(warnings: Sequence[str]) -> str:
    text = " ".join(warnings).lower()
    if "unsupported mesh format" in text:
        return "unsupported_format"
    if "unsafe" in text or "escaped" in text:
        return "unsafe_path"
    if "unsupported" in text and ("scheme" in text or "file uri host" in text):
        return "unsupported_scheme"
    return "resolve_failed"


def _stage_visual_meshes(payload: Json, scene_dir: Path, output_path: Path) -> None:
    repo_root = Path.cwd().resolve()
    scene_id = str(payload.get("scene", {}).get("id") or scene_dir.name)
    asset_root = (repo_root / "build" / "workcell_studio_web_scene" / "assets" / scene_id).resolve()
    asset_root.mkdir(parents=True, exist_ok=True)
    sections: Sequence[str] = ("robots", "tools", "assets", "sensors", "zones")
    for section in sections:
        for item in payload.get(section, []):
            if not isinstance(item, dict):
                continue
            candidates = _mesh_candidates(item)
            original = candidates[0][1] if candidates else None
            item["original_mesh_uri"] = original
            item["mesh_staging_status"] = "no_mesh_uri"
            item["mesh_staged_path"] = None
            item["mesh_resolve_warning"] = None
            if not candidates:
                continue
            warnings: List[str] = []
            resolved: Optional[Path] = None
            source_root = "local"
            dest_rel: Optional[Path] = None
            for _field, uri in candidates:
                if uri.startswith("package://"):
                    resolved, source_root, dest_rel, warning = _resolve_package_uri(uri, repo_root)
                else:
                    resolved, source_root, dest_rel, warning = _resolve_local_mesh_uri(uri, scene_dir, repo_root)
                if resolved is not None:
                    break
                if warning:
                    warnings.append(warning)
            if resolved is None:
                item["mesh_staging_status"] = _staging_failure_status(warnings)
                item["mesh_resolve_warning"] = "; ".join(warnings) if warnings else "No mesh URI candidate could be resolved."
                continue
            if dest_rel is None and _is_relative_to(resolved, repo_root):
                rel_parts = resolved.relative_to(repo_root).parts
                dest_rel = Path(source_root, *rel_parts[1:]) if rel_parts and rel_parts[0] == source_root else Path(source_root, *rel_parts)
            elif dest_rel is None:
                dest_rel = Path(source_root, resolved.name)
            safe_parts = _safe_relative_parts(dest_rel)
            if safe_parts is None:
                item["mesh_staging_status"] = "unsafe_destination"
                item["mesh_resolve_warning"] = f"Unsafe staged mesh destination rejected for {resolved}"
                continue
            dest = (asset_root / Path(*safe_parts)).resolve()
            if not _is_relative_to(dest, asset_root):
                item["mesh_staging_status"] = "unsafe_destination"
                item["mesh_resolve_warning"] = f"Staged mesh destination escaped asset root: {dest}"
                continue
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(resolved, dest)
            rewritten = os.path.relpath(dest, repo_root).replace(os.sep, "/")
            item["mesh_uri"] = rewritten
            item["mesh_staging_status"] = "staged"
            item["mesh_staged_path"] = rewritten


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
        "mesh_uri", "source_path", "mesh_path", "resolved_source_path", "scale", "mesh_scale", "source_layer", "active_visual_source",
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


def _supplement_missing_tool_meshes(data: Dict[str, Any], generated: Dict[str, List[Json]]) -> None:
    """Add capability-described tool visuals when the flattened index omitted them.

    This is intentionally metadata-driven: the scene manifest/cell/environment says
    which tool profile and links are expected, and normal staging still resolves the
    package URIs.  It does not special-case a scene id or bake browser-only files.
    """
    existing_text = " ".join(
        str(item.get("id", "")) + " " + str(item.get("link", "")) + " " + str(item.get("mesh_uri", ""))
        for item in generated.get("tools", [])
    ).lower()
    if "robotiq_85" in existing_text or "gripper_base_link" in existing_text:
        return
    manifest = _as_map(data.get("scene_manifest"))
    env = _as_map(data.get("environment"))
    cell = _as_map(data.get("cell_definition"))
    tool_meta = _as_map(manifest.get("end_effector") or manifest.get("tool"))
    if not tool_meta:
        tool_meta = _as_map(env.get("tool") or env.get("end_effector"))
    if not tool_meta:
        tool_meta = _as_map(cell.get("end_effector") or cell.get("tool"))
    profile = " ".join(str(tool_meta.get(k, "")) for k in ("id", "model", "profile", "type")).lower()
    if "robotiq_85" not in profile:
        return
    expected_links = [str(v) for v in _as_list(tool_meta.get("visual_links"))]
    if not expected_links:
        expected_links = list(ROBOTIQ_85_VISUAL_MESHES)
    robot_items = generated.get("robots", [])
    wrist_pose = None
    for item in robot_items:
        if "wrist_3" in str(item.get("link", "")):
            wrist_pose = item.get("pose") or item.get("world_pose") or item.get("baked_world_visual_pose")
            break
    for link in expected_links:
        mesh_name = ROBOTIQ_85_VISUAL_MESHES.get(link)
        if not mesh_name:
            continue
        item: Json = {
            "id": f"generated_tool::{link}::visual_0",
            "type": "mesh",
            "category": "tool",
            "role": "gripper",
            "display_name": link,
            "link": link,
            "object_name": link,
            "visual": "visual_0",
            "pose": wrist_pose or {"xyz": [0, 0, 0], "rpy": [0, 0, 0]},
            "geometry_type": "mesh",
            "primitive_geometry_type": "mesh",
            "package_uri": f"package://robotiq_85_description/meshes/visual/{mesh_name}",
            "mesh_uri": f"package://robotiq_85_description/meshes/visual/{mesh_name}",
            "source_path": f"package://robotiq_85_description/meshes/visual/{mesh_name}",
            "mesh_path": f"assets/end_effectors/robotiq_85_gripper/robotiq_85_description/meshes/visual/{mesh_name}",
            "scale": [1, 1, 1],
            "mesh_scale": [1, 1, 1],
            "source_layer": "locked_generated_urdf_visual",
            "active_visual_source": "mesh_preview",
            "render_expected": True,
            "mesh_available": True,
            "resolved": True,
            "locked": True,
            "editable": False,
            "source_kind": "generated_preview",
            "provenance": _provenance(
                ("id", "link", "mesh_uri", "package_uri", "pose", "locked", "editable", "source_kind"),
                "scene_manifest.yaml|generated/scene_visual_mesh_index.json",
            ),
        }
        generated["tools"].append(item)


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


def _has_mesh_reference(item: Mapping[str, Any]) -> bool:
    return any(isinstance(item.get(field), str) and bool(str(item.get(field)).strip()) for field in MESH_URI_FIELDS)


def _drop_shadowed_metadata_primitives(items: List[Json], generated_items: List[Json], tokens: Sequence[str]) -> List[Json]:
    if not any(_has_mesh_reference(item) for item in generated_items):
        return items
    kept: List[Json] = []
    for item in items:
        if item.get("source_kind") == "generated_preview" or _has_mesh_reference(item):
            kept.append(item)
            continue
        text = _identity_text(item)
        if any(token in text for token in tokens):
            continue
        kept.append(item)
    return kept


def build_web_scene(scene_dir: Path, *, stage_assets: bool = False, output_path: Optional[Path] = None) -> Json:
    scene_dir = scene_dir.resolve()
    warnings: List[Json] = []
    data = _load_inputs(scene_dir, warnings)

    manifest = _as_map(data.get("scene_manifest"))
    scene_meta = _as_map(manifest.get("scene"))
    cell_scene = _as_map(_as_map(data.get("cell_definition")).get("cell"))
    env_scene = _as_map(_as_map(data.get("environment")).get("scene"))
    scene_name = _first_present(scene_meta.get("name"), cell_scene.get("name"), cell_scene.get("id"), env_scene.get("name"), env_scene.get("id"), scene_dir.name)

    generated = _generated_preview_items(_as_map(data.get("visual_mesh_index")), scene_dir, warnings) if data.get("visual_mesh_index") is not None else {"robots": [], "tools": [], "assets": [], "sensors": [], "zones": []}
    _supplement_missing_tool_meshes(data, generated)
    authored = _authored_sections(data, scene_dir, warnings)
    top_robots, top_tools, top_sensors = _top_level_entities(data, warnings)

    robots = _drop_shadowed_metadata_primitives(top_robots + generated["robots"], generated["robots"], ("robot", "ur5", "ur3", "ur10"))
    tools = _drop_shadowed_metadata_primitives(top_tools + generated["tools"], generated["tools"], ("tool", "gripper", "robotiq", "end_effector"))
    sensors = _drop_shadowed_metadata_primitives(top_sensors + authored["sensors"] + generated["sensors"], generated["sensors"], ("camera", "realsense", "sensor"))
    assets = _drop_shadowed_metadata_primitives(authored["assets"] + generated["assets"], generated["assets"], ("table", "workbench", "support_surface"))
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
    if stage_assets:
        _stage_visual_meshes(output, scene_dir, output_path or Path("build/workcell_studio_web_scene/scene.web_scene.json"))
    return output


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Export a deterministic Workcell Studio web scene JSON file.")
    parser.add_argument("--scene", required=True, help="Scene directory, for example scenes/ur5_2f_test")
    parser.add_argument("--output", required=True, help="Output JSON path, typically under build/")
    parser.add_argument("--stage-assets", dest="stage_assets", action="store_true", default=True, help="Copy resolvable mesh assets into build/workcell_studio_web_scene/assets/<scene_id> and rewrite mesh_uri for browser loading. Enabled by default.")
    parser.add_argument("--no-stage-assets", dest="stage_assets", action="store_false", help="Disable mesh asset staging and leave mesh URI fields unchanged.")
    args = parser.parse_args(argv)

    scene_dir = Path(args.scene)
    output_path = Path(args.output)
    if not scene_dir.exists() or not scene_dir.is_dir():
        print(f"error: --scene must be an existing directory: {scene_dir}", file=sys.stderr)
        return 2

    payload = build_web_scene(scene_dir, stage_assets=args.stage_assets, output_path=output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
