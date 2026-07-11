#!/usr/bin/env python3
"""Export a dependency-light Workcell Studio scene description for web viewers.

This exporter is intentionally read-only with respect to scene inputs. It only
loads authoring metadata plus generated preview cache files, normalizes the small
subset needed by a browser preview, and writes one deterministic JSON document to
--output.

``generated/scene_visual_mesh_index.json`` is treated as generated cache/build
output that the GUI/Web refresh path can regenerate automatically; it is not a
tracked source-of-truth scene file and should not be committed under
``scenes/*/generated/``.  Write web scene JSON exports under
``build/workcell_studio_web_scene/`` or another ignored output location, not as
committed scene artifacts.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import re
import copy
import xml.etree.ElementTree as ET
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
ROBOTIQ_85_FALLBACK_LINK_TRANSFORMS = {
    "gripper_base_link": {"parent_link": "tool0", "joint_origin": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}},
    "gripper_finger1_knuckle_link": {"parent_link": "gripper_base_link", "joint_origin": {"xyz": [0.05490451627, 0.03060114443, 0.0], "rpy": [3.141592653589793, 0.0, 0.0]}},
    "gripper_finger2_knuckle_link": {"parent_link": "gripper_base_link", "joint_origin": {"xyz": [0.05490451627, -0.03060114443, 0.0], "rpy": [0.0, 0.0, 0.0]}},
    "gripper_finger1_finger_link": {"parent_link": "gripper_finger1_knuckle_link", "joint_origin": {"xyz": [-0.00408552455, -0.03148604435, 0.0], "rpy": [0.0, 0.0, 0.0]}},
    "gripper_finger2_finger_link": {"parent_link": "gripper_finger2_knuckle_link", "joint_origin": {"xyz": [-0.00408552455, -0.03148604435, 0.0], "rpy": [0.0, 0.0, 0.0]}},
    "gripper_finger1_inner_knuckle_link": {"parent_link": "gripper_base_link", "joint_origin": {"xyz": [0.06142, 0.0127, 0.0], "rpy": [3.141592653589793, 0.0, 0.0]}},
    "gripper_finger2_inner_knuckle_link": {"parent_link": "gripper_base_link", "joint_origin": {"xyz": [0.06142, -0.0127, 0.0], "rpy": [0.0, 0.0, 0.0]}},
    "gripper_finger1_finger_tip_link": {"parent_link": "gripper_finger1_inner_knuckle_link", "joint_origin": {"xyz": [0.04303959807, -0.03759940821, 0.0], "rpy": [0.0, 0.0, 0.0]}},
    "gripper_finger2_finger_tip_link": {"parent_link": "gripper_finger2_inner_knuckle_link", "joint_origin": {"xyz": [0.04303959807, -0.03759940821, 0.0], "rpy": [0.0, 0.0, 0.0]}},
}
UR_VISUAL_MESH_TOKEN = "package://ur_description/meshes/"
ROBOTIQ_VISUAL_MESH_TOKEN = "package://robotiq_85_description/meshes/visual/"

DEFAULT_ROBOT_PREVIEW_JOINT_VALUES = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.5708,
    "elbow_joint": 1.5708,
    "wrist_1_joint": -1.5708,
    "wrist_2_joint": -1.5708,
    "wrist_3_joint": 0.0,
}
EXPECTED_ROBOT_PREVIEW_LINKS = [
    "base_link",
    "base_link_inertia",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "tool0",
    "gripper_base_link",
]

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
GENERATED_OUTPUT_SECTIONS = ("robots", "tools", "assets", "sensors", "zones", "frames")
RENDERABLE_OUTPUT_SECTIONS = ("robots", "tools", "assets", "sensors", "zones")

Json = Dict[str, Any]


class BlockingExportError(RuntimeError):
    """Raised when a canonical scene cannot be exported honestly."""


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




def _xml_attr(value: Any) -> str:
    return str(value if value is not None else "").replace("&", "&amp;").replace('"', "&quot;").replace("<", "&lt;").replace(">", "&gt;")


def _pose_xyz_rpy_text(pose: Mapping[str, Any]) -> Tuple[str, str]:
    xyz = pose.get("xyz") if isinstance(pose, Mapping) else None
    rpy = pose.get("rpy") if isinstance(pose, Mapping) else None
    xyz_vals = xyz if isinstance(xyz, list) and len(xyz) >= 3 else [0, 0, 0]
    rpy_vals = rpy if isinstance(rpy, list) and len(rpy) >= 3 else [0, 0, 0]
    return " ".join(str(float(v)) for v in xyz_vals[:3]), " ".join(str(float(v)) for v in rpy_vals[:3])


def _synthesize_robot_preview_urdf_from_rows(payload: Json) -> Optional[str]:
    rows: List[Json] = []
    for section in ("robots", "tools"):
        rows.extend([item for item in payload.get(section, []) if isinstance(item, dict) and item.get("mesh_uri")])
    if not rows:
        return None
    links = {"base_link"}
    joints: Dict[str, Tuple[str, Json]] = {}
    visuals: Dict[str, List[Json]] = {}
    for item in rows:
        link = str(item.get("link_name") or item.get("link") or item.get("object_name") or "").strip()
        if not link:
            continue
        parent = str(item.get("parent_link") or item.get("joint_parent_link") or item.get("immediate_parent_link") or "").strip()
        links.add(link)
        if parent:
            links.add(parent)
            joints.setdefault(link, (parent, item))
        visuals.setdefault(link, []).append(item)
    out = ['<?xml version="1.0" ?>', '<robot name="workcell_studio_preview_robot">']
    for link in sorted(links):
        out.append(f'  <link name="{_xml_attr(link)}">')
        for idx, item in enumerate(visuals.get(link, [])):
            xyz, rpy = _pose_xyz_rpy_text(_as_map(item.get("visual_origin")))
            mesh = _xml_attr(item.get("mesh_uri"))
            out.extend([
                f'    <visual name="visual_{idx}">',
                f'      <origin xyz="{xyz}" rpy="{rpy}"/>',
                '      <geometry>',
                f'        <mesh filename="{mesh}"/>',
                '      </geometry>',
                '    </visual>',
            ])
        out.append('  </link>')
    for child, (parent, item) in sorted(joints.items()):
        joint = str(item.get("joint_name") or item.get("parent_joint_name") or f"{parent}_to_{child}")
        jtype = str(item.get("joint_type") or item.get("parent_joint_type") or "fixed")
        xyz, rpy = _pose_xyz_rpy_text(_as_map(item.get("joint_origin") or item.get("parent_joint_origin") or item.get("parent_to_child_pose")))
        axis = item.get("joint_axis") if isinstance(item.get("joint_axis"), list) else [1, 0, 0]
        axis_text = " ".join(str(float(v)) for v in axis[:3])
        out.extend([
            f'  <joint name="{_xml_attr(joint)}" type="{_xml_attr(jtype)}">',
            f'    <parent link="{_xml_attr(parent)}"/>',
            f'    <child link="{_xml_attr(child)}"/>',
            f'    <origin xyz="{xyz}" rpy="{rpy}"/>',
            f'    <axis xyz="{axis_text}"/>',
            '  </joint>',
        ])
    out.append('</robot>')
    return "\n".join(out) + "\n"


def _child_text(element: ET.Element, tag: str, attr: str) -> str:
    child = element.find(tag)
    return child.get(attr, "") if child is not None else ""


def _extract_robot_preview_urdf(expanded_urdf_text: str, source: Path) -> str:
    """Extract the robot/tool branch from a full xacro-expanded scene URDF."""
    try:
        full_robot = ET.fromstring(expanded_urdf_text)
    except ET.ParseError as exc:
        raise BlockingExportError(f"failed to parse expanded scene URDF {source}: {exc}") from exc
    if full_robot.tag != "robot":
        raise BlockingExportError(f"expanded scene URDF {source} does not contain a <robot> root")

    links = {link.get("name", ""): link for link in full_robot.findall("link") if link.get("name")}
    joints = [joint for joint in full_robot.findall("joint") if joint.get("name")]
    if "base_link" not in links:
        raise BlockingExportError(f"expanded scene URDF {source} does not contain required robot base link 'base_link'")

    child_to_joint: Dict[str, ET.Element] = {}
    children_by_parent: Dict[str, List[ET.Element]] = {}
    for joint in joints:
        parent = _child_text(joint, "parent", "link")
        child = _child_text(joint, "child", "link")
        if not parent or not child:
            continue
        child_to_joint[child] = joint
        children_by_parent.setdefault(parent, []).append(joint)

    # Retain the exact fixed world/root-to-base chain, if the expanded scene has
    # one, then recurse downward from base_link only. This naturally excludes
    # table/camera/bin/zone sibling branches mounted under world.
    selected_links = {"base_link"}
    selected_joints: List[ET.Element] = []
    cursor = "base_link"
    seen = set()
    while cursor in child_to_joint and cursor not in seen:
        seen.add(cursor)
        joint = child_to_joint[cursor]
        parent = _child_text(joint, "parent", "link")
        if not parent:
            break
        if joint.get("type", "fixed") != "fixed":
            raise BlockingExportError(
                f"expanded scene URDF {source} has non-fixed root-to-base joint "
                f"{joint.get('name')} ({parent} -> {cursor}); robot preview requires an exact fixed world/root-to-base transform"
            )
        selected_links.add(parent)
        selected_joints.insert(0, joint)
        cursor = parent

    stack = ["base_link"]
    while stack:
        parent = stack.pop()
        for joint in sorted(children_by_parent.get(parent, []), key=lambda j: j.get("name", "")):
            child = _child_text(joint, "child", "link")
            if not child or child in selected_links:
                continue
            selected_joints.append(joint)
            selected_links.add(child)
            stack.append(child)

    required = {"base_link", "tool0", "gripper_base_link"}
    missing = sorted(link for link in required if link not in selected_links)
    if missing:
        raise BlockingExportError(
            f"expanded scene URDF {source} robot-preview extraction is missing required robot/tool links: {', '.join(missing)}"
        )

    material_names = {
        material.get("name", "")
        for link_name in selected_links
        for material in links[link_name].findall(".//material")
        if material.get("name")
    }
    out = ET.Element("robot", {"name": f"{full_robot.get('name', 'workcell')}_robot_preview"})
    for material in full_robot.findall("material"):
        if material.get("name") in material_names:
            out.append(copy.deepcopy(material))
    for link_name in sorted(selected_links):
        out.append(copy.deepcopy(links[link_name]))
    for joint in selected_joints:
        out.append(copy.deepcopy(joint))
    ET.indent(out, space="  ")
    return ET.tostring(out, encoding="unicode", xml_declaration=True) + "\n"


def _stage_expanded_robot_urdf(payload: Json, scene_dir: Path, output_path: Path, warnings: List[Json]) -> None:
    """Stage a browser robot-preview URDF extracted from the xacro-expanded scene URDF."""
    data = _as_map(payload.get("_visual_mesh_index_source"))
    rel = str(data.get("source_expanded_urdf_path") or "generated/expanded_scene_preview.urdf")
    source = (Path.cwd() / rel).resolve() if rel and not Path(rel).is_absolute() else Path(rel).resolve()
    if not source.is_file():
        source = scene_dir / "generated" / "expanded_scene_preview.urdf"
    synthesized_text = None
    scene_id = str(payload.get("scene", {}).get("id") or scene_dir.name)
    canonical_scene_requires_real_expanded_urdf = scene_id == "ur5_2f_test"
    if not source.is_file():
        if canonical_scene_requires_real_expanded_urdf:
            raise BlockingExportError(
                f"blocking export error: {scene_id} requires real xacro-expanded scene URDF at "
                f"{scene_dir / 'generated' / 'expanded_scene_preview.urdf'}; run the visual mesh index extractor with real xacro expansion first. "
                "Refusing to synthesize robot preview from flattened mesh rows."
            )
        synthesized_text = _synthesize_robot_preview_urdf_from_rows(payload)
        if not synthesized_text:
            _warn(warnings, "expanded_robot_urdf_missing", "Expanded robot URDF was not available for browser robot preview; legacy rows remain as fallback metadata only.", rel)
            return
    repo_root = Path.cwd().resolve()
    dest = (repo_root / "build" / "workcell_studio_web_scene" / f"{scene_id}.robot_preview.urdf").resolve()
    dest.parent.mkdir(parents=True, exist_ok=True)
    if synthesized_text is not None:
        text = synthesized_text
        preview_mode = "legacy_flattened_rows_robot_preview"
        source_mode = "legacy_flattened_rows_robot_preview"
        rviz_parity = False
    else:
        try:
            text = _extract_robot_preview_urdf(source.read_text(encoding="utf-8"), source)
        except BlockingExportError:
            if canonical_scene_requires_real_expanded_urdf:
                raise
            _warn(warnings, "robot_preview_extraction_failed", "Could not extract robot subtree from expanded URDF; using temporary legacy flattened-row fallback without RViz parity.", str(source))
            synthesized_text = _synthesize_robot_preview_urdf_from_rows(payload)
            if not synthesized_text:
                return
            text = synthesized_text
            preview_mode = "legacy_flattened_rows_robot_preview"
            source_mode = "legacy_flattened_rows_robot_preview"
            rviz_parity = False
        else:
            preview_mode = "expanded_urdf_loader"
            source_mode = "expanded_urdf_robot_subtree"
            rviz_parity = True
    asset_root = (repo_root / "build" / "workcell_studio_web_scene" / "assets" / scene_id).resolve()
    asset_root.mkdir(parents=True, exist_ok=True)

    def rewrite(match: re.Match[str]) -> str:
        uri = match.group(0)
        resolved, package, dest_rel, warning = _resolve_package_uri(uri, repo_root)
        if resolved is None or dest_rel is None:
            _warn(warnings, "expanded_robot_urdf_mesh_unresolved", warning or f"Could not resolve package mesh URI: {uri}", uri)
            return uri
        target = (asset_root / dest_rel).resolve()
        if not _is_relative_to(target, asset_root):
            _warn(warnings, "expanded_robot_urdf_mesh_unsafe", f"Staged URDF mesh destination escaped asset root: {target}", uri)
            return uri
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(resolved, target)
        return os.path.relpath(target, repo_root).replace(os.sep, "/")

    text = re.sub(r"package://[^\s'\"<>]+", rewrite, text)
    dest.write_text(text, encoding="utf-8")
    payload["robot_preview"] = {
        "mode": preview_mode,
        "source_mode": source_mode,
        "urdf_url": os.path.relpath(dest, repo_root).replace(os.sep, "/"),
        "robot_root_link": "base_link",
        "rviz_parity": rviz_parity,
        "joint_values": dict(DEFAULT_ROBOT_PREVIEW_JOINT_VALUES),
        "expected_links": list(EXPECTED_ROBOT_PREVIEW_LINKS),
    }

def _stage_visual_meshes(payload: Json, scene_dir: Path, output_path: Path) -> None:
    repo_root = Path.cwd().resolve()
    scene_id = str(payload.get("scene", {}).get("id") or scene_dir.name)
    asset_root = (repo_root / "build" / "workcell_studio_web_scene" / "assets" / scene_id).resolve()
    asset_root.mkdir(parents=True, exist_ok=True)
    sections: Sequence[str] = RENDERABLE_OUTPUT_SECTIONS
    for section in sections:
        for item in payload.get(section, []):
            if not isinstance(item, dict):
                continue
            candidates = _mesh_candidates(item)
            original = candidates[0][1] if candidates else None
            item["original_mesh_uri"] = original
            item["original_package_uri"] = next((uri for _field, uri in candidates if uri.startswith("package://")), original if isinstance(original, str) and original.startswith("package://") else None)
            item["original_source_path"] = item.get("source_path") or item.get("mesh_path") or original
            item["mesh_format"] = _mesh_format_from_uri(original or item.get("mesh_uri") or item.get("mesh_path") or item.get("source_path"))
            item["mesh_staging_status"] = "no_mesh_uri"
            item["mesh_staged_path"] = None
            item["mesh_url"] = None
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
            item["resolved_source_path"] = os.path.relpath(resolved, repo_root).replace(os.sep, "/") if _is_relative_to(resolved, repo_root) else str(resolved)
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
            item["mesh_url"] = rewritten


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


def _is_transform_anchor(item: Mapping[str, Any]) -> bool:
    return str(item.get("type", "")).lower() == "transform_anchor" or str(item.get("role", "")).lower() == "transform_anchor"


def _mesh_uri_values(item: Mapping[str, Any]) -> List[str]:
    return [str(item.get(field)) for field in MESH_URI_FIELDS if isinstance(item.get(field), str) and str(item.get(field))]


def _has_ur_visual_mesh_reference(item: Mapping[str, Any]) -> bool:
    return any(UR_VISUAL_MESH_TOKEN in value and "/visual/" in value for value in _mesh_uri_values(item))


def _has_robotiq_visual_mesh_reference(item: Mapping[str, Any]) -> bool:
    return any(ROBOTIQ_VISUAL_MESH_TOKEN in value for value in _mesh_uri_values(item))


def _normalize_generated_urdf_mesh_preview_item(item: Json) -> None:
    """Classify generated URDF mesh rows before direct web export bucketing.

    ``ensure_workcell_studio_web_scene_fresh.py`` also normalizes the generated
    visual mesh index on disk, but callers of this exporter must not depend on
    that refresh wrapper.  Normalize the copied row in memory so direct
    ``export_workcell_studio_web_scene.py --no-stage-assets`` exports preserve
    UR robot links, Robotiq tool links, and the meshless ``tool0`` anchor
    contract even when the extractor emitted plain mesh rows with no role or
    category.
    """
    link = str(item.get("link") or item.get("link_name") or item.get("object_name") or item.get("frame") or "")
    if link:
        item.setdefault("link", link)
        item.setdefault("link_name", link)

    if _is_transform_anchor(item) and link == "tool0":
        item["id"] = "urdf_frame_anchor_tool0"
        item["type"] = "transform_anchor"
        item["role"] = "transform_anchor"
        item["category"] = "frame"
        item["render_expected"] = False
        item["mesh_available"] = False
        item["mesh_load_required"] = False
        item["meshless_frame"] = True
        item["active_visual_source"] = "frame_anchor"
        for field in MESH_URI_FIELDS:
            item.pop(field, None)
        return

    if _has_ur_visual_mesh_reference(item):
        item["role"] = "robot"
        item["category"] = "robot_static_mesh_visual"
        item["source_layer"] = "locked_generated_urdf_visual"
        item["active_visual_source"] = "mesh_preview"
        item["primitive_geometry_type"] = "mesh"
        item["geometry_type"] = "mesh"
        item["mesh_available"] = True
        item["render_expected"] = True
        item["primitive_fallback"] = False
        item["fallback_reason"] = ""
        return

    if _has_robotiq_visual_mesh_reference(item):
        item["role"] = "gripper"
        item["category"] = "tool"
        item["source_layer"] = "locked_generated_urdf_visual"
        item["active_visual_source"] = "mesh_preview"
        item["primitive_geometry_type"] = "mesh"
        item["geometry_type"] = "mesh"
        item["mesh_available"] = True
        item["render_expected"] = True
        item["primitive_fallback"] = False
        item["fallback_reason"] = ""


def _section_from_item(item: Mapping[str, Any]) -> str:
    if _is_transform_anchor(item):
        return "frames"
    text = _identity_text(item)
    category = str(item.get("category", "")).lower()
    role = str(item.get("role", "")).lower()
    if _is_helper(item):
        return "zones"
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
    return "assets"


def _mesh_unit_evidence_from_path(path: Path) -> Optional[Json]:
    """Return non-rendering mesh unit evidence for exporter diagnostics.

    COLLADA files can declare their authored linear unit in ``asset/unit``.
    The D435 asset used by the canonical scene declares ``meter=1`` and has
    vertex coordinates in the expected physical camera range, so the exporter
    records that evidence rather than applying an implicit scale.
    """
    if path.suffix.lower() != ".dae" or not path.is_file():
        return None
    try:
        root = ET.parse(path).getroot()
    except Exception:  # noqa: BLE001 - unit evidence is diagnostic only
        return None
    unit = None
    for elem in root.iter():
        if elem.tag.rsplit("}", 1)[-1] == "unit":
            unit = elem
            break
    if unit is None:
        return {"source": "collada_asset_unit", "declared_unit": "unspecified", "unit_scale_to_m": None}
    meter_raw = unit.attrib.get("meter")
    try:
        meter = float(meter_raw) if meter_raw is not None else None
    except ValueError:
        meter = None
    name = unit.attrib.get("name") or "unspecified"
    if meter == 1.0:
        interpretation = "mesh_vertices_already_meters_no_exporter_unit_conversion_applied"
    elif meter == 0.01:
        interpretation = "mesh_vertices_declared_centimeters"
    elif meter == 0.001:
        interpretation = "mesh_vertices_declared_millimeters"
    else:
        interpretation = "mesh_vertices_declared_custom_or_unknown_unit"
    return {
        "source": "collada_asset_unit",
        "declared_unit": name,
        "unit_scale_to_m": meter,
        "interpretation": interpretation,
    }


def _annotate_mesh_unit_evidence(item: Json, scene_dir: Path) -> None:
    if item.get("mesh_unit_evidence"):
        return
    repo_root = Path.cwd().resolve()
    for _field, uri in _mesh_candidates(item):
        resolved: Optional[Path]
        if uri.startswith("package://"):
            resolved, _source_root, _dest_rel, _warning = _resolve_package_uri(uri, repo_root)
        else:
            resolved, _source_root, _dest_rel, _warning = _resolve_local_mesh_uri(uri, scene_dir, repo_root)
        if resolved is None:
            continue
        evidence = _mesh_unit_evidence_from_path(resolved)
        if evidence:
            evidence["source_path"] = os.path.relpath(resolved, repo_root).replace(os.sep, "/") if _is_relative_to(resolved, repo_root) else str(resolved)
            item["mesh_unit_evidence"] = evidence
            item.setdefault("mesh_unit_conversion", {"applied": False, "scale": [1.0, 1.0, 1.0], "reason": evidence.get("interpretation")})
            return


def _canonical_generated_transform(raw: Mapping[str, Any]) -> Tuple[Optional[Any], Optional[str], Optional[str]]:
    """Return the browser-ready world-from-visual pose and its source field.

    Generated mesh rows may carry link-frame, visual-origin, and already-baked
    world visual transforms.  The web viewer consumes the baked transform as a
    final render pose; it must not multiply the visual origin a second time.
    """
    baked_source = raw.get("baked_world_visual_transform_source")
    # Prefer the already-baked visible mesh world pose when present.  Older
    # generated indexes may also carry link-frame final_transform/world_from_visual
    # values, and using those first makes the browser re-apply visual_origin and
    # explode generated URDF meshes.
    for field in ("urdf_fk_visual_world_pose", "baked_world_visual_pose", "expected_visual_pose", "final_transform", "world_from_visual", "pose", "world_pose"):
        value = raw.get(field)
        if value not in (None, "", [], {}):
            return value, str(baked_source or field), field
    return None, None, None


def _pose_with_xyz_offset(base_pose: Any, offset: Sequence[float]) -> Json:
    pose = dict(base_pose) if isinstance(base_pose, Mapping) else {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
    xyz = pose.get("xyz") if isinstance(pose.get("xyz"), list) else [0.0, 0.0, 0.0]
    rpy = pose.get("rpy") if isinstance(pose.get("rpy"), list) else [0.0, 0.0, 0.0]
    pose["xyz"] = [float(xyz[i] if i < len(xyz) else 0.0) + float(offset[i] if i < len(offset) else 0.0) for i in range(3)]
    pose["rpy"] = [float(rpy[i] if i < len(rpy) else 0.0) for i in range(3)]
    return pose


def _generated_preview_items(index: Mapping[str, Any], scene_dir: Path, warnings: List[Json]) -> Dict[str, List[Json]]:
    sections = {section: [] for section in GENERATED_OUTPUT_SECTIONS}
    items = _as_list(index.get("visual_items") or index.get("items"))
    if not items:
        _warn(warnings, "visual_mesh_index_items_missing", "Visual mesh index has no visual_items/items list.", INPUTS["visual_mesh_index"])
        return sections
    fields = (
        "id", "type", "category", "role", "display_name", "link", "object_name", "visual", "pose", "world_pose",
        "final_transform", "world_from_visual", "transform_source",
        "urdf_fk_source", "urdf_fk_world_pose", "urdf_fk_link_world_pose", "urdf_fk_visual_world_pose", "urdf_fk_verified_against_ros_tf",
        "urdf_joint_parent", "urdf_joint_child", "urdf_joint_origin", "urdf_visual_origin",
        "baked_world_visual_pose", "expected_visual_pose", "link_world_pose", "frame_world_pose", "visual_origin", "baked_world_visual_matrix",
        "baked_world_visual_quaternion", "baked_world_visual_transform_source", "geometry_type",
        "parent_link", "immediate_parent_link", "root_link", "link_chain", "joint_parent_link",
        "parent_joint", "parent_joint_name", "parent_joint_type", "parent_joint_origin",
        "parent_joint_axis", "parent_joint_value", "parent_joint_value_source", "joint_type",
        "joint_origin", "joint_name", "joint_value", "applied_joint_value", "joint_axis",
        "joint_value_source", "applied_joint_value_source", "transform_chain",
        "primitive_geometry_type", "package_uri", "mesh_uri", "source_path", "mesh_path",
        "resolved_source_path", "scale", "mesh_scale", "source_layer", "active_visual_source",
        "mesh_bounds", "local_bounds", "bounds", "dimensions", "size",
        "support_surface_kind", "support_kind", "semantic_type", "top_surface_z_m", "topSurfaceZM", "support_surface_height_m", "supportSurfaceHeightM",
        "expected_support_footprint_m", "support_footprint_m", "footprint_m", "footprint",
        "render_expected", "mesh_available", "resolved", "warning",
    )
    for i, raw in enumerate(items):
        if not isinstance(raw, Mapping):
            _warn(warnings, "visual_mesh_index_item_invalid", f"Skipping non-object visual index item at offset {i}.", INPUTS["visual_mesh_index"])
            continue
        item = _copy_fields(raw, fields, INPUTS["visual_mesh_index"], scene_dir)
        item.setdefault("id", _stable_id("generated_preview", i))
        final_transform, transform_source, transform_field = _canonical_generated_transform(raw)
        if final_transform is not None:
            item["final_transform"] = final_transform
            item["world_from_visual"] = final_transform
            item["transform_source"] = transform_source
            if transform_field in {"urdf_fk_visual_world_pose", "baked_world_visual_pose", "expected_visual_pose"}:
                item["workcell_web_render_pose_mode"] = "baked_visible_world_pose"
                item["visual_origin_application"] = "baked_into_web_preview_pose"
                if "link_world_pose" in raw:
                    item["original_link_world_pose"] = raw.get("link_world_pose")
                if "frame_world_pose" in raw:
                    item["original_frame_world_pose"] = raw.get("frame_world_pose")
        elif raw.get("baked_world_visual_transform_source"):
            item["transform_source"] = raw.get("baked_world_visual_transform_source")
        item["locked"] = True
        item["editable"] = False
        item["source_kind"] = "generated_preview"
        item["provenance"].update({"locked": INPUTS["visual_mesh_index"], "editable": INPUTS["visual_mesh_index"], "source_kind": INPUTS["visual_mesh_index"]})
        _normalize_generated_urdf_mesh_preview_item(item)
        if _is_transform_anchor(raw):
            item["render_expected"] = False
            item["mesh_available"] = False
            item["mesh_load_required"] = False
            item.setdefault("active_visual_source", "frame_anchor")
            item["provenance"].update({
                "render_expected": INPUTS["visual_mesh_index"],
                "mesh_available": INPUTS["visual_mesh_index"],
                "mesh_load_required": INPUTS["visual_mesh_index"],
                "active_visual_source": INPUTS["visual_mesh_index"],
            })
        if final_transform is not None:
            item["provenance"].update({
                "final_transform": INPUTS["visual_mesh_index"],
                "world_from_visual": INPUTS["visual_mesh_index"],
                "transform_source": INPUTS["visual_mesh_index"],
            })
            if transform_field in {"urdf_fk_visual_world_pose", "baked_world_visual_pose", "expected_visual_pose"}:
                item["provenance"].update({
                    "workcell_web_render_pose_mode": INPUTS["visual_mesh_index"],
                    "visual_origin_application": INPUTS["visual_mesh_index"],
                })
                if "original_link_world_pose" in item:
                    item["provenance"]["original_link_world_pose"] = INPUTS["visual_mesh_index"]
                if "original_frame_world_pose" in item:
                    item["provenance"]["original_frame_world_pose"] = INPUTS["visual_mesh_index"]
        if any(token in _identity_text(raw) for token in ("camera", "realsense", "d435")):
            _annotate_mesh_unit_evidence(item, scene_dir)
        sections[_section_from_item(item)].append(item)
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
    def _frame_pose(item: Mapping[str, Any]) -> Optional[Json]:
        """Return explicit link/frame world pose metadata, never render poses."""
        for field in ("link_world_pose", "frame_world_pose"):
            value = item.get(field)
            if isinstance(value, Mapping):
                return dict(value)
        return None

    def _link_name(item: Mapping[str, Any]) -> str:
        return str(item.get("link") or item.get("link_name") or item.get("frame") or item.get("object_name") or "")

    def _is_non_rendered_anchor(item: Mapping[str, Any]) -> bool:
        if item.get("render_expected") is False:
            return True
        if item.get("mesh_available") is False or item.get("resolved") is False:
            return True
        if not any(item.get(field) for field in MESH_URI_FIELDS):
            geometry = str(item.get("geometry_type") or item.get("primitive_geometry_type") or "").lower()
            return geometry in {"", "frame", "anchor", "none"}
        return False

    preview_items = [
        item
        for section in GENERATED_OUTPUT_SECTIONS
        for item in generated.get(section, [])
        if isinstance(item, Mapping)
    ]
    link_poses: Dict[str, Json] = {}
    for item in preview_items:
        link = _link_name(item)
        pose = _frame_pose(item)
        if link and pose is not None and (link not in link_poses or _is_non_rendered_anchor(item)):
            link_poses[link] = pose
    tool0_anchor_pose = next(
        (
            pose
            for item in preview_items
            if _link_name(item) == "tool0" and _is_non_rendered_anchor(item)
            for pose in [_frame_pose(item)]
            if pose is not None
        ),
        None,
    )
    if tool0_anchor_pose is not None:
        link_poses["tool0"] = tool0_anchor_pose
    elif "tool0" not in link_poses:
        link_poses["tool0"] = {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}
    for link in expected_links:
        mesh_name = ROBOTIQ_85_VISUAL_MESHES.get(link)
        if not mesh_name:
            continue
        transform_meta = ROBOTIQ_85_FALLBACK_LINK_TRANSFORMS.get(link, {})
        joint_origin = _as_map(transform_meta.get("joint_origin"))
        parent_link = str(transform_meta.get("parent_link") or "tool0")
        parent_pose = link_poses.get(parent_link) or link_poses["tool0"]
        link_world_pose = _pose_with_xyz_offset(parent_pose, _as_list(joint_origin.get("xyz") or [0.0, 0.0, 0.0]))
        final_transform = dict(link_world_pose)
        link_poses[link] = link_world_pose
        item: Json = {
            "id": f"generated_tool::{link}::visual_0",
            "type": "mesh",
            "category": "tool",
            "role": "gripper",
            "display_name": link,
            "link": link,
            "object_name": link,
            "visual": "visual_0",
            "pose": final_transform,
            "world_pose": final_transform,
            "final_transform": final_transform,
            "world_from_visual": final_transform,
            "link_world_pose": link_world_pose,
            "frame_world_pose": link_world_pose,
            "parent_link": parent_link,
            "joint_parent_link": parent_link,
            "joint_origin": joint_origin,
            "parent_joint_origin": joint_origin,
            "visual_origin": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "transform_source": "robotiq_85_fallback_link_metadata",
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
                ("id", "link", "mesh_uri", "package_uri", "pose", "world_pose", "final_transform",
                 "world_from_visual", "link_world_pose", "frame_world_pose", "parent_link", "joint_origin",
                 "visual_origin", "transform_source", "locked", "editable", "source_kind"),
                "scene_manifest.yaml|generated/scene_visual_mesh_index.json",
            ),
        }
        generated["tools"].append(item)


def _pose_xyz(pose: Any) -> Optional[List[float]]:
    if not isinstance(pose, Mapping):
        return None
    xyz = pose.get("xyz")
    if not isinstance(xyz, list) or len(xyz) < 3:
        return None
    try:
        return [float(xyz[0]), float(xyz[1]), float(xyz[2])]
    except (TypeError, ValueError):
        return None


def _set_item_pose(item: Json, pose: Json, source: str) -> None:
    for field in ("pose", "world_pose", "final_transform", "world_from_visual", "link_world_pose", "frame_world_pose", "baked_world_visual_pose"):
        if field in item or field in {"pose", "world_pose", "final_transform", "world_from_visual", "frame_world_pose"}:
            item[field] = dict(pose)
            item.setdefault("provenance", {})[field] = source
    item["transform_source"] = source
    item.setdefault("provenance", {})["transform_source"] = source


def _apply_web_scene_transform_parity_fallbacks(data: Dict[str, Any], generated: Dict[str, List[Json]], warnings: List[Json]) -> None:
    """Keep browser export transforms plausible when flattened URDF metadata is stale.

    Some generated static visual indexes include correct mesh identity but leave
    non-arm fixed descendants (tool/camera) at their local origin.  Product View
    should still consume canonical world-space final transforms, so this exporter
    applies conservative scene-metadata fallbacks only when the generated mesh
    pose is effectively collapsed at the world origin.
    """
    generated_items = [
        item
        for section in GENERATED_OUTPUT_SECTIONS
        for item in generated.get(section, [])
        if isinstance(item, Mapping)
    ]

    def _link_key(item: Mapping[str, Any]) -> str:
        return str(item.get("link") or item.get("link_name") or item.get("frame") or item.get("object_name") or "")

    def _frame_pose(item: Mapping[str, Any], *, prefer_frame: bool = False) -> Optional[Json]:
        # Parent-frame composition must use explicit frame/link world metadata only.
        # Render-only visual poses such as final_transform/world_from_visual/pose/
        # baked_world_visual_pose may include visual_origin and must not affect
        # gripper attachment.
        fields = ("frame_world_pose", "link_world_pose") if prefer_frame else ("link_world_pose", "frame_world_pose")
        for field in fields:
            value = item.get(field)
            if isinstance(value, Mapping):
                return dict(value)
        return None

    frame_pose_by_link: Dict[str, Json] = {}
    for item in generated_items:
        link = _link_key(item)
        if not link:
            continue
        pose = _frame_pose(item, prefer_frame=(link == "tool0"))
        if pose is not None:
            frame_pose_by_link[link] = pose

    tool_parent_pose = frame_pose_by_link.get("tool0")
    transform_source = "web_export_tool0_frame_transform_parity_fallback"
    tool_parent_xyz = _pose_xyz(tool_parent_pose)
    tool0_collapsed_at_origin = tool_parent_xyz is not None and sum(v * v for v in tool_parent_xyz) < 0.05
    if (tool_parent_pose is None or tool0_collapsed_at_origin) and "wrist_3_link" in frame_pose_by_link:
        tool_parent_pose = frame_pose_by_link["wrist_3_link"]
        transform_source = "web_export_wrist_3_link_link_pose_transform_parity_fallback"
        _warn(
            warnings,
            "tool0_frame_missing_or_collapsed_using_wrist_3_link_fallback",
            "tool0 frame/link world pose metadata is missing or collapsed near the world origin in generated/scene_visual_mesh_index.json; "
            "using wrist_3_link.link_world_pose only as a temporary gripper parent fallback. "
            "Regenerate the scene visual mesh index so tool0.frame_world_pose or tool0.link_world_pose is available and non-collapsed.",
            INPUTS["visual_mesh_index"],
        )

    parent_xyz = _pose_xyz(tool_parent_pose)
    if parent_xyz is not None:
        for item in generated.get("frames", []):
            if _link_key(item) == "tool0":
                pose = item.get("final_transform") or item.get("world_from_visual") or item.get("pose") or item.get("frame_world_pose")
                xyz = _pose_xyz(pose)
                if xyz is None or sum(v * v for v in xyz) < 0.05:
                    adjusted = dict(tool_parent_pose) if isinstance(tool_parent_pose, Mapping) else {"xyz": parent_xyz, "rpy": [0.0, 0.0, 0.0]}
                    _set_item_pose(item, adjusted, transform_source)
        for item in generated.get("tools", []):
            pose = item.get("final_transform") or item.get("world_from_visual") or item.get("pose")
            xyz = _pose_xyz(pose)
            chain = " ".join(str(v) for v in _as_list(item.get("transform_chain")))
            parent_link = str(item.get("joint_parent_link") or item.get("parent_link") or "")
            if xyz is not None and sum(v * v for v in xyz) < 0.05 and ("wrist_3" in chain or "tool0" in chain or parent_link in {"tool0", "wrist_3_link"}):
                adjusted = dict(pose) if isinstance(pose, Mapping) else {"rpy": [0.0, 0.0, 0.0]}
                adjusted["xyz"] = [parent_xyz[i] + xyz[i] for i in range(3)]
                _set_item_pose(item, adjusted, transform_source)

    env = _as_map(data.get("environment"))
    authored_camera_pose = None
    for raw in _as_list(_as_map(env.get("environment")).get("assets")) + _as_list(_as_map(env.get("environment")).get("sensors")):
        if not isinstance(raw, Mapping):
            continue
        text = _identity_text(raw)
        if "camera" in text or "realsense" in text:
            xyz = raw.get("pose_xyz")
            if isinstance(xyz, list) and len(xyz) >= 3:
                rpy = (raw.get("pose_rpy") or [0.0, 0.0, 0.0])[:3]
                authored_camera_pose = {
                    "xyz": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
                    "rpy": [float(v) for v in rpy],
                }
                break
    if authored_camera_pose is not None:
        for item in generated.get("sensors", []):
            pose = item.get("final_transform") or item.get("world_from_visual") or item.get("pose")
            xyz = _pose_xyz(pose)
            if xyz is not None and sum(v * v for v in xyz) < 0.05 and ("camera" in _identity_text(item) or "realsense" in _identity_text(item)):
                _set_item_pose(item, authored_camera_pose, "web_export_authored_camera_transform_fallback")


def _rpy_xyz_matrix(rpy: Sequence[float]) -> List[List[float]]:
    import math
    r = float(rpy[0] if len(rpy) > 0 else 0.0)
    p = float(rpy[1] if len(rpy) > 1 else 0.0)
    y = float(rpy[2] if len(rpy) > 2 else 0.0)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    # Match THREE.Euler(..., 'XYZ') composition used by the browser: Rz * Ry * Rx.
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matrix_transpose(m: Sequence[Sequence[float]]) -> List[List[float]]:
    return [[float(m[j][i]) for j in range(3)] for i in range(3)]


def _matrix_multiply(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> List[List[float]]:
    return [[sum(float(a[i][k]) * float(b[k][j]) for k in range(3)) for j in range(3)] for i in range(3)]


def _matrix_vec_multiply(m: Sequence[Sequence[float]], v: Sequence[float]) -> List[float]:
    return [sum(float(m[i][k]) * float(v[k]) for k in range(3)) for i in range(3)]


def _rpy_from_xyz_matrix(m: Sequence[Sequence[float]]) -> List[float]:
    import math
    # Inverse of the Rz * Ry * Rx matrix above for THREE Euler order XYZ.
    sy = -float(m[2][0])
    if abs(sy) < 0.999999:
        pitch = math.asin(sy)
        roll = math.atan2(float(m[2][1]), float(m[2][2]))
        yaw = math.atan2(float(m[1][0]), float(m[0][0]))
    else:
        pitch = math.copysign(math.pi / 2.0, sy)
        roll = math.atan2(-float(m[1][2]), float(m[1][1]))
        yaw = 0.0
    return [roll, pitch, yaw]


def _parent_to_child_pose(parent_world_pose: Any, child_world_pose: Any) -> Optional[Json]:
    parent_xyz = _pose_xyz(parent_world_pose)
    child_xyz = _pose_xyz(child_world_pose)
    if parent_xyz is None or child_xyz is None:
        return None
    parent_rpy = _as_list(_as_map(parent_world_pose).get("rpy") or [0.0, 0.0, 0.0])
    child_rpy = _as_list(_as_map(child_world_pose).get("rpy") or [0.0, 0.0, 0.0])
    try:
        parent_rot = _rpy_xyz_matrix([float(v) for v in parent_rpy[:3]])
        child_rot = _rpy_xyz_matrix([float(v) for v in child_rpy[:3]])
        parent_inv = _matrix_transpose(parent_rot)
        delta = [child_xyz[i] - parent_xyz[i] for i in range(3)]
        local_xyz = _matrix_vec_multiply(parent_inv, delta)
        local_rot = _matrix_multiply(parent_inv, child_rot)
        local_rpy = _rpy_from_xyz_matrix(local_rot)
    except Exception:  # noqa: BLE001 - diagnostics should not break export
        return None
    return {"xyz": [float(v) for v in local_xyz], "rpy": [float(v) for v in local_rpy]}


def _annotate_parent_to_child_poses(generated: Dict[str, List[Json]]) -> None:
    """Store explicit parent-to-child local poses for browser hierarchy rendering.

    The browser attaches each child link under its parent Object3D, so the local
    transform must be parent_world^-1 * child_world (parent-to-child), not the
    ambiguous legacy parent_from_child naming and not a stale raw joint origin.
    """
    items = [
        item
        for section in GENERATED_OUTPUT_SECTIONS
        for item in generated.get(section, [])
        if isinstance(item, dict)
    ]
    by_link: Dict[str, Json] = {}
    for item in items:
        link = str(item.get("link_name") or item.get("link") or item.get("frame") or item.get("object_name") or "")
        if link and (link not in by_link or item.get("meshless_frame") or item.get("role") == "transform_anchor"):
            by_link[link] = item
    for item in items:
        link = str(item.get("link_name") or item.get("link") or item.get("frame") or item.get("object_name") or "")
        parent = str(item.get("parent_link") or item.get("joint_parent_link") or item.get("immediate_parent_link") or "")
        if not link or not parent or parent not in by_link:
            continue
        parent_pose = by_link[parent].get("frame_world_pose") or by_link[parent].get("link_world_pose")
        child_pose = item.get("frame_world_pose") or item.get("link_world_pose")
        pose = _parent_to_child_pose(parent_pose, child_pose)
        if pose is None:
            continue
        item["parent_to_child_pose"] = pose
        item["parent_from_child"] = pose
        item["joint_origin"] = pose
        item["parent_joint_origin"] = pose
        item["parent_to_child_pose_source"] = "web_export_parent_world_inverse_times_child_world"
        item.setdefault("provenance", {}).update({
            "parent_to_child_pose": "web_export_parent_world_inverse_times_child_world",
            "parent_from_child": "web_export_parent_world_inverse_times_child_world",
            "joint_origin": "web_export_parent_world_inverse_times_child_world",
            "parent_joint_origin": "web_export_parent_world_inverse_times_child_world",
        })

def _authored_item(raw: Mapping[str, Any], source: str, index: int, scene_dir: Path) -> Json:
    fields = (
        "id", "type", "role", "category", "display_name", "frame", "pose", "pose_xyz", "pose_rpy", "dimensions",
        "geometry_type", "primitive_geometry_type", "mesh_uri", "package_uri", "source_path", "mesh_path", "material",
        "layout_item_ref", "support_surface_ref", "task_zone_ref", "scale", "mesh_scale", "perception_mode", "runtime_enforced", "runtime_commanded",
        "support_surface_kind", "support_kind", "semantic_type", "top_surface_z_m", "topSurfaceZM", "support_surface_height_m", "supportSurfaceHeightM",
        "expected_support_footprint_m", "support_footprint_m", "footprint_m", "footprint", "table_height", "table_top_z", "surface_height_m",
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



def _contains_unresolved_substitution(value: Any) -> bool:
    if not isinstance(value, str):
        return False
    text = value.strip()
    return text == "${mesh}" or ("${" in text and "}" in text)


def _has_unresolved_placeholder_mesh_reference(item: Mapping[str, Any]) -> bool:
    return any(_contains_unresolved_substitution(item.get(field)) for field in MESH_URI_FIELDS)


def _supported_mesh_uri(value: Any) -> bool:
    if not isinstance(value, str) or _contains_unresolved_substitution(value):
        return False
    parsed = urlparse(value)
    path = unquote(parsed.path if parsed.scheme else value)
    return Path(path).suffix.lower() in SUPPORTED_MESH_SUFFIXES


def _has_supported_mesh_reference(item: Mapping[str, Any]) -> bool:
    return any(_supported_mesh_uri(item.get(field)) for field in MESH_URI_FIELDS)


def _valid_generated_robot_mesh_reference(item: Mapping[str, Any]) -> bool:
    """Return true for generated robot mesh-preview rows that can replace stale xacro placeholders."""
    if str(item.get("active_visual_source", "")).lower() != "mesh_preview":
        return False
    if str(item.get("category", "")).lower() != "robot_static_mesh_visual":
        return False
    if str(item.get("role", "")).lower() != "robot":
        return False
    return any(
        isinstance(item.get(field), str)
        and not _contains_unresolved_substitution(item.get(field))
        and "package://ur_description/meshes/" in str(item.get(field))
        and "/visual/" in str(item.get(field))
        and Path(unquote(urlparse(str(item.get(field))).path)).suffix.lower() in SUPPORTED_MESH_SUFFIXES
        for field in MESH_URI_FIELDS
    )


def _robot_family_from_item(item: Mapping[str, Any]) -> Optional[str]:
    text = " ".join(str(item.get(field, "")) for field in MESH_URI_FIELDS + ("id", "role", "category", "display_name", "link")).lower()
    for family in ("ur3", "ur5", "ur10"):
        if f"/meshes/{family}/" in text or family in text:
            return family
    return None


def _normalized_robot_link(item: Mapping[str, Any]) -> str:
    link = str(item.get("link") or item.get("object_name") or "").lower()
    return link.removesuffix("_inertia")


def _is_robot_like_generated_item(item: Mapping[str, Any]) -> bool:
    text = _identity_text(item)
    role = str(item.get("role", "")).lower()
    category = str(item.get("category", "")).lower()
    return (
        item.get("source_kind") == "generated_preview"
        and (role == "robot" or category == "robot" or "robot" in text or _robot_family_from_item(item) is not None)
    )


def _has_generated_robot_mesh_replacements(generated: Mapping[str, List[Json]]) -> Tuple[set[Tuple[str, str]], set[str]]:
    replacement_keys: set[Tuple[str, str]] = set()
    replacement_families: set[str] = set()
    for section in ("robots", "assets"):
        for item in generated.get(section, []):
            if not isinstance(item, Mapping) or not _is_robot_like_generated_item(item):
                continue
            if not _valid_generated_robot_mesh_reference(item):
                continue
            family = _robot_family_from_item(item)
            link = _normalized_robot_link(item)
            if family:
                replacement_families.add(family)
                if link:
                    replacement_keys.add((family, link))
    return replacement_keys, replacement_families


def _suppress_unresolved_placeholder_robot_visuals(generated: Dict[str, List[Json]], warnings: List[Json]) -> None:
    replacement_keys, replacement_families = _has_generated_robot_mesh_replacements(generated)
    if not replacement_keys and not replacement_families:
        return
    for section in ("robots", "assets"):
        kept: List[Json] = []
        for item in generated.get(section, []):
            if not _has_unresolved_placeholder_mesh_reference(item):
                kept.append(item)
                continue
            family = _robot_family_from_item(item)
            link = _normalized_robot_link(item)
            should_suppress = (family and ((family, link) in replacement_keys or family in replacement_families))
            if not should_suppress and link and any((replacement_family, link) in replacement_keys for replacement_family in replacement_families):
                should_suppress = True
            if not should_suppress:
                kept.append(item)
                continue
            item_id = str(item.get("id") or "<unknown>")
            _warn(
                warnings,
                "unresolved_placeholder_visual_suppressed",
                f"Suppressed generated visual-index item {item_id} because its mesh reference contains an unresolved xacro substitution and a generated robot mesh replacement exists.",
                INPUTS["visual_mesh_index"],
            )
        generated[section] = kept

def _sort_items(items: List[Json]) -> List[Json]:
    return sorted(items, key=lambda x: (str(x.get("source_kind", "")), str(x.get("category", "")), str(x.get("role", "")), str(x.get("id", ""))))


def _has_mesh_reference(item: Mapping[str, Any]) -> bool:
    return any(isinstance(item.get(field), str) and bool(str(item.get(field)).strip()) for field in MESH_URI_FIELDS)




def _is_render_expected(item: Mapping[str, Any]) -> bool:
    return item.get("render_expected", True) is not False


def _mesh_format_from_uri(uri: Any) -> Optional[str]:
    if not isinstance(uri, str) or not uri:
        return None
    parsed = urlparse(uri)
    path = unquote(parsed.path if parsed.scheme else uri)
    suffix = Path(path).suffix.lower().lstrip(".")
    return suffix or None


def _is_mesh_item(item: Mapping[str, Any]) -> bool:
    geometry = str(item.get("geometry_type") or item.get("primitive_geometry_type") or item.get("type") or "").lower()
    return geometry == "mesh" or _has_mesh_reference(item)


def _core_mesh_category(item: Mapping[str, Any], section: str) -> Optional[str]:
    if _is_helper(item) or section == "zones":
        return None
    text = _identity_text(item)
    role = str(item.get("role", "")).lower()
    category = str(item.get("category", "")).lower()
    # Generated URDF preview items can include table/camera links in the same
    # visual index as robot links.  Classify explicit physical fixtures before
    # robot-family heuristics so support assets do not inherit robot metadata.
    if section == "sensors" or any(token in text for token in ("camera", "realsense")):
        return "camera_realsense"
    if any(token in text for token in ("table", "workbench", "support_surface")):
        return "table_workbench"
    if section == "robots" or role == "robot" or category in {"robot", "robot_static_mesh_visual"} or _robot_family_from_item(item):
        return "robot_arm_link"
    if section == "tools" or category in {"tool", "gripper", "end_effector"} or role in {"tool", "gripper", "end_effector"} or any(token in text for token in ("gripper", "robotiq", "suction", "end_effector")):
        return "gripper_link"
    if section == "assets" and _has_supported_mesh_reference(item):
        return "authored_asset_object"
    return None


def _all_scene_items(payload: Mapping[str, Any]) -> Iterable[Tuple[str, Json]]:
    for section in GENERATED_OUTPUT_SECTIONS:
        for item in payload.get(section, []):
            if isinstance(item, dict):
                yield section, item



def _item_pose_rpy(item: Mapping[str, Any]) -> Optional[List[float]]:
    for field in ("final_transform", "world_from_visual", "pose", "world_pose", "baked_world_visual_pose"):
        pose = item.get(field)
        if isinstance(pose, Mapping):
            rpy = _finite_num3(pose.get("rpy"))
            if rpy is not None:
                return rpy
    return _finite_num3(item.get("pose_rpy"))


def _bounds_dimensions(bounds: Tuple[List[float], List[float], str]) -> List[float]:
    mn, mx, _source = bounds
    return [abs(mx[i] - mn[i]) for i in range(3)]


def _visual_contract_category(item: Mapping[str, Any], section: str) -> str:
    text = _identity_text(item)
    role = str(item.get("role", "")).lower()
    category = str(item.get("category", "")).lower()
    if section == "zones" or _is_helper(item):
        return "zone"
    if section == "sensors" or any(token in text for token in ("camera", "realsense")):
        return "camera"
    if any(token in text for token in ("table", "workbench", "support_surface")):
        return "table"
    if section == "robots" or role == "robot" or category in {"robot", "robot_static_mesh_visual"} or _robot_family_from_item(item):
        return "robot_link"
    if section == "tools" or category in {"tool", "gripper", "end_effector"} or role in {"tool", "gripper", "end_effector"} or any(token in text for token in ("gripper", "robotiq", "suction", "end_effector")):
        return "gripper"
    return "object"


def _scene_expected_workspace_bounds_m(data: Mapping[str, Any]) -> Json:
    for root_key in ("scene_manifest", "cell_definition", "environment", "layout"):
        root = _as_map(data.get(root_key))
        candidates = (
            root.get("expected_workspace_bounds_m"),
            _as_map(root.get("visual_bounds_contract")).get("expected_workspace_bounds_m"),
            _as_map(root.get("scene")).get("expected_workspace_bounds_m"),
            _as_map(root.get("metadata")).get("expected_workspace_bounds_m"),
        )
        for candidate in candidates:
            if isinstance(candidate, Mapping):
                mn = _finite_num3(candidate.get("min") or candidate.get("min_xyz"))
                mx = _finite_num3(candidate.get("max") or candidate.get("max_xyz"))
                if mn is not None and mx is not None:
                    return {"min": mn, "max": mx, "source": root_key}
    return {"min": [-1.0, -1.0, 0.0], "max": [1.0, 1.0, 1.8], "source": "default_m1_workcell_envelope"}


def _item_visual_bounds(item: Mapping[str, Any]) -> Optional[Tuple[List[float], List[float], str]]:
    xyz = _item_pose_xyz(item)
    if xyz is None:
        return None
    local = _item_local_bounds(item)
    if local is not None:
        mn, mx, source = _scaled_bounds(local, item)
        return [xyz[i] + mn[i] for i in range(3)], [xyz[i] + mx[i] for i in range(3)], source
    return list(xyz), list(xyz), "pose"


def _authored_physical_dimension_defaults(data: Mapping[str, Any]) -> Dict[str, List[float]]:
    """Return scene-authored dimensions for generated physical fixture meshes.

    The visual mesh index can contain URDF-flattened table/camera mesh rows without
    repeating the authoring dimensions.  Preserve the mesh-backed generated preview
    while carrying over source-of-truth fixture dimensions from environment data so
    the browser preflight can distinguish normal fixtures from bad camera-framing
    bounds.
    """
    defaults: Dict[str, List[float]] = {}
    env = _as_map(data.get("environment"))
    env_root = _as_map(env.get("environment"))
    for raw in _as_list(env_root.get("support_surfaces")) + _as_list(env.get("support_surfaces")):
        if not isinstance(raw, Mapping):
            continue
        dims = _finite_num3(raw.get("dimensions") or raw.get("size"))
        text = _identity_text(raw)
        if dims is not None and any(token in text for token in ("table", "workbench", "support_surface")):
            defaults.setdefault("table", dims)
    for raw in _as_list(env_root.get("assets")) + _as_list(env_root.get("sensors")) + _as_list(env.get("assets")) + _as_list(env.get("sensors")):
        if not isinstance(raw, Mapping):
            continue
        dims = _finite_num3(raw.get("dimensions") or raw.get("size"))
        text = _identity_text(raw)
        if dims is not None and any(token in text for token in ("camera", "realsense")):
            defaults.setdefault("camera", dims)
    return defaults


def _authored_support_surface_defaults(data: Mapping[str, Any]) -> Dict[str, Any]:
    defaults: Dict[str, Any] = {}
    env = _as_map(data.get("environment"))
    env_root = _as_map(env.get("environment"))
    cell = _as_map(data.get("cell_definition"))
    cell_env = _as_map(cell.get("environment"))
    candidates = (
        _as_list(env_root.get("support_surfaces"))
        + _as_list(env.get("support_surfaces"))
        + _as_list(cell_env.get("support_surfaces"))
    )
    for raw in candidates:
        if not isinstance(raw, Mapping):
            continue
        text = _identity_text(raw)
        if not any(token in text for token in ("table", "workbench", "support_surface")):
            continue
        dims = _finite_num3(raw.get("dimensions") or raw.get("size"))
        pose = _finite_num3(raw.get("pose_xyz"))
        if pose is None:
            pose = _pose_xyz(raw.get("pose"))
        if dims is not None:
            defaults.setdefault("expected_support_footprint_m", [abs(dims[0]), abs(dims[1])])
            defaults.setdefault("expected_dimensions_m", dims)
        for source_key, target_key in (
            ("support_surface_kind", "support_surface_kind"),
            ("support_kind", "support_surface_kind"),
            ("semantic_type", "support_surface_kind"),
            ("top_surface_z_m", "top_surface_z_m"),
            ("topSurfaceZM", "top_surface_z_m"),
            ("support_surface_height_m", "support_surface_height_m"),
            ("supportSurfaceHeightM", "support_surface_height_m"),
            ("table_top_z", "top_surface_z_m"),
            ("table_height", "support_surface_height_m"),
            ("surface_height_m", "support_surface_height_m"),
        ):
            if raw.get(source_key) not in (None, ""):
                defaults.setdefault(target_key, raw.get(source_key))
        if pose is not None and dims is not None:
            top_z = pose[2] + abs(dims[2]) / 2.0
            defaults.setdefault("top_surface_z_m", top_z)
            defaults.setdefault("support_surface_height_m", top_z)
    return defaults


def _stl_mesh_bounds_m(item: Mapping[str, Any]) -> Optional[Dict[str, List[float]]]:
    uri = str(item.get("original_mesh_uri") or item.get("original_source_path") or item.get("mesh_uri") or "")
    if "workbench_description/meshes/visual/table.stl" not in uri:
        return None
    resolved = item.get("resolved_source_path")
    if not isinstance(resolved, str) or not resolved:
        return None
    path = Path(resolved)
    if not path.is_absolute():
        path = Path.cwd() / path
    if not path.is_file():
        return None
    try:
        raw = path.read_bytes()
    except OSError:
        return None
    vertices: List[List[float]] = []
    try:
        text = raw.decode("utf-8", errors="ignore")
        for line in text.splitlines():
            parts = line.strip().split()
            if len(parts) == 4 and parts[0].lower() == "vertex":
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
    except ValueError:
        vertices = []
    if not vertices and len(raw) >= 84:
        import struct

        try:
            tri_count = struct.unpack_from("<I", raw, 80)[0]
            expected = 84 + tri_count * 50
            if expected <= len(raw):
                for offset in range(84, expected, 50):
                    for vertex_index in range(3):
                        vertex_offset = offset + 12 + vertex_index * 12
                        vertices.append(list(struct.unpack_from("<fff", raw, vertex_offset)))
        except (struct.error, ValueError):
            return None
    if not vertices:
        return None
    scale = _finite_num3(item.get("scale") or item.get("mesh_scale")) or [1.0, 1.0, 1.0]
    minimum: List[float] = []
    maximum: List[float] = []
    size: List[float] = []
    for axis in range(3):
        values = [vertex[axis] * scale[axis] for vertex in vertices]
        axis_min = min(values)
        axis_max = max(values)
        minimum.append(axis_min)
        maximum.append(axis_max)
        size.append(abs(axis_max - axis_min))
    if all(v > 0.0 for v in size):
        return {"min": minimum, "max": maximum, "size": size}
    return None


def _stl_mesh_dimensions_m(item: Mapping[str, Any]) -> Optional[List[float]]:
    bounds = _stl_mesh_bounds_m(item)
    return bounds.get("size") if bounds else None


def _populate_support_surface_fields(item: Json, category: str, support_defaults: Mapping[str, Any]) -> None:
    if category != "table":
        return
    mesh_bounds = _stl_mesh_bounds_m(item)
    mesh_dims = mesh_bounds.get("size") if mesh_bounds else None
    dims = mesh_dims or _finite_num3(item.get("expected_dimensions_m")) or _finite_num3(support_defaults.get("expected_dimensions_m"))
    if mesh_dims is not None and 0.9 <= mesh_dims[0] <= 1.5 and 0.5 <= mesh_dims[1] <= 1.1 and 0.6 <= mesh_dims[2] <= 1.2:
        item.setdefault("support_surface_kind", "workbench_body")
        item["expected_dimensions_m"] = mesh_dims
    else:
        item.setdefault("support_surface_kind", support_defaults.get("support_surface_kind") or "support_surface")
    if dims is not None:
        item.setdefault("expected_support_footprint_m", [abs(dims[0]), abs(dims[1])])
    xyz = _item_pose_xyz(item)
    if xyz is None and dims is not None:
        xyz = [0.0, 0.0, 0.0]
    if xyz is not None and dims is not None:
        # STL workbench meshes may use a floor/local-corner origin rather than
        # a centered origin.  Use the actual mesh max-Z when available so
        # support-surface diagnostics match the rendered browser mesh top.
        top_z = xyz[2] + (float(mesh_bounds["max"][2]) if mesh_bounds else abs(dims[2]) / 2.0)
        item.setdefault("top_surface_z_m", top_z)
        item.setdefault("support_surface_height_m", top_z)
    for key in ("top_surface_z_m", "support_surface_height_m", "expected_support_footprint_m"):
        if key not in item and key in support_defaults:
            item[key] = support_defaults[key]


def _populate_visual_bounds_item_fields(payload: Json, data: Mapping[str, Any]) -> None:
    dimension_defaults = _authored_physical_dimension_defaults(data)
    support_defaults = _authored_support_surface_defaults(data)
    for section, item in _all_scene_items(payload):
        if not _is_mesh_item(item):
            continue
        had_explicit_expected_dimensions = _finite_num3(item.get("expected_dimensions_m")) is not None
        local = _item_local_bounds(item)
        if local is not None:
            item["expected_dimensions_m"] = _bounds_dimensions(_scaled_bounds(local, item))
        xyz = _item_pose_xyz(item)
        if xyz is not None:
            item["expected_pose_m"] = xyz
        rpy = _item_pose_rpy(item)
        if rpy is not None:
            item["expected_pose_rpy"] = rpy
        category = _visual_contract_category(item, section)
        item["mesh_contract_category"] = category
        if "expected_dimensions_m" not in item and category in dimension_defaults:
            item["expected_dimensions_m"] = dimension_defaults[category]
        _populate_support_surface_fields(item, category, support_defaults)
        item.setdefault("mesh_load_required", category in {"robot_link", "gripper", "table", "camera", "object"})
        # Unit autoscale is a browser-side asset convenience only.  Generated URDF
        # previews and robot links must keep authored units exactly as exported.
        item["allow_mesh_unit_autoscale"] = bool(
            had_explicit_expected_dimensions
            and item.get("source_kind") != "generated_preview"
            and section in {"assets", "sensors"}
            and category in {"table", "camera", "object"}
        )


def _visual_bounds_contract(payload: Json, data: Mapping[str, Any]) -> Json:
    expected = _scene_expected_workspace_bounds_m(data)
    workspace_min = expected["min"]
    workspace_max = expected["max"]
    workspace_span = [workspace_max[i] - workspace_min[i] for i in range(3)]
    scene_min: Optional[List[float]] = None
    scene_max: Optional[List[float]] = None
    sources: List[str] = []
    oversized: List[Json] = []
    collapsed: List[Json] = []
    invalid_orientation: List[Json] = []
    blockers: List[Json] = []

    for section, item in _all_scene_items(payload):
        if section == "zones" or _is_helper(item) or not _is_render_expected(item):
            continue
        item_id = str(item.get("id"))
        category = str(item.get("mesh_contract_category") or _visual_contract_category(item, section))
        bounds = _item_visual_bounds(item)
        dims = _finite_num3(item.get("expected_dimensions_m"))
        if dims is None:
            local = _item_local_bounds(item)
            dims = _bounds_dimensions(_scaled_bounds(local, item)) if local is not None else None
        if bounds is not None:
            mn, mx, source = bounds
            scene_min = mn if scene_min is None else [min(scene_min[i], mn[i]) for i in range(3)]
            scene_max = mx if scene_max is None else [max(scene_max[i], mx[i]) for i in range(3)]
            sources.append(f"{item_id}:{source}")
            outside = any(mx[i] < workspace_min[i] or mn[i] > workspace_max[i] for i in range(3))
        else:
            outside = False
        if dims is not None:
            if any(v <= 1e-6 for v in dims):
                entry = {"id": item_id, "category": category, "expected_dimensions_m": dims, "reason": "zero_or_near_zero_dimension"}
                collapsed.append(entry)
                if category in {"robot_link", "gripper", "table", "camera", "object"}:
                    blockers.append({**entry, "reason": "collapsed_item_can_break_camera_framing"})
            if any(workspace_span[i] > 0 and dims[i] > workspace_span[i] * 1.5 for i in range(3)):
                entry = {"id": item_id, "category": category, "expected_dimensions_m": dims, "reason": "dimension_exceeds_expected_workspace_span"}
                oversized.append(entry)
                blockers.append({**entry, "reason": "oversized_item_can_break_camera_framing"})
        if outside:
            blockers.append({"id": item_id, "category": category, "reason": "item_bounds_outside_expected_workspace"})
        rpy = _item_pose_rpy(item)
        if rpy is not None and (any(abs(v) > 6.5 for v in rpy) or _finite_num3(rpy) is None):
            entry = {"id": item_id, "category": category, "expected_pose_rpy": rpy, "reason": "orientation_rpy_outside_radian_range"}
            invalid_orientation.append(entry)
            blockers.append({**entry, "reason": "invalid_orientation_can_break_camera_framing"})

    camera_framing_blockers = sorted(blockers, key=lambda x: (str(x.get("id")), str(x.get("reason"))))
    status = "passed" if not camera_framing_blockers else "failed"

    return {
        "status": status,
        "expected_workspace_bounds_m": {"min": workspace_min, "max": workspace_max, "source": expected["source"]},
        "scene_bounds_m": {"min": scene_min or [0.0, 0.0, 0.0], "max": scene_max or [0.0, 0.0, 0.0], "source_count": len(sources), "sources": sorted(sources)},
        "oversized_items": sorted(oversized, key=lambda x: str(x.get("id"))),
        "collapsed_items": sorted(collapsed, key=lambda x: str(x.get("id"))),
        "invalid_orientation_items": sorted(invalid_orientation, key=lambda x: str(x.get("id"))),
        "camera_framing_blockers": camera_framing_blockers,
    }

def _populate_mesh_contract_fields(payload: Json, *, staged: bool) -> Json:
    required = 0
    staged_count = 0
    missing: List[Json] = []
    failures: List[Json] = []
    fallback_primitive_count = 0
    for section, item in _all_scene_items(payload):
        core_category = _core_mesh_category(item, section)
        if not _is_render_expected(item) or not _is_mesh_item(item):
            # Non-mesh primitives, helpers, zones, and authored overlays are not
            # required browser mesh loads. Do not count them as primitive fallback
            # core mesh failures; actual required mesh failures are reported below
            # through missing_required_meshes/core_mesh_failures.
            continue
        candidates = _mesh_candidates(item)
        original = item.get("original_mesh_uri") or (candidates[0][1] if candidates else None)
        item["original_mesh_uri"] = original
        item["original_package_uri"] = next((uri for _field, uri in candidates if uri.startswith("package://")), original if isinstance(original, str) and original.startswith("package://") else None)
        item["original_source_path"] = item.get("source_path") or item.get("mesh_path") or original
        item.setdefault("mesh_format", _mesh_format_from_uri(original or item.get("mesh_uri") or item.get("mesh_path") or item.get("source_path")))
        if "mesh_load_required" not in item:
            item["mesh_load_required"] = core_category is not None
        if core_category:
            item["core_mesh_category"] = core_category
        item["mesh_url"] = item.get("mesh_uri") if item.get("mesh_staging_status") == "staged" else None
        item.setdefault("mesh_staged_path", item.get("mesh_staged_path"))
        if item.get("mesh_load_required"):
            required += 1
            status = str(item.get("mesh_staging_status") or "")
            if status == "staged":
                staged_count += 1
            elif staged:
                entry = {"id": str(item.get("id")), "category": core_category, "status": status or "not_staged", "source": original, "warning": item.get("mesh_resolve_warning")}
                missing.append(entry)
                failures.append(entry)
    status = "passed" if not failures else "failed"
    return {
        "required_mesh_count": required,
        "staged_mesh_count": staged_count,
        "missing_required_meshes": missing,
        "fallback_primitive_count": fallback_primitive_count,
        "core_mesh_failures": failures,
        "mesh_contract_status": status,
    }

def _drop_shadowed_metadata_primitives(items: List[Json], generated_items: List[Json], tokens: Sequence[str]) -> List[Json]:
    if not any(_has_supported_mesh_reference(item) for item in generated_items):
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



def _finite_num3(value: Any) -> Optional[List[float]]:
    if not isinstance(value, list) or len(value) < 3:
        return None
    try:
        out = [float(value[0]), float(value[1]), float(value[2])]
    except (TypeError, ValueError):
        return None
    if any(not (v == v and abs(v) != float("inf")) for v in out):
        return None
    return out


def _item_pose_xyz(item: Mapping[str, Any]) -> Optional[List[float]]:
    for field in ("final_transform", "world_from_visual", "pose", "world_pose", "baked_world_visual_pose"):
        xyz = _pose_xyz(item.get(field))
        if xyz is not None:
            return xyz
    return _finite_num3(item.get("pose_xyz"))


def _item_local_bounds(item: Mapping[str, Any]) -> Optional[Tuple[List[float], List[float], str]]:
    for field in ("mesh_bounds", "local_bounds", "bounds"):
        bounds = item.get(field)
        if isinstance(bounds, Mapping):
            mn = _finite_num3(bounds.get("min") or bounds.get("min_xyz") or bounds.get("local_min"))
            mx = _finite_num3(bounds.get("max") or bounds.get("max_xyz") or bounds.get("local_max"))
            if mn is not None and mx is not None:
                return mn, mx, field
    mn = _finite_num3(item.get("local_bounds_min") or item.get("mesh_bounds_min"))
    mx = _finite_num3(item.get("local_bounds_max") or item.get("mesh_bounds_max"))
    if mn is not None and mx is not None:
        return mn, mx, "local_bounds_min/max"
    dims = _finite_num3(item.get("dimensions") or item.get("size") or item.get("primitive_dimensions"))
    if dims is not None:
        half = [abs(v) / 2.0 for v in dims]
        return [-half[0], -half[1], -half[2]], [half[0], half[1], half[2]], "dimensions"
    return None


def _scaled_bounds(bounds: Tuple[List[float], List[float], str], item: Mapping[str, Any]) -> Tuple[List[float], List[float], str]:
    mn, mx, source = bounds
    scale = _finite_num3(item.get("scale") or item.get("mesh_scale")) or [1.0, 1.0, 1.0]
    scaled_min: List[float] = []
    scaled_max: List[float] = []
    for i in range(3):
        a = mn[i] * scale[i]
        b = mx[i] * scale[i]
        scaled_min.append(min(a, b))
        scaled_max.append(max(a, b))
    return scaled_min, scaled_max, source


def _viewer_item_status(item: Mapping[str, Any]) -> str:
    status = str(item.get("mesh_staging_status") or "")
    if status == "staged":
        return "mesh_backed"
    if status in {"resolve_failed", "unsupported_format", "unsafe_path", "unsupported_scheme", "unsafe_destination"}:
        return "missing_or_failed_mesh"
    if _has_mesh_reference(item):
        return "missing_or_failed_mesh"
    return "primitive_fallback"


def _viewer_summary(payload: Json) -> Json:
    sections: Sequence[str] = RENDERABLE_OUTPUT_SECTIONS
    renderable: List[Tuple[str, Json]] = []
    for section in sections:
        for item in payload.get(section, []):
            if isinstance(item, dict):
                renderable.append((section, item))

    scene_min: Optional[List[float]] = None
    scene_max: Optional[List[float]] = None
    bounds_sources: List[str] = []
    mesh_backed = 0
    fallback = 0
    missing = 0
    required: Dict[str, Json] = {key: {"present": False, "status": "missing", "item_ids": []} for key in ("robot", "tool", "table", "camera")}

    for _section, item in renderable:
        status = _viewer_item_status(item)
        if status == "mesh_backed":
            mesh_backed += 1
        elif status == "missing_or_failed_mesh":
            missing += 1
        else:
            fallback += 1

        xyz = _item_pose_xyz(item)
        if xyz is not None:
            local = _item_local_bounds(item)
            if local is not None:
                mn, mx, source = _scaled_bounds(local, item)
                item_min = [xyz[i] + mn[i] for i in range(3)]
                item_max = [xyz[i] + mx[i] for i in range(3)]
                bounds_sources.append(f"{item.get('id')}:{source}")
            else:
                item_min = list(xyz)
                item_max = list(xyz)
                bounds_sources.append(f"{item.get('id')}:pose")
            scene_min = item_min if scene_min is None else [min(scene_min[i], item_min[i]) for i in range(3)]
            scene_max = item_max if scene_max is None else [max(scene_max[i], item_max[i]) for i in range(3)]

        categories: List[str] = []
        is_helper_or_zone = _section == "zones" or _is_helper(item)
        if not is_helper_or_zone:
            text = _identity_text(item)
            if "robot" in text or str(item.get("category", "")).lower() == "robot":
                categories.append("robot")
            if any(token in text for token in ("tool", "gripper", "robotiq", "end_effector", "suction")):
                categories.append("tool")
            if any(token in text for token in ("table", "workbench", "support_surface")):
                categories.append("table")
            if any(token in text for token in ("camera", "realsense")):
                categories.append("camera")
        for category in categories:
            entry = required[category]
            entry["present"] = True
            entry["item_ids"].append(str(item.get("id")))
            if entry["status"] in {"missing", "primitive_fallback"} or status == "missing_or_failed_mesh":
                entry["status"] = status

    for entry in required.values():
        entry["item_ids"] = sorted(set(entry["item_ids"]))
    bounds = {"min": scene_min or [0.0, 0.0, 0.0], "max": scene_max or [0.0, 0.0, 0.0], "source_count": len(bounds_sources), "sources": sorted(bounds_sources)}
    return {
        "scene_bounds": bounds,
        "renderable_count": len(renderable),
        "mesh_backed_count": mesh_backed,
        "fallback_count": fallback,
        "missing_or_failed_mesh_count": missing,
        "required_item_status": required,
    }


def _annotate_urdf_assembly_metadata(generated: Dict[str, List[Json]], scene_name: str) -> None:
    """Mark generated robot/tool URDF visuals for browser hierarchy rendering."""
    robot_instance_id = f"{scene_name}_robot"
    assembly_group = "ur5_robotiq" if "ur5" in scene_name.lower() else f"{scene_name}_robot_tool"
    for section in GENERATED_OUTPUT_SECTIONS:
        for item in generated.get(section, []):
            if not isinstance(item, dict):
                continue
            text = _identity_text(item)
            link = str(item.get("link_name") or item.get("link") or item.get("frame") or item.get("object_name") or "")
            if not link:
                continue
            is_robot_tool = (
                section in {"robots", "tools", "frames"}
                or str(item.get("role", "")).lower() in {"robot", "tool", "gripper", "end_effector"}
                or str(item.get("category", "")).lower() in {"robot", "tool", "gripper", "end_effector", "robot_static_mesh_visual"}
                or any(token in text for token in ("ur5", "robot", "wrist", "shoulder", "forearm", "upper_arm", "tool0", "gripper", "robotiq"))
            )
            if not is_robot_tool:
                continue
            item.setdefault("link_name", link)
            parent = str(item.get("parent_link") or item.get("joint_parent_link") or item.get("immediate_parent_link") or "")
            if link == "tool0" and parent == "flange":
                parent = "wrist_3_link"
                item["parent_link"] = parent
                item["joint_parent_link"] = parent
                item["immediate_parent_link"] = parent
                item["joint_origin"] = {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
            elif parent:
                item.setdefault("parent_link", parent)
            if "joint_origin" not in item and "parent_joint_origin" in item:
                item["joint_origin"] = item["parent_joint_origin"]
            item.setdefault("joint_origin", {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]})
            item.setdefault("parent_from_child", item["joint_origin"])
            item.setdefault("visual_origin", {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]})
            mesh = str(item.get("mesh_uri") or item.get("package_uri") or item.get("source_path") or item.get("mesh_path") or "")
            if mesh:
                item.setdefault("mesh_uri", mesh)
                item.setdefault("original_mesh_uri", item.get("package_uri") or item.get("source_path") or mesh)
            elif link == "tool0":
                item["render_expected"] = False
                item["geometry_type"] = "frame"
                item["primitive_geometry_type"] = "frame"
                item["meshless_frame"] = True
            item["robot_instance_id"] = robot_instance_id
            item["assembly_group"] = assembly_group
            item["robot_render_mode"] = "urdf_fk_visual_world_pose" if item.get("urdf_fk_source") == "expanded_urdf_joint_tree" else "assembled_urdf_hierarchy"
            item["baked_world_visual_pose_diagnostic_only"] = True
            item.setdefault("provenance", {}).update({
                "robot_instance_id": "web_export_urdf_assembly_metadata",
                "assembly_group": "web_export_urdf_assembly_metadata",
                "robot_render_mode": "web_export_urdf_assembly_metadata",
            })

def build_web_scene(scene_dir: Path, *, stage_assets: bool = False, output_path: Optional[Path] = None) -> Json:
    scene_dir = scene_dir.resolve()
    warnings: List[Json] = []
    data = _load_inputs(scene_dir, warnings)

    manifest = _as_map(data.get("scene_manifest"))
    scene_meta = _as_map(manifest.get("scene"))
    cell_scene = _as_map(_as_map(data.get("cell_definition")).get("cell"))
    env_scene = _as_map(_as_map(data.get("environment")).get("scene"))
    scene_name = _first_present(scene_meta.get("name"), cell_scene.get("name"), cell_scene.get("id"), env_scene.get("name"), env_scene.get("id"), scene_dir.name)

    generated = _generated_preview_items(_as_map(data.get("visual_mesh_index")), scene_dir, warnings) if data.get("visual_mesh_index") is not None else {section: [] for section in GENERATED_OUTPUT_SECTIONS}
    _supplement_missing_tool_meshes(data, generated)
    _annotate_urdf_assembly_metadata(generated, scene_name)
    _apply_web_scene_transform_parity_fallbacks(data, generated, warnings)
    _annotate_parent_to_child_poses(generated)
    _suppress_unresolved_placeholder_robot_visuals(generated, warnings)
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
        "frames": _sort_items(generated["frames"]),
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
    output["_visual_mesh_index_source"] = _as_map(data.get("visual_mesh_index"))
    if stage_assets:
        _stage_visual_meshes(output, scene_dir, output_path or Path("build/workcell_studio_web_scene/scene.web_scene.json"))
        _stage_expanded_robot_urdf(output, scene_dir, output_path or Path("build/workcell_studio_web_scene/scene.web_scene.json"), warnings)
    output.pop("_visual_mesh_index_source", None)
    _populate_visual_bounds_item_fields(output, data)
    output["metadata"] = {
        "mesh_contract": _populate_mesh_contract_fields(output, staged=stage_assets),
        "visual_bounds_contract": _visual_bounds_contract(output, data),
    }
    output["viewer_summary"] = _viewer_summary(output)
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

    try:
        payload = build_web_scene(scene_dir, stage_assets=args.stage_assets, output_path=output_path)
    except BlockingExportError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 3
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
