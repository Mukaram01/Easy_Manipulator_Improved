#!/usr/bin/env python3
"""Offline scene creation readiness checker for existing workcell_builder scenes."""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import yaml

MESH_EXTS = {".stl", ".dae", ".obj"}
SCENE_TEXT_EXTS = {".urdf", ".xacro", ".yaml", ".yml", ".json", ".xml", ".txt", ".srdf"}
FRAME_HINTS = ("world", "base_link", "tool0", "camera_link")

def _load_optional_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception:
        return {}
    return loaded if isinstance(loaded, dict) else {}


def _discover_scene_packages(scenes_dir: Path) -> list[Path]:
    return sorted([p for p in scenes_dir.iterdir() if p.is_dir()]) if scenes_dir.is_dir() else []


def _extract_mesh_refs(text: str) -> list[str]:
    refs: list[str] = []
    refs.extend(re.findall(r"filename\s*=\s*[\"']([^\"']+)[\"']", text, flags=re.IGNORECASE))
    refs.extend(re.findall(r"(?:package://[^\s\"']+\.(?:stl|dae|obj)|[^\s\"']+\.(?:stl|dae|obj))", text, flags=re.IGNORECASE))
    return refs


def _looks_absolute(ref: str) -> bool:
    lowered = ref.lower()
    return lowered.startswith("/") or re.match(r"^[a-zA-Z]:\\", ref) is not None or "/home/" in lowered


def _asset_name_from_mesh(path: Path) -> str:
    base = re.sub(r"[^a-zA-Z0-9_]+", "_", path.stem.lower()).strip("_")
    return base or "imported_asset"


def _extract_placed_objects(scene_pkg: Path) -> tuple[list[dict[str, Any]], list[str]]:
    warnings: list[str] = []
    env_yaml = scene_pkg / "environment.yaml"
    if not env_yaml.exists():
        return [], warnings
    try:
        loaded = yaml.safe_load(env_yaml.read_text(encoding="utf-8")) or {}
    except Exception as exc:
        warnings.append(f"Unable to parse environment.yaml for placed_objects: {exc}")
        return [], warnings
    if not isinstance(loaded, dict):
        warnings.append("environment.yaml root is not a mapping; skipping placed_objects checks.")
        return [], warnings

    raw = loaded.get("placed_objects")
    if raw is None:
        return [], warnings
    if not isinstance(raw, list):
        warnings.append("environment.yaml placed_objects is not a list; skipping placed_objects checks.")
        return [], warnings
    return [obj for obj in raw if isinstance(obj, dict)], warnings


def _collect_generated_urdf_text(scene_pkg: Path) -> tuple[str, list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []
    generated_files = [scene_pkg / "urdf" / "scene.urdf.xacro", scene_pkg / "urdf" / "environment.urdf.xacro"]
    texts: list[str] = []
    existing = [p for p in generated_files if p.exists()]
    for path in existing:
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except Exception as exc:
            errors.append(f"Unable to read generated URDF/Xacro file {path.relative_to(scene_pkg)}: {exc}")
            continue
        try:
            ET.fromstring(text)
        except ET.ParseError as exc:
            errors.append(f"Generated URDF/Xacro is structurally invalid ({path.relative_to(scene_pkg)}): {exc}")
        texts.append(text)
    if not existing:
        warnings.append("No generated URDF/Xacro file found for placed_objects linkage check.")
    return "\n".join(texts), errors, warnings


def _analyze_scene_package(scene_pkg: Path, repo_root: Path) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    notes: list[str] = []

    files = [p for p in scene_pkg.rglob("*") if p.is_file()]
    mesh_files = [p for p in files if p.suffix.lower() in MESH_EXTS]
    urdf_xacro_files = [p for p in files if ".urdf" in p.name.lower() or p.suffix.lower() == ".xacro"]
    text_files = [p for p in files if p.suffix.lower() in SCENE_TEXT_EXTS or any(ext in p.name.lower() for ext in (".urdf", ".xacro", ".srdf"))]

    if (scene_pkg / "package.xml").exists():
        notes.append("ROS scene package detected (package.xml present).")
    else:
        warnings.append("Scene package has no package.xml (may be non-ROS folder).")

    if not urdf_xacro_files:
        errors.append("No URDF/Xacro files found in scene package.")

    frame_hits: dict[str, int] = {frame: 0 for frame in FRAME_HINTS}
    candidate_clues = {"robot": set(), "gripper": set(), "sensor": set()}
    placed_objects, placed_warnings = _extract_placed_objects(scene_pkg)
    warnings.extend(placed_warnings)

    env_data = _load_optional_yaml(scene_pkg / "environment.yaml")
    has_task_zones_key = isinstance(env_data, dict) and "task_zones" in env_data
    task_zones = env_data.get("task_zones") if isinstance(env_data.get("task_zones"), list) else []
    zone_ids: list[str] = []
    dup_ids: list[str] = []
    invalid_dims: list[str] = []
    pick_count = 0
    place_count = 0
    for i, zone in enumerate(task_zones):
        if not isinstance(zone, dict):
            continue
        zid = str(zone.get("id") or f"task_zone_{i+1:02d}")
        ztype = str(zone.get("type") or "").lower()
        if zid in zone_ids and zid not in dup_ids:
            dup_ids.append(zid)
        zone_ids.append(zid)
        dims = zone.get("dimensions") or zone.get("size") or []
        if not (isinstance(dims, list) and len(dims) == 3 and all(isinstance(d, (int, float)) and d > 0 for d in dims)):
            invalid_dims.append(zid)
        if "pick" in ztype or "pick" in zid.lower():
            pick_count += 1
        if any(tok in ztype for tok in ("place", "target", "bin")) or "place" in zid.lower():
            place_count += 1
    if has_task_zones_key:
        if pick_count == 0:
            warnings.append("Missing pick task zone (no pick_zone-like entry in environment.task_zones).")
        if place_count == 0:
            warnings.append("Missing place task zone (no place_zone-like entry in environment.task_zones).")
    if dup_ids:
        warnings.append("Duplicate task zone IDs: " + ", ".join(dup_ids))
    if invalid_dims:
        warnings.append("Task zones with invalid dimensions: " + ", ".join(invalid_dims))

    for path in text_files:
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except Exception:
            continue

        for frame in FRAME_HINTS:
            frame_hits[frame] += len(re.findall(rf"\b{re.escape(frame)}\b", text))

        lower = (path.name + "\n" + text[:2000]).lower()
        if any(tok in lower for tok in ("ur5", "ur10", "ur3", "fanuc", "panda", "robot")):
            candidate_clues["robot"].add(path.name)
        if any(tok in lower for tok in ("robotiq", "gripper", "suction", "airpick", "end_effector")):
            candidate_clues["gripper"].add(path.name)
        if any(tok in lower for tok in ("camera", "realsense", "sensor", "rgbd")):
            candidate_clues["sensor"].add(path.name)

        mesh_refs = _extract_mesh_refs(text)
        has_visual = bool(re.search(r"<\s*visual\b|\bvisual\s*:", text, flags=re.IGNORECASE))
        has_collision = bool(re.search(r"<\s*collision\b|\bcollision\s*:", text, flags=re.IGNORECASE))
        if has_visual and not has_collision:
            warnings.append(f"Potential missing collision geometry in {path.relative_to(scene_pkg)}.")

        for ref in mesh_refs:
            if _looks_absolute(ref):
                warnings.append(f"Absolute mesh path found in {path.relative_to(scene_pkg)}: {ref}")
                continue
            if ref.startswith("package://"):
                continue
            candidate = (scene_pkg / ref).resolve()
            if not candidate.exists() and not (repo_root / ref).resolve().exists():
                warnings.append(f"Referenced mesh path not found: {ref} (from {path.relative_to(scene_pkg)}).")

    counts = Counter(p.name.lower() for p in mesh_files)
    duplicate_basenames = sorted([name for name, count in counts.items() if count > 1])
    if duplicate_basenames:
        warnings.append(f"Duplicate mesh basenames in scene package: {', '.join(duplicate_basenames)}")

    placed_summary: dict[str, Any] = {"count": len(placed_objects)}
    if placed_objects:
        generated_text, generated_errors, generated_warnings = _collect_generated_urdf_text(scene_pkg)
        errors.extend(generated_errors)
        warnings.extend(generated_warnings)

        invalid_mesh_warnings: list[str] = []
        absolute_external_mesh_warnings: list[str] = []
        missing_collision_mesh_warnings: list[str] = []
        found_names: list[str] = []

        for idx, obj in enumerate(placed_objects):
            obj_name = str(obj.get("name") or obj.get("id") or f"placed_object_{idx}")
            mesh_path = str(obj.get("mesh") or obj.get("mesh_path") or obj.get("filepath") or "").strip()
            if not mesh_path:
                invalid_mesh_warnings.append(f"{obj_name}: missing mesh filepath")
            else:
                lower = mesh_path.lower()
                if not any(lower.endswith(ext) for ext in MESH_EXTS):
                    invalid_mesh_warnings.append(f"{obj_name}: mesh does not use a supported extension ({mesh_path})")
                if _looks_absolute(mesh_path):
                    absolute_external_mesh_warnings.append(f"{obj_name}: absolute mesh path ({mesh_path})")
                if mesh_path.startswith("http://") or mesh_path.startswith("https://"):
                    absolute_external_mesh_warnings.append(f"{obj_name}: external URL mesh path ({mesh_path})")

            collision_enabled = bool(obj.get("collision_enabled", obj.get("collision", True)))
            collision_mesh = str(obj.get("collision_mesh") or obj.get("collision_mesh_path") or "").strip()
            if collision_enabled and not collision_mesh and not mesh_path:
                missing_collision_mesh_warnings.append(f"{obj_name}: collision enabled but no mesh/collision mesh provided")

            if generated_text and re.search(rf"\b{re.escape(obj_name)}\b", generated_text):
                found_names.append(obj_name)

        for msg in invalid_mesh_warnings:
            warnings.append(f"Placed object warning: {msg}")
        for msg in absolute_external_mesh_warnings:
            warnings.append(f"Placed object warning: {msg}")
        for msg in missing_collision_mesh_warnings:
            warnings.append(f"Placed object warning: {msg}")

        placed_summary.update(
            {
                "invalid_mesh_warnings": invalid_mesh_warnings,
                "absolute_external_mesh_warnings": absolute_external_mesh_warnings,
                "missing_collision_mesh_warnings": missing_collision_mesh_warnings,
                "found_in_generated_urdf_xacro": len(found_names) == len(placed_objects),
                "found_names": found_names,
                "missing_names": [str(o.get("name") or o.get("id") or f"placed_object_{i}") for i, o in enumerate(placed_objects) if str(o.get("name") or o.get("id") or f"placed_object_{i}") not in found_names],
            }
        )
    else:
        placed_summary.update(
            {
                "found_in_generated_urdf_xacro": False,
                "found_names": [],
                "missing_names": [],
                "invalid_mesh_warnings": [],
                "absolute_external_mesh_warnings": [],
                "missing_collision_mesh_warnings": [],
            }
        )

    task_intent = _load_optional_yaml(scene_pkg / "config" / "workcell_builder_task_intent.yaml")
    ti_pick_block = (task_intent.get("pick") or {}) if isinstance(task_intent, dict) else {}
    ti_pick_source = (ti_pick_block.get("source") or {}) if isinstance(ti_pick_block, dict) else {}
    ti_pick = ti_pick_source.get("id") if isinstance(ti_pick_source, dict) else None
    ti_pick_type = str(ti_pick_source.get("type") or "zone").lower() if isinstance(ti_pick_source, dict) else "zone"
    ti_pick_zone = ((ti_pick_block.get("zone") or {}).get("id")) if isinstance(ti_pick_block.get("zone"), dict) else None
    ti_place = (((task_intent.get("place") or {}).get("target") or {}).get("id")) if isinstance(task_intent, dict) else None
    if ti_pick and ti_pick_type in {"zone", "pick_zone"} and ti_pick not in zone_ids:
        warnings.append(f"Task intent pick.source.id not found in task_zones: {ti_pick}")
    if ti_pick_zone and ti_pick_zone not in zone_ids:
        warnings.append(f"Task intent pick.zone.id not found in task_zones: {ti_pick_zone}")
    if ti_place and ti_place not in zone_ids:
        warnings.append(f"Task intent place.target.id not found in task_zones: {ti_place}")

    cell_def = _load_optional_yaml(scene_pkg / "cell_definition.yaml")
    c_pick = ((((cell_def.get("task") or {}).get("pick") or {}).get("source") or {}).get("id")) if isinstance(cell_def, dict) else None
    c_place = ((((cell_def.get("task") or {}).get("place") or {}).get("target") or {}).get("id")) if isinstance(cell_def, dict) else None
    if c_pick and c_pick not in zone_ids:
        warnings.append(f"Cell definition task.pick.source.id not found in task_zones: {c_pick}")
    if c_place and c_place not in zone_ids:
        warnings.append(f"Cell definition task.place.target.id not found in task_zones: {c_place}")

    robot_cfg = env_data.get("robot") if isinstance(env_data.get("robot"), dict) else {}
    ee_cfg = env_data.get("end_effector") if isinstance(env_data.get("end_effector"), dict) else {}
    robot_mount = robot_cfg.get("robot_mount") if isinstance(robot_cfg, dict) else None
    tool_attachment = ee_cfg.get("tool_attachment") if isinstance(ee_cfg, dict) else None

    warnings.append(
        "Legacy compatibility: robot_mount "
        + ("present." if isinstance(robot_mount, dict) else "missing.")
    )
    warnings.append(
        "Legacy compatibility: tool_attachment "
        + ("present." if isinstance(tool_attachment, dict) else "missing.")
    )

    if isinstance(tool_attachment, dict):
        parent_link = str(tool_attachment.get("parent_link") or "").strip()
        child_link = str(tool_attachment.get("child_link") or "").strip()
        warnings.append(
            "Legacy compatibility: tool_attachment parent/child link completeness "
            + ("complete." if parent_link and child_link else "incomplete.")
        )

    rec_used = False
    for key in ("use_recommended_gripper_orientation", "use_recommended_orientation"):
        if key in ee_cfg:
            rec_used = bool(ee_cfg.get(key))
            break
    warnings.append(
        "Legacy compatibility: recommended gripper orientation "
        + ("used." if rec_used else "not used.")
    )

    known_gripper = str(ee_cfg.get("name") or ee_cfg.get("id") or "").lower()
    if known_gripper and any(tok in known_gripper for tok in ("robotiq", "onrobot", "airpick", "gripper", "suction")):
        src = tool_attachment if isinstance(tool_attachment, dict) else ee_cfg.get("origin")
        origin = (src.get("origin") if isinstance(src, dict) and isinstance(src.get("origin"), dict) else src) if isinstance(src, dict) else {}
        rpy = origin.get("rpy") if isinstance(origin, dict) else None
        if isinstance(rpy, list) and len(rpy) == 3:
            vals = [float(v) for v in rpy]
            if all(abs(v) < 1e-9 for v in vals):
                warnings.append("Legacy compatibility: known gripper has zero RPY (0,0,0).")

    generated_text = ""
    if placed_objects:
        # Re-use earlier generated URDF linkage checks path to avoid duplicate WARN lines.
        generated_text, _, _ = _collect_generated_urdf_text(scene_pkg)
    else:
        generated_text, generated_errors, generated_warnings = _collect_generated_urdf_text(scene_pkg)
        errors.extend(generated_errors)
        warnings.extend(generated_warnings)
    if generated_text:
        if isinstance(robot_mount, dict):
            mount_pose = robot_mount.get("pose") if isinstance(robot_mount.get("pose"), dict) else {}
            xyz = mount_pose.get("xyz")
            if isinstance(xyz, list) and len(xyz) == 3:
                xyz_str = " ".join(str(v) for v in xyz)
                warnings.append(
                    "Legacy compatibility: generated URDF robot base origin "
                    + ("contains expected pose." if xyz_str in generated_text else "missing expected pose.")
                )
        if isinstance(tool_attachment, dict):
            attach_origin = tool_attachment.get("origin") if isinstance(tool_attachment.get("origin"), dict) else {}
            xyz = attach_origin.get("xyz")
            if isinstance(xyz, list) and len(xyz) == 3:
                xyz_str = " ".join(str(v) for v in xyz)
                warnings.append(
                    "Legacy compatibility: generated URDF tool attach origin "
                    + ("contains expected pose." if xyz_str in generated_text else "missing expected pose.")
                )

    return {
        "scene_package": str(scene_pkg),
        "errors": errors,
        "warnings": warnings,
        "notes": notes,
        "counts": {
            "files": len(files),
            "meshes": len(mesh_files),
            "urdf_xacro": len(urdf_xacro_files),
        },
        "frames": {k: v for k, v in frame_hits.items() if v > 0},
        "candidates": {k: sorted(v) for k, v in candidate_clues.items() if v},
        "mesh_extensions_detected": sorted({p.suffix.lower() for p in mesh_files}),
        "placed_objects": placed_summary,
        "task_zones": {"total": len(task_zones), "pick": pick_count, "place": place_count, "duplicate_ids": dup_ids, "invalid_dimensions": invalid_dims, "zone_ids": zone_ids},
    }


def check_readiness(workcell_root: Path | None, scene_package: Path | None, strict: bool = False) -> dict[str, Any]:
    repo_root = Path(__file__).resolve().parents[1]
    errors: list[str] = []
    warnings: list[str] = []
    notes: list[str] = []
    scene_reports: list[dict[str, Any]] = []

    if workcell_root is not None:
        if not workcell_root.exists() or not workcell_root.is_dir():
            errors.append(f"Workcell root does not exist or is not a directory: {workcell_root}")
        scenes_dir = workcell_root / "scenes"
        assets_dir = workcell_root / "assets"
        if not scenes_dir.is_dir():
            errors.append(f"Missing scenes directory: {scenes_dir}")
        else:
            for pkg in _discover_scene_packages(scenes_dir):
                scene_reports.append(_analyze_scene_package(pkg, repo_root))

        if not assets_dir.is_dir():
            notes.append(
                f"Assets directory missing at {assets_dir}; existing workcell_builder default assets fallback may be used."
            )

    if scene_package is not None:
        if not scene_package.exists() or not scene_package.is_dir():
            errors.append(f"Scene package path does not exist or is not a directory: {scene_package}")
        else:
            scene_reports.append(_analyze_scene_package(scene_package, repo_root))

    if not scene_reports and not errors:
        warnings.append("No scene packages were inspected.")

    for report in scene_reports:
        errors.extend(report["errors"])
        warnings.extend(report["warnings"])
        notes.extend(report["notes"])

    result = "PASS"
    if errors:
        result = "FAIL"
    elif warnings:
        result = "WARN"

    if strict and warnings and not errors:
        result = "FAIL"

    return {
        "result": result,
        "strict": strict,
        "inputs": {
            "workcell_root": str(workcell_root) if workcell_root else None,
            "scene_package": str(scene_package) if scene_package else None,
        },
        "summary": {
            "scene_packages_checked": len(scene_reports),
            "errors": len(errors),
            "warnings": len(warnings),
        },
        "errors": errors,
        "warnings": warnings,
        "notes": notes,
        "scene_reports": scene_reports,
    }


def _print_human(payload: dict[str, Any]) -> None:
    print(f"RESULT: {payload['result']}")
    print(f"Scene packages checked: {payload['summary']['scene_packages_checked']}")
    for note in payload.get("notes", []):
        print(f"NOTE: {note}")
    for warning in payload.get("warnings", []):
        print(f"WARN: {warning}")
    for error in payload.get("errors", []):
        print(f"FAIL: {error}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workcell-root", type=Path)
    parser.add_argument("--scene-package", type=Path)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    if args.workcell_root is None and args.scene_package is None:
        parser.error("At least one of --workcell-root or --scene-package is required.")

    payload = check_readiness(args.workcell_root, args.scene_package, strict=args.strict)
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        _print_human(payload)

    return 0 if payload["result"] == "PASS" or payload["result"] == "WARN" else 1


if __name__ == "__main__":
    raise SystemExit(main())
