from __future__ import annotations

import json
from pathlib import Path
from typing import Any

try:
    from scripts.repair_ur5_scene3d_visual_index import repair_index
except Exception:  # pragma: no cover
    repair_index = None


def list_value(data: dict[str, Any], key: str) -> list[Any]:
    value = data.get(key)
    return value if isinstance(value, list) else []


def run_visual_index_postprocess(scene: Path) -> dict[str, Any]:
    idx = scene / "generated" / "scene_visual_mesh_index.json"
    result: dict[str, Any] = {
        "postprocess_helper_available": repair_index is not None,
        "postprocess_changed": False,
        "postprocess_error": "",
        "postprocess_detail": "",
    }
    if repair_index is None:
        result["postprocess_error"] = "repair module unavailable"
        return result
    if not idx.exists():
        result["postprocess_error"] = "scene_visual_mesh_index.json missing"
        return result
    try:
        changed, added_links = repair_index(idx)
        result["postprocess_changed"] = bool(changed)
        data = json.loads(idx.read_text(encoding="utf-8"))
        added_arm = list(added_links) if added_links else list_value(data, "ur5_runtime_repair_added_links")
        added_eef = list_value(data, "ur5_runtime_repair_added_end_effector_links")
        if changed and added_arm:
            result["postprocess_detail"] = "ur5_links=" + ",".join(str(v) for v in added_arm)
        elif changed and added_eef:
            result["postprocess_detail"] = "end_effector_links=" + ",".join(str(v) for v in added_eef)
        elif changed:
            result["postprocess_detail"] = "visual_rows_normalized"
        else:
            result["postprocess_detail"] = "already_safe"
    except Exception as exc:  # pragma: no cover
        result["postprocess_error"] = str(exc)
    return result


def summarize_visual_index(scene: Path, repo_root: Path) -> dict[str, Any]:
    idx = scene / "generated/scene_visual_mesh_index.json"
    data = json.loads(idx.read_text()) if idx.exists() else {}
    items = data.get("visual_items", [])
    poses = [
        item.get("pose", {}).get("xyz", [0, 0, 0])
        for item in items
        if isinstance(item.get("pose", {}).get("xyz"), list)
        and len(item.get("pose", {}).get("xyz")) == 3
    ]
    distinct_pose_count = len({tuple(round(float(v), 6) for v in xyz) for xyz in poses})
    bbox_min = [min(float(p[i]) for p in poses) for i in range(3)] if poses else [0, 0, 0]
    bbox_max = [max(float(p[i]) for p in poses) for i in range(3)] if poses else [0, 0, 0]
    safe = data.get("safe_for_preview", False)
    return {
        "scene": scene.name,
        "extraction_mode": data.get("extraction_mode", "unknown"),
        "xacro_available": data.get("xacro_available", False),
        "expanded_urdf_written": bool(data.get("source_expanded_urdf_path")),
        "safe_for_preview": safe,
        "fallback_reason": data.get("fallback_reason", ""),
        "unresolved_placeholder_count": data.get("unresolved_placeholder_count", 0),
        "mesh_backed_count": sum(1 for i in items if i.get("geometry_type") == "mesh"),
        "skipped_count": sum(1 for i in items if i.get("render_skip_reason")),
        "primitive_fallback_count": sum(1 for i in items if i.get("item_source") == "primitive_fallback" or i.get("geometry_type") in ("box", "cylinder", "sphere")),
        "stale_index": data.get("stale_index", False),
        "status": "PASS" if safe else ("FAIL" if not items else "WARN"),
        "visual_item_count": len(items),
        "unresolved_count": data.get("unresolved_placeholder_count", 0),
        "stale_or_unsafe_count": int(bool(data.get("stale_index"))) + (1 if not safe else 0),
        "generated_index_path": str(idx.relative_to(repo_root)),
        "distinct_pose_count": distinct_pose_count,
        "bounding_box_min": bbox_min,
        "bounding_box_max": bbox_max,
        "collapsed_pose_warning": bool(poses) and distinct_pose_count <= max(1, len(poses) // 3),
        "ur5_runtime_repair_applied": bool(data.get("ur5_runtime_repair_applied", False)),
        "ur5_runtime_repair_mode": data.get("ur5_runtime_repair_mode", ""),
        "ur5_runtime_repair_added_links": list_value(data, "ur5_runtime_repair_added_links"),
        "ur5_runtime_repair_added_end_effector_links": list_value(data, "ur5_runtime_repair_added_end_effector_links"),
        "ur5_runtime_repair_reasons": list_value(data, "ur5_runtime_repair_reasons"),
    }


def format_regeneration_row(row: dict[str, Any]) -> str:
    return (
        f"{row['scene']}: visual_items={row['visual_item_count']} mesh={row['mesh_backed_count']} "
        f"primitive={row['primitive_fallback_count']} distinct_pose_count={row['distinct_pose_count']} "
        f"bbox_min={row['bounding_box_min']} bbox_max={row['bounding_box_max']} "
        f"collapsed_pose_warning={row['collapsed_pose_warning']} skipped={row['skipped_count']} "
        f"safe={row['safe_for_preview']} postprocess_changed={row['postprocess_changed']} "
        f"postprocess_detail={row['postprocess_detail']} fallback_reason={row['fallback_reason']}"
    )
