#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "scenes"
OUT = ROOT / "build/workcell_studio/visual_mesh_index_regeneration_report.json"
SUPPORTED_SCENES = ["ur5_2f_test", "ur5_3f_test", "ur10_2f_test", "ur3_suction_test", "ur5_airpick4_test", "suction_test"]

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    from scripts.repair_ur5_scene3d_visual_index import repair_index
except Exception:  # pragma: no cover
    repair_index = None


def parse():
    p = argparse.ArgumentParser()
    p.add_argument("--repo-root", type=Path, default=ROOT, help="Repository root that contains scripts/ and scenes/.")
    p.add_argument("--workspace-root", type=Path, help="Optional ROS workspace root forwarded to the extractor.")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--all", action="store_true")
    g.add_argument("--scene")
    p.add_argument("--portable", action="store_true")
    p.add_argument("--prefer-xacro-expanded", action="store_true", default=True)
    p.add_argument("--fallback-best-effort", action="store_true", default=True)
    p.add_argument("--repair-existing-only", action="store_true", help="Repair existing generated indexes without rerunning extraction.")
    p.add_argument("--fail-on-unexpanded", action="store_true")
    p.add_argument("--xacro-arg", action="append", default=[])
    p.add_argument("--fail-on-unsafe", action="store_true")
    return p.parse_args()


def scene_list(a):
    scenes_root = a.repo_root / "scenes"
    if a.scene:
        return [scenes_root / a.scene]
    return [scenes_root / s for s in SUPPORTED_SCENES if (scenes_root / s).exists()]


def _list_value(data, key):
    value = data.get(key)
    return value if isinstance(value, list) else []


def postprocess_scene_index(scene):
    idx = scene / "generated" / "scene_visual_mesh_index.json"
    result = {"postprocess_helper_available": repair_index is not None, "postprocess_changed": False, "postprocess_error": "", "postprocess_detail": ""}
    if repair_index is None:
        result["postprocess_error"] = "repair helper unavailable"
        return result
    if not idx.exists():
        result["postprocess_error"] = "scene_visual_mesh_index.json missing"
        return result
    try:
        changed, added_links = repair_index(idx)
        result["postprocess_changed"] = bool(changed)
        data = json.loads(idx.read_text(encoding="utf-8"))
        added_arm = list(added_links) if added_links else _list_value(data, "ur5_runtime_repair_added_links")
        added_eef = _list_value(data, "ur5_runtime_repair_added_end_effector_links")
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


def summarize(scene, repo_root=ROOT):
    idx = scene / "generated/scene_visual_mesh_index.json"
    data = json.loads(idx.read_text()) if idx.exists() else {}
    items = data.get("visual_items", [])
    mesh_backed = sum(1 for i in items if i.get("geometry_type") == "mesh")
    primitive = sum(1 for i in items if i.get("item_source") == "primitive_fallback" or i.get("geometry_type") in ("box", "cylinder", "sphere"))
    unresolved = data.get("unresolved_placeholder_count", 0)
    poses = [i.get("pose", {}).get("xyz", [0, 0, 0]) for i in items if isinstance(i.get("pose", {}).get("xyz"), list) and len(i.get("pose", {}).get("xyz")) == 3]
    distinct_pose_count = len({tuple(round(float(v), 6) for v in xyz) for xyz in poses})
    if poses:
        mins = [min(float(p[i]) for p in poses) for i in range(3)]
        maxs = [max(float(p[i]) for p in poses) for i in range(3)]
    else:
        mins, maxs = [0, 0, 0], [0, 0, 0]
    collapsed_pose_warning = bool(poses) and distinct_pose_count <= max(1, len(poses) // 3)
    safe = data.get("safe_for_preview", False)
    status = "PASS" if safe else ("FAIL" if not items else "WARN")
    stale_unsafe = int(bool(data.get("stale_index"))) + (1 if not safe else 0)
    return {
        "scene": scene.name,
        "extraction_mode": data.get("extraction_mode", "unknown"),
        "xacro_available": data.get("xacro_available", False),
        "expanded_urdf_written": bool(data.get("source_expanded_urdf_path")),
        "safe_for_preview": safe,
        "fallback_reason": data.get("fallback_reason", ""),
        "unresolved_placeholder_count": unresolved,
        "mesh_backed_count": mesh_backed,
        "skipped_count": sum(1 for i in items if i.get("render_skip_reason")),
        "primitive_fallback_count": primitive,
        "stale_index": data.get("stale_index", False),
        "status": status,
        "visual_item_count": len(items),
        "unresolved_count": unresolved,
        "stale_or_unsafe_count": stale_unsafe,
        "generated_index_path": str(idx.relative_to(repo_root)),
        "distinct_pose_count": distinct_pose_count,
        "bounding_box_min": mins,
        "bounding_box_max": maxs,
        "collapsed_pose_warning": collapsed_pose_warning,
        "ur5_runtime_repair_applied": bool(data.get("ur5_runtime_repair_applied", False)),
        "ur5_runtime_repair_mode": data.get("ur5_runtime_repair_mode", ""),
        "ur5_runtime_repair_added_links": _list_value(data, "ur5_runtime_repair_added_links"),
        "ur5_runtime_repair_added_end_effector_links": _list_value(data, "ur5_runtime_repair_added_end_effector_links"),
        "ur5_runtime_repair_reasons": _list_value(data, "ur5_runtime_repair_reasons"),
    }


def main():
    a = parse()
    rows = []
    a.repo_root = a.repo_root.resolve()
    out = a.repo_root / "build/workcell_studio/visual_mesh_index_regeneration_report.json"
    extractor = a.repo_root / "scripts/extract_scene_urdf_visual_mesh_index.py"
    for s in scene_list(a):
        if not a.repair_existing_only:
            cmd = ["python3", str(extractor), "--scene", s.name, "--prefer-xacro-expanded"]
            if a.workspace_root is not None:
                cmd += ["--workspace-root", str(a.workspace_root)]
            for xa in a.xacro_arg:
                cmd += ["--xacro-arg", xa]
            if a.fail_on_unexpanded:
                cmd.append("--fail-on-unexpanded")
            subprocess.run(cmd, check=False)
        postprocess = postprocess_scene_index(s)
        row = summarize(s, a.repo_root)
        row.update(postprocess)
        rows.append(row)
        print(
            f"{row['scene']}: visual_items={row['visual_item_count']} mesh={row['mesh_backed_count']} "
            f"primitive={row['primitive_fallback_count']} distinct_pose_count={row['distinct_pose_count']} "
            f"bbox_min={row['bounding_box_min']} bbox_max={row['bounding_box_max']} "
            f"collapsed_pose_warning={row['collapsed_pose_warning']} skipped={row['skipped_count']} "
            f"safe={row['safe_for_preview']} postprocess_changed={row['postprocess_changed']} "
            f"postprocess_detail={row['postprocess_detail']} fallback_reason={row['fallback_reason']}"
        )
    payload = {"schema": "workcell_studio_visual_mesh_index_regeneration/v3", "scenes": rows, "summary": {"scene_count": len(rows)}}
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2) + "\n")
    print(out)
    if a.fail_on_unsafe and any(r["status"] != "PASS" for r in rows):
        return 1
    if a.fail_on_unexpanded and any(r["extraction_mode"] != "xacro_expanded" for r in rows):
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
