#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import struct
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
SUPPORTED_EXTENSIONS = {".stl", ".obj"}


def _load_json(path: Path | None) -> dict[str, Any]:
    if not path:
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _as_vec3(value: Any, default: list[float] | None = None) -> list[float]:
    if default is None:
        default = [0.0, 0.0, 0.0]
    if isinstance(value, list) and len(value) == 3:
        try:
            return [float(value[0]), float(value[1]), float(value[2])]
        except (TypeError, ValueError):
            return list(default)
    return list(default)


def _merge_item(index_item: dict[str, Any], gui_item: dict[str, Any], debug_item: dict[str, Any]) -> dict[str, Any]:
    merged: dict[str, Any] = {}
    for src in (index_item, gui_item, debug_item):
        if isinstance(src, dict):
            merged.update(src)
    return merged


def _extract_tris_bounds(path: Path, ext: str) -> tuple[bool, int, list[float], list[float], str]:
    try:
        if ext == ".obj":
            verts: list[list[float]] = []
            tris = 0
            with path.open("r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    if line.startswith("v "):
                        parts = line.split()
                        if len(parts) >= 4:
                            verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
                    elif line.startswith("f "):
                        n = len(line.split()) - 1
                        if n >= 3:
                            tris += n - 2
            if not verts:
                return True, tris, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "obj"
            mins = [min(v[i] for v in verts) for i in range(3)]
            maxs = [max(v[i] for v in verts) for i in range(3)]
            return True, tris, mins, maxs, "obj"

        if ext == ".stl":
            data = path.read_bytes()
            if len(data) >= 84:
                tri_count = struct.unpack("<I", data[80:84])[0]
                expected_len = 84 + tri_count * 50
                if expected_len == len(data):
                    mins = [float("inf")] * 3
                    maxs = [float("-inf")] * 3
                    offset = 84
                    for _ in range(tri_count):
                        offset += 12
                        for _ in range(3):
                            x, y, z = struct.unpack("<fff", data[offset : offset + 12])
                            mins[0] = min(mins[0], x); mins[1] = min(mins[1], y); mins[2] = min(mins[2], z)
                            maxs[0] = max(maxs[0], x); maxs[1] = max(maxs[1], y); maxs[2] = max(maxs[2], z)
                            offset += 12
                        offset += 2
                    if tri_count == 0:
                        mins = [0.0, 0.0, 0.0]; maxs = [0.0, 0.0, 0.0]
                    return True, tri_count, mins, maxs, "stl_binary"
            verts: list[list[float]] = []
            for line in data.decode("utf-8", errors="ignore").splitlines():
                s = line.strip()
                if s.startswith("vertex "):
                    p = s.split()
                    if len(p) == 4:
                        verts.append([float(p[1]), float(p[2]), float(p[3])])
            tris = len(verts) // 3
            if not verts:
                return True, 0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "stl_ascii"
            mins = [min(v[i] for v in verts) for i in range(3)]
            maxs = [max(v[i] for v in verts) for i in range(3)]
            return True, tris, mins, maxs, "stl_ascii"
    except Exception:
        return False, 0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "parse_error"
    return False, 0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "unsupported"


def main() -> int:
    ap = argparse.ArgumentParser(description="Audit Scene3D runtime mesh rendering diagnostics")
    ap.add_argument("--scene", required=True)
    ap.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    ap.add_argument("--index-json", type=Path, default=None)
    ap.add_argument("--gui-smoke-json", type=Path, default=None)
    ap.add_argument("--runtime-debug-json", type=Path, default=None)
    ap.add_argument("--output", type=Path, default=None)
    args = ap.parse_args()

    scene_root = args.repo_root / "scenes" / args.scene
    index_path = args.index_json or (scene_root / "generated" / "scene_visual_mesh_index.json")
    index_payload = _load_json(index_path)
    gui_payload = _load_json(args.gui_smoke_json)
    runtime_payload = _load_json(args.runtime_debug_json)

    index_items = index_payload.get("scene_urdf_visual_items") or index_payload.get("visual_items") or []
    gui_items = gui_payload.get("visual_items") if isinstance(gui_payload, dict) else []
    debug_items = runtime_payload.get("visual_items") if isinstance(runtime_payload, dict) else []

    def key_for(item: dict[str, Any]) -> str:
        return str(item.get("id") or item.get("visual") or item.get("link") or item.get("name") or "")

    merged: dict[str, dict[str, Any]] = {}
    for item in index_items:
        merged[key_for(item)] = _merge_item(item, {}, {})
    for item in gui_items or []:
        k = key_for(item)
        merged[k] = _merge_item(merged.get(k, {}), item, {})
    for item in debug_items or []:
        k = key_for(item)
        merged[k] = _merge_item(merged.get(k, {}), {}, item)

    results = []
    reason_counter = Counter()
    grouped_ids: dict[str, list[str]] = defaultdict(list)

    for k, item in merged.items():
        source_path = item.get("source_path") or item.get("mesh_path") or item.get("resolved_path") or ""
        resolved_source_path = item.get("resolved_source_path") or source_path
        final_gui_mesh_path = item.get("final_gui_mesh_path") or item.get("resolved_mesh_path") or resolved_source_path
        mesh_path = Path(final_gui_mesh_path) if final_gui_mesh_path else Path()
        ext = mesh_path.suffix.lower() if final_gui_mesh_path else ""
        exists = bool(final_gui_mesh_path) and mesh_path.exists()

        parser_attempted = exists and ext in SUPPORTED_EXTENSIONS
        parser_success = False
        parser_type = "none"
        tri_count = 0
        bounds_min = [0.0, 0.0, 0.0]
        bounds_max = [0.0, 0.0, 0.0]
        fallback_reason = ""

        if not final_gui_mesh_path:
            fallback_reason = "missing path"
        elif not exists:
            fallback_reason = "missing path"
        elif ext not in SUPPORTED_EXTENSIONS:
            fallback_reason = "unsupported extension"
        else:
            parser_success, tri_count, bounds_min, bounds_max, parser_type = _extract_tris_bounds(mesh_path, ext)
            if not parser_success:
                fallback_reason = "parse fail"
            elif tri_count == 0:
                fallback_reason = "zero tris"

        span = [bounds_max[i] - bounds_min[i] for i in range(3)]
        rejected_by_bounds_guard = bool(item.get("rejected_by_bounds_guard", False))
        if not fallback_reason and (rejected_by_bounds_guard or max(span) > 1.0e4):
            fallback_reason = "unreasonable bounds"

        rendered_as_mesh = bool(item.get("rendered_as_mesh", False))
        if not fallback_reason and not rendered_as_mesh:
            fallback_reason = "mode fallback"

        reason_key = fallback_reason or "rendered"
        reason_counter[reason_key] += 1
        grouped_ids[reason_key].append(k)

        results.append({
            "id": item.get("id", k),
            "link": item.get("link"),
            "visual": item.get("visual") or item.get("name"),
            "package_uri": item.get("package_uri"),
            "source_path": source_path,
            "resolved_source_path": resolved_source_path,
            "final_gui_mesh_path": final_gui_mesh_path,
            "file_exists": exists,
            "extension": ext,
            "parser_attempted": parser_attempted,
            "parser_success": parser_success,
            "parser_type": parser_type,
            "triangle_count": tri_count,
            "local_bounds_min": bounds_min,
            "local_bounds_max": bounds_max,
            "local_bounds_span": span,
            "mesh_scale": _as_vec3(item.get("mesh_scale") or item.get("scale"), [1.0, 1.0, 1.0]),
            "final_scene_pose": item.get("final_scene_pose") or item.get("pose") or {},
            "rejected_by_bounds_guard": rejected_by_bounds_guard,
            "rendered_as_mesh": rendered_as_mesh,
            "fallback_reason": fallback_reason,
        })

    total = len(results)
    rendered = reason_counter.get("rendered", 0)
    summary = {
        "scene": args.scene,
        "index_json": str(index_path),
        "gui_smoke_json": str(args.gui_smoke_json) if args.gui_smoke_json else None,
        "runtime_debug_json": str(args.runtime_debug_json) if args.runtime_debug_json else None,
        "total_visual_items": total,
        "rendered_mesh_items": rendered,
        "non_rendered_items": total - rendered,
        "ratio_rendered": f"{rendered}/{total}" if total else "0/0",
        "grouped_failure_reasons": [
            {"reason": reason, "count": count, "ratio": f"{count}/{total}", "visual_ids": grouped_ids[reason]}
            for reason, count in sorted(reason_counter.items(), key=lambda kv: (-kv[1], kv[0]))
        ],
        "visual_items": sorted(results, key=lambda x: str(x.get("id") or "")),
    }

    output = args.output or (args.repo_root / "build" / f"scene3d_runtime_mesh_rendering_audit_{args.scene}.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {output} ({rendered}/{total} rendered as mesh)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
