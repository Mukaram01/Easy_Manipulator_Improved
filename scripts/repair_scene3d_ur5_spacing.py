#!/usr/bin/env python3
"""Repair UR5 Scene3D visual index rows when adjacent links are too far apart."""
from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from scripts.scene3d_ur5_preview_rows import REQUIRED_UR5_LINKS, REQUIRED_UR5_VISUALS, make_ur5_preview_item

MAX_ADJACENT_DISTANCE_M = 0.75
POSE_FIELDS = ("baked_world_visual_pose", "expected_visual_pose", "link_world_pose", "world_pose", "pose")


def _text(item: dict[str, Any]) -> str:
    keys = ("link", "link_name", "canonical_link_name", "object_name", "visual_index_link", "visual_index_link_name", "id", "item_id")
    return "|".join(str(item.get(key) or "") for key in keys).lower()


def _xyz(item: dict[str, Any], field: str) -> list[float] | None:
    pose = item.get(field)
    raw = pose.get("xyz") if isinstance(pose, dict) else None
    if not isinstance(raw, list) or len(raw) < 3:
        return None
    try:
        xyz = [float(raw[i]) for i in range(3)]
    except Exception:
        return None
    return xyz if all(math.isfinite(v) for v in xyz) else None


def _positions(items: list[dict[str, Any]]) -> dict[str, list[float]]:
    out: dict[str, list[float]] = {}
    for item in items:
        for link in REQUIRED_UR5_LINKS:
            if link in out or link.lower() not in _text(item):
                continue
            for field in POSE_FIELDS:
                xyz = _xyz(item, field)
                if xyz is not None:
                    out[link] = xyz
                    break
    return out


def _distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def spacing_reasons(items: list[dict[str, Any]]) -> list[str]:
    pos = _positions(items)
    reasons: list[str] = []
    for prev, cur in zip(REQUIRED_UR5_LINKS, REQUIRED_UR5_LINKS[1:]):
        if prev not in pos or cur not in pos:
            continue
        dist = _distance(pos[prev], pos[cur])
        if dist > MAX_ADJACENT_DISTANCE_M:
            reasons.append(f"{prev}->{cur}:{dist:.3f}m")
    return reasons


def repair_index(path: Path, *, write_debug_repair: bool = False) -> tuple[bool, list[str]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    items = data.get("visual_items") if isinstance(data.get("visual_items"), list) else []
    items = [item for item in items if isinstance(item, dict)]
    reasons = spacing_reasons(items)
    if not reasons:
        return False, []

    replaced = set(REQUIRED_UR5_LINKS)
    repaired = [make_ur5_preview_item(spec, idx) for idx, spec in enumerate(REQUIRED_UR5_VISUALS)]
    kept = [item for item in items if not any(link.lower() in _text(item) for link in replaced)]
    visual_items = repaired + kept

    data["visual_items"] = visual_items
    if isinstance(data.get("items"), list):
        data["items"] = visual_items
    data["visual_count"] = len(visual_items)
    data["candidate_mesh_count"] = len(visual_items)
    data["emitted_visual_count"] = len(visual_items)
    data["safe_for_preview"] = True
    data["stale_index"] = False
    data["stale_reasons"] = []
    data["ur5_runtime_repair_applied"] = True
    data["ur5_runtime_repair_mode"] = "stable_primitive_builder_preview"
    data["ur5_runtime_repair_added_links"] = list(REQUIRED_UR5_LINKS)
    data["ur5_runtime_repair_reasons"] = ["implausible_adjacent_distance:" + reason for reason in reasons]
    data["repair_generated_at"] = datetime.now(timezone.utc).isoformat()
    if not write_debug_repair:
        return True, reasons
    # Debug 3D Preview exception: this rewrites only the generated Scene3D
    # preview index for local diagnostics. It does not modify source-of-truth
    # layout, cell-definition, URDF/xacro, launch, or planning artifacts.
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    return True, reasons


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("index_path", type=Path)
    parser.add_argument(
        "--write-debug-repair",
        action="store_true",
        help=(
            "Rewrite generated/scene_visual_mesh_index.json for Debug 3D Preview only. "
            "Default is read-only diagnostics."
        ),
    )
    args = parser.parse_args()
    changed, reasons = repair_index(args.index_path.resolve(), write_debug_repair=args.write_debug_repair)
    if changed:
        action = "wrote Debug 3D Preview repair" if args.write_debug_repair else "diagnostic only; rerun with --write-debug-repair to mutate generated preview index"
        print("[repair_scene3d_ur5_spacing] UR5 spacing issue: " + ";".join(reasons) + f" ({action}; source-of-truth layout/planning artifacts unchanged)")
    else:
        print("[repair_scene3d_ur5_spacing] no UR5 spacing repair needed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
