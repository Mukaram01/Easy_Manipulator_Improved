#!/usr/bin/env python3
"""Validate that scene_visual_mesh_index visual_0 maps to the Scene3D PreviewItem contract.

This is a focused regression guard for the UR5 base handoff from the generated
URDF visual mesh index into Workcell Builder preview assembly. It does not audit
renderer mesh loading and does not create fallback geometry.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _token(value: Any) -> str:
    return str(value or "").strip().lower().replace("-", "_").replace(" ", "_")


def _preview_item_from_visual_row(row: dict[str, Any], source_row_index: int) -> dict[str, Any]:
    link = str(row.get("link_name") or row.get("link") or "").strip()
    visual = str(row.get("visual_name") or row.get("visual") or row.get("id") or "").strip()
    row_index = str(row.get("source_row_index") if row.get("source_row_index") is not None else source_row_index)
    package_uri = str(row.get("package_uri") or row.get("mesh_uri") or "").strip()
    source_mix = "|".join(
        str(row.get(key) or "")
        for key in ("package_uri", "mesh_uri", "source_path", "resolved_source_path", "resolved_path")
    ).lower()
    category = "robot/ur5" if "ur_description/meshes/ur5/" in source_mix else "URDF Visual"
    return {
        "id": f"generated_urdf::{_token(link) or 'link_missing'}::{_token(visual) or 'visual_missing'}::{_token(row_index)}",
        "canonical_link_name": link,
        "source_layer": "locked_generated_urdf_visual",
        "active_visual_source": "mesh_preview" if _token(row.get("geometry_type")) == "mesh" else "urdf_primitive",
        "category": category,
        "type": _token(row.get("geometry_type")),
        "package_uri": package_uri,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "index",
        nargs="?",
        default="scenes/ur5_2f_test/generated/scene_visual_mesh_index.json",
        help="Path to generated/scene_visual_mesh_index.json",
    )
    args = parser.parse_args()
    path = Path(args.index)
    payload = json.loads(path.read_text())
    items = payload.get("visual_items")
    if not isinstance(items, list) or not items:
        raise SystemExit(f"FAIL: {path} has no visual_items")
    row = items[0]
    preview = _preview_item_from_visual_row(row, 0)
    checks = {
        "id_includes_visual0_base": "generated_urdf::base_link_inertia::visual_0" in preview["id"],
        "canonical_link_name": preview["canonical_link_name"] == "base_link_inertia",
        "source_layer": preview["source_layer"] == "locked_generated_urdf_visual",
        "active_visual_source": preview["active_visual_source"] == "mesh_preview",
        "category": preview["category"] in {"robot", "robot/ur5"},
        "type": preview["type"] == "mesh",
        "package_uri": "ur_description/meshes/ur5/visual/base.dae" in preview["package_uri"],
    }
    failures = [name for name, ok in checks.items() if not ok]
    if failures:
        print(json.dumps({"status": "FAIL", "failures": failures, "visual_0": row, "preview_item": preview}, indent=2))
        return 1
    print(json.dumps({"status": "PASS", "preview_item": preview}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
