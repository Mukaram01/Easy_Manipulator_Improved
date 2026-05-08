#!/usr/bin/env python3
from __future__ import annotations
import argparse
import json
from pathlib import Path
from typing import Any
import yaml

GROUPS = ["robots", "grippers", "tools", "objects", "tables", "workbenches", "sensors", "cameras", "conveyors", "fixtures", "machines", "safety", "custom"]

def _root() -> Path:
    return Path(__file__).resolve().parents[1]

def _group(item: dict[str, Any]) -> str:
    c = (item.get("category") or "").lower()
    i = (item.get("id") or "").lower()
    if c == "robots": return "robots"
    if c in {"end_effectors", "grippers"}: return "grippers"
    if c == "tools": return "tools"
    if c == "sensors": return "sensors"
    if "camera" in i: return "cameras"
    if c == "fixtures": return "fixtures"
    if c == "machines": return "machines"
    if c == "safety": return "safety"
    if "table" in i: return "tables"
    if "bench" in i: return "workbenches"
    if "conveyor" in i: return "conveyors"
    if c in {"environment", "objects"}: return "objects"
    return "custom"

def generate(root: Path | None = None) -> dict[str, Any]:
    root = root or _root()
    data = yaml.safe_load((root / "workcell_studio_catalog" / "catalog.yaml").read_text(encoding="utf-8")) or {}
    out = {k: [] for k in GROUPS}
    for item in data.get("items", []):
        out[_group(item)].append({
            "id": item.get("id"), "display_name": item.get("display_name"), "category": item.get("category"),
            "family": item.get("family"), "support_status": item.get("support_status"), "runtime_status": item.get("runtime_status"),
            "package_name": item.get("urdf_package") or item.get("moveit_config_package") or "",
            "urdf_or_xacro": item.get("urdf_path") or "", "mesh_path": item.get("mesh_path") or item.get("preview_mesh_path") or "",
            "thumbnail_path": item.get("thumbnail_path") or "", "notes": item.get("notes") or "", "tags": item.get("tags") or []
        })
    for g, i, d in [("objects", "cube_placeholder", "Cube Placeholder"), ("objects", "bin_placeholder", "Bin Placeholder"), ("conveyors", "conveyor_placeholder", "Conveyor Placeholder"), ("fixtures", "camera_mount_placeholder", "Camera Mount Placeholder")]:
        if not any(x["id"] == i for x in out[g]):
            out[g].append({"id": i, "display_name": d, "category": g, "family": "placeholder", "support_status": "preview_only", "runtime_status": "preview_only", "package_name": "", "urdf_or_xacro": "", "mesh_path": "", "thumbnail_path": "", "notes": "Primitive placeholder for GUI visibility.", "tags": ["placeholder"]})
    return out

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, default=_root())
    ap.add_argument("--output", type=Path)
    a = ap.parse_args()
    payload = generate(a.root.resolve())
    out = a.output or (a.root / "workcell_studio_catalog" / "generated" / "workcell_builder_gui_catalog.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(out)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
