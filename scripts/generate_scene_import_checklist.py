#!/usr/bin/env python3
"""Generate a checklist for importing meshes via the existing workcell_builder GUI flow."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


def _slug(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_]+", "_", value.lower()).strip("_") or "imported_object"


def _checklist_for_mesh(mesh_path: Path) -> list[str]:
    exists = mesh_path.exists()
    asset_name = _slug(mesh_path.stem)
    object_id = f"{asset_name}_obj"
    return [
        f"## Mesh: `{mesh_path}`",
        "",
        f"- [ ] File exists: **{'YES' if exists else 'NO'}**",
        f"- [ ] Recommended asset name: `{asset_name}`",
        f"- [ ] Recommended scene object ID: `{object_id}`",
        f"- [ ] Recommended visual entry: `mesh filename=\"{mesh_path.name}\"`",
        f"- [ ] Recommended collision entry: `mesh filename=\"{mesh_path.name}\"` (or simplified collision mesh)",
        "- [ ] Units check: STL may be millimetres or metres; verify scale in workcell_builder before saving.",
        "- [ ] Origin check: confirm mesh origin/alignment before final placement.",
        "- [ ] Collision check: if mesh is complex, use simplified collision geometry.",
        "- [ ] Frame check: place relative to `world` or intended parent link.",
        "- [ ] Package path check: avoid absolute filesystem paths in reusable scene packages.",
        "",
    ]


def build_checklist(meshes: list[Path]) -> str:
    lines = [
        "# Scene Import Checklist (Existing workcell_builder flow)",
        "",
        "This checklist is a helper layer only. It does not replace the existing GUI or scene format.",
        "",
        "## Operator steps in current workcell_builder",
        "",
        "1. Open/create workcell project.",
        "2. Open existing scene or create a new scene package in `scenes/`.",
        "3. Add object/link and import mesh through existing visual geometry dialog.",
        "4. Add collision geometry for each visual mesh.",
        "5. Set frame/pose relative to world or parent link.",
        "6. Save scene and run `check_scene_readiness.py`.",
        "",
    ]

    for mesh in meshes:
        lines.extend(_checklist_for_mesh(mesh))
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("meshes", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("reports/scene_import_checklist.md"))
    args = parser.parse_args()

    text = build_checklist(args.meshes)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text, encoding="utf-8")
    print(f"PASS: wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
