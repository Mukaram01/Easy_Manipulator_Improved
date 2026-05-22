#!/usr/bin/env python3
"""Report existing workcell_builder/scenes/assets path layout without modifying logic."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from scripts.workcell_studio_path_resolver import resolve_repo_root

MESH_EXTS = {".stl", ".dae", ".obj"}


def _find_scene_packages(scenes_dirs: list[Path]) -> list[Path]:
    out: list[Path] = []
    for scene_dir in scenes_dirs:
        out.extend([p for p in scene_dir.iterdir() if p.is_dir()])
    return sorted(set(out))


def build_report(repo_root: Path) -> dict[str, Any]:
    workcell_builder_path = repo_root / "workcell_builder" / "workcell_builder"
    default_assets = workcell_builder_path / "assets"
    templates = workcell_builder_path / "templates"

    scenes_dirs = sorted({p.parent for p in repo_root.rglob("scene_manifest.yaml") if p.parent.is_dir()})
    if (repo_root / "scenes").is_dir() and (repo_root / "scenes") not in scenes_dirs:
        scenes_dirs.append(repo_root / "scenes")

    assets_dirs = sorted({p for p in repo_root.rglob("assets") if p.is_dir()})
    scene_packages = _find_scene_packages([d for d in scenes_dirs if d.is_dir()])

    mesh_count = 0
    urdf_xacro_count = 0
    for pkg in scene_packages:
        for file in pkg.rglob("*"):
            if not file.is_file():
                continue
            suffix = file.suffix.lower()
            if suffix in MESH_EXTS:
                mesh_count += 1
            if suffix == ".xacro" or ".urdf" in file.name.lower():
                urdf_xacro_count += 1

    return {
        "repo_root": str(repo_root),
        "workcell_builder_package_path": str(workcell_builder_path),
        "default_assets_directory": str(default_assets) if default_assets.is_dir() else None,
        "templates_directory": str(templates) if templates.is_dir() else None,
        "scenes_directories": [str(p) for p in sorted(scenes_dirs)],
        "workcell_assets_directories": [str(p) for p in assets_dirs],
        "scene_packages": [str(p) for p in scene_packages],
        "counts": {
            "scene_packages": len(scene_packages),
            "meshes_in_scene_packages": mesh_count,
            "urdf_xacro_in_scene_packages": urdf_xacro_count,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=resolve_repo_root())
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    payload = build_report(args.repo_root.resolve())
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(f"workcell_builder package: {payload['workcell_builder_package_path']}")
        print(f"default assets: {payload['default_assets_directory']}")
        print(f"templates: {payload['templates_directory']}")
        print(f"scene directories: {len(payload['scenes_directories'])}")
        print(f"scene packages: {payload['counts']['scene_packages']}")
        print(f"scene meshes: {payload['counts']['meshes_in_scene_packages']}")
        print(f"scene urdf/xacro: {payload['counts']['urdf_xacro_in_scene_packages']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
