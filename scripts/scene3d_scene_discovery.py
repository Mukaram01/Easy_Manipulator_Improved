#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

IGNORED_SCENE_FOLDER_NAMES = {
    "generated",
    "layout",
    "task",
    "plan_preview",
    "config",
    "urdf",
    "launch",
}


def _detect_layers(scene_path: Path) -> list[str]:
    layers: set[str] = set()
    if (scene_path / "layout/workcell_studio_layout.yaml").exists():
        layers.add("editable_layout")
    if (scene_path / "generated/scene_visual_mesh_index.json").exists():
        layers.add("mesh_preview")
    if (scene_path / "layout/workcell_studio_layout.generated.yaml").exists() or (scene_path / "urdf/scene.urdf.xacro").exists():
        layers.add("locked_generated_urdf_visual")
    if (scene_path / "generated/scene_preview_metadata.json").exists():
        layers.add("overlay")
    return sorted(layers)


def discover_scene3d_scenes(scenes_root: Path) -> list[dict]:
    discoveries: list[dict] = []
    if not scenes_root.exists():
        return discoveries

    for scene_path in sorted([p for p in scenes_root.iterdir() if p.is_dir()], key=lambda p: p.name):
        detected_files: dict[str, str | None] = {}
        blockers: list[str] = []
        name = scene_path.name

        if name.startswith("."):
            discoveries.append({
                "scene": name,
                "scene_path": str(scene_path),
                "detected_files": detected_files,
                "source_layers_found": [],
                "status": "IGNORED_NON_SCENE",
                "blockers": [],
                "ignore_reason": "hidden_or_system_folder",
            })
            continue
        if name in IGNORED_SCENE_FOLDER_NAMES:
            discoveries.append({
                "scene": name,
                "scene_path": str(scene_path),
                "detected_files": detected_files,
                "source_layers_found": [],
                "status": "IGNORED_NON_SCENE",
                "blockers": [],
                "ignore_reason": "helper_or_artifact_folder_name",
            })
            continue

        pkg = scene_path / "package.xml"
        manifest = scene_path / "scene_manifest.yaml"
        env = scene_path / "environment.yaml"
        cell = scene_path / "cell_definition.yaml"
        demo = scene_path / "launch/demo.launch.py"
        detected_files = {
            "package.xml": str(pkg) if pkg.exists() else None,
            "scene_manifest.yaml": str(manifest) if manifest.exists() else None,
            "environment.yaml": str(env) if env.exists() else None,
            "cell_definition.yaml": str(cell) if cell.exists() else None,
            "launch/demo.launch.py": str(demo) if demo.exists() else None,
        }

        source_layers = _detect_layers(scene_path)

        strong_identity = pkg.exists() or manifest.exists() or (env.exists() and cell.exists()) or demo.exists()
        if not strong_identity:
            status = "IGNORED_NON_SCENE"
            if any((scene_path / n).exists() for n in IGNORED_SCENE_FOLDER_NAMES):
                blockers.append("contains helper/artifact subfolders but no strong scene identity signal")
            else:
                blockers.append("missing strong scene identity signal")
        else:
            if pkg.exists() and manifest.exists():
                status = "PASS"
            else:
                status = "LEGACY_INCOMPLETE"
                blockers.append("scene has strong identity but is missing package.xml or scene_manifest.yaml")

        discoveries.append(
            {
                "scene": scene_path.name,
                "scene_path": str(scene_path),
                "detected_files": detected_files,
                "source_layers_found": source_layers,
                "status": status,
                "blockers": blockers,
            }
        )

    return discoveries
