#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

REQUIRED_SCENE_SIGNALS = {
    "package.xml": "package.xml",
    "launch/demo.launch.py": "launch/demo.launch.py",
    "scene_manifest.yaml": "scene_manifest.yaml",
    "environment.yaml": "environment.yaml",
    "cell_definition.yaml": "cell_definition.yaml",
    "urdf/scene.urdf.xacro": "urdf/scene.urdf.xacro",
    "layout/workcell_studio_layout.yaml": "layout/workcell_studio_layout.yaml",
    "generated/scene_visual_mesh_index.json": "generated/scene_visual_mesh_index.json",
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
        missing_reasons: list[str] = []
        for key, rel in REQUIRED_SCENE_SIGNALS.items():
            candidate = scene_path / rel
            if candidate.exists():
                detected_files[key] = str(candidate)
            else:
                detected_files[key] = None
                missing_reasons.append(f"missing required signal: {candidate}")

        source_layers = _detect_layers(scene_path)

        if not (scene_path / "package.xml").exists() or not (scene_path / "scene_manifest.yaml").exists():
            status = "BLOCKED"
        elif missing_reasons:
            status = "LEGACY_INCOMPLETE"
        else:
            status = "PASS"

        discoveries.append(
            {
                "scene": scene_path.name,
                "scene_path": str(scene_path),
                "detected_files": detected_files,
                "source_layers_found": source_layers,
                "status": status,
                "blockers": missing_reasons,
            }
        )

    return discoveries
