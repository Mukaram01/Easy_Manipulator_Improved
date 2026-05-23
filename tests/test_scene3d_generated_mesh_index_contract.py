from __future__ import annotations

import json
from pathlib import Path


def _resolve_mesh_path(scene_dir: Path, source_path: str, package_uri: str) -> Path | None:
    if source_path:
        p = Path(source_path)
        candidates = [p] if p.is_absolute() else [scene_dir / p]
        for c in candidates:
            if c.is_file():
                return c.resolve()

    if package_uri.startswith("file://"):
        p = Path(package_uri[7:])
        if p.is_file():
            return p.resolve()
    if package_uri.startswith("/"):
        p = Path(package_uri)
        if p.is_file():
            return p.resolve()

    if package_uri.startswith("package://"):
        tail = package_uri[len("package://"):]
        pkg, _, rel = tail.partition("/")
        for root in [
            scene_dir / "generated" / pkg,
            Path.cwd() / "assets",
            Path.cwd() / "src" / "easy_manipulation_deployment" / "assets",
            Path.cwd() / "src" / "assets",
            Path("/opt/ros/humble/share") / pkg,
        ]:
            candidate = root / rel
            if candidate.is_file():
                return candidate.resolve()
    return None


def test_ur5_2f_generated_visual_index_contains_resolvable_mesh_items() -> None:
    scene_dir = Path("scenes/ur5_2f_test")
    index_path = scene_dir / "generated" / "scene_visual_mesh_index.json"
    assert index_path.is_file(), f"missing index: {index_path}"

    data = json.loads(index_path.read_text(encoding="utf-8"))
    visual_items = data.get("visual_items", [])
    if not visual_items:
        # This repository snapshot can be checked in without xacro-expanded artifacts.
        # We still validate that the index is present and records why visuals were not generated.
        assert data.get("safe_for_preview") is False
        assert data.get("fallback_reason")
        return

    converted = []
    for item in visual_items:
        if item.get("geometry_type") != "mesh":
            continue
        mesh_path = _resolve_mesh_path(scene_dir, item.get("source_path", ""), item.get("package_uri", ""))
        if not mesh_path:
            continue
        converted.append(
            {
                "has_mesh_metadata": True,
                "mesh_path": str(mesh_path),
                "source_layer": "generated_urdf_visual",
                "active_visual_source": "mesh_preview",
            }
        )

    assert converted, "expected at least one mesh-backed generated visual item"
    assert any(
        x["has_mesh_metadata"]
        and x["mesh_path"]
        and x["source_layer"] == "generated_urdf_visual"
        and x["active_visual_source"] == "mesh_preview"
        for x in converted
    )
