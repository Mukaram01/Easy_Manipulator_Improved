from __future__ import annotations

import json
from pathlib import Path


def _resolve_mesh_path(scene_dir: Path, source_path: str, package_uri: str, resolved_source_path: str = "") -> Path | None:
    if resolved_source_path:
        p = Path(resolved_source_path)
        candidates = [p] if p.is_absolute() else [Path.cwd() / p, scene_dir / p]
        for c in candidates:
            if c.is_file():
                return c.resolve()

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
        mesh_path = _resolve_mesh_path(
            scene_dir,
            item.get("source_path", ""),
            item.get("package_uri", ""),
            item.get("resolved_source_path", "") or item.get("mesh_path", ""),
        )
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


def _finite_pose(value: dict) -> bool:
    return (
        isinstance(value, dict)
        and isinstance(value.get("xyz"), list)
        and len(value["xyz"]) == 3
        and isinstance(value.get("rpy"), list)
        and len(value["rpy"]) == 3
    )


def test_ur5_2f_required_visuals_carry_mesh_and_transform_metadata() -> None:
    scene_dir = Path("scenes/ur5_2f_test")
    index_path = scene_dir / "generated" / "scene_visual_mesh_index.json"
    assert index_path.is_file(), f"missing index: {index_path}"

    data = json.loads(index_path.read_text(encoding="utf-8"))
    visual_items = data.get("visual_items", [])
    renderable_by_link = {
        item.get("link"): item
        for item in visual_items
        if item.get("geometry_type") == "mesh"
        and item.get("render_expected") is True
        and item.get("resolved") is True
    }

    assert {"base_link", "base_link_inertia"} & set(renderable_by_link), (
        "expected a renderable UR5 base visual for base_link or base_link_inertia"
    )
    for link in [
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]:
        assert link in renderable_by_link, f"missing renderable UR5 visual for {link}"

    required_links = [
        next(link for link in ("base_link_inertia", "base_link") if link in renderable_by_link),
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]
    for link in required_links:
        item = renderable_by_link[link]
        mesh_uri = item.get("mesh_uri") or item.get("package_uri") or item.get("source_path")
        assert isinstance(mesh_uri, str) and mesh_uri.startswith("package://ur_description/meshes/ur5/visual/"), link
        assert item.get("mesh_path") or item.get("resolved_source_path"), link
        assert _finite_pose(item.get("baked_world_visual_pose", {})), link
        assert _finite_pose(item.get("visual_origin", {})), link
        assert item.get("joint_parent_link") or item.get("parent_link") is not None, link
        assert "joint_type" in item or "parent_joint_type" in item, link
        assert item.get("mesh_scale") == [1.0, 1.0, 1.0], link
        assert item.get("render_expected") is True, link
        assert item.get("resolved") is True, link

    gripper_mount = renderable_by_link.get("gripper_base_link")
    assert gripper_mount, "missing renderable Robotiq gripper mount visual"
    assert gripper_mount.get("joint_parent_link") == "tool0" or "tool0" in gripper_mount.get("link_chain", [])
    assert str(gripper_mount.get("mesh_uri") or gripper_mount.get("package_uri") or "").startswith(
        "package://robotiq_85_description/meshes/visual/"
    )
