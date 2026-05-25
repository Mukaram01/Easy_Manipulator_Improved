from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Pose:
    tx: float
    ty: float
    tz: float


def _compose(a: Pose, b: Pose) -> Pose:
    return Pose(a.tx + b.tx, a.ty + b.ty, a.tz + b.tz)


def _transform_bounds(local_min: tuple[float, float, float], local_max: tuple[float, float, float], pose: Pose) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    return (
        (local_min[0] + pose.tx, local_min[1] + pose.ty, local_min[2] + pose.tz),
        (local_max[0] + pose.tx, local_max[1] + pose.ty, local_max[2] + pose.tz),
    )


def _compute_robot_aabb(mesh_min: tuple[float, float, float], mesh_max: tuple[float, float, float], scale: tuple[float, float, float], pose: Pose) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    scaled_min = (mesh_min[0] * scale[0], mesh_min[1] * scale[1], mesh_min[2] * scale[2])
    scaled_max = (mesh_max[0] * scale[0], mesh_max[1] * scale[1], mesh_max[2] * scale[2])
    return _transform_bounds(scaled_min, scaled_max, pose)


def _scene3d_visual_quality_pass(*, robot_only_count: int, aabb_valid: bool, mesh_loaded: bool) -> bool:
    # Regression guard: robot-only counter is necessary but not sufficient.
    return robot_only_count > 0 and aabb_valid and mesh_loaded


def _persist_base_pose_target(scene_dir: Path) -> dict:
    authoring = scene_dir / "layout" / "workcell_studio_layout.yaml"
    if authoring.exists():
        return {"ok": True, "target": str(authoring), "diagnostic": ""}
    return {
        "ok": False,
        "target": None,
        "diagnostic": "base_pose_persistence_blocked_missing_authoring_layout",
    }


def test_visual_origin_applies_expected_world_bounds_shift() -> None:
    local_min = (-0.5, -0.5, -0.5)
    local_max = (0.5, 0.5, 0.5)
    link_pose = Pose(1.0, 2.0, 3.0)
    visual_origin = Pose(0.25, -0.5, 1.0)

    world_pose = _compose(link_pose, visual_origin)
    world_min, world_max = _transform_bounds(local_min, local_max, world_pose)

    assert world_pose == Pose(1.25, 1.5, 4.0)
    assert world_min == (0.75, 1.0, 3.5)
    assert world_max == (1.75, 2.0, 4.5)


def test_transform_chain_multi_link_end_world_pose_is_deterministic() -> None:
    base = Pose(1.0, 0.0, 0.0)
    joint1 = Pose(0.0, 2.0, 0.0)
    joint2 = Pose(0.0, 0.0, 3.0)
    tool = Pose(-0.5, 0.5, 0.25)

    end_pose = _compose(_compose(_compose(base, joint1), joint2), tool)
    assert end_pose == Pose(0.5, 2.5, 3.25)


def test_robot_aabb_uses_mesh_bounds_scale_and_pose() -> None:
    mesh_min = (-1.0, -2.0, -0.5)
    mesh_max = (2.0, 1.0, 0.5)
    scale = (0.5, 2.0, 3.0)
    pose = Pose(10.0, -1.0, 4.0)

    world_min, world_max = _compute_robot_aabb(mesh_min, mesh_max, scale, pose)
    assert world_min == (9.5, -5.0, 2.5)
    assert world_max == (11.0, 1.0, 5.5)


def test_robot_only_count_is_not_visual_pass_without_aabb_and_mesh_load() -> None:
    assert _scene3d_visual_quality_pass(robot_only_count=3, aabb_valid=True, mesh_loaded=True) is True
    assert _scene3d_visual_quality_pass(robot_only_count=3, aabb_valid=False, mesh_loaded=True) is False
    assert _scene3d_visual_quality_pass(robot_only_count=3, aabb_valid=True, mesh_loaded=False) is False


def test_base_pose_persistence_target_requires_authoring_layout_or_blocked_diagnostic(tmp_path: Path) -> None:
    scene_dir = tmp_path / "scene"
    (scene_dir / "layout").mkdir(parents=True)

    blocked = _persist_base_pose_target(scene_dir)
    assert blocked["ok"] is False
    assert blocked["diagnostic"] == "base_pose_persistence_blocked_missing_authoring_layout"

    authoring = scene_dir / "layout" / "workcell_studio_layout.yaml"
    authoring.write_text("items: []\n", encoding="utf-8")
    ok = _persist_base_pose_target(scene_dir)
    assert ok["ok"] is True
    assert ok["target"] == str(authoring)
    assert ok["diagnostic"] == ""
