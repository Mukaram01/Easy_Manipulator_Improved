from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from scripts.validate_ur5_scene3d_transform_plausibility import validate_index


SCRIPT = Path("scripts/validate_ur5_scene3d_transform_plausibility.py")


def _write_index(
    path: Path,
    poses: dict[str, list[float]],
    *,
    visual_poses: dict[str, list[float]] | None = None,
) -> Path:
    visual_poses = visual_poses or poses
    visual_items = [
        {
            "link": link,
            "link_world_pose": {"xyz": xyz, "rpy": [0.0, 0.0, 0.0]},
            "pose": {"xyz": visual_poses.get(link, xyz), "rpy": [0.0, 0.0, 0.0]},
        }
        for link, xyz in poses.items()
    ]
    visual_items.append(
        {
            "link": "gripper_base_link",
            "source_path": "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae",
            "link_world_pose": {"xyz": poses["wrist_3_link"], "rpy": [0.0, 0.0, 0.0]},
            "pose": {
                "xyz": visual_poses.get("wrist_3_link", poses["wrist_3_link"]),
                "rpy": [0.0, 0.0, 0.0],
            },
        }
    )
    path.write_text(json.dumps({"visual_items": visual_items}), encoding="utf-8")
    return path


def _valid_ur5_poses() -> dict[str, list[float]]:
    return {
        "base_link_inertia": [0.0, 0.0, 0.0],
        "shoulder_link": [0.0, 0.0, 0.089159],
        "upper_arm_link": [0.0, 0.13585, 0.089159],
        "forearm_link": [0.425, 0.0165, 0.089159],
        "wrist_1_link": [0.81725, 0.01615, 0.089159],
        "wrist_2_link": [0.81725, 0.10915, 0.089509],
        "wrist_3_link": [0.81725, 0.10965, -0.005491],
    }


def test_repository_ur5_2f_scene_visual_index_is_plausible() -> None:
    index = Path("scenes/ur5_2f_test/generated/scene_visual_mesh_index.json")
    assert index.is_file(), f"missing generated mesh index: {index}"

    report = validate_index(index)

    assert report["ok"], report["errors"]
    assert report["distances_m"]["base_to_origin"] <= 0.25
    assert report["distances_m"]["upper_arm_link_to_forearm_link"] <= 0.55


def test_validator_cli_returns_zero_for_plausible_visual_items(tmp_path: Path) -> None:
    index = _write_index(tmp_path / "scene_visual_mesh_index.json", _valid_ur5_poses())

    run = subprocess.run(
        [sys.executable, str(SCRIPT), "--index", str(index), "--json"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert run.returncode == 0, run.stdout + run.stderr
    report = json.loads(run.stdout)
    assert report["ok"] is True
    assert not report["errors"]


def test_validator_cli_returns_nonzero_for_implausible_adjacent_distance(
    tmp_path: Path,
) -> None:
    poses = _valid_ur5_poses()
    poses["forearm_link"] = [9.0, 0.0, 0.0]
    index = _write_index(tmp_path / "scene_visual_mesh_index.json", poses)

    run = subprocess.run(
        [sys.executable, str(SCRIPT), "--index", str(index), "--json"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert run.returncode == 1
    report = json.loads(run.stdout)
    assert report["ok"] is False
    assert any("upper_arm_link->forearm_link" in error for error in report["errors"])


def test_validator_rejects_nan_or_infinite_required_link_pose(tmp_path: Path) -> None:
    poses = _valid_ur5_poses()
    poses["wrist_3_link"] = [float("nan"), 0.0, 0.0]
    index = _write_index(tmp_path / "scene_visual_mesh_index.json", poses)

    report = validate_index(index)

    assert report["ok"] is False
    assert any("wrist_3_link" in error for error in report["errors"])


def test_validator_rejects_exploded_visuals_even_when_link_frames_are_plausible(
    tmp_path: Path,
) -> None:
    poses = _valid_ur5_poses()
    visual_poses = dict(poses)
    visual_poses["forearm_link"] = [8.0, 0.0, 0.0]
    index = _write_index(
        tmp_path / "scene_visual_mesh_index.json", poses, visual_poses=visual_poses
    )

    report = validate_index(index)

    assert report["ok"] is False
    assert any(
        "rendered visual center distance upper_arm_link->forearm_link" in error
        for error in report["errors"]
    )


def test_validator_rejects_collapsed_visuals_even_when_link_frames_are_plausible(
    tmp_path: Path,
) -> None:
    poses = _valid_ur5_poses()
    visual_poses = dict(poses)
    visual_poses["forearm_link"] = visual_poses["upper_arm_link"]
    index = _write_index(
        tmp_path / "scene_visual_mesh_index.json", poses, visual_poses=visual_poses
    )

    report = validate_index(index)

    assert report["ok"] is False
    assert any(
        "rendered visual centers upper_arm_link->forearm_link are collapsed" in error
        for error in report["errors"]
    )
