from __future__ import annotations

from pathlib import Path

import yaml

from tools.workcell_studio_streamlit import backend


def test_robot_base_pose_persistence_uses_valid_authoring_target(tmp_path: Path) -> None:
    scene = tmp_path / "scene"
    scene.mkdir(parents=True)
    (scene / "environment.yaml").write_text(
        yaml.safe_dump(
            {
                "robot": {
                    "robot_mount": {
                        "pose": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
                    }
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    result = backend.write_robot_base_pose(scene, {"xyz": [0.1, 0.2, 0.3], "rpy": [0.0, 0.1, 0.2]})
    assert result["status"] == "ok"
    assert result["target"] == "environment_yaml"
    read_back = backend.read_robot_base_pose(scene)
    assert read_back["pose"]["xyz"] == [0.1, 0.2, 0.3]


def test_robot_base_pose_blocked_diagnostic_is_clear_without_authoring_target(tmp_path: Path) -> None:
    scene = tmp_path / "scene"
    scene.mkdir(parents=True)
    (scene / "environment.yaml").write_text("robot: {name: ur5}\n", encoding="utf-8")
    result = backend.write_robot_base_pose(scene, {"xyz": [0, 0, 0], "rpy": [0, 0, 0]})
    assert result["status"] == "blocked"
    assert result["robot_pose_source"] == "blocked:no_authoring_target"
    assert "No supported robot base pose authoring target found" in result["diagnostic"]
