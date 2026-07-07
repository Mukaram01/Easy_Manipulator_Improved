from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "extract_scene_urdf_visual_mesh_index.py"
SCENE_INDEX = ROOT / "scenes" / "ur5_2f_test" / "generated" / "scene_visual_mesh_index.json"


def test_ur5_2f_generated_mesh_index_includes_tool0_frame_anchor() -> None:
    run = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--scene",
            "ur5_2f_test",
            "--allow-xacro-lite-fallback",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert run.returncode == 0, run.stdout + run.stderr
    assert SCENE_INDEX.is_file(), f"missing generated mesh index: {SCENE_INDEX}"

    payload = json.loads(SCENE_INDEX.read_text(encoding="utf-8"))
    anchors = [
        item
        for item in payload.get("visual_items", [])
        if item.get("link") == "tool0" and item.get("role") == "transform_anchor"
    ]
    assert anchors, "expected a non-rendered transform anchor for the visual-less tool0 frame"

    anchor = anchors[0]
    assert anchor["id"] == "urdf_frame_anchor_tool0"
    assert anchor["type"] == "transform_anchor"
    assert anchor["category"] == "frame"
    assert anchor["parent_link"]
    assert anchor["joint_parent_link"] == anchor["parent_link"]
    assert isinstance(anchor["joint_origin"], dict)
    assert isinstance(anchor["link_world_pose"], dict)
    assert anchor["frame_world_pose"] == anchor["link_world_pose"]
    assert anchor["world_pose"] == anchor["link_world_pose"]
    assert anchor["pose"] == anchor["link_world_pose"]
    assert anchor["render_expected"] is False
    assert anchor["mesh_available"] is False
    assert anchor["resolved"] is True
    assert "mesh_uri" not in anchor
