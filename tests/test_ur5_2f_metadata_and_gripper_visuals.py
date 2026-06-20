import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "ur5_2f_test"


def _first(mapping, paths):
    for path in paths:
        node = mapping
        for key in path:
            if not isinstance(node, dict) or key not in node:
                node = None
                break
            node = node[key]
        if isinstance(node, str) and node.strip():
            return node.strip()
    return ""


def test_ur5_2f_canonical_metadata_priority_resolves_tooling():
    docs = []
    for rel in ("environment.yaml", "cell_definition.yaml", "scene_manifest.yaml"):
        with (SCENE / rel).open("r", encoding="utf-8") as handle:
            docs.append(yaml.safe_load(handle) or {})

    robot = end_effector = mount = grasp = ""
    for doc in docs:
        robot = robot or _first(doc, [("robot", "model"), ("robot", "id"), ("robot", "name")])
        end_effector = end_effector or _first(doc, [
            ("end_effector", "id"), ("end_effector", "model"), ("end_effector", "profile"),
            ("tool", "id"), ("tool", "model"),
        ])
        mount = mount or _first(doc, [("end_effector", "mount_link"), ("tool", "mount_link"), ("robot", "tool_mount_link")])
        grasp = grasp or _first(doc, [("end_effector", "grasp_frame"), ("tool", "grasp_frame"), ("robot", "ee_link")])

    assert robot == "ur5"
    assert end_effector == "robotiq_85_gripper"
    assert mount == "tool0"
    assert grasp == "ee_palm"


def test_ur5_2f_visual_index_extracts_robotiq_gripper_visuals_without_motion():
    subprocess.run(
        [sys.executable, "scripts/extract_scene_urdf_visual_mesh_index.py", "--scene", "ur5_2f_test"],
        cwd=ROOT,
        check=True,
    )
    payload = json.loads((SCENE / "generated" / "scene_visual_mesh_index.json").read_text(encoding="utf-8"))
    items = payload.get("visual_items", [])
    gripper_items = [
        item for item in items
        if "gripper" in str(item.get("id", "")).lower()
        or "gripper" in str(item.get("link", "")).lower()
        or "robotiq_85_description" in str(item.get("package_uri", ""))
    ]
    ids = [item.get("id") for item in items]
    assert len(ids) == len(set(ids))
    assert gripper_items, "expected at least one Robotiq/gripper visual item"
    assert any("robotiq_85_description" in str(item.get("package_uri", "")) for item in gripper_items)
