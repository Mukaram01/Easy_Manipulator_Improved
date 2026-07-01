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


def test_ur5_2f_web_export_keeps_distinct_robotiq_link_transforms(tmp_path):
    subprocess.run(
        [sys.executable, "scripts/extract_scene_urdf_visual_mesh_index.py", "--scene", "ur5_2f_test"],
        cwd=ROOT,
        check=True,
    )
    output = tmp_path / "web_scene.json"
    subprocess.run(
        [
            sys.executable,
            "scripts/export_workcell_studio_web_scene.py",
            "--scene",
            "scenes/ur5_2f_test",
            "--output",
            str(output),
        ],
        cwd=ROOT,
        check=True,
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    gripper_items = [
        item for item in payload.get("tools", [])
        if "gripper" in str(item.get("link", "")).lower()
        or "robotiq_85_description" in str(item.get("package_uri", ""))
    ]
    assert len(gripper_items) >= 3
    poses = [json.dumps(item.get("world_from_visual") or item.get("final_transform") or item.get("pose"), sort_keys=True) for item in gripper_items]
    explicit_identical = any(item.get("transform_source") == "explicit_identical_transform" for item in gripper_items)
    assert explicit_identical or len(set(poses)) > 1
    for item in gripper_items:
        assert item.get("parent_link") is not None
        assert item.get("joint_origin") is not None
        assert item.get("visual_origin") is not None
        assert item.get("transform_source") in {
            "urdf_fk_link_world_times_visual_origin",
            "robotiq_85_fallback_link_metadata",
        }
        xyz = (item.get("world_from_visual") or item.get("pose") or {}).get("xyz")
        assert xyz is not None
        assert abs(float(xyz[0])) < 1.0
        assert abs(float(xyz[1])) < 1.0
        assert abs(float(xyz[2])) < 1.0
