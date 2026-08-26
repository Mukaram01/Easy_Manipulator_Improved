import ast
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from scripts import extract_scene_urdf_visual_mesh_index as mesh_index
from scripts import export_workcell_studio_web_scene as web_exporter


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "suction_test"
LAYOUT = SCENE / "layout" / "workcell_studio_layout.yaml"
LAUNCH = SCENE / "launch" / "demo.launch.py"
XACRO = SCENE / "urdf" / "scene.urdf.xacro"


def _layout_items(scene=SCENE):
    document = yaml.safe_load((scene / "layout/workcell_studio_layout.yaml").read_text(encoding="utf-8"))
    return {item["id"]: item for item in document["items"]}


def _launch_helpers():
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    wanted_assignments = {"CANONICAL_LAYOUT_REL_PATH", "CANONICAL_LAYOUT_SCHEMA"}
    wanted_functions = {"_canonical_physical_role", "load_canonical_layout_poses"}
    nodes = [
        node for node in tree.body
        if (
            isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id in wanted_assignments for target in node.targets)
        ) or (isinstance(node, ast.FunctionDef) and node.name in wanted_functions)
    ]
    namespace = {"math": math, "os": __import__("os"), "yaml": yaml, "scene_pkg": "suction_test"}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(LAUNCH), "exec"), namespace)
    return namespace


def _rotation_xyz(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _matmul(lhs, rhs):
    return tuple(tuple(sum(lhs[row][k] * rhs[k][col] for k in range(3)) for col in range(3)) for row in range(3))


def _matvec(matrix, vector):
    return tuple(sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3))


def test_suction_launch_and_extractor_resolve_differently_named_canonical_owners():
    items = _layout_items()
    poses = _launch_helpers()["load_canonical_layout_poses"](layout_path=LAYOUT)
    assert poses["support_surface"]["id"] == "table_main"
    assert poses["camera"]["id"] == "realsense_suction_overhead"
    assert poses["support_surface"]["xyz"] == pytest.approx(items["table_main"]["pose"]["xyz"])
    assert poses["camera"]["xyz"] == pytest.approx(items["realsense_suction_overhead"]["pose"]["xyz"])

    request = mesh_index._extract_scene_launch_xacro_request(SCENE, {})
    assert [float(value) for value in request["mappings"]["table_world_xyz"].split()] == pytest.approx(
        items["table_main"]["pose"]["xyz"]
    )
    assert [float(value) for value in request["mappings"]["camera_world_xyz"].split()] == pytest.approx(
        items["realsense_suction_overhead"]["pose"]["xyz"]
    )


def test_generic_pose_resolution_rejects_competing_physical_owners(tmp_path):
    document = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    duplicate = dict(next(item for item in document["items"] if item["id"] == "table_main"))
    duplicate["id"] = "second_table"
    document["items"].append(duplicate)
    copied = tmp_path / "layout.yaml"
    copied.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    with pytest.raises(RuntimeError, match="exactly one physical support_surface owner"):
        _launch_helpers()["load_canonical_layout_poses"](layout_path=copied)


def test_camera_world_pose_does_not_inherit_table_translation(tmp_path):
    document = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    items = {item["id"]: item for item in document["items"]}
    authored_camera = tuple(items["realsense_suction_overhead"]["pose"]["xyz"])
    items["table_main"]["pose"]["xyz"][0] += 0.75
    copied = tmp_path / "layout.yaml"
    copied.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    poses = _launch_helpers()["load_canonical_layout_poses"](layout_path=copied)
    assert poses["support_surface"]["xyz"][0] == pytest.approx(1.25)
    assert poses["camera"]["xyz"] == pytest.approx(authored_camera)


def test_suction_xacro_places_table_and_camera_independently_in_world():
    source = XACRO.read_text(encoding="utf-8")
    assert '<origin xyz="$(arg table_world_xyz)" rpy="$(arg table_world_rpy)"/>' in source
    assert '<xacro:sensor_d435i parent="$(arg world_frame)"' in source
    assert '<origin xyz="$(arg camera_world_xyz)" rpy="$(arg camera_world_rpy)"/>' in source
    assert 'parent="table_"' not in source


def test_generated_table_and_camera_mount_match_authored_world_poses():
    items = _layout_items()
    expanded = ET.parse(SCENE / "generated" / "expanded_scene_preview.urdf").getroot()
    joints = {joint.get("name"): joint for joint in expanded.findall("joint")}
    table_joint = joints["table_base_joint_"]
    camera_joint = joints["camera_joint"]
    assert table_joint.find("parent").get("link") == "world"
    assert camera_joint.find("parent").get("link") == "world"
    assert [float(value) for value in table_joint.find("origin").get("xyz").split()] == pytest.approx(
        items["table_main"]["pose"]["xyz"]
    )
    assert [float(value) for value in camera_joint.find("origin").get("xyz").split()] == pytest.approx(
        items["realsense_suction_overhead"]["pose"]["xyz"]
    )

    index = json.loads((SCENE / "generated" / "scene_visual_mesh_index.json").read_text(encoding="utf-8"))
    table_row = next(item for item in index["visual_items"] if item.get("link") == "table_")
    assert table_row["world_pose"]["xyz"] == pytest.approx(items["table_main"]["pose"]["xyz"])


def test_suction_mount_maps_cup_approach_axis_downward():
    index = json.loads((SCENE / "generated" / "scene_visual_mesh_index.json").read_text(encoding="utf-8"))
    tool0 = next(item for item in index["visual_items"] if item.get("link") == "tool0")
    tool_rotation = _rotation_xyz(tool0["world_pose"]["rpy"])
    mount_rotation = _rotation_xyz((-1.5708, -1.5708, 0.0))
    cup_rotation = _rotation_xyz((0.0, math.pi / 2.0, 0.0))
    cup_z_world = _matvec(_matmul(_matmul(tool_rotation, mount_rotation), cup_rotation), (0.0, 0.0, 1.0))
    assert cup_z_world == pytest.approx((0.0, 0.0, -1.0), abs=1e-4)
    assert 'rpy="$(arg suction_mount_rpy)"' in XACRO.read_text(encoding="utf-8")


def test_suction_export_has_one_primary_support_and_camera_owner():
    payload = web_exporter.build_web_scene(SCENE, stage_assets=False)
    primary = [
        item for section in ("assets", "sensors") for item in payload.get(section, [])
        if item.get("render_policy") == "primary"
    ]
    assert sum(item.get("readiness_category") == "workbench_support_surface" for item in primary) == 1
    assert sum(item.get("readiness_category") == "configured_camera" for item in primary) == 1
    assert payload["render_ownership_summary"]["duplicate_primary_identities"] == 0
