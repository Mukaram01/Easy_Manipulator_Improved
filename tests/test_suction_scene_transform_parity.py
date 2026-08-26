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


def test_suction_mount_exactly_matches_ur5_2f_reference_mount():
    suction = ET.parse(XACRO).getroot()
    reference = ET.parse(ROOT / "scenes/ur5_2f_test/urdf/scene.urdf.xacro").getroot()
    namespace = "http://www.ros.org/wiki/xacro"
    suction_arg = next(
        item for item in suction.findall(f"{{{namespace}}}arg") if item.get("name") == "suction_mount_rpy"
    )
    reference_macro = next(
        item for item in reference.findall(f"{{{namespace}}}robotiq_85_gripper")
    )
    reference_origin = reference_macro.find("origin")
    assert suction_arg.get("default") == "1.5707 -1.5707 0"
    assert reference_origin.get("xyz") == "0 0 0"
    assert reference_origin.get("rpy") == suction_arg.get("default")


def test_suction_cup_link_uses_real_visual_and_collision_mesh():
    asset = ET.parse(
        ROOT / "assets/end_effectors/single_suction_gripper/single_suction_description/urdf/single_suction_gripper.urdf.xacro"
    ).getroot()
    macro = next(item for item in asset if item.tag == "macro" or item.tag.endswith("}macro"))
    cup = next(item for item in macro.findall("link") if item.get("name") == "suction_cup_link")
    expected = "package://single_suction_description/meshes/suction_cup_link.STL"
    assert cup.find("visual/geometry/mesh").get("filename") == expected
    assert cup.find("collision/geometry/mesh").get("filename") == expected
    assert cup.find("visual/origin").get("xyz") == "0 0 0"
    assert cup.find("collision/origin").get("xyz") == "0 0 0"


def test_generated_index_contains_complete_locked_suction_tool():
    index = json.loads((SCENE / "generated" / "scene_visual_mesh_index.json").read_text(encoding="utf-8"))
    tool_rows = [item for item in index["visual_items"] if item.get("link") in {"wrist_fixture", "suction_cup_link"}]
    assert {item.get("link") for item in tool_rows} == {"wrist_fixture", "suction_cup_link"}
    assert any(str(item.get("mesh_uri") or "").endswith("/suction_cup_link.STL") for item in tool_rows)
    assert all(item.get("resolved") is True for item in tool_rows)
    assert all(
        math.isfinite(value)
        for item in tool_rows
        for value in (*item["world_pose"]["xyz"], *item["world_pose"]["rpy"])
    )


def test_suction_export_has_one_primary_support_and_camera_owner():
    payload = web_exporter.build_web_scene(SCENE, stage_assets=False)
    primary = [
        item for section in ("assets", "sensors") for item in payload.get(section, [])
        if item.get("render_policy") == "primary"
    ]
    assert sum(item.get("readiness_category") == "workbench_support_surface" for item in primary) == 1
    assert sum(item.get("readiness_category") == "configured_camera" for item in primary) == 1
    assert payload["render_ownership_summary"]["duplicate_primary_identities"] == 0
