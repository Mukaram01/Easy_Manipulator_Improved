from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "workcell_builder/workcell_builder/templates/ros2/launch/demo.launch.py"
FIXTURE = ROOT / "tests/fixtures/environment.yaml"


def _load_append_fn():
    src = LAUNCH.read_text(encoding="utf-8")
    start = src.index("def _as_vec3")
    end = src.index("def extract_end_effector_metadata")
    ns = {"os": __import__("os"), "ET": __import__("xml.etree.ElementTree", fromlist=["ElementTree"]), "warnings": __import__("warnings")}
    exec(src[start:end], ns)
    return ns["_append_environment_placed_objects_urdf"]


def _joint_origin_pose(urdf_text: str, joint_name: str):
    root = ET.fromstring(urdf_text)
    for joint in root.findall("joint"):
        if joint.get("name") == joint_name:
            origin = joint.find("origin")
            return origin.get("xyz"), origin.get("rpy")
    raise AssertionError(f"Joint not found: {joint_name}")


def test_round_trip_save_reload_pose_update_preview_and_scene_pose_match(tmp_path):
    append_fn = _load_append_fn()
    env = yaml.safe_load(FIXTURE.read_text(encoding="utf-8"))

    out_1 = tmp_path / "environment_1.yaml"
    out_1.write_text(yaml.safe_dump(env, sort_keys=False), encoding="utf-8")

    reloaded = yaml.safe_load(out_1.read_text(encoding="utf-8"))
    reloaded["placed_objects"][0]["xyz"] = [0.6, 0.1, 0.25]
    reloaded["placed_objects"][0]["rpy"] = [0.0, 0.0, 1.0]

    out_2 = tmp_path / "environment_2.yaml"
    out_2.write_text(yaml.safe_dump(reloaded, sort_keys=False), encoding="utf-8")

    robot = '<robot name="demo"><link name="world"/></robot>'
    preview_urdf = append_fn(robot, yaml.safe_load(out_2.read_text(encoding="utf-8")))
    scene_urdf = append_fn(robot, reloaded)

    preview_pose = _joint_origin_pose(preview_urdf, "table_01_joint")
    scene_pose = _joint_origin_pose(scene_urdf, "table_01_joint")
    assert preview_pose == scene_pose
