from pathlib import Path

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


def test_generated_scene_urdf_contains_expected_placed_object_tokens():
    append_fn = _load_append_fn()
    env = yaml.safe_load(FIXTURE.read_text(encoding="utf-8"))
    robot = '<robot name="demo"><link name="world"/></robot>'
    urdf = append_fn(robot, env)

    assert "table_01_link" in urdf
    assert "table_01_joint" in urdf
    assert "package://demo_scene/meshes/table_01.stl" in urdf
    assert "<collision>" in urdf
    assert 'xyz="0.5 0.0 0.2"' in urdf
    assert 'rpy="0.0 0.0 1.57"' in urdf


def test_safety_words_not_introduced_for_real_hardware_execution():
    docs = (ROOT / "docs/manuals/SCENE_CREATION_WORKFLOW.md").read_text(encoding="utf-8")
    assert "fake hardware" in docs.lower()
    assert "no real hardware" in docs.lower() or "does not" in docs.lower()
