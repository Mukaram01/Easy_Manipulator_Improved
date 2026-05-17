import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXPORT = ROOT / "scripts/export_builder_scene_to_cell_definition.py"
VALIDATE = ROOT / "scripts/validate_cell_definition.py"


def test_cell_definition_export_supports_robot_and_tool_pose_keys_and_validator_pass_fixture(tmp_path):
    scene = tmp_path / "scene"
    scene.mkdir(parents=True)
    (scene / "environment.yaml").write_text("schema_version: workcell_scene/v1\nrobot:\n  robot_mount:\n    pose:\n      xyz: [0.1,0.2,0.3]\n      rpy: [0,0,1.57]\ntool:\n  tool_attachment:\n    origin:\n      rpy: [-1.5708,-1.5708,0.0]\n", encoding="utf-8")
    out = tmp_path / "out"
    subprocess.run(["python3", str(EXPORT), str(scene), "--output-dir", str(out)], check=True)
    yaml_gen = (ROOT / "workcell_builder/workcell_builder/include/yaml_parser/generate_yaml.h").read_text(encoding="utf-8")
    assert "robot_mount" in yaml_gen
    assert "tool_attachment" in yaml_gen

    subprocess.run(["python3", str(VALIDATE), str(ROOT / "tests/fixtures/cell_definition_pick_place.yaml"), "--json"], check=True)
