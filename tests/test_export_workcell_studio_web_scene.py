import json
import subprocess
import sys
from pathlib import Path

import yaml


SCRIPT = Path("scripts/export_workcell_studio_web_scene.py")


def test_export_web_scene_contract_and_determinism(tmp_path):
    scene = tmp_path / "scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    (scene / "scene_manifest.yaml").write_text(
        yaml.safe_dump({"scene": {"name": "demo"}, "robot": {"model": "ur5"}}, sort_keys=False),
        encoding="utf-8",
    )
    (scene / "cell_definition.yaml").write_text(
        yaml.safe_dump({"cell": {"id": "demo"}, "robot": {"model": "ur5", "planning_group": "manipulator"}}, sort_keys=False),
        encoding="utf-8",
    )
    (scene / "environment.yaml").write_text(
        yaml.safe_dump(
            {"environment": {"assets": [{"id": "bin", "type": "target_bin", "pose_xyz": [1, 2, 3]}]}},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "layout/workcell_studio_layout.yaml").write_text(
        yaml.safe_dump(
            {"schema_version": "workcell_studio_layout/v1", "items": [{"id": "table", "role": "support_surface", "mesh_path": str(scene / "meshes/table.stl")}]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps(
            {
                "visual_items": [
                    {"id": "robot_mesh", "category": "robot", "role": "robot", "mesh_path": str(scene / "meshes/robot.dae")},
                    {"id": "camera_mesh", "category": "camera", "role": "camera", "package_uri": "package://camera/mesh.dae"},
                ]
            }
        ),
        encoding="utf-8",
    )

    out1 = tmp_path / "build/scene.web_scene.json"
    out2 = tmp_path / "build/scene_2.web_scene.json"
    cmd1 = [sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out1)]
    cmd2 = [sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out2)]
    subprocess.run(cmd1, check=True)
    subprocess.run(cmd2, check=True)

    assert out1.read_text(encoding="utf-8") == out2.read_text(encoding="utf-8")
    payload = json.loads(out1.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "workcell_studio_web_scene/v1"
    assert payload["robots"][-1]["id"] == "robot_mesh"
    assert payload["robots"][-1]["locked"] is True
    assert payload["robots"][-1]["editable"] is False
    table = next(item for item in payload["assets"] if item["id"] == "table")
    assert table["locked"] is False
    assert table["editable"] is True
    assert table["mesh_path"] == "meshes/table.stl"
    assert payload["robots"][-1]["mesh_path"] == "meshes/robot.dae"
    assert payload["sensors"][-1]["package_uri"] == "package://camera/mesh.dae"
    assert payload["robots"][-1]["provenance"]["mesh_path"] == "generated/scene_visual_mesh_index.json"


def test_export_web_scene_warns_for_missing_optional_inputs(tmp_path):
    scene = tmp_path / "scene"
    scene.mkdir()
    out = tmp_path / "build/scene.web_scene.json"

    subprocess.run([sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out)], check=True)

    payload = json.loads(out.read_text(encoding="utf-8"))
    missing = [w for w in payload["warnings"] if w["code"] == "optional_file_missing"]
    assert {w["source"] for w in missing} == {
        "scene_manifest.yaml",
        "cell_definition.yaml",
        "environment.yaml",
        "layout/workcell_studio_layout.yaml",
        "generated/scene_visual_mesh_index.json",
    }
