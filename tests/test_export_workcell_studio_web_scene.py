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
                    {
                        "id": "robot_mesh",
                        "category": "robot",
                        "role": "robot",
                        "mesh_path": str(scene / "meshes/robot.dae"),
                        "pose": {"xyz": [9, 9, 9], "rpy": [0, 0, 0]},
                        "world_pose": {"xyz": [8, 8, 8], "rpy": [0, 0, 0]},
                        "link_world_pose": {"xyz": [1, 2, 3], "rpy": [0.1, 0.2, 0.3]},
                        "visual_origin": {"xyz": [0.1, 0.2, 0.3], "rpy": [0, 0, 1.57]},
                        "baked_world_visual_pose": {"xyz": [1.1, 2.2, 3.3], "rpy": [0.1, 0.2, 1.87]},
                        "baked_world_visual_matrix": [[1, 0, 0, 1.1], [0, 1, 0, 2.2], [0, 0, 1, 3.3], [0, 0, 0, 1]],
                        "baked_world_visual_quaternion": [0, 0, 0, 1],
                        "mesh_scale": [1, 1, 1],
                    },
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
    assert payload["robots"][-1]["final_transform"] == {"xyz": [1.1, 2.2, 3.3], "rpy": [0.1, 0.2, 1.87]}
    assert payload["robots"][-1]["world_from_visual"] == payload["robots"][-1]["final_transform"]
    assert payload["robots"][-1]["transform_source"] == "baked_world_visual_pose"
    assert payload["robots"][-1]["link_world_pose"] == {"xyz": [1, 2, 3], "rpy": [0.1, 0.2, 0.3]}
    assert payload["robots"][-1]["visual_origin"] == {"xyz": [0.1, 0.2, 0.3], "rpy": [0, 0, 1.57]}
    assert payload["robots"][-1]["mesh_scale"] == [1, 1, 1]
    assert payload["robots"][-1]["baked_world_visual_matrix"] == [[1, 0, 0, 1.1], [0, 1, 0, 2.2], [0, 0, 1, 3.3], [0, 0, 0, 1]]
    assert payload["robots"][-1]["baked_world_visual_quaternion"] == [0, 0, 0, 1]
    assert payload["sensors"][-1]["package_uri"] == "package://camera/mesh.dae"
    assert payload["robots"][-1]["provenance"]["mesh_path"] == "generated/scene_visual_mesh_index.json"
    assert payload["robots"][-1]["provenance"]["final_transform"] == "generated/scene_visual_mesh_index.json"


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


def test_export_web_scene_stages_safe_mesh_assets(tmp_path):
    scene = tmp_path / "scene"
    mesh_dir = scene / "meshes"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    mesh_dir.mkdir()
    (mesh_dir / "table.stl").write_text("solid table\nendsolid table\n", encoding="utf-8")
    (scene / "scene_manifest.yaml").write_text(yaml.safe_dump({"scene": {"name": "staging_test"}}), encoding="utf-8")
    (scene / "cell_definition.yaml").write_text("{}", encoding="utf-8")
    (scene / "environment.yaml").write_text("{}", encoding="utf-8")
    (scene / "layout/workcell_studio_layout.yaml").write_text(
        yaml.safe_dump({"items": [{"id": "table", "mesh_uri": "meshes/table.stl"}]}),
        encoding="utf-8",
    )
    (scene / "generated/scene_visual_mesh_index.json").write_text(json.dumps({"visual_items": []}), encoding="utf-8")

    out = tmp_path / "build/scene.web_scene.json"
    subprocess.run([sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out)], check=True)

    payload = json.loads(out.read_text(encoding="utf-8"))
    table = next(item for item in payload["assets"] if item["id"] == "table")
    assert table["original_mesh_uri"] == "meshes/table.stl"
    assert table["mesh_staging_status"] == "staged"
    assert table["mesh_resolve_warning"] is None
    assert table["mesh_uri"] == "build/workcell_studio_web_scene/assets/staging_test/external_" + table["mesh_uri"].split("/external_", 1)[1]
    assert Path(table["mesh_staged_path"]).is_file()


def test_export_web_scene_rejects_unsafe_or_unsupported_mesh_assets(tmp_path):
    scene = tmp_path / "scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    (scene / "scene_manifest.yaml").write_text(yaml.safe_dump({"scene": {"name": "unsafe_test"}}), encoding="utf-8")
    (scene / "cell_definition.yaml").write_text("{}", encoding="utf-8")
    (scene / "environment.yaml").write_text("{}", encoding="utf-8")
    (scene / "layout/workcell_studio_layout.yaml").write_text(
        yaml.safe_dump(
            {
                "items": [
                    {"id": "traversal", "mesh_uri": "../secret.stl"},
                    {"id": "remote", "mesh_uri": "https://example.invalid/mesh.stl"},
                    {"id": "texture", "mesh_uri": "meshes/texture.png"},
                ]
            }
        ),
        encoding="utf-8",
    )
    (scene / "generated/scene_visual_mesh_index.json").write_text(json.dumps({"visual_items": []}), encoding="utf-8")

    out = tmp_path / "build/scene.web_scene.json"
    subprocess.run([sys.executable, str(SCRIPT), "--scene", str(scene), "--output", str(out)], check=True)

    payload = json.loads(out.read_text(encoding="utf-8"))
    by_id = {item["id"]: item for item in payload["assets"]}
    assert by_id["traversal"]["mesh_staging_status"] == "unsafe_path"
    assert "Unsafe relative mesh path" in by_id["traversal"]["mesh_resolve_warning"]
    assert by_id["remote"]["mesh_staging_status"] == "unsupported_scheme"
    assert "Unsupported mesh URI scheme" in by_id["remote"]["mesh_resolve_warning"]
    assert by_id["texture"]["mesh_staging_status"] == "unsupported_format"
    assert "Unsupported mesh format" in by_id["texture"]["mesh_resolve_warning"]


def test_web_viewer_prefers_canonical_final_transform_without_visual_origin_recomposition():
    viewer = Path("workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
    pose_block = viewer.split("function poseOf(item)", 1)[1].split("function scaleOf", 1)[0]
    assert "item.final_transform || item.world_from_visual || item.baked_world_visual_pose" in pose_block
    assert "visual_origin" not in pose_block
