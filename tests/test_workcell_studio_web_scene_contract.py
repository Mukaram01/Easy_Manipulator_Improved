import json
import subprocess
import sys
from pathlib import Path

import yaml
from jsonschema import Draft202012Validator


REPO_ROOT = Path(__file__).resolve().parents[1]
EXPORTER = REPO_ROOT / "scripts" / "export_workcell_studio_web_scene.py"
SCHEMA = REPO_ROOT / "schemas" / "workcell_studio_web_scene_v1.schema.json"


def _write_yaml(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def _write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, sort_keys=True), encoding="utf-8")


def _tiny_scene_fixture(tmp_path: Path) -> Path:
    scene = tmp_path / "tiny_scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()

    _write_yaml(
        scene / "environment.yaml",
        {
            "scene": {"id": "tiny_scene", "name": "Tiny Scene"},
            "environment": {
                "assets": [
                    {
                        "id": "workbench",
                        "type": "table",
                        "role": "support_surface",
                        "pose_xyz": [0.0, 0.0, 0.0],
                        "dimensions": [1.0, 0.7, 0.05],
                    }
                ],
                "sensors": [
                    {
                        "id": "inspection_camera",
                        "category": "camera",
                        "role": "camera",
                        "pose_xyz": [0.4, -0.2, 1.1],
                    }
                ],
            },
        },
    )
    _write_yaml(
        scene / "layout" / "workcell_studio_layout.yaml",
        {
            "schema_version": "workcell_studio_layout/v1",
            "items": [
                {
                    "id": "layout_bin",
                    "type": "target_bin",
                    "category": "bin",
                    "pose_xyz": [0.45, 0.25, 0.08],
                    "mesh_path": str(scene / "meshes" / "bin.stl"),
                }
            ],
        },
    )
    _write_json(
        scene / "generated" / "scene_visual_mesh_index.json",
        {
            "visual_items": [
                {
                    "id": "ur5_visual",
                    "category": "robot",
                    "role": "robot",
                    "mesh_path": str(scene / "meshes" / "ur5.dae"),
                },
                {
                    "id": "robotiq_2f_visual",
                    "category": "tool",
                    "role": "gripper",
                    "mesh_path": str(scene / "meshes" / "robotiq_2f.dae"),
                },
            ]
        },
    )
    return scene


def _export(scene: Path, output: Path) -> dict:
    subprocess.run(
        [sys.executable, str(EXPORTER), "--scene", str(scene), "--output", str(output)],
        cwd=REPO_ROOT,
        check=True,
    )
    return json.loads(output.read_text(encoding="utf-8"))


def test_web_scene_schema_file_exists_and_is_valid_json():
    assert SCHEMA.exists()
    with SCHEMA.open(encoding="utf-8") as fh:
        schema = json.load(fh)
    assert schema["$id"].endswith("/workcell_studio_web_scene_v1.schema.json")
    assert schema["properties"]["schema_version"]["const"] == "workcell_studio_web_scene/v1"
    Draft202012Validator.check_schema(schema)


def test_tiny_scene_exports_generated_and_authored_asset_contract(tmp_path):
    scene = _tiny_scene_fixture(tmp_path)
    out1 = tmp_path / "out" / "scene.web_scene.json"
    out2 = tmp_path / "out" / "scene.again.web_scene.json"

    payload1 = _export(scene, out1)
    payload2 = _export(scene, out2)

    assert out1.read_bytes() == out2.read_bytes()
    assert payload1 == payload2
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    Draft202012Validator(schema).validate(payload1)
    assert payload1["schema_version"] == "workcell_studio_web_scene/v1"
    assert payload1["scene"]["units"] == {"distance": "metre", "angle": "radian"}
    assert payload1["scene"]["coordinate_system"]["convention"] == "ros_world_z_up"
    assert payload1["inputs"]["environment"]["present"] is True
    assert payload1["inputs"]["layout"]["present"] is True
    assert payload1["inputs"]["visual_mesh_index"]["present"] is True

    robot = next(item for item in payload1["robots"] if item["id"] == "ur5_visual")
    tool = next(item for item in payload1["tools"] if item["id"] == "robotiq_2f_visual")
    for generated in (robot, tool):
        assert generated["source_kind"] == "generated_preview"
        assert generated["locked"] is True
        assert generated["editable"] is False
        assert generated["provenance"]["mesh_path"] == "generated/scene_visual_mesh_index.json"

    layout_asset = next(item for item in payload1["assets"] if item["id"] == "layout_bin")
    environment_asset = next(item for item in payload1["assets"] if item["id"] == "workbench")
    environment_sensor = next(item for item in payload1["sensors"] if item["id"] == "inspection_camera")
    for authored in (layout_asset, environment_asset, environment_sensor):
        assert authored["source_kind"] == "user_authored"
        assert authored["editable"] is True
        assert authored["locked"] is False

    actions = {action["id"]: action for action in payload1["backend_actions"]}
    assert actions["validate"]["request_kind"] == "backend_request"
    assert actions["generate_scene_package"]["enabled"] is True
    assert actions["plan_simulate"]["enabled"] is False
    assert "real hardware execution is not exposed" in actions["plan_simulate"]["safety_note"]


def test_missing_optional_inputs_warn_in_output_json_without_crashing(tmp_path):
    scene = tmp_path / "missing_optional_inputs_scene"
    scene.mkdir()

    payload = _export(scene, tmp_path / "out" / "scene.web_scene.json")

    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    Draft202012Validator(schema).validate(payload)
    assert payload["schema_version"] == "workcell_studio_web_scene/v1"
    missing = [warning for warning in payload["warnings"] if warning["code"] == "optional_file_missing"]
    assert {warning["source"] for warning in missing} == {
        "scene_manifest.yaml",
        "cell_definition.yaml",
        "environment.yaml",
        "layout/workcell_studio_layout.yaml",
        "generated/scene_visual_mesh_index.json",
    }
    assert all(payload["inputs"][name]["present"] is False for name in payload["inputs"])


def test_ur5_2f_web_scene_stages_required_product_meshes_without_required_fallbacks(tmp_path):
    payload = _export(REPO_ROOT / "scenes" / "ur5_2f_test", tmp_path / "out" / "ur5_2f_test.web_scene.json")
    required = {
        "ur5": [item for item in payload["robots"] if "ur_description/meshes/ur5/visual" in str(item.get("original_mesh_uri") or "")],
        "robotiq": [item for item in payload["tools"] if "robotiq_85_description/meshes/visual" in str(item.get("original_mesh_uri") or "")],
        "table": [item for item in payload["assets"] if "workbench_description/meshes/visual/table.stl" in str(item.get("original_mesh_uri") or "")],
        "camera": [item for item in payload["sensors"] if "realsense2_description/meshes/d435.dae" in str(item.get("original_mesh_uri") or "")],
    }
    assert len(required["ur5"]) >= 7
    assert required["robotiq"]
    assert required["table"]
    assert required["camera"]

    for group in required.values():
        for item in group:
            assert item["mesh_staging_status"] == "staged"
            assert item["mesh_uri"].startswith("build/workcell_studio_web_scene/assets/ur5_2f_test/")
            assert not item["mesh_uri"].startswith(("package://", "file://", "/"))
            assert "://" not in item["mesh_uri"]
            assert item.get("mesh_resolve_warning") in (None, "")

    required_ids = {item["id"] for group in required.values() for item in group}
    fallback_warnings = [
        warning for warning in payload["warnings"]
        if warning.get("code") in {"mesh_primitive_fallback", "mesh_stage_failed"}
    ]
    assert not [warning for warning in fallback_warnings if warning.get("source") in required_ids]
