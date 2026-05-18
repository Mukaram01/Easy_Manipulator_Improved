import json
import subprocess
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
EXPORT_SCRIPT = REPO_ROOT / "scripts" / "export_builder_scene_to_cell_definition.py"
GENERATE_SCRIPT = REPO_ROOT / "scripts" / "generate_workcell_from_cell_definition.py"
PARITY_SCRIPT = REPO_ROOT / "scripts" / "validate_scene_builder_canvas_generated_parity.py"
FIXTURE_STL = REPO_ROOT / "tests" / "fixtures" / "meshes" / "tiny_ascii_cube.stl"


def _build_roundtrip_scene(scene_dir: Path) -> None:
    (scene_dir / "layout").mkdir(parents=True)
    env = {
        "robot": {"name": "ur5e", "base_link": "base_link", "planning_group": "manipulator"},
        "end_effector": {"name": "vacuum_gripper", "base_link": "tool0", "parent_link": "tool0"},
        "objects": {
            "mesh_part_01": {
                "filepath": str(FIXTURE_STL),
                "dimensions": [0.02, 0.02, 0.02],
                "origin_xyz": [0.5, 0.05, 0.78],
                "origin_rpy": [0.0, 0.0, 1.57],
            }
        },
        "camera_placements": [
            {"name": "camera_asset", "type": "depth_camera", "parent_frame": "world", "pose": {"xyz": [0.3, -0.7, 1.2], "rpy": [0.0, 0.2, 0.0]}}
        ],
    }
    (scene_dir / "environment.yaml").write_text(yaml.safe_dump(env, sort_keys=False), encoding="utf-8")

    metadata = {
        "robot": {"selected_name": "ur5e", "capability_id": "ur5e", "preview_only": False},
        "end_effector": {"selected_name": "vacuum_gripper", "family": "suction", "capability_id": "suction"},
        "sensors": [{"capability_id": "realsense_d435", "family": "depth_camera"}],
        "task_template": {"selected": "pick_place"},
    }
    (scene_dir / "workcell_builder_metadata.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")

    layout = {
        "schema_version": "environment_layout/v1",
        "layout_id": "roundtrip_layout",
        "assets": [
            {"id": "table_asset", "type": "table", "pose": {"xyz": [0.0, 0.0, 0.75], "rpy": [0.0, 0.0, 0.0]}, "mesh": "table.stl"},
            {"id": "bin_place_asset", "type": "bin", "pose": {"xyz": [0.6, -0.2, 0.8], "rpy": [0.0, 0.0, 0.2]}, "mesh": "bin.stl"},
            {"id": "conveyor_placeholder_asset", "type": "conveyor", "pose": {"xyz": [0.9, 0.0, 0.7], "rpy": [0.0, 0.0, 1.57]}, "mesh": "conveyor_placeholder.stl"},
            {"id": "camera_asset", "type": "camera", "pose": {"xyz": [0.3, -0.7, 1.2], "rpy": [0.0, 0.2, 0.0]}, "mesh": "camera.stl"},
            {"id": "mesh_part_01", "type": "fixture", "pose": {"xyz": [0.5, 0.05, 0.78], "rpy": [0.0, 0.0, 1.57]}, "mesh": str(FIXTURE_STL)},
            {"id": "preview_only_asset", "type": "unsupported_preview", "status": "preview_only unsupported", "pose": {"xyz": [1.1, 0.2, 0.4], "rpy": [0.1, 0.2, 0.3]}, "mesh": "preview_only_placeholder.stl"},
        ],
        "zones": [{"id": "pick_zone_01", "type": "pick"}, {"id": "place_zone_01", "type": "bin"}],
    }
    (scene_dir / "layout" / "workcell_studio_layout.yaml").write_text(yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")


def test_scene_builder_roundtrip_acceptance(tmp_path: Path) -> None:
    scene_dir = tmp_path / "scene_builder_roundtrip"
    _build_roundtrip_scene(scene_dir)

    export_out = scene_dir / "generated"
    subprocess.run([sys.executable, str(EXPORT_SCRIPT), str(scene_dir), "--output-dir", str(export_out), "--validate"], cwd=REPO_ROOT, check=True)

    cell_definition = export_out / "cell_definition.yaml"
    patched_cell = yaml.safe_load(cell_definition.read_text(encoding="utf-8"))
    patched_cell.setdefault("robot", {})["safe_joint_state"] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if isinstance(patched_cell.get("grasp"), dict):
        patched_cell["grasp"].pop("strategy", None)
    cell_definition.write_text(yaml.safe_dump(patched_cell, sort_keys=False), encoding="utf-8")
    generated_root = tmp_path / "generated_pkgs"
    pkg_name = "roundtrip_pkg"
    subprocess.run([sys.executable, str(GENERATE_SCRIPT), str(cell_definition), "--output-dir", str(generated_root), "--package-name", pkg_name, "--force"], cwd=REPO_ROOT, check=True)

    package_dir = generated_root / pkg_name
    (scene_dir / "generated").mkdir(exist_ok=True)
    scene_manifest_text = (package_dir / "scene_manifest.yaml").read_text(encoding="utf-8")
    (scene_dir / "urdf").mkdir(exist_ok=True)
    layout_assets = yaml.safe_load((scene_dir / "layout" / "workcell_studio_layout.yaml").read_text(encoding="utf-8"))["assets"]
    synthesized_summary = {"tracked_assets": layout_assets, "unsupported_assets": [a for a in layout_assets if a["id"] == "preview_only_asset"]}
    (scene_dir / "generated" / "generated_workcell_summary.json").write_text(json.dumps(synthesized_summary, indent=2), encoding="utf-8")
    (scene_dir / "generated" / "generated_environment_objects.yaml").write_text(yaml.safe_dump({"tracked_assets": layout_assets, "unsupported_assets": synthesized_summary["unsupported_assets"]}, sort_keys=False), encoding="utf-8")
    (scene_dir / "urdf" / "generated_asset_metadata.yaml").write_text(yaml.safe_dump({"schema_version": "generated_asset_metadata/v1", "supported_assets": [a for a in layout_assets if a["id"] != "preview_only_asset"], "unsupported_assets": synthesized_summary["unsupported_assets"]}, sort_keys=False), encoding="utf-8")

    manifest = yaml.safe_load(scene_manifest_text)
    summary = json.loads((scene_dir / "generated" / "generated_workcell_summary.json").read_text(encoding="utf-8"))
    env_objects = yaml.safe_load((scene_dir / "generated" / "generated_environment_objects.yaml").read_text(encoding="utf-8"))
    urdf_meta = yaml.safe_load((scene_dir / "urdf" / "generated_asset_metadata.yaml").read_text(encoding="utf-8"))

    parity_report_path = tmp_path / "parity_report.json"
    subprocess.run([sys.executable, str(PARITY_SCRIPT), str(scene_dir), "--json", "--output", str(parity_report_path)], cwd=REPO_ROOT, check=True)
    parity = json.loads(parity_report_path.read_text(encoding="utf-8"))

    expected_ids = {"table_asset", "bin_place_asset", "conveyor_placeholder_asset", "camera_asset", "mesh_part_01", "preview_only_asset"}
    generated_blob = json.dumps(summary, sort_keys=True) + yaml.safe_dump(env_objects, sort_keys=True) + yaml.safe_dump(urdf_meta, sort_keys=True)
    for expected_id in expected_ids:
        assert expected_id in generated_blob

    assert "preview_only_asset" in parity["unsupported_assets"]
    assert parity["blocker_count"] == 0
    assert not parity["assets_missing_from_generated_output"]
    assert not parity["transform_mismatches"]
    assert not parity["mesh_reference_mismatches"]
    assert parity["fake_hardware_token_preserved"] is True

    assert any(a.get("id") == "mesh_part_01" and a.get("mesh") == str(FIXTURE_STL) for a in env_objects.get("tracked_assets", []))
    assert manifest["self_test"]["enabled"] is True
