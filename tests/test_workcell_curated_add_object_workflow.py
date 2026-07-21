import hashlib
import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/add_workcell_studio_curated_object.py"
CURATION = ROOT / "workcell_builder/workcell_builder/config/asset_profiles/curated_add_objects.json"
CONTROLLER = ROOT / "workcell_builder/workcell_builder/gui/embedded_web_curated_add_controller.hpp"
BOOTSTRAP = ROOT / "workcell_builder/workcell_builder/gui/embedded_web_curated_add_bootstrap.hpp"
UI_HEADER = ROOT / "workcell_builder/workcell_builder/include/workcell_builder_ui_utils.hpp"


def _scene(tmp_path: Path) -> Path:
    scene = tmp_path / "scenes/tiny_scene"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    layout = {
        "schema_version": "workcell_studio_layout/v1",
        "items": [
            {
                "id": "table", "type": "support_surface", "role": "support_surface",
                "category": "work_surface", "pose": {"xyz": [0.0, 0.0, 0.05], "rpy": [0, 0, 0]},
                "dimensions": [1.2, 0.8, 0.1], "geometry_type": "box", "editable": True, "locked": False,
            },
            {
                "id": "existing_box", "type": "object", "role": "pick_object",
                "pose": {"xyz": [0.0, 0.0, 0.15], "rpy": [0, 0, 0]},
                "dimensions": [0.2, 0.2, 0.1], "geometry_type": "box", "editable": True, "locked": False,
            },
            {
                "id": "pick_zone", "type": "pick_zone", "role": "pick_zone",
                "pose": {"xyz": [0.0, 0.0, 0.105], "rpy": [0, 0, 0]}, "dimensions": [0.8, 0.6, 0.01],
            },
        ],
    }
    (scene / "layout/workcell_studio_layout.yaml").write_text(yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")
    (scene / "environment.yaml").write_text("sentinel: unchanged\n", encoding="utf-8")
    (scene / "generated/sentinel.txt").write_text("unchanged\n", encoding="utf-8")
    return scene


def _run(scene: Path, asset_id: str, *extra: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--scene", str(scene), "--asset-id", asset_id, "--json", *extra],
        cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )


def test_curated_manifest_is_small_support_surface_only_and_excludes_structural_assets():
    manifest = json.loads(CURATION.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == "workcell_studio_curated_add_objects/v1"
    ids = {item["asset_id"] for item in manifest["objects"]}
    assert ids == {"bin_small", "bin_large", "tray", "tote_box", "fixture_plate", "calibration_cube", "pick_box", "cylinder_object"}
    assert all(item["placement_policy"] == "support_surface" for item in manifest["objects"])
    assert not ids.intersection({"table_small", "workbench", "pallet", "robot_base_plate", "camera_stand", "safety_fence_panel"})


def test_dry_run_plans_collision_free_support_placement_without_mutation(tmp_path: Path):
    scene = _scene(tmp_path)
    layout = scene / "layout/workcell_studio_layout.yaml"
    before = layout.read_bytes()
    result = _run(scene, "pick_box")
    assert result.returncode == 0, result.stdout + result.stderr
    plan = json.loads(result.stdout)
    assert plan["status"] == "planned"
    assert plan["support_surface_id"] == "table"
    assert plan["instance_id"] == "pick_box_01"
    assert plan["pose_xyz"] != [0.0, 0.0, 0.14]
    assert layout.read_bytes() == before


def test_confirmed_write_is_atomic_backed_up_and_bottom_aligned(tmp_path: Path):
    scene = _scene(tmp_path)
    layout_path = scene / "layout/workcell_studio_layout.yaml"
    before_environment = (scene / "environment.yaml").read_bytes()
    before_generated = (scene / "generated/sentinel.txt").read_bytes()
    plan = json.loads(_run(scene, "pick_box").stdout)
    result = _run(
        scene, "pick_box", "--instance-id", plan["instance_id"],
        "--expected-layout-sha256", plan["layout_sha256"], "--write", "--backup",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    written = json.loads(result.stdout)
    assert written["status"] == "written"
    assert Path(written["backup_path"]).is_file()
    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    item = next(value for value in layout["items"] if value["id"] == "pick_box_01")
    assert item["source"] == "layout/workcell_studio_layout.yaml"
    assert item["source_layer"] == "editable_layout"
    assert item["editable"] is True and item["locked"] is False
    assert item["catalog_asset_id"] == "pick_box"
    assert item["support_surface_ref"] == "table"
    assert item["primitive_geometry_type"] == "box"
    assert abs((item["pose"]["xyz"][2] - item["dimensions"][2] / 2.0) - 0.1) < 1e-9
    assert (scene / "environment.yaml").read_bytes() == before_environment
    assert (scene / "generated/sentinel.txt").read_bytes() == before_generated


def test_stale_layout_and_non_curated_asset_are_rejected(tmp_path: Path):
    scene = _scene(tmp_path)
    stale = hashlib.sha256(b"older layout").hexdigest()
    stale_result = _run(scene, "pick_box", "--expected-layout-sha256", stale, "--write")
    assert stale_result.returncode != 0
    assert "layout changed after preview" in json.loads(stale_result.stdout)["error"]
    structural = _run(scene, "table_small")
    assert structural.returncode != 0
    assert "not approved" in json.loads(structural.stdout)["error"]


def test_curated_mesh_asset_keeps_repo_relative_mesh_reference(tmp_path: Path):
    scene = _scene(tmp_path)
    plan = json.loads(_run(scene, "bin_small").stdout)
    result = _run(scene, "bin_small", "--instance-id", plan["instance_id"],
                  "--expected-layout-sha256", plan["layout_sha256"], "--write")
    assert result.returncode == 0, result.stdout + result.stderr
    layout = yaml.safe_load((scene / "layout/workcell_studio_layout.yaml").read_text(encoding="utf-8"))
    item = next(value for value in layout["items"] if value["id"] == "bin_small_01")
    assert item["geometry_type"] == "mesh"
    assert item["mesh_path"].startswith("workcell_builder/workcell_builder/assets/environment/")
    assert not Path(item["mesh_path"]).is_absolute()


def test_qt_workflow_is_curated_async_scene_guarded_and_requires_clean_editor():
    source = CONTROLLER.read_text(encoding="utf-8")
    for token in [
        "Add object", "QInputDialog::getItem", "curated_add_objects.json", "environment_assets.json",
        "add_workcell_studio_curated_object.py", "--expected-layout-sha256", "--backup",
        "run_workcell_studio_web_edit_workflow.py", "--generate-and-validate",
        "ensure_workcell_studio_web_scene_fresh.py", "--stage-assets", "--force",
        "Scene changed—reload required", "Planning…", "Adding…", "Generating…", "Refreshing…", "Added",
        "!state.value(QStringLiteral(\"dirty\")).toBool()", "view_->url() == expected_url_", "QProcess",
    ]:
        assert token in source
    assert "QFileDialog" not in source
    assert "waitForFinished" not in source
    assert "window.__WORKCELL_EDITOR_API_V1__" in source
    assert "embedded_web_curated_add_bootstrap.hpp" in UI_HEADER.read_text(encoding="utf-8")
    assert "Q_COREAPP_STARTUP_FUNCTION" in BOOTSTRAP.read_text(encoding="utf-8")


def test_no_browser_yaml_write_or_robot_motion_commands_are_added():
    combined = "\n".join(path.read_text(encoding="utf-8") for path in (SCRIPT, CONTROLLER, BOOTSTRAP))
    for forbidden in (
        "ros2 launch", "execute_trajectory", "move_group", "GetMotionPlan", "real_hardware_enabled: true",
        "fetch(\"file://", "environment.yaml\").write", "scene_manifest.yaml\").write",
    ):
        assert forbidden not in combined
