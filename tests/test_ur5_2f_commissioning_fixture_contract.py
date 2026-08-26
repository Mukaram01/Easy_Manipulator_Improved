from pathlib import Path

import yaml

from scripts import export_workcell_studio_web_scene as web_scene_exporter
from scripts import generate_scene_from_cell_definition as scene_generator
from scripts import check_scene_readiness as scene_readiness
from scripts import validate_cell_definition as cell_validator


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "ur5_2f_test"


def _load(relative_path: str) -> dict:
    return yaml.safe_load((SCENE / relative_path).read_text(encoding="utf-8"))


def test_canonical_task_consumes_perception_without_a_fixed_object() -> None:
    for scene_name in ("ur5_2f_test", "suction_test"):
        scene = ROOT / "scenes" / scene_name
        production_yaml = "\n".join(
            path.read_text(encoding="utf-8")
            for path in [
                scene / "cell_definition.yaml",
                scene / "scene_manifest.yaml",
                scene / "environment.yaml",
                scene / "config/task_recipe.yaml",
                scene / "config/workcell_builder_task_intent.yaml",
            ]
        )
        assert "commissioning_object" not in production_yaml

        cell = yaml.safe_load((scene / "cell_definition.yaml").read_text(encoding="utf-8"))
        assert cell["objects"] == []
        assert cell["task"]["object_source"] == "perception"
        assert cell["task"]["perception_source"] == "detected_objects/v1"
        assert cell["task"]["pick_zone"] == "pick_zone_main"
        assert cell["commissioning"]["self_test_enabled"] is False

        summary = cell_validator.validate_cell_definition(cell, scene / "cell_definition.yaml", "pyyaml", [])
        assert summary.errors == []
        generated_manifest = scene_generator.build_scene_manifest(cell)
        assert "self_test" not in generated_manifest
        assert generated_manifest["task_recipe"]["pick"]["object_source"] == "perception"
        assert generated_manifest["task_recipe"]["pick"]["source"] == "detected_objects/v1"


def test_physical_and_task_layers_remain_separate_and_safe() -> None:
    environment = _load("environment.yaml")
    layout = _load("layout/workcell_studio_layout.yaml")
    manifest = _load("scene_manifest.yaml")
    recipe = _load("config/task_recipe.yaml")

    physical_ids = {
        item["id"]
        for collection in (environment["environment"]["support_surfaces"], environment["environment"]["assets"])
        for item in collection
    }
    assert {"support_surface_table", "target_bin_default", "realsense_overhead"} <= physical_ids
    assert "commissioning_object" not in physical_ids
    assert "commissioning_object" not in {item["id"] for item in layout["items"]}
    assert "default_drop_zone" in {zone["id"] for zone in environment["task_zones"]}
    assert "default_drop_zone" not in physical_ids
    assert manifest["task_recipe"]["inputs"]["perception_mode"] == "epd_snapshot"
    assert manifest["task_recipe"]["pick"]["object_source"] == "perception"
    assert recipe["task"]["object_source"] == "perception"
    assert environment["robot"]["id"] == "ur5"
    assert environment["tool"]["id"] == "robotiq_85_gripper"
    assert environment["safety"] == {
        "fake_hardware_first": True,
        "real_hardware_enabled": False,
        "runtime_execution_enabled": False,
        "motion_command_sent": False,
    }


def test_normal_web_scene_does_not_export_fixture_as_physical_equipment() -> None:
    payload = web_scene_exporter.build_web_scene(SCENE, stage_assets=False)
    rendered_ids = {
        item.get("id")
        for collection in ("assets", "sensors", "robots", "tools")
        for item in payload.get(collection, [])
    }

    assert "commissioning_object" not in rendered_ids
    assert "target_bin_default" in rendered_ids
    assert "realsense_overhead" in rendered_ids


def test_canonical_web_scene_has_one_primary_owner_per_support_surface_and_camera() -> None:
    for scene_name, support_id, camera_id in (
        ("ur5_2f_test", "support_surface_table", "realsense_overhead"),
        ("suction_test", "table_main", "realsense_suction_overhead"),
    ):
        payload = web_scene_exporter.build_web_scene(ROOT / "scenes" / scene_name, stage_assets=False)
        records = [item for section in ("assets", "sensors") for item in payload.get(section, [])]
        primary = [item for item in records if item.get("render_policy") == "primary"]
        support = [item for item in primary if item.get("readiness_category") == "workbench_support_surface"]
        cameras = [item for item in primary if item.get("readiness_category") == "configured_camera"]
        assert len(support) == 1
        assert len(cameras) == 1
        assert any(owner.get("id") == support_id for owner in payload["ui_selection_owners"])
        assert any(owner.get("id") == camera_id for owner in payload["ui_selection_owners"])
        assert payload["render_ownership_summary"]["duplicate_primary_identities"] == 0


def test_perception_pick_source_is_not_misclassified_as_a_missing_zone() -> None:
    for scene_name in ("ur5_2f_test", "suction_test"):
        report = scene_readiness.check_readiness(None, ROOT / "scenes" / scene_name)
        assert not [warning for warning in report["warnings"] if "detected_objects/v1" in warning]
