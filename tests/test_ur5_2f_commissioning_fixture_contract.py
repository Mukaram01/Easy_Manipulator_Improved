from pathlib import Path

import yaml

from scripts import export_workcell_studio_web_scene as web_scene_exporter
from scripts import generate_scene_from_cell_definition as scene_generator
from scripts import validate_cell_definition as cell_validator


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "ur5_2f_test"


def _load(relative_path: str) -> dict:
    return yaml.safe_load((SCENE / relative_path).read_text(encoding="utf-8"))


def test_commissioning_object_is_a_deterministic_self_test_fixture() -> None:
    cell = _load("cell_definition.yaml")

    assert cell["objects"] == []
    assert cell["self_test"] == {
        "enabled": True,
        "object": {
            "id": "commissioning_object",
            "shape": "box",
            "class": "part",
            "color": "red",
            "material": "plastic",
            "frame": "world",
            "pose_xyz": [0.45, 0.25, 0.1],
            "pose_rpy": [0.0, 0.0, 0.0],
            "dimensions": [0.05, 0.05, 0.05],
        },
    }
    assert cell["task"]["source_object"] == "commissioning_object"
    assert cell["task"]["object_source"] == "self_test"

    summary = cell_validator.validate_cell_definition(cell, SCENE / "cell_definition.yaml", "pyyaml", [])
    assert summary.errors == []
    generated_manifest = scene_generator.build_scene_manifest(cell)
    assert generated_manifest["self_test"]["object"]["id"] == "commissioning_object"
    assert generated_manifest["task_recipe"]["pick"]["object_source"] == "self_test"


def test_physical_and_task_layers_remain_separate_and_safe() -> None:
    environment = _load("environment.yaml")
    layout = _load("environment_layout.yaml")
    manifest = _load("scene_manifest.yaml")
    recipe = _load("config/task_recipe.yaml")

    physical_ids = {
        item["id"]
        for collection in (environment["environment"]["support_surfaces"], environment["environment"]["assets"])
        for item in collection
    }
    assert {"support_surface_table", "target_bin_default", "realsense_overhead"} <= physical_ids
    assert "commissioning_object" not in physical_ids
    assert "commissioning_object" not in {item["id"] for item in layout["objects"]}
    assert "default_drop_zone" in {zone["id"] for zone in environment["task_zones"]}
    assert "default_drop_zone" not in physical_ids
    assert manifest["task_recipe"]["inputs"]["perception_mode"] == "off"
    assert manifest["task_recipe"]["pick"]["object_source"] == "self_test"
    assert recipe["task"]["object_source"] == "self_test"
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
