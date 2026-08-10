from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from scripts.workcell_builder_gui_workflow import generate_files_from_yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE = REPO_ROOT / "scenes" / "ur5_2f_test"
PACKAGE_GENERATOR = REPO_ROOT / "scripts" / "generate_workcell_from_cell_definition.py"
MAINWINDOW_CPP = (
    REPO_ROOT / "workcell_builder" / "workcell_builder" / "gui" / "mainwindow.cpp"
).read_text(encoding="utf-8")
COMMAND_BUILDERS_CPP = (
    REPO_ROOT
    / "workcell_builder"
    / "workcell_builder"
    / "src_workcell_builder_command_builders.cpp"
).read_text(encoding="utf-8")


def _without_nondeterministic_fields(path: Path):
    """Load output while excluding the documented generation timestamp keys."""
    if path.suffix == ".json":
        value = json.loads(path.read_text(encoding="utf-8"))
    else:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))

    def clean(item):
        if isinstance(item, dict):
            return {key: clean(val) for key, val in item.items()
                    if key not in {"generated_at", "generated_at_utc", "generation_timestamp"}}
        if isinstance(item, list):
            return [clean(val) for val in item]
        return item

    return clean(value)


def test_existing_scene_regeneration_uses_saved_layout_without_rewriting_authored_files(tmp_path):
    scene = tmp_path / "ur5_2f_test"
    shutil.copytree(SCENE, scene)
    layout_path = scene / "layout" / "workcell_studio_layout.yaml"
    environment_path = scene / "environment.yaml"
    environment_layout_path = scene / "environment_layout.yaml"
    task_intent_path = scene / "config" / "workcell_builder_task_intent.yaml"

    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    items = {item["id"]: item for item in layout["items"]}
    edited_xyz = [0.71, -0.19, 0.24]
    items["target_bin_default"]["pose"]["xyz"] = edited_xyz
    items["place_zone_default"]["pose"]["xyz"] = edited_xyz
    layout_path.write_text(yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")

    authored_before = {
        environment_path: environment_path.read_bytes(),
        environment_layout_path: environment_layout_path.read_bytes(),
        layout_path: layout_path.read_bytes(),
        task_intent_path: task_intent_path.read_bytes(),
    }
    first = generate_files_from_yaml(scene)
    assert first["ok"], first
    expected_names = (
        "cell_definition.yaml",
        "environment_layout.yaml",
        "task_recipe_from_builder_intent.yaml",
        "offline_plan_preview_request.yaml",
        "selected_assets.json",
        "compatibility_report.json",
        "builder_export_summary.json",
    )
    assert first["generated_files"] == [str(scene / "generated" / name) for name in expected_names]

    generated_layout = yaml.safe_load(
        (scene / "generated" / "environment_layout.yaml").read_text(encoding="utf-8")
    )
    generated_items = {item["id"]: item for item in generated_layout["items"]}
    target = generated_items["target_bin_default"]
    destination = generated_items["place_zone_default"]
    assert target["pose"]["xyz"] == edited_xyz
    assert destination["target_ref"] == target["id"]
    assert destination["transform_group"] == target["transform_group"]
    assert destination["pose"] == target["pose"]
    generated_cell = yaml.safe_load(
        (scene / "generated" / "cell_definition.yaml").read_text(encoding="utf-8")
    )
    place = generated_cell["task"]["place"]["target"]
    assert place["id"] == "default_drop_zone"
    assert place["layout_item_ref"] == "place_zone_default"
    assert place["target_ref"] == "target_bin_default"
    generated_destination = generated_cell["task"]["destinations"][0]
    assert generated_destination["target_ref"] == "target_bin_default"
    assert generated_destination["pose_xyz"] == edited_xyz
    assert generated_destination["pose_rpy"] == target["pose"]["rpy"]
    assert {path: path.read_bytes() for path in authored_before} == authored_before

    first_generation = {
        Path(path).name: _without_nondeterministic_fields(Path(path))
        for path in first["generated_files"]
    }
    second = generate_files_from_yaml(scene)
    assert second["ok"], second
    second_generation = {
        Path(path).name: _without_nondeterministic_fields(Path(path))
        for path in second["generated_files"]
    }
    assert second_generation == first_generation
    assert {path: path.read_bytes() for path in authored_before} == authored_before


def test_scene_package_generation_refreshes_selected_package_in_place_without_duplicate(tmp_path):
    repo_root = tmp_path / "easy_manipulation_deployment"
    scene = repo_root / "scenes" / "ur5_2f_test"
    shutil.copytree(SCENE, scene)
    workspace_duplicate = tmp_path / "scenes" / scene.name
    preserved_relative_paths = [
        "package.xml",
        "CMakeLists.txt",
        "launch/demo.launch.py",
        "urdf/scene.urdf.xacro",
        "config/task_recipe.yaml",
        "config/workcell_builder_task_intent.yaml",
        "layout/workcell_studio_layout.yaml",
        "environment.yaml",
        "environment_layout.yaml",
        "cell_definition.yaml",
        "scene_manifest.yaml",
    ]
    preserved = [scene / relative for relative in preserved_relative_paths]
    before = {path: path.read_bytes() for path in preserved}
    yaml_result = generate_files_from_yaml(scene)
    assert yaml_result["ok"], yaml_result
    assert {path: path.read_bytes() for path in preserved} == before
    readiness_path = scene / "generated" / "scene_package_readiness.json"
    readiness_path.write_text("stale generated artifact\n", encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            str(PACKAGE_GENERATOR),
            str(scene / "cell_definition.yaml"),
            "--output-dir",
            str(scene.parent),
            "--package-name",
            scene.name,
            "--existing-package-dir",
            str(scene),
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert {path: path.read_bytes() for path in preserved} == before
    assert readiness_path.is_file()
    assert readiness_path.read_text(encoding="utf-8") != "stale generated artifact\n"
    generated_layout_path = scene / "generated" / "environment_layout.yaml"
    generated_layout = yaml.safe_load(generated_layout_path.read_text(encoding="utf-8"))
    assert generated_layout["schema_version"] == "environment_layout/v1"
    assert "commissioning_pick_pose" in {
        zone["id"] for zone in generated_layout["zones"]
    }
    assert "default_drop_zone" in {
        target["id"] for target in generated_layout["targets"]
    }

    environment_validation = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "scripts" / "validate_environment_layout.py"),
            str(generated_layout_path),
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert environment_validation.returncode == 0, (
        environment_validation.stdout + environment_validation.stderr
    )

    scene_validation = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "scripts" / "validate_builder_generated_scene.py"),
            str(scene),
            "--require-generated",
            "--json",
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    scene_validation_output = scene_validation.stdout + scene_validation.stderr
    assert scene_validation.returncode == 0, scene_validation_output
    for previous_error in (
        "schema_version must be exactly 'environment_layout/v1'",
        "Routing target does not exist",
        "commissioning_pick_pose does not exist",
        "default_drop_zone does not exist",
    ):
        assert previous_error not in scene_validation_output
    assert {path: path.read_bytes() for path in preserved} == before

    xacro = (scene / "urdf" / "scene.urdf.xacro").read_text(encoding="utf-8")
    for runtime_marker in (
        "xacro:ur_robot",
        "robotiq_85_gripper",
        "table_world_xyz",
        "camera_world_xyz",
    ):
        assert runtime_marker in xacro

    launch = (scene / "launch" / "demo.launch.py").read_text(encoding="utf-8")
    for runtime_marker in (
        "load_canonical_layout_poses",
        "allow_trajectory_execution",
        "use_fake_hardware",
    ):
        assert runtime_marker in launch
    assert not workspace_duplicate.exists()
    assert not (scene.parent / f"{scene.name}.tmp-generation").exists()


def test_workcell_builder_keeps_exact_scene_identity_through_package_refresh():
    assert '<< "--existing-package-dir" << scene_dir' in COMMAND_BUILDERS_CPP
    assert '<< "--force"' not in COMMAND_BUILDERS_CPP
    assert "const fs::path selected_scene_dir" in MAINWINDOW_CPP
    assert "candidate.scene_dir == selected_scene_dir" in MAINWINDOW_CPP
    assert "invalidate_workcell_studio_scene_metadata_snapshot(selected_scene_dir" in MAINWINDOW_CPP


@pytest.mark.parametrize(
    ("break_contract", "unresolved_id"),
    [
        ("missing_pick", "missing_pick_zone"),
        ("broken_target_ref", "missing_target_bin"),
        ("inconsistent_group", "target_bin_default"),
        ("invalid_intent", "invalid/task-intent-schema"),
    ],
)
def test_existing_scene_regeneration_blocks_unresolved_authored_task_contract(
    tmp_path, break_contract, unresolved_id
):
    scene = tmp_path / "ur5_2f_test"
    shutil.copytree(SCENE, scene)
    layout_path = scene / "layout" / "workcell_studio_layout.yaml"
    intent_path = scene / "config" / "workcell_builder_task_intent.yaml"

    if break_contract == "missing_pick":
        intent = yaml.safe_load(intent_path.read_text(encoding="utf-8"))
        intent["pick"]["source"]["id"] = unresolved_id
        intent_path.write_text(yaml.safe_dump(intent, sort_keys=False), encoding="utf-8")
    elif break_contract == "broken_target_ref":
        layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
        next(item for item in layout["items"] if item["id"] == "place_zone_default")["target_ref"] = unresolved_id
        layout_path.write_text(yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")
    elif break_contract == "inconsistent_group":
        layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
        next(item for item in layout["items"] if item["id"] == unresolved_id)["transform_group"] = "different_group"
        layout_path.write_text(yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")
    else:
        intent = yaml.safe_load(intent_path.read_text(encoding="utf-8"))
        intent["schema"] = unresolved_id
        intent_path.write_text(yaml.safe_dump(intent, sort_keys=False), encoding="utf-8")

    result = generate_files_from_yaml(scene)

    assert not result["ok"]
    assert str(intent_path) in result["error"]
    assert str(layout_path) in result["error"]
    assert unresolved_id in result["error"]
    assert not (scene / "generated" / "cell_definition.yaml").exists()
