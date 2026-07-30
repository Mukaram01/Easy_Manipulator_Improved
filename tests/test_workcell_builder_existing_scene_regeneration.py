from __future__ import annotations

import json
import shutil
from pathlib import Path

import yaml

from scripts.workcell_builder_gui_workflow import generate_files_from_yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE = REPO_ROOT / "scenes" / "ur5_2f_test"


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

    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    items = {item["id"]: item for item in layout["items"]}
    edited_xyz = [0.71, -0.19, 0.24]
    items["target_bin_default"]["pose"]["xyz"] = edited_xyz
    items["place_zone_default"]["pose"]["xyz"] = edited_xyz
    layout_path.write_text(yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")

    authored_before = {
        environment_path: environment_path.read_bytes(),
        layout_path: layout_path.read_bytes(),
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
