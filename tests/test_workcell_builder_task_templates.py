from __future__ import annotations

from pathlib import Path

import yaml

from scripts.validate_builder_task_intent import validate
from scripts.workcell_builder_studio_summary import summarize_builder_scene


def _write_yaml(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _scene(tmp_path: Path, template_id: str, task_type: str = "pick_place") -> tuple[Path, Path]:
    scene = tmp_path / f"scene_{template_id}"
    (scene / "generated").mkdir(parents=True, exist_ok=True)
    (scene / "package.xml").write_text("<package/>", encoding="utf-8")
    _write_yaml(scene / "environment.yaml", {"robot": {"name": "ur5"}, "end_effector": {"name": "2f"}})
    _write_yaml(scene / "generated" / "environment_layout.yaml", {
        "assets": [{"id": "cam1", "type": "camera"}],
        "zones": [{"id": "pick_zone_main", "type": "pick_zone"}, {"id": "bin_red", "type": "bin"}],
    })
    _write_yaml(scene / "workcell_builder_metadata.yaml", {"generator_version": "test"})
    intent = scene / "generated" / "workcell_builder_task_intent.yaml"
    _write_yaml(intent, {
        "schema": "workcell_builder_task_intent/v1",
        "scene_package": str(scene),
        "task": {"id": "t1", "type": task_type, "template": template_id},
        "task_template": {"id": template_id, "runtime_status": "preview_only" if template_id in {"inspection", "machine_tending"} else "supported"},
        "pick": {"source": {"id": "pick_zone_main", "type": "pick_zone"}},
        "grasp": {"strategy_ref": "finger_pinch_basic", "approach_distance_m": 0.1, "retreat_distance_m": 0.1},
        "place": {"target": {"id": "bin_red", "type": "bin"}},
        "routing": {"rules": [{"place_target": "bin_red"}]},
        "safety": {"metadata_only": True, "runtime_io_applied": False, "motion_started": False, "ros_launch_started": False},
    })
    return scene, intent


def test_pick_place_metadata_validates(tmp_path: Path) -> None:
    scene, intent = _scene(tmp_path, "pick_place")
    report = validate(intent, scene)
    assert report["status"] == "PASS"


def test_sorting_metadata_validates(tmp_path: Path) -> None:
    scene, intent = _scene(tmp_path, "sorting", task_type="sort_by_colour")
    report = validate(intent, scene)
    assert report["status"] == "PASS"


def test_preview_templates_warn_only(tmp_path: Path) -> None:
    for template in ("inspection", "machine_tending"):
        scene, intent = _scene(tmp_path, template)
        report = validate(intent, scene)
        assert report["status"] == "WARN"
        assert any("preview-only" in warning for warning in report["warnings"])


def test_generated_metadata_marks_sorting_as_template_scenario(tmp_path: Path) -> None:
    scene, _ = _scene(tmp_path, "sorting", task_type="sort_by_colour")
    summary = summarize_builder_scene(scene)
    assert summary["task_template"] == "sorting"
    assert summary["task_template_metadata"]["id"] == "sorting"
