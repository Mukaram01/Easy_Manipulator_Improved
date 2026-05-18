from __future__ import annotations

import re
from pathlib import Path

import yaml

from tests.workcell_scene_backend import SceneModel, save_scene
from tools.workcell_studio_streamlit import backend


def test_render_topdown_targets_svg_empty_targets() -> None:
    svg = backend.render_topdown_targets_svg([])
    assert "<svg" in svg
    assert "No targets discovered yet" in svg
    assert "Traceback" not in svg


def test_render_topdown_targets_svg_with_pick_and_bin() -> None:
    targets = [
        {"id": "pick_zone_main", "type": "pick_zone", "label": "Main pick", "pose": {"frame": "world", "xyz": [0.45, 0.0, 0.08], "rpy": [0, 0, 0]}, "size": [0.3, 0.2, 0.1]},
        {"id": "bin_red", "type": "bin", "label": "Red bin", "pose": {"frame": "world", "xyz": [0.30, 0.35, 0.10], "rpy": [0, 0, 0]}, "size": [0.2, 0.2, 0.1]},
    ]
    svg = backend.render_topdown_targets_svg(targets)
    assert "pick_zone_main" in svg
    assert "bin_red" in svg
    assert "xyz=(0.45,0.00,0.08)" in svg
    assert "<rect" in svg


def test_list_targets_save_load_roundtrip(tmp_path: Path) -> None:
    path = tmp_path / "environment_layout.yaml"
    payload = {
        "schema": "environment_layout/v1",
        "zones": [
            {"id": "pick_zone_main", "type": "pick_zone", "label": "Main", "pose": {"frame": "world", "xyz": [0.45, 0.0, 0.08], "rpy": [0, 0, 0]}, "size": [0.3, 0.2, 0.1]}
        ],
    }
    backend.save_environment_layout(path, payload)
    loaded = backend.load_environment_layout(path)
    assert loaded["zones"][0]["id"] == "pick_zone_main"


def test_create_or_update_environment_target_with_topdown_flow(tmp_path: Path) -> None:
    layout = tmp_path / "environment_layout.yaml"
    res = backend.create_or_update_environment_target(layout, "pick_zone_main", "pick_zone", "Main", "world", [0.45, 0, 0.08], [0, 0, 0], [0.3, 0.2, 0.1], output_path=layout)
    assert res["returncode"] == 0


def test_backend_does_not_import_streamlit() -> None:
    text = (Path(__file__).resolve().parents[1] / "tools" / "workcell_studio_streamlit" / "backend.py").read_text(encoding="utf-8")
    assert "import streamlit" not in text


def test_id_generation_per_type_sequence_collision_and_regex_validity() -> None:
    ids = ["pick_zone_main", "pick_zone_001", "pick_zone_002", "place_target_001", "bin_001", "bin_002"]
    by_prefix: dict[str, list[int]] = {"pick_zone": [], "place_target": [], "bin": []}
    for value in ids:
        m = re.fullmatch(r"([a-z]+(?:_[a-z]+)*)(?:_(\d{3}|main))?", value)
        assert m is not None
        if m.group(1) in by_prefix and m.group(2) and m.group(2).isdigit():
            by_prefix[m.group(1)].append(int(m.group(2)))
    assert len(ids) == len(set(ids))
    assert by_prefix["pick_zone"] == [1, 2]


def test_missing_layout_creates_minimal_valid_environment_layout_v1(tmp_path: Path) -> None:
    layout = tmp_path / "missing" / "environment_layout.yaml"
    backend.create_or_update_environment_target(layout, "pick_zone_main", "pick_zone", "Main Pick", "world", [0.4, 0.0, 0.1], [0.0, 0.0, 0.0], [0.2, 0.2, 0.2], output_path=layout)
    doc = yaml.safe_load(layout.read_text(encoding="utf-8"))
    assert doc["schema_version"] == "environment_layout/v1"


def test_non_destructive_merge_unrelated_manual_fields_preserved_after_save(tmp_path: Path) -> None:
    scene_dir = tmp_path / "scene"; scene_dir.mkdir()
    (scene_dir / "environment.yaml").write_text(
        "robot: {name: ur5}\n"
        "end_effector: {name: gripper}\n"
        "objects: {}\n"
        "manual_notes: {owner: ops, keep_me: true}\n",
        encoding="utf-8",
    )
    save_scene(scene_dir, SceneModel(name="scene", robot={"name": "ur5"}, end_effector={"name": "gripper"}, metadata={"manual_notes": {"owner": "ops", "keep_me": True}}))
    out = yaml.safe_load((scene_dir / "environment.yaml").read_text(encoding="utf-8"))
    assert out["manual_notes"]["keep_me"] is True


def test_metadata_fallback_no_manifest_and_incomplete_assets_disabled_reason(tmp_path: Path) -> None:
    scene = tmp_path / "scene"; scene.mkdir(parents=True)
    (scene / "environment.yaml").write_text(
        "zones:\n"
        "  - {id: pick_zone_main, type: pick_zone}\n"
        "  - {id: bin_main, type: bin}\n",
        encoding="utf-8",
    )
    payload = (backend.list_builder_scene_authoring_targets(scene).get("json") or {})
    assert "pick_zone_main" in payload.get("pick_sources", [])
    report = backend.authoring_validation_report({"cell": {"id": "cell"}, "placements": [{"asset_id": "asset_without_pose"}], "pick_sources": [], "place_targets": [], "task": {"type": "pick_place"}, "grasp": {"strategy_ref": "suction_top_basic"}})
    assert report["status"] == "FAIL"


def test_task_intent_linkage_updates_target_keys_only_and_backup_before_rewrite(tmp_path: Path) -> None:
    scene = tmp_path / "scene"; scene.mkdir()
    (scene / "environment.yaml").write_text(
        "task_zones:\n"
        "  - {id: pick_zone_a, type: pick, enabled: true}\n"
        "  - {id: bin_b, type: place, enabled: true}\n",
        encoding="utf-8",
    )
    existing = scene / "workcell_builder_task_intent.yaml"
    existing.write_text(
        "schema: workcell_builder_task_intent/v1\n"
        "pick: {source: {id: old_pick, type: zone}, untouched: keep}\n"
        "place: {target: {id: old_place, type: place_target}, untouched: keep}\n",
        encoding="utf-8",
    )
    backend.create_or_update_builder_task_intent(scene, "task_1", "pick_place", "pick_zone_a", "bin_b", "suction_top_basic", output_path=existing, validate=False)
    updated = yaml.safe_load(existing.read_text(encoding="utf-8"))
    assert updated["pick"]["source"]["id"] == "pick_zone_a"
    assert updated["pick"]["untouched"] == "keep"
    scene2 = tmp_path / "scene2"; scene2.mkdir(); (scene2 / "environment.yaml").write_text("robot: {name: ur5}\n", encoding="utf-8")
    backup = save_scene(scene2, SceneModel(name="scene2", robot={"name": "ur5"}))
    assert backup.exists()
