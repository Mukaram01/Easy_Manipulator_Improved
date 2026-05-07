from __future__ import annotations

import tempfile
from pathlib import Path

from scripts.workcell_builder_grasp_strategy import normalize_grasp_strategy
from scripts.export_builder_scene_to_cell_definition import export_scene


def test_default_grasp_strategy_exists() -> None:
    payload, warns = normalize_grasp_strategy({}, {"family": "finger", "capability_id": "robotiq_2f_85"})
    assert payload["strategy_id"] == "auto"
    assert payload["approach_axis"] == "z_down"
    assert payload["compatibility_status"] == "PASS"
    assert not warns


def test_suction_top_allowed_only_for_suction_tools() -> None:
    suction_payload, suction_warns = normalize_grasp_strategy({"strategy_id": "suction_top"}, {"family": "suction"})
    assert suction_payload["compatibility_status"] == "PASS"
    assert not suction_warns

    finger_payload, finger_warns = normalize_grasp_strategy({"strategy_id": "suction_top"}, {"family": "finger", "capability_id": "robotiq_2f_85"})
    assert finger_payload["compatibility_status"] == "WARN"
    assert any("incompatible" in w or "typically" in w for w in finger_warns)


def test_finger_strategies_allowed_for_robotiq_2f() -> None:
    for sid in ("finger_top", "finger_side"):
        payload, warns = normalize_grasp_strategy({"strategy_id": sid}, {"family": "finger", "capability_id": "robotiq_2f_85"})
        assert payload["compatibility_status"] == "PASS"
        assert warns == []


def test_invalid_combinations_produce_warn_metadata() -> None:
    payload, warns = normalize_grasp_strategy({"strategy_id": "suction_side"}, {"family": "finger", "capability_id": "robotiq_2f_85"})
    assert payload["compatibility_status"] == "WARN"
    assert payload["compatibility_warnings"]
    assert any("placeholder" in w for w in warns)


def test_generated_metadata_includes_grasp_strategy() -> None:
    with tempfile.TemporaryDirectory() as d:
        root = Path(d) / "scene"
        root.mkdir()
        (root / "package.xml").write_text("<package/>", encoding="utf-8")
        (root / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.5)", encoding="utf-8")
        (root / "environment.yaml").write_text(
            "robot: {name: ur5}\nend_effector: {name: robotiq_2f}\nobjects: {box1: {filepath: meshes/box.stl, dimensions: [0.1,0.1,0.1]}}\n",
            encoding="utf-8",
        )
        (root / "workcell_builder_metadata.yaml").write_text(
            '{"robot":{"capability_id":"ur5"},"end_effector":{"capability_id":"robotiq_2f_85","family":"finger"},"grasp_strategy":{"strategy_id":"finger_top","orientation_mode":"fixed","approach_distance_m":0.12}}',
            encoding="utf-8",
        )
        out = root / "generated"
        summary = export_scene(root, out, validate=False)
        assert summary["generated_by"] == "workcell_builder"
        cell = (out / "cell_definition.yaml").read_text(encoding="utf-8")
        assert "strategy_ref: finger_top" in cell
        assert "orientation_mode: fixed" in cell
