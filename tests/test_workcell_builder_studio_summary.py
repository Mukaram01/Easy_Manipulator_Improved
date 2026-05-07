from __future__ import annotations

import json
from pathlib import Path

from scripts.workcell_builder_studio_summary import summarize_builder_scene, write_builder_export_summary


def _write_scene(scene: Path) -> None:
    (scene / "package.xml").write_text("<package/>", encoding="utf-8")
    (scene / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.5)", encoding="utf-8")
    (scene / "environment.yaml").write_text(
        "robot: {name: ur5}\nend_effector: {name: robotiq_2f}\nobjects: {}\n", encoding="utf-8"
    )
    (scene / "workcell_builder_metadata.yaml").write_text("fake_hardware_ready: true\n", encoding="utf-8")
    gen = scene / "generated"
    gen.mkdir()
    (gen / "cell_definition.yaml").write_text("schema_version: cell_definition/v1\n", encoding="utf-8")
    (gen / "environment_layout.yaml").write_text(
        "schema_version: environment_layout/v1\nassets:\n - id: table_1\n   type: table\n - id: cam\n   type: camera\nzones:\n - id: pick_zone_a\n   type: pick_zone\n - id: bin_a\n   type: bin\n",
        encoding="utf-8",
    )


def test_summary_contains_required_fields(tmp_path: Path) -> None:
    _write_scene(tmp_path)
    summary = summarize_builder_scene(tmp_path)
    assert summary["selected_robot"] == "ur5"
    assert summary["selected_end_effector"] == "robotiq_2f"
    assert summary["generated_scene_package_path"] == str(tmp_path)
    assert summary["use_fake_hardware_default"] is True
    assert summary["readiness_status"] in {"OK", "WARN", "FAIL"}
    assert summary["no_runtime_execution_default"] is True


def test_write_builder_export_summary_artifact(tmp_path: Path) -> None:
    _write_scene(tmp_path)
    output = write_builder_export_summary(tmp_path)
    assert output.name == "builder_export_summary.json"
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["selected_robot"] == "ur5"
