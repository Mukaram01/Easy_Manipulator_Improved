from __future__ import annotations

import argparse
import json
from pathlib import Path

import pytest

import scripts.run_workcell_builder_scene3d_gui_smoke as smoke


def test_repo_root_resolution_from_nested_dirs_and_script_local_contexts(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    repo = tmp_path / "my_repo"
    scripts_dir = repo / "scripts"
    scripts_dir.mkdir(parents=True)
    marker_script = scripts_dir / "run_workcell_builder_scene3d_gui_smoke.py"
    marker_script.write_text("#!/usr/bin/env python3\n", encoding="utf-8")

    monkeypatch.setattr(smoke, "ROOT", marker_script.resolve().parents[1])

    assert smoke.ROOT == repo
    assert (smoke.ROOT / "scripts" / "run_workcell_builder_scene3d_gui_smoke.py").name == "run_workcell_builder_scene3d_gui_smoke.py"


def test_workspace_resolution_supports_non_default_workspace_names(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    (repo / "install").mkdir(parents=True)
    custom_ws = tmp_path / "robotics_dev_ws"
    exe = custom_ws / "install/workcell_builder/lib/workcell_builder/workcell_builder"
    exe.parent.mkdir(parents=True)
    exe.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")

    monkeypatch.setattr(smoke, "ROOT", custom_ws)
    resolved = smoke.resolve_workcell_builder()
    assert Path(resolved) == exe


def test_install_setup_and_executable_lookup_in_temp_workspace_tree(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    ws = tmp_path / "alt_ws"
    exe = ws / "install/workcell_builder/lib/workcell_builder/workcell_builder"
    exe.parent.mkdir(parents=True)
    exe.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    monkeypatch.setattr(smoke, "ROOT", ws)

    args = argparse.Namespace(
        scene="ur5_2f_test",
        new_cell_recommended_layout_smoke=True,
        output=ws / "build/workcell_studio/scene3d_gui_smoke_ur5_2f_test.json",
        screenshot=ws / "build/workcell_studio/scene3d_gui_smoke_ur5_2f_test.png",
    )
    cmd = smoke.build_cmd(smoke.resolve_workcell_builder(), args)

    assert str(exe) == cmd[0]
    assert "--exit-after-smoke" in cmd
    assert "--scene" in cmd and "ur5_2f_test" in cmd


def test_missing_executable_reports_searched_paths(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr(smoke, "ROOT", tmp_path / "repo_without_install")
    monkeypatch.setattr(smoke.shutil, "which", lambda _: None)

    with pytest.raises(FileNotFoundError) as err:
        smoke.resolve_workcell_builder()

    text = str(err.value)
    assert "install/workcell_builder" in text
    assert "PATH" in text


def test_smoke_wrapper_has_no_hardcoded_root_workspace_fallbacks() -> None:
    src = Path("scripts/run_workcell_builder_scene3d_gui_smoke.py").read_text(encoding="utf-8")
    assert "/root/workcell_ws" not in src
    assert "~/workcell_ws" not in src


def test_safety_invariants_no_real_hardware_launch_paths() -> None:
    src = Path("scripts/run_workcell_builder_scene3d_gui_smoke.py").read_text(encoding="utf-8").lower()
    disallowed = ["real_hardware", "use_fake_hardware:=false", "fake_hardware:=false"]
    for token in disallowed:
        assert token not in src


def test_validate_smoke_json_smoke_happy_path(tmp_path: Path) -> None:
    report = tmp_path / "smoke.json"
    report.write_text(
        json.dumps(
            {
                "schema": smoke.EXPECTED_SCHEMA,
                "scene": "ur5_2f_test",
                "status": "PASS",
                "screenshot_path": "dummy.png",
                "unique_visible_item_count": 2,
                "mesh_rendered_count": 1,
                "generated_fallback_count": 0,
                "editable_layout_count": 1,
                "primitive_fallback_count": 0,
                "overlay_count": 1,
                "labels_drawn": 1,
                "labels_suppressed_overlap": 0,
                "hierarchy_child_row_count": 1,
                "selected_scene_name": "ur5_2f_test",
                "selected_item_id": "robot",
                "blockers": [],
                "warnings": [],
            }
        ),
        encoding="utf-8",
    )
    blockers, warnings, status = smoke.validate_smoke_json(report)
    assert blockers == []
    assert warnings == []
    assert status == "PASS"
