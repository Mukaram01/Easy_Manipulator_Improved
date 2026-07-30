import json
from pathlib import Path

from scripts import workcell_builder_gui_workflow

REPO = Path(__file__).resolve().parents[1]
SCENE_SELECT_CPP = REPO / "workcell_builder/workcell_builder/gui/scene_select.cpp"
SCENE_SELECT_H = REPO / "workcell_builder/workcell_builder/gui/scene_select.h"
WEB_VIEWER_FILES = (
    REPO / "workcell_studio_web/viewer/index.html",
    REPO / "workcell_studio_web/viewer/main.js",
    REPO / "workcell_studio_web/viewer/README.md",
)


def _cpp() -> str:
    return SCENE_SELECT_CPP.read_text(encoding="utf-8")


def _between(source: str, start: str, end: str) -> str:
    return source[source.index(start) : source.index(end, source.index(start))]


def _generate_validate_action_section() -> str:
    source = _cpp()
    return _between(
        source,
        'auto * generate_validate_scene_action = new QPushButton',
        'connect(ui->fit_cell_action',
    )


def _execute_generate_validate_section() -> str:
    source = _cpp()
    return _between(
        source,
        'bool SceneSelect::execute_generate_validate_after_web_edit',
        'bool SceneSelect::run_generate_validate_after_web_edit',
    )


def _run_generate_validate_section() -> str:
    source = _cpp()
    return _between(
        source,
        'bool SceneSelect::run_generate_validate_after_web_edit',
        'bool SceneSelect::execute_web_edit_patch_workflow',
    )


def test_generate_validate_label_or_after_web_edit_label_exists():
    source = _cpp()
    assert '"Generate & Validate Scene"' in source or '"Generate & Validate After Web Edit"' in source
    assert "generate_validate_after_web_edit_action" in source
    assert "on_generate_validate_after_web_edit_clicked" in source
    assert "on_generate_validate_after_web_edit_clicked" in SCENE_SELECT_H.read_text(encoding="utf-8")


def test_generate_validate_action_uses_selected_scene_path():
    run_section = _run_generate_validate_section()
    execute_section = _execute_generate_validate_section()
    assert "current_scene_index()" in run_section
    assert "scene_dir_for_current_selection()" in run_section
    assert "selected scene" in run_section
    assert "execute_generate_validate_after_web_edit(repo_root, scene_dir" in run_section
    assert '"--scene", QString::fromStdString(scene_dir.string())' in execute_section


def test_generate_validate_action_calls_supported_web_edit_workflow_script():
    combined = _generate_validate_action_section() + _execute_generate_validate_section() + _run_generate_validate_section()
    assert "scripts/run_workcell_studio_web_edit_workflow.py" in combined
    assert 'process.start("python3", args)' in combined
    assert "validate_workcell_studio_web_scene_edit_patch.py" not in combined
    assert "apply_workcell_studio_web_scene_edit_patch.py" not in combined
    assert "verify_workcell_studio_web_scene_edit_persistence.py" not in combined


def test_generate_validate_action_passes_generate_and_validate_flag():
    execute_section = _execute_generate_validate_section()
    assert '"--generate-and-validate"' in execute_section
    assert '"--generate"' not in execute_section.replace('"--generate-and-validate"', "")
    assert '"--patch"' not in execute_section
    assert '"--write"' not in execute_section


def test_generate_validate_action_has_no_ros_rviz_moveit_or_gazebo_launch_commands():
    combined = _generate_validate_action_section() + _execute_generate_validate_section() + _run_generate_validate_section()
    forbidden_launch_tokens = (
        "ros2 launch",
        "launch_rviz",
        "rviz2",
        "move_group",
        "controller_manager",
        "gazebo",
        "ign gazebo",
        "gz sim",
        "isaac",
        "use_fake_hardware:=false",
        "real_hardware:=true",
    )
    for token in forbidden_launch_tokens:
        assert token.lower() not in combined.lower()


def test_generate_validate_requires_confirmation_before_mutating_generated_outputs():
    run_section = _run_generate_validate_section()
    confirm_index = run_section.index("Confirm Generate & Validate Scene")
    mutation_warning_index = run_section.index("may mutate generated scene/package outputs")
    question_index = run_section.index("QMessageBox::question")
    execute_index = run_section.index("execute_generate_validate_after_web_edit(repo_root, scene_dir")
    assert question_index < confirm_index < execute_index
    assert mutation_warning_index < question_index < execute_index
    assert "QMessageBox::No" in run_section
    assert "cancelled before mutating generated outputs" in run_section


def test_generate_validate_pass_fail_messaging_is_present():
    combined = _execute_generate_validate_section() + _run_generate_validate_section()
    assert "Generate & Validate Scene PASS" in combined
    assert "Generate & Validate Scene FAIL" in combined
    assert 'QString(ok ? "PASS" : "FAIL")' in combined
    assert "Exit code:" in combined
    assert "STDOUT:" in combined
    assert "STDERR:" in combined


def test_generation_cli_prints_false_result_and_returns_nonzero(monkeypatch, capsys, tmp_path):
    result = {"ok": False, "error": f"Required canonical output is absent: {tmp_path / 'generated/cell_definition.yaml'}"}
    monkeypatch.setattr(workcell_builder_gui_workflow, "generate_files_from_yaml", lambda scene: result)

    rc = workcell_builder_gui_workflow.main(["--generate-from-yaml", str(tmp_path)])

    assert rc != 0
    assert json.loads(capsys.readouterr().out) == result


def test_generation_cli_prints_exact_generated_files_on_success(monkeypatch, capsys, tmp_path):
    generated = [str(tmp_path / "generated" / name) for name in (
        "cell_definition.yaml",
        "environment_layout.yaml",
        "task_recipe_from_builder_intent.yaml",
        "offline_plan_preview_request.yaml",
        "selected_assets.json",
        "compatibility_report.json",
        "builder_export_summary.json",
    )]
    result = {"ok": True, "generated_files": generated}
    monkeypatch.setattr(workcell_builder_gui_workflow, "generate_files_from_yaml", lambda scene: result)

    rc = workcell_builder_gui_workflow.main(["--generate-from-yaml", str(tmp_path)])

    assert rc == 0
    assert json.loads(capsys.readouterr().out)["generated_files"] == generated


def test_no_direct_browser_yaml_write_logic_is_added_to_web_viewer_or_builder_action():
    action_source = _generate_validate_action_section() + _execute_generate_validate_section() + _run_generate_validate_section()
    browser_source = "\n".join(path.read_text(encoding="utf-8") for path in WEB_VIEWER_FILES if path.exists())
    assert "browser never writes YAML directly" in action_source
    assert "Export Edit Patch" in browser_source
    forbidden_browser_write_tokens = (
        "environment.yaml",
        "workcell_studio_layout.yaml",
        "cell_definition.yaml",
        "scene_manifest.yaml",
        "js-yaml",
        "yaml.dump",
        "YAML.stringify",
        "fetch(\"file://",
    )
    for token in forbidden_browser_write_tokens:
        assert token not in action_source
    js_source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in WEB_VIEWER_FILES
        if path.exists() and path.suffix in {".js", ".html"}
    )
    for token in forbidden_browser_write_tokens:
        assert token not in js_source


def test_no_qt_scene3d_visual_topology_or_screenshot_scope_is_added_to_action():
    combined = _generate_validate_action_section() + _execute_generate_validate_section() + _run_generate_validate_section()
    forbidden_scope_tokens = ("Qt Scene3D visual", "topology", "screenshot")
    for token in forbidden_scope_tokens:
        assert token not in combined
