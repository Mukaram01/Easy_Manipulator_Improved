from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCENE_SELECT = REPO / "workcell_builder/workcell_builder/gui/scene_select.cpp"
HEADER = REPO / "workcell_builder/workcell_builder/gui/scene_select.h"
DOC = REPO / "docs/WORKCELL_STUDIO_WEB_3D_EDIT_PATCH.md"
VIEWER_README = REPO / "workcell_studio_web/viewer/README.md"


def _cpp() -> str:
    return SCENE_SELECT.read_text(encoding="utf-8")


def test_visible_workcell_builder_patch_actions_are_registered():
    source = _cpp()
    assert "Validate Web Edit Patch…" in source
    assert "Dry Run Web Edit Patch…" in source
    assert "Apply Web Edit Patch…" in source
    assert "Generate & Validate Scene" in source
    assert "validate_web_edit_patch_action" in source
    assert "dry_run_web_edit_patch_action" in source
    assert "apply_web_edit_patch_action" in source
    assert "generate_validate_after_web_edit_action" in source
    header = HEADER.read_text(encoding="utf-8")
    assert "on_validate_web_edit_patch_clicked" in header
    assert "on_generate_validate_after_web_edit_clicked" in header


def test_generate_validate_scene_action_uses_backend_without_patch_and_requires_confirmation():
    source = _cpp()
    section = source[source.index("bool SceneSelect::run_generate_validate_after_web_edit") : source.index("bool SceneSelect::execute_web_edit_patch_workflow")]
    assert "Confirm Generate & Validate Scene" in section
    assert "QMessageBox::question" in section
    assert "QFileDialog::getOpenFileName" not in section
    assert "--patch" not in section
    assert "--generate-and-validate" in source
    assert "python3 scripts/run_workcell_studio_web_edit_workflow.py" in source
    assert "Generate & Validate Scene PASS" in source
    assert "Generate & Validate Scene FAIL" in source
    assert "Patch applied and verified. You can now Generate & Validate Scene." in source
    assert "generation/validation is explicit" in source
    assert "browser never writes YAML directly" in source
    assert "this does not launch fake hardware" in source
    assert "this does not move real hardware" in source


def test_workflow_script_is_reused_instead_of_duplicate_backend_logic():
    source = _cpp()
    assert "scripts" in source
    assert "run_workcell_studio_web_edit_workflow.py" in source
    assert "validate_workcell_studio_web_scene_edit_patch.py" not in source
    assert "apply_workcell_studio_web_scene_edit_patch.py" not in source
    assert "verify_workcell_studio_web_scene_edit_persistence.py" not in source


def test_dry_run_command_omits_write_and_apply_write_is_confirmation_gated():
    source = _cpp()
    dry_run_section = source[source.index("bool SceneSelect::execute_web_edit_patch_workflow") : source.index("bool SceneSelect::run_web_edit_patch_workflow")]
    assert 'args << "--dry-run-apply"' in dry_run_section
    assert 'if (write) {\n    args << "--write";' in dry_run_section
    workflow_section = source[source.index("bool SceneSelect::run_web_edit_patch_workflow") :]
    confirm_index = workflow_section.index("Confirm Apply Web Edit Patch")
    write_call_index = workflow_section.index("execute_web_edit_patch_workflow(repo_root, scene_dir, patch_path, false, true")
    assert confirm_index < write_call_index
    first_dry_run_index = workflow_section.index("execute_web_edit_patch_workflow(repo_root, scene_dir, patch_path, validate_only, false")
    assert first_dry_run_index < confirm_index


def test_default_patch_folder_and_patch_json_validation():
    source = _cpp()
    assert 'repo_root / "build" / "workcell_studio_web_scene"' in source
    assert "QFileDialog::getOpenFileName" in source
    assert 'patch_info.suffix().compare("json"' in source
    assert "QJsonParseError" in source


def test_generated_directory_write_target_is_not_introduced():
    source = _cpp()
    workflow_source = source[source.index("void SceneSelect::on_validate_web_edit_patch_clicked") :]
    assert ' / "generated"' not in workflow_source
    assert 'generated/' not in workflow_source[source.index("Web edit patch workflow safety") :]


def test_safety_wording_mentions_dry_run_confirmation_and_no_robot_motion():
    combined = "\n".join(path.read_text(encoding="utf-8") for path in (SCENE_SELECT, DOC, VIEWER_README))
    assert "dry-run" in combined.lower()
    assert "explicit confirmation" in combined.lower()
    assert "no real robot motion" in combined.lower()
    assert "RViz/MoveIt remains" in combined
    assert "browser edits are preview-only" in combined


def test_no_direct_browser_yaml_write_or_frontend_stack_logic_added():
    combined = "\n".join(path.read_text(encoding="utf-8") for path in (SCENE_SELECT, DOC, VIEWER_README))
    assert "browser never writes source YAML" in combined
    assert "npm" not in SCENE_SELECT.read_text(encoding="utf-8").lower()
    assert "package.json" not in SCENE_SELECT.read_text(encoding="utf-8").lower()


def test_no_qt_scene3d_visual_topology_screenshot_scope_reintroduced():
    workflow_source = _cpp()[_cpp().index("void SceneSelect::on_validate_web_edit_patch_clicked") :]
    forbidden = ("Qt Scene3D visual", "topology", "screenshot")
    for token in forbidden:
        assert token not in workflow_source
