from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GUI = ROOT / "workcell_builder/workcell_builder/gui"
CONTROLLER = GUI / "embedded_web_edit_save_controller.hpp"
UI_UTILS = GUI / "workcell_builder_ui_utils.cpp"
WORKFLOW = ROOT / "scripts/run_workcell_studio_web_edit_workflow.py"
APPLICATOR = ROOT / "scripts/apply_workcell_studio_web_scene_edit_patch.py"


def test_qt_product_view_installs_one_contextual_save_action():
    controller = CONTROLLER.read_text(encoding="utf-8")
    ui_utils = UI_UTILS.read_text(encoding="utf-8")

    assert '#include "embedded_web_edit_save_controller.hpp"' in ui_utils
    assert "installEmbeddedWebEditSaveControllers(widget);" in ui_utils
    assert 'objectName() == QStringLiteral("scenePreviewWidget")' in controller
    assert 'findChild<QWebEngineView *>(QStringLiteral("embeddedWeb3dProductView"))' in controller
    assert 'setObjectName(QStringLiteral("embeddedSaveLayoutButton"))' in controller
    assert 'QPushButton(QStringLiteral("Save layout")' in controller
    assert 'property("workcell_embedded_save_controller")' in controller


def test_qt_reads_patch_from_existing_browser_editor_api_and_checks_identity():
    source = CONTROLLER.read_text(encoding="utf-8")
    for token in [
        "window.__WORKCELL_EDITOR_API_V1__",
        "api.getState()",
        "api.getEditPatch()",
        'kPatchSchema = "workcell_studio_web_scene_edit_patch/v1"',
        'kPatchCreator = "static_web_viewer"',
        'patch.value(QStringLiteral("scene_id")).toString() != scene_id_',
        "view_->url() == expected_url_",
        "sceneIdFromViewerUrl(view_->url()) == scene_id_",
        "Scene changed—reload required",
        "No changes",
        "Validation failed",
        "Saved",
    ]:
        assert token in source


def test_qt_writes_patch_atomically_then_runs_dry_run_before_confirmation_and_write():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert "QSaveFile output(patch_path_)" in source
    assert "output.commit()" in source
    assert 'QStringLiteral("--dry-run-apply")' in source
    assert 'QStringLiteral("--write") << QStringLiteral("--generate-and-validate")' in source
    assert "WorkflowPhase::DryRun" in source
    assert "WorkflowPhase::Write" in source
    assert "Confirm Save Product View Layout" in source
    assert source.index("startWorkflow(WorkflowPhase::DryRun)") < source.index("Confirm Save Product View Layout")
    assert source.index("Confirm Save Product View Layout") < source.index("startWorkflow(WorkflowPhase::Write)")
    assert "QProcess::MergedChannels" in source
    assert "waitForFinished" not in source


def test_successful_qt_save_reuses_backend_refreshes_and_reloads_product_view():
    controller = CONTROLLER.read_text(encoding="utf-8")
    workflow = WORKFLOW.read_text(encoding="utf-8")

    assert "scripts/run_workcell_studio_web_edit_workflow.py" in controller
    assert "scripts/apply_workcell_studio_web_scene_edit_patch.py" not in controller
    assert "scripts/validate_workcell_studio_web_scene_edit_patch.py" not in controller
    assert "if (view_) view_->reload();" in controller

    assert 'write_cmd = [*dry_cmd, "--write", "--backup"]' in workflow
    assert "persistence verification" in workflow
    assert "_product_view_refresh_cmd" in workflow
    assert "ensure_workcell_studio_web_scene_fresh.py" in workflow
    assert '"--stage-assets"' in workflow
    assert '"--force"' in workflow
    assert "Product View refresh result" in workflow


def test_source_yaml_write_is_allowlisted_backed_up_and_atomic():
    source = APPLICATOR.read_text(encoding="utf-8")
    for token in [
        '"layout/workcell_studio_layout.yaml"',
        '"environment.yaml"',
        'FORBIDDEN_TARGET_PARTS = {"generated"}',
        'FORBIDDEN_TARGET_NAMES = {"cell_definition.yaml", "scene_manifest.yaml"}',
        "shutil.copy2(path, path.with_name",
        "tempfile.mkstemp",
        "stream.flush()",
        "os.fsync(stream.fileno())",
        "os.replace(temporary, path)",
        "temporary.unlink()",
    ]:
        assert token in source
    assert "path.write_text(yaml.safe_dump" not in source


def test_save_roundtrip_has_no_browser_source_writes_or_robot_motion():
    combined = "\n".join(
        path.read_text(encoding="utf-8").lower()
        for path in (CONTROLLER, WORKFLOW, APPLICATOR)
    )
    for forbidden in [
        "execute_trajectory",
        "getmotionplan",
        "/plan_kinematic_path",
        "real_hardware_enabled: true",
        "move_group.execute",
        "ros2 launch",
        "qwebchannel",
    ]:
        assert forbidden not in combined
    assert "no robot motion is started" in combined
    assert "does not launch controllers" in combined
    assert "move real hardware" in combined


def test_roundtrip_change_stays_focused():
    assert len(CONTROLLER.read_text(encoding="utf-8").splitlines()) < 520
    assert len(WORKFLOW.read_text(encoding="utf-8").splitlines()) < 340
    assert len(APPLICATOR.read_text(encoding="utf-8").splitlines()) < 290
