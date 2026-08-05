from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "workcell_builder/workcell_builder/gui/embedded_web_edit_save_controller.hpp"


def _section(source: str, start: str, end: str) -> str:
    return source.split(start, 1)[1].split(end, 1)[0]


def test_strict_prewrite_context_check_keeps_exact_viewer_identity():
    source = CONTROLLER.read_text(encoding="utf-8")
    strict = _section(source, "bool saveContextIsCurrent() const", "bool saveTargetContextIsActive() const")

    assert "view_->url() != expected_url_" in strict
    assert "sceneIdFromViewerUrl(view_->url()) != scene_id_" in strict
    assert "return saveTargetContextIsActive();" in strict


def test_postwrite_target_check_tolerates_same_scene_navigation_churn():
    source = CONTROLLER.read_text(encoding="utf-8")
    active = _section(source, "bool saveTargetContextIsActive() const", "void reportSceneChanged()")

    assert "sceneIdFromViewerUrl(view_->url()) != scene_id_" in active
    assert "view_->url() != expected_url_" not in active
    assert "current.scene_id.trimmed() == scene_id_" in active
    assert "canonicalPath(current.absolute_repo_root.trimmed()) == repo_root_" in active
    assert "canonicalPath(current.absolute_scene_dir.trimmed()) == scene_dir_" in active


def test_successful_write_is_classified_before_context_churn_and_starts_one_refresh():
    source = CONTROLLER.read_text(encoding="utf-8")
    callback = _section(
        source,
        "connect(process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished)",
        "process_->start();",
    )

    ok_index = callback.index("const bool ok = exit_status == QProcess::NormalExit && exit_code == 0;")
    dry_run_context_index = callback.index(
        "if (phase == WorkflowPhase::DryRun && !saveContextIsCurrent())"
    )
    failure_index = callback.index("if (!ok)")
    write_target_index = callback.index("if (!saveTargetContextIsActive())")
    refresh_index = callback.index("request_post_save_product_view_refresh()")

    assert ok_index < dry_run_context_index < failure_index < write_target_index < refresh_index
    assert callback.count("request_post_save_product_view_refresh()") == 1
    assert "reportSavedButSceneChanged();" in callback


def test_successful_write_is_never_relabelled_as_validation_failure():
    source = CONTROLLER.read_text(encoding="utf-8")
    report = _section(source, "void reportSavedButSceneChanged()", "bool resolveSaveContext")

    assert "persisted YAML remains authoritative" in report
    assert "The authored YAML was saved and verified" in report
    assert "validation failed" not in report
    assert "Web3D edits preserved" not in report
