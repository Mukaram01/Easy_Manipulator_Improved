from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")


def _between(start: str, end: str) -> str:
    begin = CPP.index(start)
    return CPP[begin:CPP.index(end, begin)]


def test_product_view_prepare_uses_qprocess_with_split_arguments_and_environment():
    prepare = _between(
        "void ScenePreviewWidget::start_embedded_web_prepare",
        "void ScenePreviewWidget::on_embedded_web_prepare_finished",
    )
    assert 'setProgram(QStringLiteral("python3"))' in prepare
    assert "QStringList args" in prepare
    assert '"scripts/ensure_workcell_studio_web_scene_fresh.py"' in prepare
    assert "setArguments(args)" in prepare
    assert "setWorkingDirectory(repo_root)" in prepare
    assert "setProcessEnvironment(QProcessEnvironment::systemEnvironment())" in prepare
    assert "std::system" not in prepare
    assert '"/bin/sh"' not in prepare
    assert '"/bin/bash"' not in prepare


def test_process_failures_report_executable_arguments_cwd_status_stderr_and_timeout():
    helper = _between(
        "QString summarize_prepare_process_failure",
        "void maybe_warn_overlay_fit_dominance",
    )
    for token in [
        "executable=%1",
        "arguments=%2",
        "working_directory=%3",
        "exit_status=%4",
        "exit_code=%5",
        "start_error=%1",
        "stderr=%1",
    ]:
        assert token in helper

    prepare = _between(
        "void ScenePreviewWidget::start_embedded_web_prepare",
        "void ScenePreviewWidget::on_embedded_web_prepare_finished",
    )
    assert "&QProcess::errorOccurred" in prepare
    assert "QTimer::singleShot(120000" in prepare
    assert 'QStringLiteral("timeout after 120000 ms")' in prepare
    assert 'QStringLiteral("timeout")' in prepare
    assert "process->kill()" in prepare


def test_nonzero_exit_and_contract_failures_use_actionable_process_detail():
    finished = _between(
        "void ScenePreviewWidget::on_embedded_web_prepare_finished",
        "void ScenePreviewWidget::start_embedded_web_readiness_polling",
    )
    assert "auto reject_prepare" in finished
    assert "summarize_prepare_process_failure(" in finished
    assert "diagnostic.stderr_tail" in finished
    assert '"prepare command failed with exit code %1; old output is rejected even if present"' in finished
    assert "show_embedded_web_preparation_failure(identity, detail)" in finished
