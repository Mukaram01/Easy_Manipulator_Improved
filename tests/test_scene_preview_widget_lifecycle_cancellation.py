from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
HDR = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.h").read_text(encoding="utf-8")


def _between(start: str, end: str) -> str:
    begin = CPP.index(start)
    return CPP[begin:CPP.index(end, begin)]


def test_lifecycle_cancellation_is_declared_and_used_for_destructor_scene_changes_and_force_refresh():
    assert "~ScenePreviewWidget() override;" in HDR
    assert "void cancel_embedded_web_lifecycle(bool stop_owned_server);" in HDR
    assert "ScenePreviewWidget::~ScenePreviewWidget()\n{\n  cancel_embedded_web_lifecycle(true);" in CPP

    context = _between("void ScenePreviewWidget::set_preview_context", "void ScenePreviewWidget::activate_native_compatibility_preview")
    scene_name = _between("void ScenePreviewWidget::set_preview_scene_name", "bool ScenePreviewWidget::diagnostic_debug_logging_enabled")
    refresh = _between("void ScenePreviewWidget::request_embedded_web_product_view_refresh", "ScenePreviewWidget::EmbeddedWebRequestIdentity")
    assert "if (context_changed) cancel_embedded_web_lifecycle(false);" in context
    assert "cancel_embedded_web_lifecycle(false);" in scene_name
    assert "if (force) cancel_embedded_web_lifecycle(false);" in refresh


def test_cancellation_retires_callbacks_and_owns_process_shutdown():
    helper = _between("void ScenePreviewWidget::cancel_embedded_web_lifecycle", "void ScenePreviewWidget::request_embedded_web_product_view_refresh")
    for token in [
        "++embedded_web_request_generation_",
        "++embedded_web_navigation_token_",
        "embedded_editor_polling_ = false",
        "embedded_web_readiness_deadline_ = QDateTime()",
        "process->terminate()",
        "process->kill()",
        "stop_owned_server && embedded_web_server_is_owned_",
    ]:
        assert token in helper


def test_async_web_callbacks_guard_their_captured_request_identity():
    prepare = _between("void ScenePreviewWidget::on_embedded_web_prepare_finished", "void ScenePreviewWidget::start_embedded_web_readiness_polling")
    editor = _between("void ScenePreviewWidget::run_embedded_editor_command", "QString ScenePreviewWidget::embedded_snap_command")
    editor_poll = _between("void ScenePreviewWidget::poll_embedded_editor_events", "#else\nvoid ScenePreviewWidget::run_embedded_editor_command")
    assert "!embedded_web_identity_is_current(identity)" in prepare
    assert "[this, identity]" in editor
    assert "if (!embedded_web_identity_is_current(identity)) return;" in editor
    assert "[this, identity]" in editor_poll
    assert "if (embedded_web_identity_is_current(identity)) poll_embedded_editor_events();" in editor_poll
