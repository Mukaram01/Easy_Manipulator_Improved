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
    assert "void stop_embedded_web_navigation_for_handoff();" in HDR
    assert "ScenePreviewWidget::~ScenePreviewWidget()\n{\n  cancel_embedded_web_lifecycle(true);" in CPP

    context = _between("void ScenePreviewWidget::set_preview_context", "void ScenePreviewWidget::activate_native_compatibility_preview")
    scene_name = _between("void ScenePreviewWidget::set_preview_scene_name", "bool ScenePreviewWidget::diagnostic_debug_logging_enabled")
    refresh = _between("void ScenePreviewWidget::request_embedded_web_product_view_refresh", "ScenePreviewWidget::EmbeddedWebRequestIdentity")
    assert "if (context_changed) cancel_embedded_web_lifecycle(false);" in context
    assert "cancel_embedded_web_lifecycle(false);" in scene_name
    assert "if (force) cancel_embedded_web_lifecycle(false);" in refresh


def test_cancellation_retires_callbacks_and_owns_process_shutdown():
    helper = _between("void ScenePreviewWidget::cancel_embedded_web_lifecycle", "void ScenePreviewWidget::request_embedded_web_product_view_refresh")
    handoff = _between("void ScenePreviewWidget::stop_embedded_web_navigation_for_handoff", "void ScenePreviewWidget::cancel_embedded_web_lifecycle")
    for token in [
        "++embedded_web_request_generation_",
        "process->terminate()",
        "process->kill()",
        "stop_owned_server && embedded_web_server_is_owned_",
    ]:
        assert token in helper
    for token in [
        "++embedded_web_navigation_token_",
        "embedded_editor_polling_ = false",
        "embedded_web_readiness_deadline_ = QDateTime()",
        "embedded_web_view_->stop()",
        "embedded_web_view_->setVisible(false)",
    ]:
        assert token in handoff
    assert "stop_embedded_web_navigation_for_handoff();" in helper


def test_async_web_callbacks_guard_their_captured_request_identity():
    prepare = _between("void ScenePreviewWidget::on_embedded_web_prepare_finished", "void ScenePreviewWidget::start_embedded_web_readiness_polling")
    editor = _between("void ScenePreviewWidget::run_embedded_editor_command", "QString ScenePreviewWidget::embedded_snap_command")
    editor_poll = _between("void ScenePreviewWidget::poll_embedded_editor_events", "#else\nvoid ScenePreviewWidget::run_embedded_editor_command")
    assert "!embedded_web_identity_is_current(identity)" in prepare
    assert "[this, identity]" in editor
    assert "if (!embedded_web_identity_is_current(identity)) return;" in editor
    assert "[this, identity]" in editor_poll
    assert "if (embedded_web_identity_is_current(identity)) poll_embedded_editor_events();" in editor_poll


def test_web_request_identity_binds_root_and_selected_port_for_every_async_gate():
    identity = HDR[HDR.index("struct EmbeddedWebRequestIdentity"):HDR.index("struct EmbeddedWebPreparationDiagnostic")]
    assert "QString absolute_repo_root;" in identity
    assert "int selected_server_port" in identity
    assert "absolute_repo_root == other.absolute_repo_root" in identity
    assert "selected_server_port == other.selected_server_port" in identity

    probes = _between("void ScenePreviewWidget::start_embedded_web_server_probes", "void ScenePreviewWidget::fail_embedded_web_server_probe")
    assert "identity.absolute_repo_root != repo_root" in probes
    assert "identity.selected_server_port != port" in probes
    assert "load_prepared_embedded_web_scene(identity);" in probes

    server = _between("void ScenePreviewWidget::start_owned_embedded_web_server", "void ScenePreviewWidget::cancel_embedded_web_lifecycle")
    assert '"--directory", repo_root' in server
    assert "identity.selected_server_port != port" in server
    assert "&QProcess::started" in server

    browser = _between("void ScenePreviewWidget::load_prepared_embedded_web_scene", "#ifdef WORKCELL_BUILDER_HAS_WEBENGINE")
    assert "identity.selected_server_port <= 0" in browser
    assert "viewer_url.setPort(identity.selected_server_port);" in browser


def test_serialized_browser_navigation_handoff_queues_only_current_scene_load():
    request = _between("void ScenePreviewWidget::request_embedded_web_product_view_refresh", "ScenePreviewWidget::EmbeddedWebRequestIdentity")
    browser = _between("void ScenePreviewWidget::load_prepared_embedded_web_scene", "#ifdef WORKCELL_BUILDER_HAS_WEBENGINE")
    mode = _between("void ScenePreviewWidget::refresh_mode_and_state", "QRectF ScenePreviewWidget::rendered_items_bounds_2d")

    assert "!embedded_web_active_identity_.matches_effective_request(identity)" in request
    assert "stop_embedded_web_navigation_for_handoff();" in request
    assert "embedded_web_view_->stop();" in browser
    assert "embedded_web_view_->setVisible(false);" in browser
    assert "QTimer::singleShot(0, this, [this, identity, queued_navigation_token, viewer_url]()" in browser
    stale_guard = browser.index("if (!embedded_web_identity_is_current(identity)")
    assert browser.index("embedded_web_view_->load(viewer_url);", stale_guard) > stale_guard
    assert "queued_navigation_token != embedded_web_navigation_token_" in browser
    assert "embedded_web_loading_navigation_token_ != queued_navigation_token" in browser
    assert "embedded_web_loading_identity_ != identity" in browser
    assert "embedded_web_expected_viewer_url_ != viewer_url" in browser
    assert "++embedded_web_browser_navigations_started_;" in browser
    assert "embedded_product_view_state_ == EmbeddedProductViewState::Ready" in mode
