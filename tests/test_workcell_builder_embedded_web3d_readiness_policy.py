from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP_PATH = ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp"
HDR_PATH = ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.h"
MAINWINDOW_PATH = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
CPP = CPP_PATH.read_text(encoding="utf-8")
HDR = HDR_PATH.read_text(encoding="utf-8")
MAINWINDOW = MAINWINDOW_PATH.read_text(encoding="utf-8")


def _between(text: str, start: str, end: str) -> str:
    start_i = text.index(start)
    return text[start_i:text.index(end, start_i)]


def test_load_finished_true_does_not_mark_embedded_product_view_ready():
    handler = _between(CPP, "&QWebEngineView::loadFinished", "simple_3d_view_ = embedded_web_view_")
    assert "set_embedded_product_view_state(EmbeddedProductViewState::Loading" in handler
    assert "start_embedded_web_readiness_polling" in handler
    assert "set_embedded_product_view_state(EmbeddedProductViewState::Ready" not in handler


def test_javascript_failed_status_sets_failed_with_stage_and_error_detail():
    poll = _between(CPP, "void ScenePreviewWidget::poll_embedded_web_readiness", "void ScenePreviewWidget::load_prepared_embedded_web_scene")
    assert "boot_state == QStringLiteral(\"failed\")" in poll
    assert "set_embedded_product_view_state(EmbeddedProductViewState::Failed, detail)" in poll
    assert "failed_stage" in poll
    assert "fatal_error" in poll
    assert "fatal_stack" in poll


def test_scene_fetch_failure_is_reported_to_qt_as_failed_javascript_status():
    viewer = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
    bundle = (ROOT / "workcell_studio_web/viewer/dist/viewer.bundle.js").read_text(encoding="utf-8")
    assert "Failed to load scene from" in viewer
    assert "showError" in viewer
    assert "viewer_boot_state: 'failed'" in viewer
    assert "Failed to load scene from" in bundle
    assert "Failed to load scene from" in bundle


def test_readiness_timeout_sets_failed_and_logs_last_boot_status():
    poll = _between(CPP, "void ScenePreviewWidget::poll_embedded_web_readiness", "static const char kStatusScript[]")
    assert "startup timed out after 45s" in poll
    assert "embedded_web_last_boot_status_" in poll
    assert "set_embedded_product_view_state(EmbeddedProductViewState::Failed, detail)" in poll


def test_ready_requires_scene_json_and_renderer_readiness():
    poll = _between(CPP, "const bool expected_json_loaded", "set_embedded_product_view_state(EmbeddedProductViewState::Loading")
    assert "scene_json_loaded && source_json == expected_json_path" in poll
    assert "robot_ready_required" in poll
    assert "robot_state == QStringLiteral(\"ready\")" in poll
    assert "boot_state == QStringLiteral(\"ready\") && expected_json_loaded && robot_ready" in poll


def test_preparation_accepts_only_rebuilt_or_current_and_rejects_malformed_wrong_scene_missing_and_failed_command():
    prepare = _between(CPP, "void ScenePreviewWidget::on_embedded_web_prepare_finished", "void ScenePreviewWidget::start_embedded_web_readiness_polling")
    assert "freshener_status != QStringLiteral(\"current\") && freshener_status != QStringLiteral(\"rebuilt\")" in prepare
    assert "freshener did not return a valid JSON object" in prepare
    assert "expected output missing or unreadable" in prepare
    assert "prepared output JSON validation failed" in prepare
    assert "prepared output semantic validation failed" in prepare
    assert "prepare command failed with exit code" in prepare
    assert "old output is rejected even if present" in prepare


def test_repo_root_resolution_covers_home_workspace_repo_symlink_env_override_and_rejects_partial_roots():
    resolver = _between(CPP, "QString ScenePreviewWidget::resolve_embedded_web_repo_root", "QString ScenePreviewWidget::embedded_web_prepare_command_for_log")
    assert "canonicalFilePath" in resolver  # symlinked scene path is canonicalized before walking upward.
    assert "selected scene directory upward walk" in resolver  # repository root from a selected scene path.
    assert "WORKCELL_STUDIO_REPO_ROOT secondary override" in resolver  # valid explicit environment override.
    assert "QDir::currentPath()" in resolver  # home/workspace/repo launch cwd fallback.
    assert "QCoreApplication::applicationDirPath()" in resolver
    for marker in ["workcell_studio_web/viewer/index.html", "scripts/ensure_workcell_studio_web_scene_fresh.py", "scenes"]:
        assert marker in resolver
    assert "candidate rejected" in resolver
    assert "missing %3" in resolver


def test_native_backend_is_suppressed_unless_explicitly_requested_or_webengine_unavailable():
    assert "WORKCELL_BUILDER_PRODUCT_VIEW_BACKEND" in CPP
    assert "native_scene3d_explicitly_requested" in CPP
    assert "product_view_backend_ = ProductViewBackend::EmbeddedWeb3D" in CPP
    assert "product_view_backend_ = ProductViewBackend::NativeScene3D" in CPP
    assert "WORKCELL_BUILDER_ALLOW_NATIVE_3D_FALLBACK" in (ROOT / "workcell_builder/workcell_builder/CMakeLists.txt").read_text(encoding="utf-8")
    assert "scene_preview_widget_->is_native_product_view_backend()" in MAINWINDOW
    assert "auto * viewport = active_native_viewport();" in CPP
    assert "ProductViewBackend { EmbeddedWeb3D, NativeScene3D }" in HDR


def test_embedded_refresh_identity_coalesces_duplicate_context_and_payload_requests():
    assert "struct EmbeddedWebRequestIdentity" in HDR
    for field in ["scene_id", "absolute_scene_dir", "payload_revision", "generation"]:
        assert field in HDR
    assert "matches_context" in HDR
    refresh = _between(CPP, "void ScenePreviewWidget::request_embedded_web_product_view_refresh", "ScenePreviewWidget::EmbeddedWebRequestIdentity")
    assert "embedded_web_active_identity_.matches_context(identity)" in refresh
    assert "pending_embedded_web_identity_.matches_context(identity)" in refresh
    assert "pending_embedded_web_request_ = true" in refresh
    labels_handler = _between(CPP, "connect(labels_selector_", "connect(snap_mode_selector_")
    assert "refresh_embedded_web_product_view" not in labels_handler


def test_manual_refresh_creates_a_new_generation_and_invalidates_old_callbacks():
    refresh = _between(CPP, "void ScenePreviewWidget::request_embedded_web_product_view_refresh", "ScenePreviewWidget::EmbeddedWebRequestIdentity")
    assert "++embedded_web_request_generation_" in refresh
    assert "embedded_web_active_identity_ = identity" in refresh
    assert "if (force) cancel_embedded_web_lifecycle(false);" in refresh
    prepare = _between(CPP, "void ScenePreviewWidget::start_embedded_web_prepare", "void ScenePreviewWidget::on_embedded_web_prepare_finished")
    assert "[this, identity]" in prepare
    finished = _between(CPP, "void ScenePreviewWidget::on_embedded_web_prepare_finished", "void ScenePreviewWidget::start_embedded_web_readiness_polling")
    assert "embedded_web_identity_is_current(identity)" in finished
    polling = _between(CPP, "void ScenePreviewWidget::poll_embedded_web_readiness", "void ScenePreviewWidget::load_prepared_embedded_web_scene")
    assert "[this, identity, navigation_token, expected_json_path, viewer_url]" in polling
