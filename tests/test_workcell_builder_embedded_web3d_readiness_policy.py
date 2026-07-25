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
    assert "set_embedded_product_view_state(EmbeddedProductViewState::WaitingForBrowserReadiness" in handler
    assert "start_embedded_web_readiness_polling" in handler
    assert "set_embedded_product_view_state(EmbeddedProductViewState::Ready" not in handler


def test_javascript_failed_status_sets_failed_with_stage_and_error_detail():
    poll = _between(CPP, "void ScenePreviewWidget::poll_embedded_web_readiness", "void ScenePreviewWidget::load_prepared_embedded_web_scene")
    assert "boot_state == QStringLiteral(\"failed\")" in poll
    assert "handle_embedded_web_runtime_failure(identity, navigation_token, detail)" in poll
    assert "failed_stage" in poll
    assert "fatal_error" in poll
    assert "fatal_stack" in poll


def test_scene_fetch_failure_is_reported_to_qt_as_failed_javascript_status():
    viewer = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
    bundle = (ROOT / "workcell_studio_web/viewer/dist/viewer.bundle.js").read_text(encoding="utf-8")
    assert "Failed to load scene from" in viewer
    assert "showError" in viewer
    assert "lifecycle_state: 'scene_failed'" in viewer
    assert "Failed to load scene from" in bundle
    assert "Failed to load scene from" in bundle


def test_readiness_timeout_sets_failed_and_logs_last_boot_status():
    poll = _between(CPP, "void ScenePreviewWidget::poll_embedded_web_readiness", "static const char kStatusScript[]")
    assert "startup timed out after 45s" in poll
    assert "embedded_web_last_boot_status_" in poll
    assert "handle_embedded_web_runtime_failure(identity, navigation_token, detail)" in poll


def test_ready_requires_scene_json_and_renderer_readiness():
    poll = _between(CPP, "const QString expected_builder_revision", "set_embedded_product_view_state(EmbeddedProductViewState::WaitingForBrowserReadiness")
    assert "contract_version != 1" in poll
    assert "lifecycle_state != QStringLiteral(\"scene_ready\")" in poll
    assert "!terminal" in poll
    assert "reported_scene_id != identity.scene_id" in poll
    assert "source_json != expected_json_path" in poll
    assert "builder_revision != expected_builder_revision" in poll
    assert "failed_required_count != 0" in poll


def test_accepted_scene_ready_transition_refreshes_visible_view_after_ready_state():
    poll = _between(CPP, "if (contract_reason.isEmpty())", "poll_embedded_editor_events();")
    ready_transition = (
        "set_embedded_product_view_state(EmbeddedProductViewState::Ready, "
        "QStringLiteral(\"viewer ready\"));"
    )
    commit = "embedded_web_has_committed_surface_ = true;"
    assert poll.index(commit) < poll.index(ready_transition)
    assert poll.index(ready_transition) < poll.index("show_embedded_web_product_view();")

    show = _between(
        CPP,
        "void ScenePreviewWidget::show_embedded_web_product_view()",
        "void ScenePreviewWidget::set_preview_context",
    )
    assert "embedded_web_view_->setVisible(scene_selected_)" in show
    assert "refresh_mode_and_state();" in show


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
    for field in ["scene_id", "absolute_scene_dir", "payload_fingerprint", "payload_revision", "generation"]:
        assert field in HDR
    assert "matches_context" in HDR
    refresh = _between(CPP, "void ScenePreviewWidget::request_embedded_web_product_view_refresh", "ScenePreviewWidget::EmbeddedWebRequestIdentity")
    assert "embedded_web_active_identity_.matches_effective_request(request_key)" in refresh
    assert "pending_embedded_web_identity_.matches_effective_request(request_key)" in refresh
    assert "pending_embedded_web_request_ = true" in refresh
    labels_handler = _between(CPP, "connect(labels_selector_", "connect(snap_mode_selector_")
    assert "refresh_embedded_web_product_view" not in labels_handler


def test_manual_refresh_creates_a_new_generation_and_invalidates_old_callbacks():
    refresh = _between(CPP, "void ScenePreviewWidget::request_embedded_web_product_view_refresh", "ScenePreviewWidget::EmbeddedWebRequestIdentity")
    assert "++embedded_web_request_generation_" in refresh
    assert "embedded_web_active_identity_ = identity" in refresh
    assert "if (force) {" in refresh
    assert "cancel_embedded_web_lifecycle(false);" in refresh
    assert "embedded_web_request_generation_ = identity.generation;" in refresh
    prepare = _between(CPP, "void ScenePreviewWidget::start_embedded_web_prepare", "void ScenePreviewWidget::on_embedded_web_prepare_finished")
    assert "[this, identity" in prepare
    finished = _between(CPP, "void ScenePreviewWidget::on_embedded_web_prepare_finished", "void ScenePreviewWidget::start_embedded_web_readiness_polling")
    assert "embedded_web_identity_is_current(identity)" in finished
    polling = _between(CPP, "void ScenePreviewWidget::poll_embedded_web_readiness", "void ScenePreviewWidget::load_prepared_embedded_web_scene")
    assert "[this, identity, navigation_token, expected_json_path, viewer_url]" in polling


def test_qt_readiness_contract_rejects_mismatches_and_infrastructure_states():
    poll = _between(CPP, "const QString expected_builder_revision", "set_embedded_product_view_state(EmbeddedProductViewState::WaitingForBrowserReadiness")
    for token in [
        "contract_version != 1",
        "server_ready is infrastructure state",
        "lifecycle_state != QStringLiteral(\"scene_ready\")",
        "!terminal",
        "reported_scene_id != identity.scene_id",
        "source_json != expected_json_path",
        "builder_revision != expected_builder_revision",
        "!scene_json_loaded",
        "failed_required_count != 0",
        "component status incomplete",
    ]:
        assert token in poll


def test_runtime_failure_accounting_is_request_scoped_and_deduplicated():
    failure = _between(CPP, "void ScenePreviewWidget::handle_embedded_web_runtime_failure", "void ScenePreviewWidget::ensure_embedded_web_server_started")
    assert "embedded_web_terminal_runtime_failures_" in HDR
    assert "embedded_web_terminal_runtime_failures_.contains(terminal_key)" in failure
    assert "Suppressed duplicate Embedded Product View terminal failure" in failure
    assert "Ignored stale Embedded Product View runtime failure" in failure
    assert "set_embedded_product_view_state(EmbeddedProductViewState::Failed, detail)" in failure


def test_stale_readiness_and_terminal_failure_cannot_commit_or_label_replacement_ready():
    poll = _between(CPP, "void ScenePreviewWidget::poll_embedded_web_readiness", "void ScenePreviewWidget::load_prepared_embedded_web_scene")
    stale_guard = poll.index("if (!embedded_web_identity_is_current(identity)")
    commit = poll.index("embedded_web_has_committed_surface_ = true;")
    assert stale_guard < commit
    assert "navigation_token != embedded_web_navigation_token_" in poll[:commit]

    failure = _between(CPP, "void ScenePreviewWidget::handle_embedded_web_runtime_failure", "void ScenePreviewWidget::ensure_embedded_web_server_started")
    assert "set_embedded_product_view_state(EmbeddedProductViewState::Failed, detail)" in failure
    assert "embedded_web_has_committed_surface_ = true" not in failure
    assert "EmbeddedProductViewState::Ready" not in failure
