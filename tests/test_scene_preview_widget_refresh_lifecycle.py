from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
MAINWINDOW_CPP = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


@dataclass(frozen=True)
class RequestKey:
    scene: str
    directory: str
    backend: str
    generated_json_path: str
    fingerprint: bytes
    revision: int


class RefreshLifecycle:
    """Small behavioral model for the Product View request lifecycle contract."""

    def __init__(self):
        self.generation = 0
        self.active = None
        self.pending = None
        self.started = []
        self.retired = []
        self.navigations = []
        self.backend_switches = []
        self.camera_state = {"target": [1, 2, 3], "distance": 4}
        self.camera_resets = 0
        self.suppression_logs = []
        self._last_suppression_key = None

    def request(self, key, force=False):
        if not force and ((self.active and key == self.active[0]) or (self.pending and key == self.pending[0])):
            suppression_key = key
            if self._last_suppression_key != suppression_key:
                self.suppression_logs.append("Duplicate Product View navigation suppressed")
                self._last_suppression_key = suppression_key
            return
        if force:
            self.active = None
            self.pending = None
        self._last_suppression_key = None
        self.generation += 1
        request = (key, self.generation)
        self.active = request
        self.pending = request

    def start_pending(self):
        if self.pending is not None:
            self.started.append(self.pending)
            self.navigations.append(self.pending)
            if self.pending[0].backend != "embedded_web3d":
                self.backend_switches.append(self.pending[0].backend)
            self.camera_state = {"target": [0, 0, 0], "distance": 10}
            self.camera_resets += 1
            self.pending = None

    def finish(self, request):
        if request != self.active:
            self.retired.append(request)
            self.start_pending()
            return
        self.start_pending()


def test_repeated_automatic_notifications_keep_one_generation_and_active_request():
    lifecycle = RefreshLifecycle()
    key = RequestKey(
        "ur5_2f_test",
        "/cells/ur5_2f_test",
        "embedded_web3d",
        "build/workcell_studio_web_scene/ur5_2f_test.web_scene.json",
        b"same",
        7,
    )
    lifecycle.request(key)
    active = lifecycle.active
    lifecycle.request(key)
    lifecycle.request(key)

    assert lifecycle.generation == 1
    assert lifecycle.active == active
    assert lifecycle.pending == active


def test_scene_or_payload_change_retires_old_completion_and_starts_replacement():
    lifecycle = RefreshLifecycle()
    old = RequestKey(
        "ur5_2f_test",
        "/cells/ur5_2f_test",
        "embedded_web3d",
        "build/workcell_studio_web_scene/ur5_2f_test.web_scene.json",
        b"old",
        7,
    )
    changed_scene = RequestKey(
        "ur5_3f_test",
        "/cells/ur5_3f_test",
        "embedded_web3d",
        "build/workcell_studio_web_scene/ur5_3f_test.web_scene.json",
        b"new",
        8,
    )
    lifecycle.request(old)
    lifecycle.start_pending()
    lifecycle.request(changed_scene)
    lifecycle.finish((old, 1))

    assert lifecycle.retired == [(old, 1)]
    assert lifecycle.started == [(old, 1), (changed_scene, 2)]
    assert lifecycle.active == (changed_scene, 2)


def test_one_manual_refresh_invalidates_once_and_creates_one_retry():
    lifecycle = RefreshLifecycle()
    key = RequestKey(
        "ur5_2f_test",
        "/cells/ur5_2f_test",
        "embedded_web3d",
        "build/workcell_studio_web_scene/ur5_2f_test.web_scene.json",
        b"same",
        7,
    )
    lifecycle.request(key)
    lifecycle.start_pending()
    lifecycle.request(key, force=True)

    assert lifecycle.generation == 2
    assert lifecycle.active == (key, 2)
    assert lifecycle.pending == (key, 2)


def test_three_identical_refreshes_prepare_navigate_load_once_and_emit_one_suppression_group():
    lifecycle = RefreshLifecycle()
    key = RequestKey(
        "ur5_2f_test",
        "/cells/ur5_2f_test",
        "embedded_web3d",
        "build/workcell_studio_web_scene/ur5_2f_test.web_scene.json",
        b"same-fingerprint",
        7,
    )
    lifecycle.request(key)
    lifecycle.request(key)
    lifecycle.request(key)
    lifecycle.start_pending()

    assert lifecycle.generation == 1
    assert lifecycle.started == [(key, 1)]
    assert lifecycle.navigations == [(key, 1)]
    assert lifecycle.backend_switches == []
    assert lifecycle.camera_resets == 1
    assert lifecycle.suppression_logs == ["Duplicate Product View navigation suppressed"]


def test_cpp_coalesces_before_generation_and_retires_stale_process_before_ui_work():
    refresh_start = CPP.index("void ScenePreviewWidget::request_embedded_web_product_view_refresh")
    identity_start = CPP.index("ScenePreviewWidget::EmbeddedWebRequestIdentity", refresh_start)
    refresh = CPP[refresh_start:identity_start]
    assert refresh.index("matches_effective_request(request_key)") < refresh.index("++embedded_web_request_generation_")
    assert "Duplicate Product View navigation suppressed" in refresh
    assert "embedded_web_last_suppressed_duplicate_key_ != suppression_key" in refresh
    assert "const PreviewContext normalized_context = normalized_preview_context(preview_context_);" in CPP
    assert "identity.product_view_backend =" in CPP
    assert "identity.generated_web_scene_path =" in CPP
    assert "identity.payload_fingerprint = preview_payload_fingerprint_;" in CPP

    prepare_start = CPP.index("void ScenePreviewWidget::on_embedded_web_prepare_finished")
    readiness_start = CPP.index("void ScenePreviewWidget::start_embedded_web_readiness_polling", prepare_start)
    prepare = CPP[prepare_start:readiness_start]
    stale = prepare.index("if (!embedded_web_identity_is_current(identity))")
    assert prepare.index("embedded_web_prepare_process_ = nullptr;", stale) > stale
    assert prepare.index("maybe_start_next_embedded_web_prepare();", stale) > stale
    assert stale < prepare.index("const QString scene = identity.scene_id;")


def test_selected_scene_status_refreshes_leave_the_committed_product_payload_identity_unchanged():
    """Status/audit work must not create another Web3D payload preparation."""

    class SelectedSceneStatusRefresh:
        def __init__(self):
            self.preparations = 0
            self.payload_revision = 0
            self.payload_generation = 0
            self.committed_fingerprint = None

        def apply_filtered_payload(self, fingerprint):
            if fingerprint == self.committed_fingerprint:
                return
            self.committed_fingerprint = fingerprint
            self.payload_revision += 1
            self.payload_generation += 1
            self.preparations += 1

        def refresh_status_and_audit(self):
            # The extracted status/audit helper intentionally has no payload commit.
            return "camera-fit-and-audit-updated"

    refresh = SelectedSceneStatusRefresh()
    refresh.apply_filtered_payload(b"stable-filtered-payload")
    before = (refresh.preparations, refresh.payload_revision, refresh.payload_generation)

    assert refresh.refresh_status_and_audit() == "camera-fit-and-audit-updated"
    assert refresh.refresh_status_and_audit() == "camera-fit-and-audit-updated"
    assert (refresh.preparations, refresh.payload_revision, refresh.payload_generation) == before

    filter_start = MAINWINDOW_CPP.index("void MainWindow::apply_scene3d_preview_layer_filters")
    audit_start = MAINWINDOW_CPP.index("void MainWindow::refresh_scene3d_product_view_status_and_audit", filter_start)
    filter_block = MAINWINDOW_CPP[filter_start:audit_start]
    audit_end = MAINWINDOW_CPP.index("void MainWindow::populate_scene_hierarchy", audit_start)
    audit_block = MAINWINDOW_CPP[audit_start:audit_end]

    assert "preview_payload_matches(filtered_items)" in filter_block
    assert "if (filtered_payload_changed)" in filter_block
    assert "scene_preview_widget_->set_preview_items(filtered_items);" in filter_block
    assert "viewport->ingest_preview_items(filtered_items);" not in filter_block
    assert "set_preview_items" not in audit_block
    assert "fit_product_view();" in audit_block
    assert "set_preview_status_summary" in audit_block


def test_transient_web3d_runtime_failures_keep_web3d_selected_and_never_activate_qt3d():
    failure_handler_start = CPP.index("void ScenePreviewWidget::handle_embedded_web_runtime_failure")
    server_start = CPP.index("void ScenePreviewWidget::ensure_embedded_web_server_started", failure_handler_start)
    failure_handler = CPP[failure_handler_start:server_start]

    assert "native_compatibility_fallback_active_ = false;" in failure_handler
    assert 'mode_selector_->setItemText(0, QStringLiteral("Web3D Product View"))' in failure_handler
    assert "embedded_web_view_->setVisible(true);" in failure_handler
    assert "compatibility_scene3d_viewport_->setVisible(false);" in failure_handler
    assert 'embedded_fit_button_->setText(QStringLiteral("Retry"))' in failure_handler
    assert "activate_native_compatibility_preview" not in failure_handler

    for marker in (
        "browser failed to load Product View page",
        "browser render process terminated",
        "viewer JavaScript failed",
    ):
        marker_index = CPP.index(marker)
        block = CPP[max(0, marker_index - 700):marker_index + 1800]
        assert "handle_embedded_web_runtime_failure" in block
        assert "activate_native_compatibility_preview" not in block

    server_probe_start = CPP.index("void ScenePreviewWidget::fail_embedded_web_server_probe")
    server_probe_end = CPP.index("QString ScenePreviewWidget::embedded_web_recovery_key", server_probe_start)
    server_probe_block = CPP[server_probe_start:server_probe_end]
    assert "handle_embedded_web_runtime_failure(identity, navigation_token, detail);" in server_probe_block
    assert "activate_native_compatibility_preview" not in server_probe_block

    for marker in ("local server exited with code", "local server startup failure"):
        marker_index = CPP.index(marker)
        block = CPP[max(0, marker_index - 700):marker_index + 700]
        assert "fail_embedded_web_server_probe" in block


def test_web3d_runtime_failure_does_not_start_automatic_forced_recovery():
    key_start = CPP.index("QString ScenePreviewWidget::embedded_web_recovery_key")
    handler_start = CPP.index("void ScenePreviewWidget::handle_embedded_web_runtime_failure", key_start)
    key_block = CPP[key_start:handler_start]
    handler_end = CPP.index("void ScenePreviewWidget::ensure_embedded_web_server_started", handler_start)
    handler = CPP[handler_start:handler_end]

    assert "payload_fingerprint.toHex()" in key_block
    assert "identity.generation" not in key_block
    assert "navigation_token" not in key_block
    assert "Embedded Product View terminal failure accepted" in handler
    assert "embedded_web_terminal_runtime_failures_.insert(terminal_key)" in handler
    assert "request_embedded_web_product_view_refresh(true);" not in handler
    assert "automatic recovery attempt" not in handler
    assert "Retry" in handler


def test_explicit_legacy_backend_selection_still_uses_native_scene3d():
    constructor_backend_start = CPP.index("const QString requested_product_view_backend")
    constructor_backend_end = CPP.index("#else", constructor_backend_start)
    backend_block = CPP[constructor_backend_start:constructor_backend_end]

    assert "native_scene3d_explicitly_requested" in backend_block
    assert 'requested_product_view_backend == QStringLiteral("native_scene3d")' in backend_block
    assert "product_view_backend_ = ProductViewBackend::NativeScene3D;" in backend_block
    assert 'simple_3d_view_->setObjectName("scene3dViewportWidget");' in backend_block


def test_web3d_selection_keeps_webengine_surface_and_blocks_native_fallback():
    refresh_start = CPP.index("void ScenePreviewWidget::refresh_mode_and_state")
    refresh_end = CPP.index("QRectF ScenePreviewWidget::rendered_items_bounds_2d", refresh_start)
    refresh = CPP[refresh_start:refresh_end]

    assert 'mode == QStringLiteral("Web3D Product View")' in refresh
    assert 'Native fallback prevented because Web3D is selected' in refresh
    assert 'native_compatibility_fallback_active_ = false;' in refresh
    assert 'const bool show_native_compatibility = use3d && scene_selected_ && !web3d_selected' in refresh
    assert 'compatibility_scene3d_viewport_->setVisible(show_native_compatibility)' in refresh
    assert 'embedded_web_view_->setVisible(use3d && scene_selected_)' in refresh
    assert 'Product View failed — Retry' in refresh

    show_start = CPP.index("void ScenePreviewWidget::show_embedded_web_product_view")
    context_start = CPP.index("void ScenePreviewWidget::set_preview_context", show_start)
    show = CPP[show_start:context_start]
    assert 'native_compatibility_fallback_active_ = false;' in show
    assert 'compatibility_scene3d_viewport_->setVisible(false)' in show
    assert 'embedded_web_view_->setVisible(scene_selected_)' in show
    assert 'mode_selector_->setCurrentIndex(0)' in show

    activate_start = CPP.index("void ScenePreviewWidget::activate_native_compatibility_preview")
    status_start = CPP.index("QString ScenePreviewWidget::runtime_preview_status_text", activate_start)
    activate = CPP[activate_start:status_start]
    assert 'mode_selector_->currentText() == QStringLiteral("Web3D Product View")' in activate
    assert 'Native fallback prevented because Web3D is selected' in activate
    assert 'set_embedded_product_view_state(EmbeddedProductViewState::Failed, reason)' in activate
    assert 'embedded_fit_button_->setText(QStringLiteral("Retry"))' in activate
    assert 'compatibility_scene3d_viewport_->setVisible(false)' in activate
    assert 'embedded_web_view_->setVisible(scene_selected_)' in activate
    assert 'return;' in activate[:activate.index('native_compatibility_fallback_active_ = true;')]


def test_web3d_status_chip_uses_required_lifecycle_labels():
    status_start = CPP.index("QString ScenePreviewWidget::runtime_preview_status_text")
    status_end = CPP.index("void ScenePreviewWidget::refresh_toolbar_status_chip", status_start)
    status = CPP[status_start:status_end]

    for label in (
        'Web3D Product View — preparing',
        'Web3D Product View — loading',
        'Web3D Product View — ready',
        'Web3D Product View — failed, Retry',
    ):
        assert label in status

@dataclass(frozen=True)
class SceneContextKey:
    scene: str = ""
    scene_dir: str = ""
    repo_root: str = ""
    generated_json_path: str = ""
    fingerprint: bytes = b""
    revision: int = 0


class SelectionLifecycleGate:
    def __init__(self):
        self.generation = 0
        self.active = None
        self.started = []
        self.terminal = []
        self.deferred = 0
        self.coalesced = 0
        self.cancelled = []

    def ready(self, key):
        return all((key.scene, key.scene_dir, key.repo_root, key.generated_json_path, key.fingerprint)) and key.revision > 0

    def request(self, key, origin="automatic", force=False):
        if not self.ready(key):
            self.deferred += 1
            return None
        if not force and self.active and self.active[0] == key:
            self.coalesced += 1
            return self.active
        if self.active and self.active[0].scene != key.scene:
            self.cancelled.append(self.active)
        self.generation += 1
        self.active = (key, self.generation, origin)
        self.started.append(self.active)
        return self.active

    def callback(self, request, state):
        if request != self.active:
            return "stale_ignored"
        self.terminal.append((request, state))
        return state


def test_initial_incomplete_scene_context_starts_no_preparation_until_payload_identity_ready():
    lifecycle = SelectionLifecycleGate()
    incomplete = SceneContextKey(scene="ur5_2f_test", scene_dir="/cells/ur5_2f_test", repo_root="/repo")
    ready = SceneContextKey("ur5_2f_test", "/cells/ur5_2f_test", "/repo", "build/workcell_studio_web_scene/ur5_2f_test.web_scene.json", b"fp", 1)

    assert lifecycle.request(incomplete, origin="scene_context_ready") is None
    assert lifecycle.started == []
    assert lifecycle.deferred == 1
    assert lifecycle.request(ready, origin="payload_commit") == (ready, 1, "payload_commit")
    assert lifecycle.started == [(ready, 1, "payload_commit")]


def test_normal_selection_has_one_terminal_scene_ready_and_stale_callbacks_cannot_clear_it():
    lifecycle = SelectionLifecycleGate()
    old = SceneContextKey("ur5_2f_test", "/cells/ur5_2f_test", "/repo", "build/workcell_studio_web_scene/ur5_2f_test.web_scene.json", b"old", 1)
    new = SceneContextKey("ur5_3f_test", "/cells/ur5_3f_test", "/repo", "build/workcell_studio_web_scene/ur5_3f_test.web_scene.json", b"new", 1)
    old_request = lifecycle.request(old, origin="payload_commit")
    new_request = lifecycle.request(new, origin="scene_switch")

    assert lifecycle.cancelled == [old_request]
    assert lifecycle.callback(old_request, "failed") == "stale_ignored"
    assert lifecycle.callback(new_request, "scene_ready") == "scene_ready"
    assert lifecycle.terminal == [(new_request, "scene_ready")]


def test_cpp_defers_until_complete_context_and_payload_and_logs_origin():
    refresh_start = CPP.index("void ScenePreviewWidget::request_embedded_web_product_view_refresh")
    identity_start = CPP.index("ScenePreviewWidget::EmbeddedWebRequestIdentity", refresh_start)
    refresh = CPP[refresh_start:identity_start]

    assert "const bool context_ready" in refresh
    assert "request_key.payload_revision > 0" in refresh
    assert "Product View lifecycle deferred" in refresh
    assert "origin=%4" in refresh
    assert refresh.index("if (!context_ready)") < refresh.index("++embedded_web_effective_refresh_requests_received_")
    assert "Product View lifecycle requested" in refresh


def test_cpp_counts_effective_preparation_starts_not_incomplete_context_requests():
    refresh_wrapper = CPP[CPP.index("void ScenePreviewWidget::refresh_embedded_web_product_view"):CPP.index("void ScenePreviewWidget::maybe_start_next_embedded_web_prepare")]
    prepare = CPP[CPP.index("void ScenePreviewWidget::start_embedded_web_prepare"):CPP.index("void ScenePreviewWidget::on_embedded_web_prepare_finished")]

    assert "++embedded_web_preparation_request_count_" not in refresh_wrapper
    assert "++embedded_web_preparation_request_count_" in prepare


def test_scene_identity_handoff_failure_late_readiness_and_return_to_original_scene():
    """A failed replacement scene must never reveal or revive the old surface."""

    class ProductSurface:
        def __init__(self):
            self.generation = 0
            self.scene = ""
            self.committed = None
            self.presentation = None
            self.ready = False

        def select(self, scene):
            self.generation += 1
            self.scene = scene
            self.committed = None
            self.ready = False
            self.presentation = ("loading", scene)
            return scene, self.generation

        def preparation_failed(self, request, error):
            if request != (self.scene, self.generation):
                return "stale_ignored"
            self.presentation = ("failure", self.scene, error, "Retry")
            return "failed"

        def readiness(self, request):
            if request != (self.scene, self.generation):
                return "stale_ignored"
            self.committed = self.scene
            self.presentation = ("ready", self.scene)
            self.ready = True
            return "ready"

    surface = ProductSurface()
    old_ur5 = surface.select("ur5_2f_test")
    assert surface.readiness(old_ur5) == "ready"

    failing_suction = surface.select("suction_test")
    assert surface.committed is None
    assert surface.presentation == ("loading", "suction_test")
    assert surface.preparation_failed(failing_suction, "missing authored environment") == "failed"
    assert surface.presentation == (
        "failure", "suction_test", "missing authored environment", "Retry"
    )

    assert surface.readiness(old_ur5) == "stale_ignored"
    assert surface.scene == "suction_test"
    assert surface.ready is False

    new_ur5 = surface.select("ur5_2f_test")
    assert new_ur5 != old_ur5
    assert surface.readiness(new_ur5) == "ready"
    assert surface.presentation == ("ready", "ur5_2f_test")


def test_cpp_scene_handoff_invalidates_every_async_surface_and_renders_scene_documents():
    handoff_start = CPP.index("void ScenePreviewWidget::invalidate_embedded_web_scene_handoff")
    handoff_end = CPP.index("void ScenePreviewWidget::cancel_embedded_web_lifecycle", handoff_start)
    handoff = CPP[handoff_start:handoff_end]
    context_start = CPP.index("void ScenePreviewWidget::set_preview_context")
    context_end = CPP.index("ScenePreviewWidget::PreviewContext ScenePreviewWidget::preview_context", context_start)
    context = CPP[context_start:context_end]

    assert "scene_identity_changed" in context
    assert "invalidate_embedded_web_scene_handoff(normalized.scene_id)" in context
    assert "cancel_embedded_web_lifecycle(false)" in handoff
    assert "++embedded_web_readiness_token_" in handoff
    assert "++embedded_web_browser_load_token_" in handoff
    assert "embedded_web_server_probe_ = EmbeddedWebServerProbe{}" in handoff
    assert "embedded_web_has_committed_surface_ = false" in handoff
    assert "clear_embedded_editor_state_for_scene_handoff()" in handoff
    assert "show_embedded_web_loading_document(scene_id)" in handoff

    assert "Loading Product View" in CPP
    assert "Product View preparation failed" in CPP
    assert "Correct the scene-authoring blockers" in CPP
    assert "workcell-retry://%1" in CPP
    assert "identity.scene_id == selected_scene" in CPP
    assert "readiness_token != embedded_web_readiness_token_" in CPP
    assert "browser_load_token != embedded_web_browser_load_token_" in CPP
