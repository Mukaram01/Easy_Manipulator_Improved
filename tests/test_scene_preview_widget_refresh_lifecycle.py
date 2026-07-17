from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
MAINWINDOW_CPP = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


@dataclass(frozen=True)
class RequestKey:
    scene: str
    directory: str
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

    def request(self, key, force=False):
        if not force and ((self.active and key == self.active[0]) or (self.pending and key == self.pending[0])):
            return
        if force:
            self.active = None
            self.pending = None
        self.generation += 1
        request = (key, self.generation)
        self.active = request
        self.pending = request

    def start_pending(self):
        if self.pending is not None:
            self.started.append(self.pending)
            self.pending = None

    def finish(self, request):
        if request != self.active:
            self.retired.append(request)
            self.start_pending()
            return
        self.start_pending()


def test_repeated_automatic_notifications_keep_one_generation_and_active_request():
    lifecycle = RefreshLifecycle()
    key = RequestKey("ur5_2f_test", "/cells/ur5_2f_test", b"same", 7)
    lifecycle.request(key)
    active = lifecycle.active
    lifecycle.request(key)
    lifecycle.request(key)

    assert lifecycle.generation == 1
    assert lifecycle.active == active
    assert lifecycle.pending == active


def test_scene_or_payload_change_retires_old_completion_and_starts_replacement():
    lifecycle = RefreshLifecycle()
    old = RequestKey("ur5_2f_test", "/cells/ur5_2f_test", b"old", 7)
    changed_scene = RequestKey("ur5_3f_test", "/cells/ur5_3f_test", b"new", 8)
    lifecycle.request(old)
    lifecycle.start_pending()
    lifecycle.request(changed_scene)
    lifecycle.finish((old, 1))

    assert lifecycle.retired == [(old, 1)]
    assert lifecycle.started == [(old, 1), (changed_scene, 2)]
    assert lifecycle.active == (changed_scene, 2)


def test_one_manual_refresh_invalidates_once_and_creates_one_retry():
    lifecycle = RefreshLifecycle()
    key = RequestKey("ur5_2f_test", "/cells/ur5_2f_test", b"same", 7)
    lifecycle.request(key)
    lifecycle.start_pending()
    lifecycle.request(key, force=True)

    assert lifecycle.generation == 2
    assert lifecycle.active == (key, 2)
    assert lifecycle.pending == (key, 2)


def test_cpp_coalesces_before_generation_and_retires_stale_process_before_ui_work():
    refresh_start = CPP.index("void ScenePreviewWidget::request_embedded_web_product_view_refresh")
    identity_start = CPP.index("ScenePreviewWidget::EmbeddedWebRequestIdentity", refresh_start)
    refresh = CPP[refresh_start:identity_start]
    assert refresh.index("matches_context(request_key)") < refresh.index("++embedded_web_request_generation_")
    assert "const PreviewContext normalized_context = normalized_preview_context(preview_context_);" in CPP
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
