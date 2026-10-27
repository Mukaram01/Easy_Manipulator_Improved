from pathlib import Path

ROOT = Path('workcell_builder/workcell_builder/gui')
HDR = (ROOT / 'scene_preview_widget.h').read_text(encoding='utf-8')
PREVIEW_CPP = (ROOT / 'scene_preview_widget.cpp').read_text(encoding='utf-8')
MAINWINDOW_CPP = (ROOT / 'mainwindow.cpp').read_text(encoding='utf-8')


def test_embedded_web3d_runtime_signal_refreshes_outer_status_ui():
    assert 'embedded_product_view_runtime_state_changed(const QString & state, bool has_usable_content)' in HDR
    assert 'emit embedded_product_view_runtime_state_changed(state_text, runtime_preview_has_usable_content());' in PREVIEW_CPP
    connection = MAINWINDOW_CPP[MAINWINDOW_CPP.index('embedded_product_view_runtime_state_changed'):]
    for token in [
        'refresh_scene_builder_view_chips();',
        'refresh_scene_workflow_rail();',
        'refresh_preview_launch_ui();',
        'refresh_new_cell_checklist();',
    ]:
        assert token in connection


def test_embedded_ready_status_does_not_depend_on_native_counters():
    # Usable content means the viewer reported terminal scene_ready; no paint counters exist any more.
    usable = PREVIEW_CPP[PREVIEW_CPP.index('bool ScenePreviewWidget::runtime_preview_has_usable_content() const'):]
    usable = usable[:usable.index('\n}\n')]
    assert 'return embedded_product_view_state_ == EmbeddedProductViewState::Ready;' in usable
    assert 'render_debug_counters' not in usable
    chip_fn = MAINWINDOW_CPP[MAINWINDOW_CPP.index('void MainWindow::refresh_scene_builder_view_chips()'):]
    chip_fn = chip_fn[:chip_fn.index('\n}\n')]
    assert 'render_debug_counters' not in chip_fn


def test_saved_clean_layout_does_not_recommend_save_layout():
    rec_fn = MAINWINDOW_CPP[MAINWINDOW_CPP.index('std::vector<MainWindow::RecommendedWorkflowAction> MainWindow::resolve_recommended_workflow_actions()'):]
    assert 'const bool derived_layout_ready = !layout_dirty_ && (' in rec_fn
    assert 'derive_layout_state_model(s.scene_dir, canvas_model, canonical_path_match)' in rec_fn
    assert 'if (!derived_layout_ready)' in rec_fn
    assert 'if (layout_dirty_ || !layout_saved_)' not in rec_fn


def test_workflow_warning_status_renders_as_warnings():
    assert 'case SceneWorkflowStepStatus::Warning: return "Warnings";' in MAINWINDOW_CPP
    assert 'case SceneWorkflowStepStatus::Warning: return "Missing";' not in MAINWINDOW_CPP


def test_failed_embedded_state_still_renders_failed():
    assert 'if (state == EmbeddedProductViewState::Failed) state_text = QStringLiteral("failed");' in PREVIEW_CPP
    status = PREVIEW_CPP[PREVIEW_CPP.index('QString ScenePreviewWidget::runtime_preview_status_text() const'):]
    status = status[:status.index('\n}\n')]
    assert 'return QStringLiteral("Web3D Product View — failed, Retry");' in status
    assert 'return QStringLiteral("Preview failed");' in status
    # No second renderer can turn a failure into a "ready" preview.
    assert 'compatibility mode' not in PREVIEW_CPP
    assert 'CompatibilityReady' not in PREVIEW_CPP


def test_launch_artifacts_are_not_labeled_fake_hardware_ready():
    assert 'Launch Artifacts: %1' in MAINWINDOW_CPP
    assert 'fake_hardware_launch=%3' in MAINWINDOW_CPP


def test_embedded_product_view_ready_requires_browser_scene_ready_not_shell_http_200():
    status_start = PREVIEW_CPP.index('static const char kStatusScript')
    status_script = PREVIEW_CPP[status_start : PREVIEW_CPP.index('embedded_web_view_->page()->runJavaScript', status_start)]
    readiness_start = PREVIEW_CPP.index('const QVariantMap status = value.toMap();')
    readiness_handler = PREVIEW_CPP[readiness_start : PREVIEW_CPP.index('QTimer::singleShot(750', readiness_start)]
    assert "s.web3d_readiness_state || s.web3dReadinessState" in status_script
    # server_ready is infrastructure state; the readiness handler refuses it as scene readiness.
    assert 'server_ready is infrastructure state, not scene readiness' in readiness_handler
    assert 'failed_required_item_count' in status_script
    assert 'scene_id: s.scene_id || s.sceneId' in status_script
    assert 'expected_physical_item_count' in status_script
    assert 'rendered_physical_item_count' in status_script
    assert 'robot_status' in status_script
    assert 'tool_status' in status_script
    assert 'environment_status' in status_script
    assert 'camera_status' in status_script
    assert 'final_lifecycle_state' in status_script
    # Since 7667bc12 each readiness condition rejects separately with a reason
    # (lifecycle, terminal, scene, source JSON, builder revision, loaded JSON,
    # failed required items, component statuses); only then is the view Ready.
    for condition in [
        'else if (lifecycle_state != QStringLiteral("scene_ready"))',
        'else if (!terminal)',
        'else if (reported_scene_id != identity.scene_id)',
        'else if (source_json != expected_json_path)',
        'else if (builder_revision != expected_builder_revision)',
        'else if (!scene_json_loaded)',
        'else if (failed_required_count != 0)',
        'else if (!component_ready(robot_status) || !component_ready(tool_status) || !component_ready(environment_status) || !component_ready(camera_status))',
    ]:
        assert condition in readiness_handler, condition
    assert readiness_handler.index('if (contract_reason.isEmpty()) {') > readiness_handler.index('else if (failed_required_count != 0)')
    assert 'boot_state == QStringLiteral("ready")' not in readiness_handler
    assert 'failed_required=%7' in readiness_handler
