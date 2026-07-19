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
    chip_fn = MAINWINDOW_CPP[MAINWINDOW_CPP.index('void MainWindow::refresh_scene_builder_view_chips()'):]
    assert 'runtime_preview_has_usable_content()' in chip_fn
    assert 'Embedded Web3D readiness is runtime-based and does not depend on' in chip_fn
    assert 'preview_chip_status = QStringLiteral("Ready");' in chip_fn


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
    chip_fn = MAINWINDOW_CPP[MAINWINDOW_CPP.index('void MainWindow::refresh_scene_builder_view_chips()'):]
    assert 'runtime_preview_status != QStringLiteral("Preview failed")' in chip_fn
    assert 'preview_chip_status = QStringLiteral("Failed");' in chip_fn


def test_launch_artifacts_are_not_labeled_fake_hardware_ready():
    assert 'Launch Artifacts: %1' in MAINWINDOW_CPP
    assert 'fake_hardware_launch=%3' in MAINWINDOW_CPP
