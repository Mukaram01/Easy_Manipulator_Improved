from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text()
H = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.h').read_text()
MW = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()


def test_authoritative_scene_context_is_passed_to_preview_widget():
    assert 'struct PreviewContext' in H
    assert 'absolute_scene_dir' in H
    assert 'scene_preview_widget_->set_preview_context(preview_context);' in MW
    assert 'preview_context.absolute_scene_dir = selected_scene_state_.path;' in MW


def test_cwd_is_not_used_to_invent_primary_selected_scene_path():
    start = CPP.index('void ScenePreviewWidget::start_embedded_web_prepare')
    body = CPP[start:CPP.index('void ScenePreviewWidget::on_embedded_web_prepare_finished', start)]
    assert 'preview_context_.absolute_scene_dir' in body
    assert 'QDir(QDir::currentPath()).absoluteFilePath(QStringLiteral("scenes/%1")' not in body


def test_repo_root_resolution_prefers_scene_walk_then_secondary_fallbacks():
    order = [
        'selected scene directory upward walk',
        'caller-provided repository root after selected scene',
        'WORKCELL_STUDIO_REPO_ROOT secondary override',
        'fallback application path upward walk',
    ]
    positions = [CPP.index(token) for token in order]
    assert positions == sorted(positions)


def test_web3d_failure_activates_native_compatibility_and_preserves_error():
    assert 'void ScenePreviewWidget::activate_native_compatibility_preview' in CPP
    assert 'embedded_web_last_error_ = reason;' in CPP
    assert 'Web3D unavailable — using native compatibility preview' in CPP
    assert 'activate_native_compatibility_preview(detail);' in CPP
    assert 'activate_native_compatibility_preview(reason);' in CPP


def test_refresh_preview_can_retry_web3d_and_clear_fallback_on_success():
    assert 'request_embedded_web_product_view_refresh(bool force' in H
    prepare = CPP[CPP.index('void ScenePreviewWidget::start_embedded_web_prepare'):CPP.index('void ScenePreviewWidget::on_embedded_web_prepare_finished')]
    assert 'native_compatibility_fallback_active_ = false;' in prepare
    assert 'set_embedded_product_view_state(EmbeddedProductViewState::Ready' in CPP


def test_populated_scene_never_recommends_add_asset():
    assert 'if (!has_asset_items && editable_layout_item_count_ == 0 && preview_fallback_item_count_ == 0)' in MW


def test_disabled_legacy_perception_is_not_actionable_warning():
    total = CPP[CPP.index('int ScenePreviewWidget::total_warning_count() const'):CPP.index('bool ScenePreviewWidget::task_is_ready() const')]
    assert 'legacy disabled' in total
    assert 'perception disabled' in total
    assert 'return false;' in total


def test_recovered_stale_mesh_paths_not_actionable_warning():
    total = CPP[CPP.index('int ScenePreviewWidget::total_warning_count() const'):CPP.index('bool ScenePreviewWidget::task_is_ready() const')]
    assert 'resolved_source_path is stale' in total
    assert 'package_uri' in total
    assert 'source_path_resolution_outcome.contains' in total


def test_root_resolution_failure_summary_deduplicated():
    assert 'root_resolution_summary_keys_' in H
    assert 'root_resolution_failed' in CPP
    assert '!root_resolution_summary_keys_.contains(summary_key)' in CPP


def test_stale_embedded_prepare_result_is_discarded_without_changing_current_preview_state():
    start = CPP.index('void ScenePreviewWidget::on_embedded_web_prepare_finished')
    body = CPP[start:CPP.index('void ScenePreviewWidget::load_prepared_embedded_web_scene', start)]
    stale_start = body.index('if (!still_current)')
    stale_body = body[stale_start:body.index('const QString absolute_output_path', stale_start)]

    assert 'Discarded embedded Product View preparation result for completed scene %1 revision %2' in stale_body
    assert '.arg(scene)' in stale_body
    assert '.arg(revision)' in stale_body
    assert 'process->deleteLater();' in body[:stale_start]
    assert 'embedded_web_prepare_process_ = nullptr;' in body[:stale_start]
    assert 'maybe_start_next_embedded_web_prepare();' in stale_body
    assert 'activate_native_compatibility_preview' not in stale_body
    assert 'set_embedded_product_view_state' not in stale_body
    assert 'simple_3d_view_' not in stale_body
    assert 'embedded_web_last_error_' not in stale_body


def test_current_embedded_prepare_failures_still_use_compatibility_preview():
    start = CPP.index('void ScenePreviewWidget::on_embedded_web_prepare_finished')
    body = CPP[start:CPP.index('void ScenePreviewWidget::load_prepared_embedded_web_scene', start)]
    current_failure = body[body.index('auto reject_prepare'):]
    assert 'activate_native_compatibility_preview(reason);' in current_failure
