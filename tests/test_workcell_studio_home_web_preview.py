from pathlib import Path

PREVIEW = Path(
    "workcell_builder/workcell_builder/include/home_workcell_preview_web.hpp"
).read_text(encoding="utf-8")
MAIN = Path(
    "workcell_builder/workcell_builder/gui/main.cpp"
).read_text(encoding="utf-8")


def test_home_installs_snapshot_preview_after_target_shell():
    assert '#include "home_workcell_preview_web.hpp"' in MAIN
    assert "configure_target_shell(this, workspace)" in MAIN
    assert "install_home_snapshot_preview(this, workspace)" in MAIN
    assert "install_home_web_preview(this, workspace)" not in MAIN


def test_home_does_not_create_a_second_webengine_product_view():
    # Home is image-cheap. The only QWebEngineView reference is the canonical
    # ScenePreviewWidget surface used as a snapshot source.
    assert 'embeddedWeb3dProductView' in PREVIEW
    assert 'new QWebEngineView' not in PREVIEW
    assert 'studioTargetWebPreview' not in PREVIEW
    assert 'setUrl(viewer_url)' not in PREVIEW
    assert 'homeMirroredProductViewUrl' not in PREVIEW
    assert 'workcell-home-preview-style' not in PREVIEW


def test_home_snapshot_cache_is_outside_scene_source_and_revision_aware():
    for token in [
        "QStandardPaths::CacheLocation",
        'home_previews',
        'scene_content_fingerprint(workspace_root, scene_id)',
        'exact_home_preview_cache_path',
        'prune_old_home_preview_cache',
        'cache_path + QStringLiteral(".json")',
        'scene_fingerprint',
    ]:
        assert token in PREVIEW

    # Selecting Home must never generate or write scene-owned preview artifacts.
    assert "generate_workcell_static_preview.py" not in PREVIEW
    assert "preview/static_preview.svg" not in PREVIEW
    assert "preview/static_preview.html" not in PREVIEW
    assert "QProcess" not in PREVIEW


def test_home_captures_only_the_existing_canonical_canvas_after_product_view_is_ready():
    for token in [
        '#include "scene_preview_widget.h"',
        'runtime_preview_has_usable_content()',
        'preview_context()',
        'embedded_product_view_runtime_state_changed',
        'post_save_product_view_refresh_finished',
        "document.getElementById('scene-canvas')",
        "canvas.toDataURL('image/png')",
        'QByteArray::fromBase64',
        'pixmap.save(cache_path, "PNG")',
        'lifecycle_state != QStringLiteral("scene_ready")',
        '!terminal',
        'rendered_physical_count <= 0',
        'reported_scene_id != scene_id',
        'completed_home_preview_contract',
    ]:
        assert token in PREVIEW


def test_home_preview_uses_current_cache_then_scene_specific_stale_fallback():
    for token in [
        'show_fast_home_preview',
        'Current Product View snapshot',
        'newest_home_preview_cache_path',
        'Cached Product View snapshot · refreshing in background',
        'Preparing scene preview…',
    ]:
        assert token in PREVIEW
    for forbidden in [
        'Stored scene snapshot',
        'find_preview_path(workspace_root, scene_id)',
    ]:
        assert forbidden not in PREVIEW
    assert "NO PREVIEW IMAGE" not in PREVIEW
    assert "PREVIEW ERROR" not in PREVIEW
    assert "PREPARING LIVE 3D" not in PREVIEW


def test_stale_home_preview_is_a_transitional_state_using_existing_capture_path():
    assert 'homeSnapshotState", QStringLiteral("refreshing")' in PREVIEW
    assert "cache=stale revision=%3 state=regenerating" in PREVIEW
    assert "capture_canonical_product_view_snapshot(safe_window, table, safe_label, workspace_root)" in PREVIEW
    assert 'homeSnapshotState", QStringLiteral("current")' in PREVIEW


def test_home_preview_logs_each_scene_revision_cache_event_once():
    assert "log_home_preview_event_once" in PREVIEW
    assert 'homeSnapshotLastLogEvent' in PREVIEW
    assert 'scene_id + QStringLiteral("|hit|") + fingerprint' in PREVIEW
    assert 'scene_id + QStringLiteral("|stale|") + fingerprint' in PREVIEW
    assert 'scene_id + QStringLiteral("|miss|") + fingerprint' in PREVIEW
    assert 'scene_id + QStringLiteral("|generated|") + scene_fingerprint' in PREVIEW


def test_late_snapshot_cannot_replace_newer_selection_or_revision():
    for token in [
        'homePreviewSelectionGeneration',
        'selection_generation',
        'selected_home_scene_id(safe_table) != scene_id',
        'scene_content_fingerprint(workspace_root, scene_id) != scene_fingerprint',
    ]:
        assert token in PREVIEW


def test_selected_preview_preserves_aspect_ratio_and_does_not_upscale():
    assert 'target.boundedTo(pixmap.size())' in PREVIEW
    assert 'Qt::KeepAspectRatio' in PREVIEW
    assert 'setScaledContents(false)' in PREVIEW


def test_home_preview_does_not_add_runtime_or_startup_mutation_paths():
    for forbidden in [
        "ros2 launch",
        "fake_hardware:=false",
        "ros2_control",
        "Q_COREAPP_STARTUP_FUNCTION",
        "installEventFilter",
        "QTimer::singleShot",
        "setCellWidget",
    ]:
        assert forbidden not in PREVIEW
