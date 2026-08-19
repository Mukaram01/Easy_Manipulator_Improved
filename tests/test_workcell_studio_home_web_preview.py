from pathlib import Path

PREVIEW = Path(
    "workcell_builder/workcell_builder/include/home_workcell_preview_web.hpp"
).read_text(encoding="utf-8")
MAIN = Path(
    "workcell_builder/workcell_builder/gui/main.cpp"
).read_text(encoding="utf-8")


def test_home_installs_preview_after_target_shell():
    assert '#include "home_workcell_preview_web.hpp"' in MAIN
    assert "configure_target_shell(this, workspace)" in MAIN
    assert "install_home_web_preview(this, workspace)" in MAIN


def test_home_preview_mirrors_the_canonical_product_view_instead_of_generating_svg():
    for token in [
        '#include "scene_preview_widget.h"',
        'embeddedWeb3dProductView',
        'runtime_preview_has_usable_content()',
        'preview_context()',
        'source_web->url()',
        'is_canonical_product_view_url',
        'homeMirroredProductViewUrl',
        'web->setUrl(viewer_url)',
        'embedded_product_view_runtime_state_changed',
    ]:
        assert token in PREVIEW

    # Home must not mutate scene directories just to obtain a picture.
    assert "generate_workcell_static_preview.py" not in PREVIEW
    assert "preview/static_preview.svg" not in PREVIEW
    assert "preview/static_preview.html" not in PREVIEW
    assert "QProcess" not in PREVIEW


def test_mirrored_product_view_is_viewport_only_and_read_only():
    for token in [
        "JavascriptEnabled, true",
        "studioTargetWebPreview",
        "workcell-home-preview-style",
        ".topbar, .object-panel, .details-panel",
        "#scene-canvas",
        "WA_TransparentForMouseEvents",
        "LocalContentCanAccessRemoteUrls, false",
        "127.0.0.1",
        "/workcell_studio_web/viewer/index.html",
    ]:
        assert token in PREVIEW


def test_home_preview_has_non_error_fallback_while_real_viewer_prepares():
    for token in [
        "find_preview_path(workspace_root, scene_id)",
        "Stored scene snapshot",
        "PREPARING LIVE 3D",
        "preparing_product_view_html",
    ]:
        assert token in PREVIEW
    assert "NO PREVIEW IMAGE" not in PREVIEW
    assert "PREVIEW ERROR" not in PREVIEW
    assert "static_preview.svg" not in PREVIEW


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
