from pathlib import Path

PREVIEW = Path(
    "workcell_builder/workcell_builder/include/home_workcell_preview_web.hpp"
).read_text(encoding="utf-8")
MAIN = Path(
    "workcell_builder/workcell_builder/gui/main.cpp"
).read_text(encoding="utf-8")


def test_home_installs_read_only_web_preview_after_target_shell():
    assert '#include "home_workcell_preview_web.hpp"' in MAIN
    assert "configure_target_shell(this, workspace)" in MAIN
    assert "install_home_web_preview(this, workspace)" in MAIN


def test_preview_prefers_real_evidence_then_static_preview():
    for token in [
        "find_preview_path(workspace_root, scene_id)",
        'preview/static_preview.svg',
        'preview/static_preview.html',
        'generate_workcell_static_preview.py',
        'cell_definition.yaml',
        'environment_layout.yaml',
        'workcell_builder_task_intent.yaml',
    ]:
        assert token in PREVIEW


def test_preview_uses_local_read_only_webengine_and_never_displays_error_panel():
    for token in [
        "WORKCELL_BUILDER_HAS_WEBENGINE",
        "QWebEngineView",
        "JavascriptEnabled, false",
        "LocalContentCanAccessFileUrls, true",
        "LocalContentCanAccessRemoteUrls, false",
        "studioTargetWebPreview",
        "metadata_schematic_html",
    ]:
        assert token in PREVIEW
    assert "NO PREVIEW IMAGE" not in PREVIEW
    assert "PREVIEW ERROR" not in PREVIEW


def test_preview_generation_is_visual_only_and_does_not_launch_robot_runtime():
    assert 'process.start(QStringLiteral("python3"), arguments)' in PREVIEW
    assert "ros2 launch" not in PREVIEW
    assert "MoveIt" not in PREVIEW
    assert "fake_hardware:=false" not in PREVIEW
    assert "Q_COREAPP_STARTUP_FUNCTION" not in PREVIEW
    assert "installEventFilter" not in PREVIEW
    assert "QTimer::singleShot" not in PREVIEW
