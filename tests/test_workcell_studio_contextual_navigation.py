from pathlib import Path

NAV = Path(
    "workcell_builder/workcell_builder/include/home_contextual_navigation.hpp"
).read_text(encoding="utf-8")
MAIN = Path(
    "workcell_builder/workcell_builder/gui/main.cpp"
).read_text(encoding="utf-8")


def test_context_navigation_is_installed_after_home_shell_and_preview():
    assert '#include "home_contextual_navigation.hpp"' in MAIN
    assert "configure_target_shell(this, workspace)" in MAIN
    assert "install_home_snapshot_preview(this, workspace)" in MAIN
    assert "install_contextual_navigation(this)" in MAIN


def test_home_uses_sidebar_only_and_non_home_pages_use_compact_topbar():
    for token in [
        'page_index == 0',
        'studioTargetSidebar',
        'sidebar->setVisible(home_page)',
        'studioTargetTopbar',
        'old_topbar->hide()',
        'studioContextTopbar',
        'context->setVisible(!home_page)',
        'legacy_studio_toolbar',
        'legacy->hide()',
    ]:
        assert token in NAV


def test_scene_topbar_shows_context_not_duplicate_navigation():
    for token in [
        '←  Home',
        'studioContextSceneTitle',
        'studioContextSceneId',
        'studioContextPage',
        'studioContextStatus',
        'studioContextRobotTool',
        '●  FAKE HARDWARE',
        '▣  REAL HARDWARE LOCKED',
        'Needs Attention',
        'Plan & Simulate',
        'Validation',
        'Export',
    ]:
        assert token in NAV


def test_home_inspector_does_not_repeat_sidebar_workflow_actions():
    for token in [
        'studioTargetPrimaryAction',
        'studioTargetSecondaryAction',
        'studioTargetSimulateAction',
        'studioTargetInspectorMore',
        'button->hide()',
        'more->hide()',
        'View details →',
    ]:
        assert token in NAV


def test_contextual_shell_does_not_reintroduce_runtime_polish_or_scene_mutation():
    for forbidden in [
        'Q_COREAPP_STARTUP_FUNCTION',
        'installEventFilter',
        'QTimer::singleShot',
        'setCellWidget',
        'deleteLater',
        'ros2 launch',
        'fake_hardware:=false',
        'generate_workcell_static_preview.py',
    ]:
        assert forbidden not in NAV
