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
    assert "install_contextual_navigation(" in MAIN
    assert "show_workcells_home();" in MAIN


def test_home_is_strictly_sidebar_only_and_hides_every_toolbar_recursively():
    for token in [
        'page_index == 0',
        'studioTargetSidebar',
        'sidebar->setVisible(home_page)',
        'studioTargetTopbar',
        'old_topbar->hide()',
        'old_topbar->setMaximumHeight(0)',
        'studioContextTopbar',
        'context->setVisible(!home_page)',
        'hide_all_legacy_studio_toolbars(window)',
        'window->findChildren<QToolBar *>()',
        'toolbar->setMaximumHeight(0)',
    ]:
        assert token in NAV


def test_scene_topbar_is_minimal_context_not_another_navigation_system():
    for token in [
        'context->setFixedHeight(40)',
        '← Workcells',
        'studioContextSceneTitle',
        'studioContextPage',
        'studioContextStatus',
        '●  FAKE HARDWARE',
        '▣  REAL LOCKED',
        'Plan & Simulate',
        'Validation',
        'Export',
    ]:
        assert token in NAV

    # Canonical id and robot/tool detail belong in inspectors/tooltips, not in
    # the compact navigation bar.
    assert 'studioContextSceneId' not in NAV
    assert 'studioContextRobotTool' not in NAV
    assert 'REAL HARDWARE LOCKED' not in NAV


def test_home_inspector_does_not_repeat_sidebar_workflow_actions():
    assert 'studioTargetInspectorMore' in NAV
    assert 'more->show()' in NAV
    for removed in ['studioTargetPrimaryAction', 'studioTargetSecondaryAction', 'studioTargetSimulateAction']:
        assert removed not in NAV


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


def test_workcells_button_delegates_to_canonical_mainwindow_navigation():
    assert "if (navigate_home) navigate_home();" in NAV
    assert "pages->setCurrentIndex(0)" not in NAV
