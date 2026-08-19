from pathlib import Path

UI_UTILS = Path(
    "workcell_builder/workcell_builder/gui/workcell_builder_ui_utils.cpp"
).read_text(encoding="utf-8")
MAINWINDOW = Path(
    "workcell_builder/workcell_builder/gui/mainwindow.cpp"
).read_text(encoding="utf-8")


def test_home_v2_has_engineering_dashboard_shell_and_safety_chips():
    for token in [
        'studioHomeSidebar',
        'studioHomeSidebarNav',
        'studioTopSafetyChip',
        'Simulation Mode   FAKE HARDWARE',
        'studioTopHardwareChip',
        'Real hardware   LOCKED',
        'Welcome to Workcell Studio',
    ]:
        assert token in UI_UTILS


def test_home_v2_starts_interactive_main_window_maximized_but_preserves_scene3d_smoke():
    assert 'window->setMinimumSize(1100, 700);' in UI_UTILS
    assert 'window->showMaximized();' in UI_UTILS
    assert '--scene3d-smoke' in UI_UTILS
    smoke_guard = UI_UTILS.index('--scene3d-smoke')
    maximize = UI_UTILS.index('window->showMaximized();')
    assert smoke_guard < maximize


def test_home_v2_has_scene_filters_sorting_and_persistent_favorites():
    for token in [
        'studioHomeRobotFilter',
        'studioHomeToolFilter',
        'studioHomeFavoritesOnly',
        'studioHomeSort',
        'studio_home/pinned_scenes',
        'Sort: Recently updated',
        'Sort: Pinned first',
        'Pin Favorite',
    ]:
        assert token in UI_UTILS


def test_home_v2_has_direct_selected_scene_actions_without_weakening_safety():
    for token in [
        'studioHomeOpenSceneButton',
        'studioHomeValidateButton',
        'studioHomeGenerateButton',
        'studioHomeSimulateButton',
        'Simulate (Fake Hardware)',
        'Generate Scene Package',
        'Open in Scene Builder',
    ]:
        assert token in UI_UTILS

    # Existing guarded runtime contract remains in the production MainWindow path.
    for safety_token in [
        'use_fake_hardware',
        'allow_real_hardware_motion',
        'real_robot_locked',
    ]:
        assert safety_token in MAINWINDOW


def test_home_v2_extends_the_existing_scene_table_without_shifting_canonical_columns():
    assert 'constexpr int kHomeSceneColumn = 0;' in UI_UTILS
    assert 'constexpr int kHomeLaunchColumn = 5;' in UI_UTILS
    assert 'constexpr int kHomeUpdatedColumn = 6;' in UI_UTILS
    assert 'constexpr int kHomePinColumn = 7;' in UI_UTILS
    assert 'setColumnCount(kHomeColumnCount)' in UI_UTILS
