from pathlib import Path

UI_HEADER = Path(
    "workcell_builder/workcell_builder/include/workcell_builder_ui_utils.hpp"
).read_text(encoding="utf-8")
HOME_V3 = Path(
    "workcell_builder/workcell_builder/include/workcell_home_polish_v3.hpp"
).read_text(encoding="utf-8")
MAINWINDOW = Path(
    "workcell_builder/workcell_builder/gui/mainwindow.cpp"
).read_text(encoding="utf-8")
STARTUP_DIALOG = Path(
    "workcell_builder/workcell_builder/gui/startup_dialog.cpp"
).read_text(encoding="utf-8")


def test_home_v3_is_the_active_composition_and_keeps_home_as_workcell_library():
    assert '#include "workcell_home_polish_v3.hpp"' in STARTUP_DIALOG
    assert '#include "workcell_home_polish_v3.hpp"' not in UI_HEADER
    assert "Keep shared UI utilities link-safe" in UI_HEADER
    for token in [
        'Your workcells',
        'Select a workcell or start a new robotic cell.',
        'Search workcells...',
        'Total Workcells',
        'Needs Attention',
        'homeV3KpiBlocked',
        'homeV3SceneThumb',
        'homeV3InspectorContent',
    ]:
        assert token in HOME_V3


def test_home_v3_removes_redundant_primary_navigation_entries_without_reindexing_stack():
    assert 'nav->item(2)->setHidden(true)' in HOME_V3
    assert 'nav->item(3)->setHidden(true)' in HOME_V3
    assert 'Home already is the workcell library' in HOME_V3
    assert 'Demo Mode is secondary' in HOME_V3


def test_home_v3_uses_canonical_product_view_for_preview_with_cached_fallback():
    for token in [
        'runtime_preview_has_usable_content',
        'embeddedWeb3dProductView',
        'scene3dViewportWidget',
        'embedded_product_view_runtime_state_changed',
        'post_save_product_view_refresh_finished',
        'Live read-only preview from the canonical Product View renderer.',
        'PREVIEW PREPARING',
        'smoke/scene3d_gui_smoke.png',
        'acceptance/scene3d_gui_smoke.png',
    ]:
        assert token in HOME_V3


def test_home_v3_hides_redundant_visible_action_strip_behind_overflow():
    assert 'homeV3InspectorMore' in HOME_V3
    assert 'Open in Scene Builder' in HOME_V3
    assert 'Open Product View' in HOME_V3
    assert 'Simulate · Fake Hardware' in HOME_V3
    for visible_button_object in [
        'homeV3OpenButton',
        'homeV3ValidateButton',
        'homeV3GenerateButton',
        'homeV3SimulateButton',
    ]:
        assert visible_button_object not in HOME_V3


def test_home_v3_preserves_canonical_scene_table_columns_and_fake_hardware_safety():
    for token in [
        'constexpr int kSceneColumn = 0;',
        'constexpr int kStatusColumn = 1;',
        'constexpr int kRobotColumn = 2;',
        'constexpr int kToolColumn = 3;',
        'constexpr int kTaskColumn = 4;',
        'constexpr int kLaunchColumn = 5;',
        'constexpr int kUpdatedColumn = 6;',
        'constexpr int kPinColumn = 7;',
    ]:
        assert token in HOME_V3

    # Existing production safety wording/guards remain untouched by this Home-only pass.
    for safety_token in [
        'Fake hardware default / Real robot locked',
        'Plan / Simulate',
        'RViz / MoveIt Preview',
    ]:
        assert safety_token in MAINWINDOW
