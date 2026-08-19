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


def test_home_v3_is_the_active_stable_startup_composition():
    assert '#include "workcell_home_polish_v3.hpp"' in STARTUP_DIALOG
    assert '#include "workcell_home_polish_v4.hpp"' not in STARTUP_DIALOG
    assert '#include "workcell_home_polish_v5.hpp"' not in STARTUP_DIALOG
    assert '#include "workcell_home_polish_v3.hpp"' not in UI_HEADER
    assert "Keep shared UI utilities link-safe" in UI_HEADER
    for token in [
        'Your workcells',
        'Select a workcell or start a new robotic cell.',
        'Search workcells...',
        'Total Workcells',
        'Needs Attention',
        'homeV3KpiBlocked',
        'homeV3InspectorContent',
    ]:
        assert token in HOME_V3


def test_home_v3_removes_redundant_primary_navigation_entries_without_reindexing_stack():
    assert 'nav->item(2)->setHidden(true)' in HOME_V3
    assert 'nav->item(3)->setHidden(true)' in HOME_V3


def test_home_v3_table_has_one_visible_presentation_path():
    # v2 remains the canonical table owner. v3 must not lay a second scene/status/
    # robot/tool/date QWidget presentation over the same QTableWidgetItem text.
    assert 'home_polish_v2::polishTable(window)' in HOME_V3
    assert 'setCellWidget(row, kSceneColumn' not in HOME_V3
    assert 'setCellWidget(row, kStatusColumn' not in HOME_V3
    assert 'setCellWidget(row, kRobotColumn' not in HOME_V3
    assert 'setCellWidget(row, kToolColumn' not in HOME_V3
    assert 'setCellWidget(row, kUpdatedColumn' not in HOME_V3
    assert 'table->setRowHeight(row, 46)' in HOME_V3
    assert 'QStringLiteral("Pinned")' in HOME_V3


def test_home_v3_restores_a_useful_selection_without_synthetic_clicks():
    for token in [
        'studio_home/last_selected_scene',
        'pinnedScenes()',
        'firstVisibleRow',
        'table->setCurrentCell(row, kSceneColumn)',
        'table->selectRow(row)',
    ]:
        assert token in HOME_V3
    # Startup selection is visual/inspector state only. MainWindow's canonical
    # scene owner is updated by the real cellClicked connection when the user acts.
    assert 'QMetaObject::invokeMethod(table, "cellClicked"' not in HOME_V3


def test_home_v3_preview_uses_workspace_scene_roots_and_explicit_fallback():
    for token in [
        'sceneRootCandidates',
        'startup/last_workspace',
        'src/easy_manipulation_deployment/scenes',
        'QDir::homePath()',
        'smoke/scene3d_gui_smoke.png',
        'acceptance/scene3d_gui_smoke.png',
        'NO PREVIEW IMAGE',
        'Open Product View once to render this workcell',
        'source->isVisible()',
    ]:
        assert token in HOME_V3
    assert 'PREVIEW PREPARING' not in HOME_V3


def test_home_v3_details_panel_has_no_duplicate_workflow_controls():
    # Home is for choosing a workcell. Operational workflow actions remain on
    # their dedicated pages/navigation instead of being repeated in the details card.
    for token in [
        'homeV3InspectorMore',
        'homeV3ViewDetails',
        'Open in Scene Builder',
        'Generate Package',
        'Simulate · Fake Hardware',
    ]:
        assert token not in HOME_V3
    assert 'homeV3InspectorPin' in HOME_V3


def test_home_v3_normalizes_common_robot_tool_labels():
    for token in [
        'robotiq_85_gripper',
        'robotiq_2f_85_gripper',
        'Robotiq 2F-85',
        'OnRobot AirPick4',
        'Single Suction',
    ]:
        assert token in HOME_V3


def test_home_v3_preserves_canonical_scene_columns_and_fake_hardware_safety():
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

    for safety_token in [
        'Fake hardware default / Real robot locked',
        'Plan / Simulate',
        'RViz / MoveIt Preview',
    ]:
        assert safety_token in MAINWINDOW
