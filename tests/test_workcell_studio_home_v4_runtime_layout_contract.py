from pathlib import Path

HOME_V4 = Path(
    "workcell_builder/workcell_builder/include/workcell_home_polish_v4.hpp"
).read_text(encoding="utf-8")
STARTUP_DIALOG = Path(
    "workcell_builder/workcell_builder/gui/startup_dialog.cpp"
).read_text(encoding="utf-8")
UI_HEADER = Path(
    "workcell_builder/workcell_builder/include/workcell_builder_ui_utils.hpp"
).read_text(encoding="utf-8")


def test_home_v4_is_active_without_reintroducing_shared_helper_linkage():
    assert '#include "workcell_home_polish_v4.hpp"' in STARTUP_DIALOG
    assert 'workcell_home_polish_v4.hpp' not in UI_HEADER
    assert '#include "workcell_home_polish_v3.hpp"' in HOME_V4


def test_home_v4_uses_one_delegate_instead_of_transparent_widget_overlays():
    for token in [
        'class HomeTableDelegate final : public QStyledItemDelegate',
        'deleteLegacyCellWidgets',
        'table->removeCellWidget(row, column)',
        'table->setItemDelegate(new HomeTableDelegate(table))',
        'table->setColumnHidden(kStatusColumn, false)',
        'table->setColumnHidden(kRobotColumn, false)',
        'table->setColumnHidden(kToolColumn, false)',
        'table->setColumnHidden(kUpdatedColumn, false)',
        'table->setColumnWidth(kPinColumn, 62)',
        'QStringLiteral("Pinned")',
    ]:
        assert token in HOME_V4

    # Regression guard: never make canonical QTableWidgetItem text transparent
    # again.  Model refreshes can legitimately destroy presentation widgets.
    assert 'item->setForeground(QBrush(Qt::transparent))' not in HOME_V4
    assert 'hideBackingItemText' not in HOME_V4


def test_home_v4_preview_is_deterministic_and_does_not_wait_forever_on_hidden_3d():
    for token in [
        'sceneRootCandidates',
        'startup/last_workspace',
        'src/easy_manipulation_deployment/scenes',
        'reliableCachedPreviewPath',
        'smoke/scene3d_gui_smoke.png',
        'acceptance/scene3d_gui_smoke.png',
        'home_polish_v3::liveCanonicalPreview',
        'NO PREVIEW GENERATED',
        'Open Product View to render this workcell',
    ]:
        assert token in HOME_V4

    assert 'PREVIEW PREPARING' not in HOME_V4
    assert 'new Scene3DViewportWidget' not in HOME_V4
    assert 'refresh_scene_builder_state_from_active_scene()' not in HOME_V4


def test_home_v4_restores_a_useful_selection_and_keeps_inspector_in_sync():
    for token in [
        'studio_home/last_selected_scene',
        'home_polish_v3::pinnedScenes()',
        'firstVisibleRow',
        'table->setCurrentCell(row, kSceneColumn)',
        'QMetaObject::invokeMethod(table, "cellClicked"',
        'scheduleHomePreview(window, scene)',
        'repairInspectorText',
        'friendlyTool(tool_raw)',
    ]:
        assert token in HOME_V4


def test_home_v4_hides_duplicate_top_bar_labels_by_content_not_fragile_object_name():
    for token in [
        'top_bar->findChildren<QLabel *>()',
        'QStringLiteral("WORKCELL STUDIO")',
        'QStringLiteral("Studio ready")',
        'homeV3InspectorMore',
        'homeV3ViewDetails',
        'label->setText(QStringLiteral("Workcells"))',
    ]:
        assert token in HOME_V4
    assert 'more->hide()' in HOME_V4
    assert 'details->hide()' in HOME_V4


def test_home_v4_keeps_canonical_table_data_untouched_for_filters_and_selection():
    # The delegate reads DisplayRole/ToolTipRole and paints friendly presentation
    # without rewriting the canonical table item text or UserRole identity.
    assert 'index.data(Qt::DisplayRole)' in HOME_V4
    assert 'index.data(Qt::ToolTipRole)' in HOME_V4
    assert 'setData(Qt::UserRole' not in HOME_V4
    assert 'item->setText(' not in HOME_V4
