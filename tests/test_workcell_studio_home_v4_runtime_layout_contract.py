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


def test_home_v4_hides_backing_table_item_text_instead_of_clearing_canonical_data():
    for token in [
        'hideBackingItemText',
        'item->setForeground(QBrush(Qt::transparent))',
        'item->setBackground(QBrush(Qt::transparent))',
        'kSceneColumn, kStatusColumn, kRobotColumn, kToolColumn, kUpdatedColumn',
        'table->setColumnWidth(kPinColumn, 74)',
        'QStringLiteral("Pinned")',
    ]:
        assert token in HOME_V4
    assert 'item->setText(QString())' not in HOME_V4


def test_home_v4_owns_a_visible_native_read_only_preview_from_canonical_scene_payload():
    for token in [
        'homeV4InspectorViewport',
        'new Scene3DViewportWidget',
        'active_scene_preview_widget()',
        'source_preview->findChild<Scene3DViewportWidget *>',
        'source_viewport->items',
        'home_viewport->ingest_preview_items(source_viewport->items)',
        'home_viewport->fit_product_view()',
        'refresh_scene_builder_state_from_active_scene()',
        'contextMatchesScene',
        'LOADING 3D PREVIEW…',
    ]:
        assert token in HOME_V4


def test_home_v4_restores_a_useful_selection_and_keeps_inspector_in_sync():
    for token in [
        'studio_home/last_selected_scene',
        'home_polish_v3::pinnedScenes()',
        'firstVisibleRow',
        'table->setCurrentCell(row, kSceneColumn)',
        'QMetaObject::invokeMethod(table, "cellClicked"',
        'scheduleHomePreview(window, sceneNameAt(table, row))',
    ]:
        assert token in HOME_V4


def test_home_v4_removes_visible_redundant_controls_and_duplicate_top_branding():
    for token in [
        'studioTopBrand',
        'studioTopStatusChip',
        'homeV3InspectorMore',
        'homeV3ViewDetails',
        'label->setText(QStringLiteral("Workcells"))',
    ]:
        assert token in HOME_V4
    assert 'more->hide()' in HOME_V4
    assert 'details->hide()' in HOME_V4
