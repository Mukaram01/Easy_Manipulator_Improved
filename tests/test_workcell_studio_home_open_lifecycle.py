from pathlib import Path


MAINWINDOW = Path(
    "workcell_builder/workcell_builder/gui/mainwindow.cpp"
).read_text(encoding="utf-8")
TARGET = Path(
    "workcell_builder/workcell_builder/include/home_workcells_target_shell.hpp"
).read_text(encoding="utf-8")
MAIN = Path(
    "workcell_builder/workcell_builder/gui/main.cpp"
).read_text(encoding="utf-8")
PREVIEW = Path(
    "workcell_builder/workcell_builder/include/home_workcell_preview_web.hpp"
).read_text(encoding="utf-8")
SCENE_PREVIEW = Path(
    "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp"
).read_text(encoding="utf-8")


def section(source: str, start: str, end: str) -> str:
    return source.split(start, 1)[1].split(end, 1)[0]


CANONICAL_OPEN = section(
    MAINWINDOW,
    "bool MainWindow::open_scene_builder_for_scene_index",
    "bool MainWindow::open_home_scene_in_builder",
)
HOME_OPEN = section(
    MAINWINDOW,
    "bool MainWindow::open_home_scene_in_builder",
    "bool MainWindow::open_scene_builder_for_selected_scene",
)
SETUP = section(
    MAINWINDOW,
    "void MainWindow::setup_studio_shell()",
    "void MainWindow::build_studio_header_actions()",
)
HOME_POPULATION = section(
    MAINWINDOW,
    "void MainWindow::refresh_studio_home_scene_table()",
    "QString MainWindow::home_scene_id_at_row",
)
BROWSER_REFRESH = section(
    MAINWINDOW,
    "void MainWindow::refresh_scene_browser_ui()",
    "void MainWindow::refresh_studio_home_scene_table()",
)


def test_home_single_click_selects_the_cell_but_never_loads_editor_state():
    selection = section(
        SETUP,
        "connect(dashboard_scene_table_, &QTableWidget::itemSelectionChanged",
        "// The production Home shell filters",
    )
    # The Home row is the one selected cell for Generate, Validation and Plan / Simulate.
    for required in ["find_scene_by_identity", "select_scene_index(scene_index)", "setEnabled(selected)",
                     "selected_scene_home_identity()", "setCurrentCell(r, 0)"]:
        assert required in selection
    for forbidden in [
        "home_selected_scene_id_",
        "refresh_scene_builder_selection_state_ui",
        "load_scene_builder_editor_for_selected_scene",
        "refresh_scene_builder_left_explorer",
        "populate_scene_hierarchy",
        "set_preview_context",
        "open_home_scene_in_builder",
    ]:
        assert forbidden not in selection


def test_home_population_restores_stable_identity_with_signals_blocked():
    for required in [
        "QSignalBlocker table_blocker(dashboard_scene_table_)",
        "QSignalBlocker library_blocker(dashboard_library_list_)",
        "home_scene_path_at_row(row) == selected_identity",
        "Qt::UserRole + 42",
    ]:
        assert required in HOME_POPULATION
    for forbidden in [
        "select_scene_index",
        "selected_scene_index_ =",
        "invokeMethod",
        'itemSelectionChanged"',
        "open_home_scene_in_builder",
    ]:
        assert forbidden not in HOME_POPULATION


def test_filter_sort_pin_and_inspector_refresh_are_home_ui_only():
    for required in [
        "apply_composed_filters",
        "sort_target_rows",
        "QSignalBlocker blocker(table)",
        "refresh_target_details",
        "restore_pinned_state",
    ]:
        assert required in TARGET
    for forbidden in [
        "open_home_scene_in_builder",
        "select_scene_index",
        "refresh_scene_builder_selection_state_ui",
        "ScenePreviewWidget",
        "Scene3DViewportWidget",
        "QWebEngineView",
    ]:
        assert forbidden not in TARGET
    assert '[refresh_home]() { refresh_home(); }' not in TARGET
    assert 'QMetaObject::invokeMethod(table, "cellClicked"' not in TARGET
    assert 'QMetaObject::invokeMethod(table, "itemSelectionChanged"' not in TARGET


def test_double_click_calls_canonical_helper_directly_once_without_inline_editing():
    double_click = section(
        SETUP,
        "connect(dashboard_scene_table_, &QTableWidget::cellDoubleClicked",
        "connect(dashboard_scene_table_, &QTableWidget::itemSelectionChanged",
    )
    assert double_click.count("open_home_scene_in_builder(scene_id, scene_path);") == 1
    assert "dashboard_open_scene_action_->trigger" not in double_click
    assert "setEditTriggers(QAbstractItemView::NoEditTriggers)" in TARGET
    assert "item->setFlags(item->flags() & ~Qt::ItemIsEditable)" in TARGET
    assert "item->setFlags(item->flags() & ~Qt::ItemIsEditable)" in HOME_POPULATION
    assert "studioTargetPrimaryAction" not in TARGET
    assert "Open Scene Builder" not in TARGET
    assert "bind_home_target_shell_actions" not in MAIN


def test_filtering_clears_a_selected_row_that_becomes_hidden():
    assert "table->isRowHidden(selected_row)" in TARGET
    assert "table->clearSelection();" in TARGET
    assert "table->setCurrentCell(-1, -1);" in TARGET


def test_modified_sort_role_does_not_overwrite_canonical_scene_path_role():
    assert "constexpr int kModifiedRole = Qt::UserRole + 43;" in TARGET
    assert "Qt::UserRole + 42" in MAINWINDOW


def test_explicit_home_open_uses_captured_id_and_cannot_run_after_navigation():
    assert "currentRow" not in HOME_OPEN
    assert "stable_scene_id" in HOME_OPEN
    assert "canonical_scene_path" in HOME_OPEN
    assert "find_scene_by_identity" in HOME_OPEN
    assert "StudioPage::DashboardPage" in HOME_OPEN
    assert HOME_OPEN.count("open_scene_builder_for_scene_index") == 1
    assert HOME_OPEN.count("Home: opening Scene Builder for %1") == 1
    assert "ur5_3f_test" not in HOME_OPEN
    assert "suction_test" not in HOME_OPEN


def test_one_open_request_has_one_canonical_editor_refresh():
    assert CANONICAL_OPEN.count("load_scene_builder_editor_for_selected_scene(source_action);") == 1
    activate = CANONICAL_OPEN.index("select_scene_index(scene_index)")
    guard = CANONICAL_OPEN.index("opening_scene_builder_ = true;")
    navigate = CANONICAL_OPEN.index("show_studio_page(StudioPage::SceneBuilderPage);")
    refresh = CANONICAL_OPEN.index("load_scene_builder_editor_for_selected_scene(source_action);")
    assert activate < guard < navigate < refresh
    assert "refresh_scene_builder_left_explorer" not in CANONICAL_OPEN
    assert "populate_scene_hierarchy" not in CANONICAL_OPEN
    assert "set_preview_context" not in CANONICAL_OPEN


def test_hidden_legacy_library_list_cannot_promote_home_selection():
    assert "connect(dashboard_library_list_, &QListWidget::currentRowChanged" not in SETUP
    assert "Dashboard double-click" not in MAINWINDOW


def test_home_entry_refresh_does_not_refresh_scene_builder_surfaces():
    assert SETUP.count("refresh_scene_browser_ui();") == 2  # Home entry plus initial discovery.
    assert "refresh_scene_builder_left_explorer();" not in SETUP
    for forbidden in [
        "populate_scene_files_tab",
        "refresh_scene_builder_selection_state_ui",
        "populate_scene_hierarchy",
        "set_preview_context",
    ]:
        assert forbidden not in BROWSER_REFRESH


def test_home_selection_reads_cache_only_and_never_starts_product_view():
    selection = section(
        PREVIEW,
        "QObject::connect(table, &QTableWidget::itemSelectionChanged",
        "#ifdef WORKCELL_BUILDER_HAS_WEBENGINE",
    )
    assert "show_fast_home_preview" in selection
    assert "capture_canonical_product_view_snapshot" not in selection
    assert "new QWebEngineView" not in PREVIEW
    assert "request_embedded_web_product_view_refresh" not in PREVIEW


def test_internal_retry_uri_was_removed_in_favor_of_native_retry_button():
    assert "workcell-retry" not in SCENE_PREVIEW
    assert "preparation_failure_retry" not in SCENE_PREVIEW
    assert "use Retry in the Product View toolbar" in SCENE_PREVIEW
    assert "EmbeddedProductViewState::Failed" in SCENE_PREVIEW
    assert 'QStringLiteral("user_retry")' in SCENE_PREVIEW


def test_selected_cell_is_one_identity_scoped_per_cell_and_remembered():
    core = section(MAINWINDOW, "bool MainWindow::select_scene_index(int scene_index)",
                   "void MainWindow::show_selected_scene_validation_record()")
    for required in ["confirm_scene_change", "leave_cell_session_for(scene_index)", "cancel_mate_for_other_cell(scene_index);",
                     "show_selected_scene_validation_record();", "refresh_task_intent_panel();",
                     "refresh_preview_launch_ui();",
                     "home/last_selected_scene_path"]:
        assert required in core
    # Editor state (canvas, hierarchy, Product View) loads only when Scene Builder is shown.
    for forbidden in ["refresh_scene_builder_left_explorer", "populate_scene_hierarchy", "set_preview_context"]:
        assert forbidden not in core
    record = section(MAINWINDOW, "void MainWindow::show_selected_scene_validation_record()",
                     "void MainWindow::refresh_selected_scene_details_card()")
    assert "validation_state_.clear();" in record and "scene_content_readiness" in record
    assert "scene_builder_editor_scene_path_ != selected_scene_path()" in SETUP
    assert "home/last_selected_scene_path" in TARGET
