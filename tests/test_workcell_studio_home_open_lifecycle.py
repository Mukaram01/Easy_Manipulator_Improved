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
BIND_HOME = section(
    MAINWINDOW,
    "void MainWindow::bind_home_target_shell_actions()",
    "bool MainWindow::load_scene_for_scene3d_smoke",
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


def test_home_single_click_is_identity_only_and_never_loads_editor_state():
    selection = section(
        SETUP,
        "connect(dashboard_scene_table_, &QTableWidget::itemSelectionChanged",
        "// The production Home shell filters",
    )
    for required in ["home_selected_scene_id_", "home_selected_scene_path_", "setEnabled(selected)"]:
        assert required in selection
    for forbidden in [
        "select_scene_by_row",
        "refresh_scene_builder_selection_state_ui",
        "populate_scene_hierarchy",
        "set_preview_context",
        "open_home_scene_in_builder",
    ]:
        assert forbidden not in selection


def test_home_population_restores_stable_identity_with_signals_blocked():
    for required in [
        "QSignalBlocker table_blocker(dashboard_scene_table_)",
        "QSignalBlocker library_blocker(dashboard_library_list_)",
        "home_selected_scene_id_",
        "home_selected_scene_path_",
        "Qt::UserRole + 42",
        'findChild<QPushButton *>(QStringLiteral("studioTargetPrimaryAction"))',
        "open_button->setEnabled(restored_home_selection)",
    ]:
        assert required in HOME_POPULATION
    for forbidden in [
        "selected_scene_index_",
        "select_scene_by_row",
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
        "select_scene_by_row",
        "refresh_scene_builder_selection_state_ui",
        "ScenePreviewWidget",
        "Scene3DViewportWidget",
        "QWebEngineView",
    ]:
        assert forbidden not in TARGET
    assert '[refresh_home]() { refresh_home(); }' not in TARGET
    assert 'QMetaObject::invokeMethod(table, "cellClicked"' not in TARGET
    assert 'QMetaObject::invokeMethod(table, "itemSelectionChanged"' not in TARGET


def test_double_click_and_primary_button_call_same_helper_directly_once():
    double_click = section(
        SETUP,
        "connect(dashboard_scene_table_, &QTableWidget::cellDoubleClicked",
        "connect(dashboard_scene_table_, &QTableWidget::itemSelectionChanged",
    )
    assert double_click.count("open_home_scene_in_builder(scene_id, scene_path);") == 1
    assert "dashboard_open_scene_action_->trigger" not in double_click
    assert BIND_HOME.count("open_home_scene_in_builder(scene_id, scene_path);") == 1
    assert "QAction::trigger" not in BIND_HOME
    assert "bind_home_target_shell_actions();" in MAIN
    primary = section(
        TARGET,
        'new QPushButton(QStringLiteral("▣  Open Scene Builder")',
        "const auto add_backed_more_action",
    )
    assert "QAction::trigger" not in primary


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
    assert CANONICAL_OPEN.count("refresh_scene_builder_selection_state_ui();") == 1
    navigate = CANONICAL_OPEN.index("show_studio_page(StudioPage::SceneBuilderPage);")
    activate = CANONICAL_OPEN.index("selected_scene_index_ = scene_index;")
    refresh = CANONICAL_OPEN.index("refresh_scene_builder_selection_state_ui();")
    assert navigate < activate < refresh
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
