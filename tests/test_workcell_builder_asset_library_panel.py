from pathlib import Path

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
HDR = Path('workcell_builder/workcell_builder/gui/mainwindow.h').read_text(encoding='utf-8')


def _between(start, end):
    s = MAIN.index(start)
    return MAIN[s:MAIN.index(end, s)]


def test_asset_library_uses_existing_catalog_not_thumbnail_database():
    body = _between('void MainWindow::populate_asset_catalog()', 'void MainWindow::open_add_asset_dialog()')
    assert 'discover_asset_catalog(' in body
    assert 'QDirIterator' not in body
    assert 'thumbnail' not in body.lower()
    assert 'asset_catalog_entries_.push_back(ui_entry);' in body


def test_asset_library_search_and_categories_are_catalog_driven():
    setup = _between('auto * hierarchy_card = new QFrame', 'auto * files_card = new QFrame')
    assert 'Asset Library' in setup
    assert 'assetLibrarySearchBox' in setup
    assert 'asset_filter_combo_->addItem("All")' in setup
    body = _between('void MainWindow::populate_asset_catalog()', 'void MainWindow::open_add_asset_dialog()')
    assert 'categories.insert(ui_entry.category);' in body
    assert 'asset_filter_combo_->addItems(sorted);' in body
    filt = _between('void MainWindow::on_asset_filter_changed', 'void MainWindow::update_asset_library_preview')
    for field in ['e.display_name', 'e.category', 'e.tags', 'e.asset_id']:
        assert field in filt


def test_asset_library_selection_is_separate_from_scene_selection():
    preview = _between('void MainWindow::update_asset_library_preview()', 'void MainWindow::on_hierarchy_item_selected')
    forbidden = ['apply_scene_selection(', 'select_canvas_item(', 'clearSelection()', 'select_preview_item(']
    assert not any(token in preview for token in forbidden)
    assert 'preview.selectable = false;' in preview
    assert 'preview.editable = false;' in preview


def test_asset_library_uses_one_live_preview_and_explicit_failures():
    setup = _between('auto * hierarchy_card = new QFrame', 'auto * files_card = new QFrame')
    assert setup.count('new ScenePreviewWidget(catalog_card)') == 1
    preview = _between('void MainWindow::update_asset_library_preview()', 'void MainWindow::on_hierarchy_item_selected')
    assert 'asset_library_preview_->set_preview_items({preview});' in preview
    assert 'missing_file' in preview
    assert 'mesh_load_warning' in preview
    assert 'Source URI:' in preview


def test_asset_library_add_delegates_to_place_asset_without_direct_yaml_write():
    add = _between('void MainWindow::add_asset_to_canvas_from_catalog', 'QPointF MainWindow::compute_default_canvas_pose')
    assert 'arm_place_asset_mode(category, display_name, source_path);' in add
    assert 'commit_armed_asset_placement' not in add
    assert 'mark_layout_dirty' not in add
    assert '.yaml' not in add.lower()


def test_asset_library_add_button_requires_scene_and_placeable_asset():
    validate = _between('void MainWindow::validate_asset_catalog_selection()', 'QString MainWindow::selected_catalog_item_path')
    assert '!digital_twin_scene_' in validate
    assert 'CatalogRolePlaceable' in validate
    assert 'setToolTip' in validate
    assert 'Start existing Place Asset mode' in validate
    assert 'ScenePreviewWidget * asset_library_preview_' in HDR
