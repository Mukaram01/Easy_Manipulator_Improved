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
    assert '{"Robots", "robot"}' in setup
    assert '{"Imported", "imported"}' in setup
    filt = _between('void MainWindow::on_asset_filter_changed', 'void MainWindow::update_asset_library_preview')
    for field in ['e.display_name', 'e.category', 'e.tags', 'e.asset_id']:
        assert field in filt


def test_asset_library_is_compact_for_the_narrow_scene_builder_side_panel():
    setup = _between('auto * hierarchy_card = new QFrame', 'auto * files_card = new QFrame')
    for contract in [
        'setIconSize(QSize(64, 48))',
        'QTreeWidget::item{height:58px',
        'asset_catalog_tree_->setColumnHidden(1, true);',
        'asset_catalog_tree_->setColumnHidden(2, true);',
        'asset_catalog_tree_->setColumnHidden(3, true);',
        'asset_catalog_tree_->header()->hide();',
        'import_asset_button->setFixedWidth(34);',
        'asset_library_thumbnail_preview_->hide();',
        'asset_library_preview_status_->setMaximumHeight(38);',
    ]:
        assert contract in setup


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
    add = MAIN[MAIN.index('connect_button(add_to_canvas_button_'):MAIN.index('connect_button(add_asset_button_')]
    assert 'data(0, CatalogRoleAssetId).toString().trimmed()' in add
    assert add.count('arm_place_asset_mode(asset_id)') == 1
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


def test_shared_placement_backend_rejects_stale_disabled_and_noneditable_catalog_entries():
    body = _between('bool MainWindow::place_catalog_asset_at_world_position', 'bool MainWindow::configure_asset_placement_transform')
    assert 'asset_id.isEmpty()' in body
    assert 'asset_catalog_entries_.cend()' in body
    assert 'stale catalog identity' in body
    assert '!match->disabled_reason.trimmed().isEmpty()' in body
    assert '!match->editable' in body
    assert body.count('arm_place_asset_mode(') == 1
    assert body.count('commit_armed_asset_placement(') == 1


def test_fresh_import_replaces_stale_selection_by_canonical_asset_id():
    """A1 steps 1-5: refresh must resolve B, never keep previously armed A."""
    helper = _between(
        'QTreeWidgetItem * find_asset_catalog_item_by_id',
        'QString canonical_skip_reason_key',
    )
    imported = _between(
        'void MainWindow::import_stl_to_asset_library()',
        'void MainWindow::open_add_asset_dialog()',
    )

    # Stable-ID lookup walks the refreshed tree and compares the dedicated role,
    # rather than relying on a stale row index, label, or source filename.
    assert 'requested_asset_id.trimmed()' in helper
    assert 'CatalogRoleAssetId' in helper
    assert 'root->child(child_index)' in helper
    assert 'find_asset_catalog_item_by_id(asset_catalog_tree_, asset_id)' in imported

    refresh = imported.index('populate_asset_catalog();')
    lookup = imported.index('find_asset_catalog_item_by_id(asset_catalog_tree_, asset_id)')
    selection = imported.index('asset_catalog_tree_->setCurrentItem(imported_item);')
    identity_check = imported.index(
        'imported_item->data(0, CatalogRoleAssetId).toString().trimmed() != asset_id'
    )
    assert refresh < lookup < selection < identity_check
    assert imported[:refresh].count('clear_armed_asset_placement();') >= 1
    assert 'asset_catalog_tree_->currentItem() != imported_item' in imported

    failure = imported[imported.index('auto block_imported_asset_placement'):lookup]
    assert 'clear_armed_asset_placement();' in failure
    assert 'set_canvas_interaction_mode(CanvasInteractionMode::Select);' in failure
    assert 'asset_catalog_tree_->setCurrentItem(nullptr);' in failure
    assert 'not found after Asset Library refresh' in failure


def test_catalog_identity_b_flows_to_one_editable_layout_item_and_refreshes_product_view():
    """A1 steps 6 and 9-11: B is the only identity consumed by placement."""
    place = _between(
        'bool MainWindow::place_catalog_asset_at_world_position',
        'bool MainWindow::configure_asset_placement_transform',
    )
    commit = _between(
        'void MainWindow::commit_armed_asset_placement',
        'void MainWindow::validate_asset_catalog_selection',
    )

    assert 'arm_place_asset_mode(asset_id)' in place
    assert place.count('commit_armed_asset_placement(') == 1
    assert 'items().size() == item_count_before + 1' in place

    assert 'workcell_studio_next_id' in commit
    assert 'item->setData(RoleSource, source_path);' in commit
    assert 'QStringLiteral("editable_layout")' in commit
    assert 'item->setData(RoleLocked, false);' in commit
    assert commit.count('digital_twin_scene_->addItem(item);') == 1
    assert 'preview_item.mesh_path = source_path;' in commit
    assert 'preview_item.editable = true;' in commit
    assert 'preview_item.locked = false;' in commit
    assert 'undo_stack_.push_back({"add", new_id' in commit
    assert 'redo_stack_.clear();' in commit
    assert 'mark_layout_dirty("Place Asset Mode: Add to 3D Canvas")' in commit
    assert 'select_canvas_item(item);' in commit
    assert 'apply_scene3d_preview_layer_filters(false);' in commit
    assert 'scene_preview_widget_->select_preview_item(new_id);' in commit

    imported = _between(
        'void MainWindow::import_stl_to_asset_library()',
        'void MainWindow::open_add_asset_dialog()',
    )
    assert 'scene_dir / "assets" / "imported"' in imported
    assert 'stem.toStdString() + "." + extension.toStdString()' in imported


def test_a7_mesh_import_has_explicit_units_format_preflight_and_portable_metadata():
    imported = _between(
        'void MainWindow::import_stl_to_asset_library()',
        'void MainWindow::open_add_asset_dialog()',
    )
    helpers = _between('QString import_mesh_format_label', 'void MainWindow::populate_asset_catalog()')

    for extension in ['stl', 'obj', 'dae']:
        assert f'QStringLiteral("{extension}")' in helpers
    for contract in [
        'mesh exceeds the 250 MB import limit',
        'OBJ must contain both vertex and face records',
        'DAE file does not contain a COLLADA document root',
        'symbolic-link sources are not allowed',
    ]:
        assert contract in helpers

    assert 'Source geometry units:' in imported
    assert 'millimetres' in imported and 'centimetres' in imported and 'metres' in imported
    assert 'Scale to metres:' in imported
    assert 'Origin policy: preserve mesh-local origin' in imported
    assert 'Bounds check: %7' in imported
    assert 'required when Product View loads' in imported
    assert 'validated before import' in imported
    assert 'inspect_import_mesh_geometry(source)' in imported
    assert 'geometry has no usable triangle surface' in helpers
    assert 'geometry contains a non-finite transformed vertex' in helpers
    assert 'geometry bounds are collapsed on two or more axes' in helpers
    assert 'Bounds after unit conversion:' in imported
    assert 'Origin to bounds centre:' in imported
    assert 'maximum_dimension < 1e-6 || maximum_dimension > 1000.0' in imported
    for metadata in [
        'source_units',
        'unit_scale_to_m',
        'mesh_origin_policy',
        'bounds_validation',
        'geometry_inspection_backend',
        'geometry_vertex_count',
        'geometry_face_count',
        'source_bounds',
        'dimensions_m',
        'origin_to_bounds_center_m',
        'original_filename',
        'source_sha256',
        'workcell_studio_mesh_import/v2',
    ]:
        assert metadata in imported
    assert 'updated_assets' in imported
    assert 'existing["id"].as<std::string>() != asset_id.toStdString()' in imported
