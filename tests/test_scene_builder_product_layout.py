from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "workcell_builder/workcell_builder/gui/main.cpp"
MAINWINDOW = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
LAYOUT = ROOT / "workcell_builder/workcell_builder/gui/scene_builder_product_layout.hpp"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_product_layout_is_applied_explicitly_from_mainwindow_wrapper():
    main = text(MAIN)
    assert '#include "gui/scene_builder_product_layout.hpp"' in main
    assert "configure_scene_builder_product_layout(this)" in main


def test_product_layout_has_one_primary_scene_toolbar():
    source = text(LAYOUT)
    for token in (
        'QStringLiteral("Select")',
        'QStringLiteral("Move")',
        'QStringLiteral("Rotate")',
        'QStringLiteral("Place Asset")',
        'QStringLiteral("Add Object")',
        'QStringLiteral("Fit")',
        'QStringLiteral("Undo")',
        'QStringLiteral("Redo")',
        'QStringLiteral("Save Layout")',
        'overflow->setText(QStringLiteral("View"))',
        'sceneBuilderEmbeddedPreviewChrome',
        'previewToolbarChip',
    ):
        assert token in source


def test_toolbar_groups_edit_modes_and_makes_save_primary_only_when_dirty():
    layout = text(LAYOUT)
    mainwindow = text(MAINWINDOW)
    for token in (
        'sceneBuilderEditModeGroup',
        'sceneBuilderModeSelect',
        'sceneBuilderModeMove',
        'sceneBuilderModeRotate',
        'layout->addSpacing(10)',
        'sceneBuilderToolbarUndo',
        'sceneBuilderToolbarRedo',
        'sceneBuilderToolbarSave[layoutDirty="true"]',
        'sceneBuilderToolbarSave[layoutDirty="false"]',
    ):
        assert token in layout
    assert 'save_layout_button_->setProperty("layoutDirty", true)' in mainwindow
    assert 'save_layout_button_->setProperty("layoutDirty", false)' in mainwindow


def test_product_layout_keeps_duplicate_contextual_without_orphan_button_overlap():
    source = text(LAYOUT)
    for token in (
        'QPushButton * duplicate = button_with_text(host, QStringLiteral("Duplicate"))',
        'duplicate->setFixedSize(0, 0)',
        'duplicate->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed)',
        'duplicate->setFocusPolicy(Qt::NoFocus)',
    ):
        assert token in source


def test_scene_item_context_menu_is_hierarchy_owned_not_3d_owned():
    layout = text(LAYOUT)
    mainwindow = text(MAINWINDOW)

    for token in (
        'keep_authoring_context_menu_in_hierarchy',
        'embedded_authoring_context_menu_requested(QPoint)',
        'customContextMenuRequested(QPoint)',
        'preview->setContextMenuPolicy(Qt::NoContextMenu)',
        'sceneBuilderHierarchyOnlyContextMenuFilter',
        'event->type() == QEvent::ContextMenu',
    ):
        assert token in layout

    # Preserve the real hierarchy menu and its item actions.
    assert 'scene_hierarchy_tree_->setContextMenuPolicy(Qt::CustomContextMenu)' in mainwindow
    assert '&QTreeWidget::customContextMenuRequested' in mainwindow
    assert '&MainWindow::show_scene_hierarchy_context_menu' in mainwindow
    assert 'else if (chosen == duplicate) duplicate_selected_item();' in mainwindow
    assert 'else if (chosen == remove) delete_selected_item();' in mainwindow
    assert 'if (chosen == rename) rename_selected_item();' in mainwindow


def test_duplicate_command_still_uses_live_authoring_path_and_global_shortcut():
    source = text(MAINWINDOW)
    for token in (
        'duplicate_action->setShortcut(QKeySequence(QStringLiteral("Ctrl+D")))',
        'duplicate_action->setShortcutContext(Qt::ApplicationShortcut)',
        'void MainWindow::duplicate_selected_item()',
        'scene_preview_widget_->duplicate_authoring_item(target.state.id, copy)',
        'apply_scene_selection(new_id, copy.role, false, false)',
        'mark_layout_dirty("Duplicate Selected")',
    ):
        assert token in source


def test_product_layout_promotes_major_tabs_and_simplifies_hierarchy():
    source = text(LAYOUT)
    for token in (
        'sceneBuilderMajorTabs',
        'QStringLiteral("Scene")',
        'QStringLiteral("Assets")',
        'QStringLiteral("Workflow")',
        'source_tabs->tabBar()->hide()',
        'sceneBuilderHierarchySearch',
        'tree->setHeaderHidden(true)',
        'tree->setColumnHidden(1, true)',
        'tree->setColumnHidden(2, true)',
        'studioSelectedItemCard',
        'selected_card->hide()',
        'sceneBuilderLayersGroup',
        'group->setChecked(false)',
    ):
        assert token in source


def test_product_layout_prioritizes_the_3d_viewport():
    source = text(LAYOUT)
    for token in (
        'left->setMinimumWidth(240)',
        'left->setMaximumWidth(340)',
        'center->setMinimumWidth(760)',
        'right->setMinimumWidth(300)',
        'right->setMaximumWidth(400)',
        'splitter->setStretchFactor(1, 10)',
        'splitter->setSizes({280, 1040, 340})',
    ):
        assert token in source


def test_inspector_position_controls_accept_negative_world_coordinates_only():
    source = text(MAINWINDOW)
    assert "for (auto * position_spin : {inspector_x_, inspector_y_, inspector_z_})" in source
    assert "position_spin->setRange(-1000.0, 1000.0)" in source
    assert "inspector_dim_x_->setRange(-1000.0, 1000.0)" not in source


def test_product_layout_removes_redundant_scene_and_debug_chrome():
    source = text(LAYOUT)
    for token in (
        'sceneBuilderCompactSceneIdentity',
        'text.startsWith(QStringLiteral("Scene Builder:"))',
        'text.startsWith(QStringLiteral("Unsaved Layout Edits:"))',
        'text.startsWith(QStringLiteral("Legend:"))',
        'text.startsWith(QStringLiteral("Scene load:"))',
        'sceneBuilderActivityStrip',
        'digital_twin_minimap',
    ):
        assert token in source


def test_product_layout_uses_clickable_activity_strip_and_existing_drawer():
    layout = text(LAYOUT)
    mainwindow = text(MAINWINDOW)
    assert 'sceneBuilderBottomStatusBar' in layout
    assert 'bottom->hide()' not in layout
    for token in (
        'sceneBuilderActivityStrip',
        'sceneBuilderLogDrawerHeader',
        'sceneBuilderLogDrawer',
        'scene_builder_log_panel_->setVisible(show)',
        'studio_log_->setVisible(show)',
    ):
        assert token in mainwindow
    assert 'sceneBuilderLogsButton' not in mainwindow


def test_asset_library_keeps_primary_workflow_and_large_cached_preview():
    source = text(MAINWINDOW)
    for token in (
        'asset_library_result_count_->setText(QString::number(visible_count))',
        'asset_filter_combo_->addItem("All assets")',
        'setObjectName("assetLibraryResetFilters")',
        'setObjectName("sceneBuilderPlaceAsset")',
        'new QPushButton("Details"',
        'setText(QString::fromUtf8("More ▾"))',
        'setObjectName("assetLibrarySelectedPreview")',
        'asset_library_selected_preview_->setMinimumHeight(240)',
        'asset_library_selected_preview_->setMaximumHeight(280)',
        'asset_library_selected_preview_->hide()',
        'asset_thumbnail_service_->request(request)',
        'setObjectName("assetLibraryPlaceAgainAction")',
    ):
        assert token in source
    assert 'new ScenePreviewWidget(catalog_card' not in source
    assert 'addAction("Preview settings")' not in source
    assert 'new QPushButton("Clear", catalog_card)' not in source
    assert 'Select an asset card to inspect available metadata.' not in source
    assert 'Select an asset to preview.' not in source


def test_selected_asset_preview_restores_thumbnail_service_contract():
    source = text(MAINWINDOW)
    update = source.split("void MainWindow::update_asset_library_preview()", 1)[1].split(
        "void MainWindow::on_hierarchy_item_selected", 1
    )[0]
    assert "resolve_visual_mesh_source_path(" in update
    assert "AssetThumbnailService::Request request" in update
    assert "e.scale, QSize(256, 192)" in update
    assert "asset_thumbnail_service_->request(request);" in update
    assert "refresh_asset_thumbnail(e.asset_id);" in update
    assert "asset_library_selected_preview_->show();" in update
    assert "asset_library_selected_preview_->hide();" in update
    assert "ScenePreviewWidget::PreviewItem" not in update


def test_product_layout_keeps_one_contextual_inspector_surface():
    source = text(LAYOUT)
    for token in (
        'QStringLiteral("Selection"), QStringLiteral("Workflow"), QStringLiteral("Readiness")',
        'tabs->setTabText(0, QStringLiteral("Inspector"))',
        'tabs->setTabText(1, QStringLiteral("Task"))',
        'tabs->setTabText(2, QStringLiteral("Checks"))',
        'card->hide()',
        'scene_builder/right_panel_visible',
    ):
        assert token in source


def test_product_layout_does_not_reintroduce_runtime_polish_or_motion_paths():
    source = text(LAYOUT)
    forbidden = (
        "Q_COREAPP_STARTUP_FUNCTION",
        "QTimer",
        "QApplication::topLevelWidgets",
        "setParent(",
        "QWebEngineView",
        "execute_trajectory",
        "real_hardware_enabled",
        "ros2 launch",
        'environment.yaml").write',
        'scene_manifest.yaml").write',
    )
    for token in forbidden:
        assert token not in source

    # One scoped viewport filter is intentional: it consumes only context-menu
    # events so the legacy native canvas cannot reopen the 3D authoring menu.
    assert source.count("installEventFilter(") == 1
    assert "HierarchyOnlyContextMenuFilter" in source
