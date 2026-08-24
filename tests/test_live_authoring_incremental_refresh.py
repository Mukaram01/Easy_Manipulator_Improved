from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
LAYOUT = (ROOT / "workcell_builder/workcell_builder/gui/scene_builder_product_layout.hpp").read_text(encoding="utf-8")
CSS = (ROOT / "workcell_studio_web/viewer/style.css").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError(f"unterminated function: {signature}")


def test_live_crud_does_not_rebuild_scene_builder_from_saved_scene():
    for signature in (
        "void MainWindow::duplicate_selected_item()",
        "void MainWindow::delete_selected_item()",
        "void MainWindow::undo_layout_edit()",
        "void MainWindow::redo_layout_edit()",
    ):
        body = function_body(MAIN, signature)
        assert "refresh_scene_builder_left_explorer();" not in body
        assert "refresh_scene_hierarchy_tree_from_current_items();" in body
        assert "refresh_minimap_card();" in body


def test_duplicate_selects_new_owner_before_hierarchy_refresh():
    body = function_body(MAIN, "void MainWindow::duplicate_selected_item()")
    assert "duplicate_authoring_item(target.state.id, copy)" in body
    selection = "apply_scene_selection(new_id, copy.role, false, false);"
    hierarchy = "refresh_scene_hierarchy_tree_from_current_items();"
    assert body.index(selection) < body.index(hierarchy)


def test_delete_updates_duplicate_action_once_after_mutation():
    body = function_body(MAIN, "void MainWindow::delete_selected_item()")
    assert "remove_authoring_item(id)" in body
    assert body.count("refresh_duplicate_selected_action();") == 1


def test_minimap_collapses_for_embedded_web3d_and_remains_available_for_fallback():
    body = function_body(MAIN, "void MainWindow::update_minimap_backend_presentation()")
    assert "embedded_web3d_presented" in body
    assert "setVisible(false)" in body
    assert "setMinimumHeight(0)" in body
    assert "setMaximumHeight(0)" in body
    assert "setMinimumHeight(90)" in body
    assert "setMaximumHeight(90)" in body
    assert "setVisible(minimap_requested_visible_)" in body
    assert 'minimap->show()' not in LAYOUT


def test_inspector_has_one_empty_selection_message_and_no_horizontal_overflow():
    body = function_body(
        MAIN,
        "void MainWindow::refresh_selected_scene_item_labels(const SelectedSceneItemState & state)",
    )
    assert 'inspector_label_->setText("No item selected")' in body
    assert 'live_coordinate_label_->setText("No item selected")' not in body
    assert "live_coordinate_label_->clear();" in body
    assert "live_coordinate_label_->setVisible(false);" in body
    assert "live_coordinate_label_->setVisible(true);" in body
    assert "setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff)" in LAYOUT
    assert "spin->setMinimumWidth(76)" in LAYOUT
    assert "card->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum)" in LAYOUT
    assert "card->setMaximumHeight(520)" in LAYOUT
    assert "card->layout()->setAlignment(Qt::AlignTop)" in LAYOUT
    assert "label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed)" in LAYOUT
    assert 'layout->addStretch(1)' in LAYOUT
    assert 'sceneBuilderTrailingStretchAdded' in LAYOUT


def test_product_layout_compacts_secondary_inspector_and_status_chrome():
    assert 'text.contains(QStringLiteral(">Scene Builder"))' in LAYOUT
    assert 'QStringLiteral("Advanced details")' in LAYOUT
    assert 'QStringLiteral("Robot Base Pose")' in LAYOUT
    assert "group->setMaximumHeight(34)" in LAYOUT
    assert 'QStringLiteral("sceneBuilderBottomStatusBar"))) bottom->hide()' in LAYOUT
    assert "splitter->setSizes({325, 1040, 370})" in LAYOUT


def test_embedded_scene_health_remains_visible_but_compact():
    assert "body.embedded-mode .scene-health" in CSS
    assert "width: min(19rem, calc(100% - 1rem));" in CSS
    assert "body.embedded-mode #scene-health" not in CSS


def test_explicit_full_scene_refresh_remains_available_for_scene_loads():
    body = function_body(MAIN, "void MainWindow::refresh_scene_builder_left_explorer()")
    assert "sync_selected_scene_state();" in body
    assert "rebuild_digital_twin_canvas();" in body
    assert "populate_scene_hierarchy();" in body
    assert "populate_asset_catalog();" in body


def test_hierarchy_keeps_presentation_labels_out_of_canonical_display_names():
    current = function_body(
        MAIN,
        "MainWindow::SelectedSceneItemState MainWindow::current_selected_scene_item() const",
    )
    hierarchy = function_body(
        MAIN,
        "void MainWindow::refresh_scene_hierarchy_tree_from_current_items()",
    )
    canonical = "state.display_name = item->data(0, TreeRoleDisplayName).toString().trimmed();"
    fallback = "if (state.display_name.isEmpty()) state.display_name = item->text(0).trimmed();"
    assert canonical in current
    assert fallback in current
    assert current.index(canonical) < current.index(fallback)
    assert "node->setData(0, TreeRoleDisplayName, p.display_name.trimmed())" in hierarchy


def test_live_web3d_transform_refreshes_hierarchy_pose_text_for_inspector():
    connection = MAIN.index("&ScenePreviewWidget::preview_item_transform_changed")
    callback = MAIN[connection : MAIN.index(
        "&ScenePreviewWidget::embedded_product_view_runtime_state_changed", connection
    )]
    assert "tree_item->setData(0, TreeRolePoseX, x);" in callback
    assert "tree_item->setData(0, TreeRolePoseY, y);" in callback
    assert "tree_item->setData(0, TreeRolePoseZ, z);" in callback
    assert "tree_item->setData(0, TreeRolePoseText" in callback
