from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.h").read_text(encoding="utf-8")
PREVIEW = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")


def test_compact_card_sits_between_existing_hierarchy_and_layers():
    setup = MAIN.split('hierarchy_layout->addWidget(scene_hierarchy_tree_);', 1)[1]
    setup = setup.split('hierarchy_layout->addWidget(preview_layers_group);', 1)[0]
    assert 'studioSelectedItemCard' in setup
    assert '<b>Selected Item</b>' in setup
    assert 'new QGroupBox("Layers", hierarchy_card)' in MAIN
    assert "QDockWidget" not in setup


def test_hierarchy_and_web3d_share_stable_id_selection():
    hierarchy = MAIN.split("void MainWindow::on_hierarchy_item_selected", 1)[1].split(
        "void MainWindow::refresh_selected_item_card", 1
    )[0]
    assert "TreeRoleId" in hierarchy
    assert "apply_scene_selection(selected_id" in hierarchy
    assert "select_preview_item(selected_id)" in MAIN
    assert "__WORKCELL_EDITOR_API_V1__&&window.__WORKCELL_EDITOR_API_V1__.selectItem" in PREVIEW
    assert "scene_hierarchy_tree_->setCurrentItem(matched)" in MAIN


def test_overlay_selection_is_inspect_only_and_can_enable_hidden_layer():
    assert 'preview_item->source_layer == QStringLiteral("overlay")' in MAIN
    assert "preview_layer_overlays_helpers_box_->setChecked(true)" in MAIN
    assert 'preview_item.target_ref = semantic_yaml_scalar(node["target_ref"])' in MAIN
    assert "preview_item.editable = false" in MAIN
    assert "preview_item.locked = true" in MAIN
    assert "This area follows %1. Move the bin instead." in MAIN
    assert "preview_item_by_id(browser_selected_id) != nullptr" in PREVIEW


def test_card_is_friendly_and_contextual_without_developer_metadata():
    card = MAIN.split("void MainWindow::refresh_selected_item_card()", 1)[1].split(
        "void MainWindow::apply_scene3d_product_view_layer_defaults", 1
    )[0]
    for text in [
        "Editable physical item",
        "Read-only destination area",
        "Generated preview · read-only",
        "Position derived from %1",
        "Destination group: %1",
        "Select an item in the hierarchy or 3D view.",
    ]:
        assert text in card
    for technical in ["source_layer", "render_policy", "source_path", "mesh_load_warning"]:
        assert f'QStringLiteral("{technical}' not in card
    assert "scene_move_mode_button_" in HEADER
    assert "scene_rotate_mode_button_" in HEADER


def test_selection_poll_remains_scene_and_request_token_guarded():
    poll = PREVIEW.split("void ScenePreviewWidget::poll_embedded_editor_events()", 1)[1].split("#else", 1)[0]
    assert "embedded_web_identity_is_current(identity)" in poll
    assert "state_request_token != embedded_editor_state_request_token_" in poll
    assert "browser_scene_id == identity.scene_id" in poll
