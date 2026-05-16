from pathlib import Path


CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
HDR = Path('workcell_builder/workcell_builder/gui/scene_select.h').read_text(encoding='utf-8')


def test_production_helpers_declared_and_defined():
    assert 'open_existing_scene(' in HDR
    assert 'save_scene(' in HDR
    assert 'duplicate_scene(' in HDR
    assert 'regenerate_scene(' in HDR
    assert 'SceneSelect::open_existing_scene(' in CPP
    assert 'SceneSelect::save_scene(' in CPP
    assert 'SceneSelect::duplicate_scene(' in CPP
    assert 'SceneSelect::regenerate_scene(' in CPP


def test_open_edit_cell_calls_production_open_and_save_helpers():
    assert 'open_existing_scene(scene_dir_for_current_selection(), &curr_scene, &open_status)' in CPP
    assert 'save_scene(scene_window.scene, scene_yaml_path, &backup_path)' in CPP


def test_save_scene_creates_timestamped_backup_and_canvas_refresh():
    assert 'environment.yaml.' in CPP and '.bak' in CPP
    assert 'QDateTime::currentDateTimeUtc().toString("yyyyMMddhhmmss")' in CPP
    assert 'refresh_canvas_from_scene(scene_window.scene);' in CPP


def test_scene_selection_hook_updates_inspector_and_canvas_selection():
    main = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    assert 'connect(scene_hierarchy_tree_, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem *item, int column){ Q_UNUSED(column); on_hierarchy_item_selected(item); });' in main
    assert "Role/Category: %3" in main and "inspector_label_->setText(QString(\"Selected: %1" in main
    assert 'if (matched_canvas_item) {\n      digital_twin_scene_->clearSelection();\n      matched_canvas_item->setSelected(true);' in main


def test_selection_sync_hooks_and_standardized_selected_item_log_format():
    main = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in [
        'connect(digital_twin_scene_, &QGraphicsScene::selectionChanged, this, &MainWindow::on_canvas_selection_changed);',
        'if (!digital_twin_scene_ || selection_update_guard_) return;',
        'apply_scene_selection(selected_id, selected_role, false, false);',
        'append_studio_log(QString("Selected item: %1 (%2)").arg(selected_id, selected_role));',
        'append_studio_log("Selected item: <none> (unknown)");',
    ]:
        assert token in main


def test_scene_builder_header_chip_and_path_present():
    main = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in ['sceneStatusChip', 'scene_builder_path_label_', 'Path: (none)']:
        assert token in main
