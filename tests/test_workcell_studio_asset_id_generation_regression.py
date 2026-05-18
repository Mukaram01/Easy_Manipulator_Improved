from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_commit_armed_asset_placement_uses_shared_id_generator_with_layout_merge():
    required_tokens = [
        'void MainWindow::commit_armed_asset_placement',
        'std::set<std::string> reserved_ids;',
        'gi->data(RoleId).toString().trimmed()',
        'selected_scene_environment_layout_path(scene_browser_result_, selected_scene_index_)',
        'workcell_builder::workcell_studio_collect_layout_ids(layout_path)',
        'reserved_ids.insert(layout_ids.begin(), layout_ids.end());',
        'workcell_builder::workcell_studio_next_id(category.toStdString(), reserved_ids)',
        'item->setData(RoleId, new_id);',
        'item->setData(RoleType, role_type);',
        'item->setData(RoleDisplayName, display_name);',
    ]
    for token in required_tokens:
        assert token in CPP


def test_no_local_id_suffix_loop_in_commit_armed_asset_placement():
    forbidden_tokens = [
        'const QString prefix = id_prefix_from_category(category);',
        'int suffix = 1;',
        'do { new_id = QString("%1_%2")',
    ]
    for token in forbidden_tokens:
        assert token not in CPP


def test_arm_place_asset_mode_only_arms_state_not_id_generation():
    arm_start = CPP.index('void MainWindow::arm_place_asset_mode')
    commit_start = CPP.index('void MainWindow::commit_armed_asset_placement')
    arm_block = CPP[arm_start:commit_start]
    assert 'workcell_studio_next_id' not in arm_block
    assert 'workcell_studio_collect_layout_ids' not in arm_block
