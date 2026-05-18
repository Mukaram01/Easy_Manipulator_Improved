from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
DISC_H = Path('workcell_builder/workcell_builder/gui/asset_catalog_discovery.h').read_text(encoding='utf-8')
DISC_CPP = Path('workcell_builder/workcell_builder/gui/asset_catalog_discovery.cpp').read_text(encoding='utf-8')
CMAKE = Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')


def test_discovered_asset_role_hint_model_is_defined_and_populated():
    assert 'std::string role_hint;' in DISC_H
    assert 'entry.role_hint = infer_role_hint(entry.category, entry.source_kind);' in DISC_CPP
    assert 'inferred.role_hint = infer_role_hint(inferred.category, inferred.source_kind);' in DISC_CPP


def test_generate_yaml_repairs_invalid_existing_cell_definition():
    assert 'existing valid cell_definition.yaml preserved' in CPP
    assert 'invalid cell_definition.yaml backed up and regenerated' in CPP
    assert 'new cell_definition.yaml generated' in CPP
    assert 'validate_cell_definition.py' in CPP


def test_pick_place_yes_path_uses_single_multi_key_write_helper():
    assert 'update_selected_scene_task_intent_bindings("Pick Zone + Pick Source"' in CPP
    assert 'update_selected_scene_task_intent_bindings("Place Zone + Place Target"' in CPP


def test_more_actions_qactions_wired_and_no_stale_hidden_buttons():
    for token in [
        '&MainWindow::generate_yaml_draft_for_selected_scene',
        '&MainWindow::generate_or_update_task_intent_for_selected_scene',
        '&MainWindow::copy_build_launch_commands_for_selected_scene',
        '&MainWindow::delete_selected_item',
        '&MainWindow::bind_selected_item_as_pick_zone',
        '&MainWindow::bind_selected_item_as_place_zone',
        '&MainWindow::bind_selected_item_as_camera',
    ]:
        assert token in CPP


def test_asset_catalog_discovery_test_links_warning_once_impl():
    assert 'ament_add_gtest(workcell_asset_catalog_discovery_test' in CMAKE
    assert 'src_workcell_warning_once.cpp' in CMAKE
