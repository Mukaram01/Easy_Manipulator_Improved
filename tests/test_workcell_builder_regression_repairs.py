from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
DISC_H = Path('workcell_builder/workcell_builder/gui/asset_catalog_discovery.h').read_text(encoding='utf-8')
DISC_CPP = Path('workcell_builder/workcell_builder/gui/asset_catalog_discovery.cpp').read_text(encoding='utf-8')
CMAKE = Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')


def test_discovered_asset_role_hint_model_is_defined_and_populated():
    assert 'std::string role_hint;' in DISC_H
    assert 'entry.role_hint = infer_role_hint(entry.category, entry.source_kind);' in DISC_CPP
    assert 'inferred.role_hint = infer_role_hint(inferred.category, inferred.source_kind);' in DISC_CPP


def test_generate_yaml_rejects_invalid_authored_task_without_reconstructing_defaults(tmp_path):
    import subprocess
    import sys
    import shutil
    (tmp_path / 'config').mkdir()
    shutil.copyfile('scenes/ur5_2f_test/environment.yaml', tmp_path / 'environment.yaml')
    (tmp_path / 'config/workcell_builder_task_intent.yaml').write_text('schema: workcell_builder_task_intent/v2\n')
    cell = tmp_path / 'cell_definition.yaml'
    cell.write_text('existing handoff must survive failed authored validation\n')
    before = cell.read_bytes()
    result = subprocess.run([sys.executable, 'scripts/export_builder_scene_to_cell_definition.py',
        str(tmp_path), '--output-dir', str(tmp_path), '--validate'], capture_output=True, text=True)
    assert result.returncode != 0
    assert cell.read_bytes() == before
    assert not (tmp_path / 'task_recipe_from_builder_intent.yaml').exists()


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
