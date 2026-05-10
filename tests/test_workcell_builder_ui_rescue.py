from pathlib import Path


def test_main_workflow_hides_unwired_shell_controls_and_focuses_actions():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for snippet in [
        'ui->asset_browser_group->hide();',
        'ui->inspector_group->hide();',
        'ui->cell_name->hide();',
        'ui->output_folder->hide();',
        'ui->browse_output_folder->hide();',
        'ui->delete_scene->hide();',
        'ui->generate_studio_pack->hide();',
        'ui->open_preview->hide();',
        'ui->show_readiness_report->hide();',
    ]:
        assert snippet in cpp


def test_scene_folder_status_is_compact_and_user_facing():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'Scenes folder: %1\\nScenes found: %2\\nLast refresh: %3' in cpp
    assert 'display_path_with_home_tilde(scenes_path)' in cpp
