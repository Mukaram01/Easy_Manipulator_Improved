from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_no_manual_double_wiring_for_autoconnect_slots():
    banned = [
        'connect(ui->create_scenario_template',
        'connect(ui->open_scene_folder',
        'connect(ui->use_recommended_layout',
    ]
    for token in banned:
        assert token not in CPP


def test_slots_still_exist_for_autoconnect():
    assert 'void SceneSelect::on_create_scenario_template_clicked()' in CPP
    assert 'void SceneSelect::on_open_scene_folder_clicked()' in CPP
