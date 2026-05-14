from pathlib import Path

MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
QSS = Path('workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss').read_text(encoding='utf-8')


def test_scene_builder_workspace_tokens_present():
    for token in [
        'Scene Hierarchy',
        'Asset Catalog',
        'Task Intent',
        'Pick-Place Configuration',
        'Grasp Strategy',
        'Approach & Retreat',
        'Validation',
        'Simulation Log',
        'Fake Hardware',
        'No Robot Motion',
        'digital_twin_canvas_',
    ]:
        assert token in MAIN_CPP


def test_scene_builder_uses_left_center_right_layout_structure():
    for token in [
        'left_panel',
        'center_panel',
        'right_panel',
        'scene_top->addWidget(left_panel)',
        'scene_top->addWidget(center_panel, 1)',
        'scene_top->addWidget(right_panel)',
        'bottom_cards',
        'sceneBuilderWorkspace',
    ]:
        assert token in MAIN_CPP


def test_studio_card_and_panel_styling_tokens_present():
    assert 'studioCard' in MAIN_CPP
    assert 'QFrame#studioCard' in QSS
    assert 'QFrame#studioPanel' in QSS
