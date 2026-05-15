from pathlib import Path


def test_task_intent_panel_wiring_and_loader_paths():
    cpp = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()
    assert 'config"/"workcell_builder_task_intent.yaml' in cpp
    assert 'config"/"task_recipe.yaml' in cpp
    assert 'scene_dir/"task_recipe.yaml"' in cpp
    assert 'scene_dir/"cell_definition.yaml"' in cpp
    assert 'scene_dir/"environment_layout.yaml"' in cpp

    for label in [
        'Task Intent',
        'Pick-Place Configuration',
        'Grasp Strategy',
        'Approach & Retreat',
        'Preview Actions',
    ]:
        assert label in cpp

    for action in [
        'Validate Task Intent',
        'Generate/Update Task Intent',
        'Open Task File',
        'Copy Task Summary',
        'Preview Offline Plan',
    ]:
        assert action in cpp

    assert 'create_or_update_builder_task_intent.py' in cpp
    assert 'Fake Hardware' in cpp
    assert 'No Robot Motion' in cpp
    assert 'Preview Only' in cpp
