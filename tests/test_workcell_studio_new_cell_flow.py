from pathlib import Path

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
SCENE = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')


def test_labels_and_handlers_exist():
    for token in [
        'New Cell', 'New Scene', 'Use Recommended Layout', 'Add to Canvas', 'Save Layout',
        'Generate/Update Task Intent', 'Run Offline Validation', 'Open RViz2 / MoveIt',
        'Run Fake-Hardware Simulation', 'Refresh Existing Scenes'
    ]:
        assert token in (MAIN + SCENE) or (token == 'Refresh Existing Scenes' and 'on_refresh_scenes_button_clicked' in SCENE)


def test_checklist_and_defaults_exist():
    for token in [
        'New Cell Checklist', 'Workspace selected', 'Cell name set', 'Robot selected (UR5 default)',
        'Tool selected (Robotiq 2F default)', 'table + pick zone + place zone + camera', 'pick_place'
    ]:
        assert token in MAIN


def test_expected_output_files_referenced():
    merged = MAIN + SCENE
    for token in ['environment.yaml', 'environment_layout.yaml', 'workcell_builder_task_intent.yaml', 'cell_definition.yaml', 'scene_manifest.yaml']:
        assert token in merged


def test_plan_and_simulate_command_contract():
    assert 'ros2 launch' in MAIN
    assert 'demo.launch.py' in MAIN
    assert 'use_fake_hardware:=true' in MAIN
    assert 'launch_rviz:=true' in MAIN
    assert 'use_fake_hardware:=false execution button' not in MAIN


def test_no_dead_buttons_in_new_cell_flow():
    banned = [
        'show_not_wired_message("New Cell")',
        'show_not_wired_message("New Scene")',
        'show_not_wired_message("Use Recommended Layout")',
        'show_not_wired_message("Add to Canvas")',
        'show_not_wired_message("Save Layout")',
        'show_not_wired_message("Generate/Update Task Intent")',
        'show_not_wired_message("Run Offline Validation")',
        'show_not_wired_message("Open RViz2 / MoveIt")',
        'show_not_wired_message("Run Fake-Hardware Simulation")',
    ]
    for token in banned:
        assert token not in MAIN + SCENE


def test_refresh_existing_scenes_after_new_cell_generation():
    assert 'update_new_scene_lifecycle_and_canvas' in SCENE
    assert 'refresh_scenes(' in SCENE
