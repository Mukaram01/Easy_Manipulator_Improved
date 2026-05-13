from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/conveyor_sorting_scenario_wizard.cpp').read_text()

def test_generated_files_tokens():
    for token in ['environment.yaml', 'config" / "environment.yaml', 'scenario.yaml', 'demo.launch.py', 'demo.rviz', '.urdf.xacro', '"preview"']:
        assert token in CPP
