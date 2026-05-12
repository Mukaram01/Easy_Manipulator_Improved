from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / 'workcell_builder' / 'workcell_builder' / 'gui' / 'scene_select.ui'


def test_ui_contains_guided_conveyor_sorting_workflow_actions():
    text = UI.read_text()
    required = [
        'Create Scenario',
        'Conveyor Sorting - Live EPD Preview',
        'Use Recommended Layout',
        'Generate Scenario',
        'Copy Sample EPD Command',
        'Export Scenario Bundle',
    ]
    for label in required:
        assert label in text, label
