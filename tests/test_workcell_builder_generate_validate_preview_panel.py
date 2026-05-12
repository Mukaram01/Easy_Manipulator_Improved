from pathlib import Path


def test_ui_contains_required_studio_status_controls():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    required = [
        'Workcell Studio Scene Status',
        'Refresh Status',
        'Validate Scene',
        'Copy Build Command',
        'Copy Launch Command',
    ]
    for token in required:
      assert token in ui or token in cpp
