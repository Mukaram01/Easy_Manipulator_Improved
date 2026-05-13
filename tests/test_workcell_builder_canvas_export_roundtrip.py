from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text()


def test_tokens_present():
    required = ['export_layout_preview', 'layout_preview.json', 'layout_preview.html', 'layout_preview.svg', 'fake_hardware_first', 'runtime_execution_enabled: false']
    # replaced per-file by sed
    for token in required:
        assert token in CPP or token in UI
