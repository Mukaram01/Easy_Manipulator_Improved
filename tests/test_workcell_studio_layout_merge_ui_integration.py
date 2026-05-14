from pathlib import Path

MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_layout_merge_actions_present_in_ui():
    for token in ['Run Layout Merge', 'Open Merge Report', 'Copy Merge Summary']:
        assert token in MAIN_CPP
