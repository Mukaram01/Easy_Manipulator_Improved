from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_diagnostics_page_strings_present():
    for needle in ['Diagnostics', 'Run Self-Test', 'Run Golden Flow Dry Run', 'Copy Diagnostics Report']:
        assert needle in CPP
