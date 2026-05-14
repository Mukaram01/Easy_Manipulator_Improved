from pathlib import Path

def test_ui_strings_present():
    text = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in [
        'Preview Launch',
        'Run Fake-Hardware Preview',
        'Stop Preview',
        'Copy Fake-Hardware Launch Command',
        'use_fake_hardware:=true',
        'Fake hardware only',
        'No robot motion commanded',
    ]:
        assert token in text
