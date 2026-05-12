from pathlib import Path

def test_ui_strings_present():
    txt = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    for token in [
        'Load Detection Snapshot',
        'Generate Sample EPD Snapshot',
        'Preview Detection Mapping',
        'adapter_metadata_only',
        'no robot motion commanded',
    ]:
        assert token in txt
