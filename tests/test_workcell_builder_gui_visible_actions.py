from pathlib import Path

REQUIRED = [
    'Validate Cell','Generate Canonical Files','Generate Workcell Package','Generate Studio Pack',
    'Open Preview','Open Output Folder','Show Readiness Report','Copy Fake-Hardware Launch Command',
    'Refresh Asset Catalog'
]

def test_visible_actions_in_scene_select_ui():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    for action in REQUIRED:
        assert action in ui
    assert 'EPD' not in ui
