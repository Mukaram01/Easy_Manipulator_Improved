from pathlib import Path

def test_workflow_sections_and_search_widgets_exist():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    for section in ['Start','Ingredients','Layout','Task','Perception ROI','Grasp','Validate &amp; Generate']:
        assert section in ui
    for name in ['asset_search_filter','asset_browser_group','inspector_group','recommended_starter_list','fake_hardware_default_label']:
        assert f'name="{name}"' in ui
    assert 'EPD' not in ui
