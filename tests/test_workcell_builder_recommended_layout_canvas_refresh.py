from pathlib import Path

def test_recommended_layout_refresh_markers_present():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for token in ['Layout applied: recommended layout saved','Next: Validate Scene','refresh_preview_status();']:
        assert token in cpp
