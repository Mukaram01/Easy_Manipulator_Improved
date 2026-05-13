from pathlib import Path
HPP = Path('workcell_builder/workcell_builder/include/workcell_scene_status.hpp').read_text()
CPP = Path('workcell_builder/workcell_builder/src_workcell_scene_status.cpp').read_text()
GUI = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_lifecycle_gating_helpers_present():
    for fn in ['lifecycle_allows_recommended_layout','lifecycle_allows_validate','lifecycle_allows_generate','lifecycle_allows_export']:
        assert fn in HPP
        assert fn in CPP

def test_export_and_launch_guidance_messages_present():
    assert 'Create or save scene YAML, then click Generate Full Scene Package.' in CPP
    assert 'Next recommended action: click Generate Full Scene Package.' in GUI
