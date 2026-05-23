from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')
HDR = Path('workcell_builder/workcell_builder/gui/mainwindow.h').read_text(encoding='utf-8')

def test_active_accessor_contract_present():
    for token in ['active_scene3d_viewport_counters()', 'active_scene_preview_widget()', 'refresh_scene_builder_state_from_active_scene()']:
        assert token in HDR or token in CPP

def test_smoke_uses_active_widget_counter_source():
    assert 'viewport_counter_source' in CPP
    assert 'active_viewport_counter_handoff_failed' in CPP
