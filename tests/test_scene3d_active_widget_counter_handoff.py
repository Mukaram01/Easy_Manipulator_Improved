from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')
HDR = Path('workcell_builder/workcell_builder/gui/mainwindow.h').read_text(encoding='utf-8')


def test_active_accessor_contract_present():
    for token in ['active_scene_preview_widget()', 'refresh_scene_builder_state_from_active_scene()']:
        assert token in HDR or token in CPP


def test_smoke_uses_active_widget_counter_source_and_json_keys():
    assert 'viewport_counter_source' in CPP
    assert 'active_viewport_counter_handoff_failed' in CPP
    assert 'root["viewport_candidates"]' in CPP
    assert 'root["active_viewport_candidate_index"]' in CPP


def test_selected_item_none_stays_warning_only_and_not_blocker():
    assert 'QString selected_item_id = "(none)";' in CPP
    assert 'if (selected_item_id == "(none)") warnings_.append("no_item_selected_by_default")' in CPP
    assert 'blockers_.append("no_item_selected_by_default")' not in CPP
