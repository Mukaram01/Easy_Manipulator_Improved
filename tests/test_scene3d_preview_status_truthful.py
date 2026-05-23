from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')


def test_preview_workflow_status_not_missing_or_unavailable_when_active_rendered_items_exist():
    assert 'workflow_preview_status' in CPP
    assert 'preview_status_untruthful' in CPP
    assert 'header_preview_status' in CPP
    assert 'counters.value("rendered_count").toInt() > 0 || counters.value("viewport_received_count").toInt() > 0' in CPP


def test_selected_item_none_is_warning_only_not_blocker():
    assert 'if (selected_item_id == "(none)") warnings_.append("no_item_selected_by_default")' in CPP
    assert 'blockers_.append("no_item_selected_by_default")' not in CPP
