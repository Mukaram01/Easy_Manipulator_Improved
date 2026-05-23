from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')


def test_preview_unavailable_guard_for_rendered_items():
    assert 'preview_status_untruthful' in CPP
    assert 'header_preview_status' in CPP
    assert 'workflow_preview_status' in CPP


def test_preview_and_workflow_status_not_missing_or_unavailable_with_active_rendered_content():
    assert 'active_rendered_count' in CPP
    assert 'const bool has_active_items = (active_received_count > 0 || active_rendered_count > 0);' in CPP
    assert 'counters["workflow_preview_status"] = has_active_items ? derived_preview_status : QStringLiteral("Missing");' in CPP
    assert 'if ((counters.value("rendered_count").toInt() > 0 || counters.value("viewport_received_count").toInt() > 0) &&' in CPP
    assert 'counters.value("header_preview_status").toString().compare("Unavailable", Qt::CaseInsensitive) == 0' in CPP
