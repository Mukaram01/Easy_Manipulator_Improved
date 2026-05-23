from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT/'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_preview_chip_not_unavailable_when_rendered_items_exist_tokens_present():
    assert 'render_debug_counters()' in CPP
    assert 'generated_fallback_count' in CPP
    assert 'Preview: %1' in CPP
    assert 'Fallback' in CPP
