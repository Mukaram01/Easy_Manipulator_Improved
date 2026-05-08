from pathlib import Path

def test_self_test_flag_exists():
    src = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')
    assert '--self-test-gui' in src
    assert '--gui-acceptance-check' in src
    assert '/tmp/workcell_builder_gui_acceptance_report.json' in src
