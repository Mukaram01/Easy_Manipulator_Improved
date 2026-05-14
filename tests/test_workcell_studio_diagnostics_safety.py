from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_diagnostics_safety_tokens_present():
    assert 'use_fake_hardware:=true' in CPP
    for token in [
        'use_fake_hardware:=false',
        'real_hardware:=true',
        'runtime_execution_enabled:=true',
        'execute:=true',
    ]:
        assert token in CPP
