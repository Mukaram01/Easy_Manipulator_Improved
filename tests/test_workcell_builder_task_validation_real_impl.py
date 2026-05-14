from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_validation_logic_present():
    for tok in ['pick source missing','place target missing','approach distance must be > 0','preview-only/runtime disabled']:
        assert tok in CPP
