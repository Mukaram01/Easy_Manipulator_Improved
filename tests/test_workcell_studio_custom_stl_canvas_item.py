from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_custom_stl_tokens_present():
    for t in ['Import STL / URDF','Add Existing STL to Canvas','Generate Simple Box/Cylinder Placeholder']:
        assert t in CPP
