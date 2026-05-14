from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

def test_canvas_model_files_and_tokens():
    h = ROOT / 'workcell_builder/workcell_builder/include/workcell_studio_canvas_model.hpp'
    c = ROOT / 'workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp'
    cm = ROOT / 'workcell_builder/workcell_builder/CMakeLists.txt'
    assert h.exists() and c.exists()
    text = c.read_text()
    for t in ['robot','reach','table','conveyor','camera','pick_zone','place_zone','bin','object','warning','Malformed or missing environment.yaml']:
        assert t in text
    assert 'src_workcell_studio_canvas_model.cpp' in cm.read_text()
