from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

def test_canvas_model_files_and_tokens():
    h = ROOT / 'workcell_builder/workcell_builder/include/workcell_studio_canvas_model.hpp'
    c = ROOT / 'workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp'
    cm = ROOT / 'workcell_builder/workcell_builder/CMakeLists.txt'
    assert h.exists() and c.exists()
    text = c.read_text()
    for t in ['robot','reach','table','conveyor','camera','pick_zone','place_zone','bin','object','warning','Malformed or missing environment.yaml',
              'deterministic_fallback_layout', 'Using deterministic 3D fallback layout because all editable layout sources are missing or invalid.']:
        assert t in text
    cm_text = cm.read_text()
    for token in ['src_workcell_studio_canvas_model.cpp', 'find_package(OpenGL REQUIRED)', 'OpenGL::GL', 'Qt5::Widgets']:
        assert token in cm_text


def test_canvas_model_depth_and_fallback_schema_tokens_present():
    c = (ROOT / "workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp").read_text(encoding="utf-8")
    for token in ["items[].size.depth", "size.depth", "has invalid or missing schema_version", "layout/workcell_studio_layout.yaml is missing"]:
        assert token in c


def test_canvas_model_fallback_role_anchor_tokens_and_log_token_present():
    c = (ROOT / "workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp").read_text(encoding="utf-8")
    for token in [
        "item.role == \"robot\"",
        "item.role == \"table\"",
        "item.role == \"conveyor\"",
        "item.role == \"pick_zone\"",
        "item.role == \"place_zone\"",
        "item.role == \"bin\"",
        "item.role == \"camera\"",
        "item.role.find(\"safety\") != std::string::npos",
        "item.role == \"object\"",
        "Using deterministic 3D fallback layout because all editable layout sources are missing or invalid.",
        "Fallback detail: ",
        "layout/workcell_studio_layout.yaml has incomplete placement metadata",
    ]:
        assert token in c
