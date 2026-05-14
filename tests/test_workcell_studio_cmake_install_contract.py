from pathlib import Path


def test_cmake_contains_required_sources_and_resources():
    text = Path("workcell_builder/workcell_builder/CMakeLists.txt").read_text(encoding="utf-8")
    for token in [
        "src_workcell_studio_canvas_model.cpp",
        "src_workcell_studio_layout_editor.cpp",
        "src_workcell_studio_layout_merge.cpp",
        "install(DIRECTORY gui/resources",
    ]:
        assert token in text
