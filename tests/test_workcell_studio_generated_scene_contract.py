from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_scene_select_uses_real_instantiator_backend():
    src = (ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
    assert "instantiate_workcell_studio_template" in src
    assert "WorkcellStudioTemplateInstantiationRequest" in src


def test_cmake_wires_instantiator_source():
    cmake = (ROOT / "workcell_builder/workcell_builder/CMakeLists.txt").read_text(encoding="utf-8")
    assert "src_workcell_studio_template_instantiator.cpp" in cmake
