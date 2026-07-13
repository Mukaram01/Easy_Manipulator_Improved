from pathlib import Path

CMAKE = Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')
PKG = Path('workcell_builder/workcell_builder/package.xml').read_text(encoding='utf-8')


def test_workcell_studio_sources_in_cmake_once():
    for src in [
        'src_workcell_studio_canvas_model.cpp',
        'src_workcell_studio_layout_editor.cpp',
        'src_workcell_studio_layout_merge.cpp',
    ]:
        assert CMAKE.count(src) >= 1


def test_qt_svg_wiring_present_for_svg_usage():
    assert 'find_package(Qt5 COMPONENTS Widgets Concurrent Svg OpenGL Network REQUIRED)' in CMAKE
    assert 'Qt5::Svg' in CMAKE
    assert 'libqt5svg5-dev' in PKG
