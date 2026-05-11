from pathlib import Path


def test_compatibility_files_and_cmake_wiring_exist():
    assert Path('workcell_builder/workcell_builder/include/robot_tool_compatibility.hpp').exists()
    assert Path('workcell_builder/workcell_builder/src_robot_tool_compatibility.cpp').exists()
    cmake = Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')
    assert 'src_robot_tool_compatibility.cpp' in cmake


def test_profile_catalog_and_tokens_exist():
    root = Path('workcell_builder/workcell_builder/config/compatibility_profiles')
    assert (root / 'robots').exists()
    assert (root / 'tools').exists()
    assert (root / 'pairs').exists()
    all_text = '\n'.join(p.read_text(encoding='utf-8') for p in root.rglob('*.json'))
    for token in ['ur5', 'robotiq', 'airpick', 'finger', 'suction', 'tcp_frame', 'mount_link', 'planning_group']:
        assert token in all_text.lower()


def test_status_labels_ui_strings_and_readiness_markers_exist():
    compat = Path('workcell_builder/workcell_builder/include/robot_tool_compatibility.hpp').read_text(encoding='utf-8') + Path('workcell_builder/workcell_builder/src_robot_tool_compatibility.cpp').read_text(encoding='utf-8')
    for status in ['COMPATIBLE', 'COMPATIBLE_WITH_WARNINGS', 'UNKNOWN_COMPATIBILITY', 'INCOMPATIBLE', 'MISSING_TCP', 'MISSING_MOUNT_LINK']:
        assert status in compat
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for marker in ['Robot / Tool Compatibility', 'Check Compatibility', 'Apply Profile Defaults', 'Manual Override', 'compatibility_status', 'compatibility_warnings']:
        assert marker in cpp
