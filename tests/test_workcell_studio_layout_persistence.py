from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / 'workcell_builder/workcell_builder/src_workcell_studio_layout_editor.cpp').read_text()
HPP = (ROOT / 'workcell_builder/workcell_builder/include/workcell_studio_layout_editor.hpp').read_text()
CMAKE = (ROOT / 'workcell_builder/workcell_builder/CMakeLists.txt').read_text()


def test_layout_persistence_helper_exists_and_is_registered():
    assert 'persist_workcell_studio_layout' in CPP
    assert 'persist_workcell_studio_layout' in HPP
    assert 'src_workcell_studio_layout_editor.cpp' in CMAKE


def test_safety_flags_and_gripper_rpy_tokens_preserved():
    for token in ['fake_hardware_first: true', 'runtime_execution_enabled: false', 'motion_command_sent: false', '-1.5708 -1.5708 0']:
        assert token in CPP


def test_malformed_yaml_warning_token_exists():
    assert 'malformed YAML returns warning, not crash' in CPP
