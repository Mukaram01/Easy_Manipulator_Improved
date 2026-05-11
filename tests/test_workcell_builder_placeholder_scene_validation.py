from pathlib import Path


def test_placeholder_robot_is_blocked_before_lookup_and_generation():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'is_placeholder_value' in cpp
    assert 'Scene is incomplete: select a robot before generating.' in cpp


def test_placeholder_end_effector_none_is_not_resolved_as_package():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'if (ee_name == "none" || ee_name.empty()) {' in cpp
    assert 'continue;' in cpp


def test_scaffold_and_incomplete_status_wording_present():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'Scene status: environment.yaml found, but scene is incomplete.' in cpp
    assert 'SCAFFOLD_ONLY/INCOMPLETE' in cpp


def test_valid_scene_success_wording_requires_validity():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'Scene valid: environment.yaml, robot, and MoveIt config found.' in cpp
