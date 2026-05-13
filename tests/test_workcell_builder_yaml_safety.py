from pathlib import Path

SCENE_SELECT_CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_load_scene_from_yaml_has_exception_guards():
    assert 'Invalid scene YAML:' in SCENE_SELECT_CPP
    assert 'catch (const YAML::Exception & error)' in SCENE_SELECT_CPP
    assert 'root must be a map' in SCENE_SELECT_CPP


def test_yaml_node_type_checks_before_iteration():
    assert 'objects must be a map' in SCENE_SELECT_CPP
    assert 'external joints must be a map' in SCENE_SELECT_CPP
    assert 'in_ext_joints.IsMap()' in SCENE_SELECT_CPP
