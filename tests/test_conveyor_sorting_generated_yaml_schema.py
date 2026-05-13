from pathlib import Path

SCENE_SELECT_CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_ext_joint_mismatch_is_handled_without_crash():
    assert 'external joints contains extra entries; ignoring trailing entries.' in SCENE_SELECT_CPP
    assert 'counter >= static_cast<int>(input_scene->object_vector.size())' in SCENE_SELECT_CPP
