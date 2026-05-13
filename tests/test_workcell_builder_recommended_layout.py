from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_recommended_layout_writes_metadata_file():
    assert 'recommended_layout.yaml' in CPP
    assert 'layout_profile: ur5_2f_safe_defaults' in CPP
    assert 'realsense_d435i' in CPP


def test_recommended_layout_next_action_message_present():
    assert 'Next recommended action: Validate Scene, then Generate Full Scene Package.' in CPP
