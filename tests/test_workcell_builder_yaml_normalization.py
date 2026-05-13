from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_recommended_layout_yaml_is_safe_and_reloadable_markers_present():
    assert 'runtime_execution_enabled: false' in CPP
    assert 'fake_hardware_first: true' in CPP
