from pathlib import Path


def test_required_checks_present():
    cpp = Path('workcell_builder/workcell_builder/src_offline_smoke_check_model.cpp').read_text()
    assert 'environment.yaml missing' in cpp
    assert 'task_recipe.yaml missing' in cpp
    assert 'fake_hardware_first' in cpp
    assert 'demo.rviz missing (optional)' in cpp
    assert 'use_fake_hardware:=true' in cpp
