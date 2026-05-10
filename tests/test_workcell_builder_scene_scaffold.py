from pathlib import Path


def test_default_paths_and_scaffold_dirs():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert '~/workcell_ws/src/scenes' in ui
    assert 'assets_path = workcell_path / "assets";' in cpp
    assert 'const fs::path scene_urdf_dir = scene_dir / "urdf";' in cpp
    assert 'write_builder_validation_helper(scene_dir);' in cpp


def test_launch_command_uses_fake_hardware_default():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'use_fake_hardware:=true' in cpp
