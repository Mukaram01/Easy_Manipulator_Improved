from pathlib import Path


def test_symlink_scene_root_logging_and_resolution_supported():
    content = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'Selected scenes directory:' in content
    assert 'Resolved scenes directory:' in content
    assert 'fs::canonical(scenes_path' in content
    assert 'Scene package:' in content
