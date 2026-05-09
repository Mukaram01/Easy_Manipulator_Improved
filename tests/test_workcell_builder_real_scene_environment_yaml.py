from pathlib import Path


def test_scene_package_generation_creates_environment_yaml_and_manifest_hooks():
    content = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'ensure_minimal_environment_yaml(scene_dir, scene_name);' in content
    assert 'refresh_scene_manifest_if_missing(scene_dir, scene_name);' in content
    assert 'README.builder.md' in content
