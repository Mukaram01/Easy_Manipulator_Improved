from pathlib import Path


def test_generate_files_requires_environment_yaml_before_proceeding():
    content = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'environment.yaml missing. Click Generate YAML files for scene first.' in content
    assert 'load_scene_from_yaml(&curr_scene)' in content
