from pathlib import Path
from scripts import workcell_builder_gui_workflow as wf


def test_new_cell_creation_creates_required_files(tmp_path):
    result = wf.create_new_cell('new_scene', tmp_path)
    assert result['ok']
    scene = Path(result['scene_dir'])
    for name in ['environment.yaml', 'scene_manifest.yaml', 'package.xml', 'CMakeLists.txt']:
        assert (scene / name).exists(), name


def test_invalid_cell_name_rejected(tmp_path):
    result = wf.create_new_cell('Bad Scene', tmp_path)
    assert not result['ok']


def test_generate_files_from_yaml_requires_environment(tmp_path):
    scene = tmp_path / 'new_scene'
    scene.mkdir()
    result = wf.generate_files_from_yaml(scene)
    assert not result['ok']
    assert 'environment.yaml' in result['error']


def test_generate_files_from_yaml_returns_build_command(tmp_path):
    scene = tmp_path / 'new_scene'
    wf.create_new_cell('new_scene', tmp_path)
    result = wf.generate_files_from_yaml(scene)
    assert result['ok']
    assert '--packages-select new_scene' in result['build_command']
