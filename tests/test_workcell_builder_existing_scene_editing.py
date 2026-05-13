from pathlib import Path
from tests.workcell_scene_backend import open_existing_scene


def test_open_existing_scene_populates_model(tmp_path: Path):
    d = tmp_path / 'scene_a'; d.mkdir()
    (d / 'environment.yaml').write_text('''robot:\n  name: ur5\nend_effector:\n  name: rg2\nobjects:\n  table_01: {links: []}\n''')
    result = open_existing_scene(d)
    assert result.status == 'YAML_READY'
    assert result.model.robot['name'] == 'ur5'
    assert result.model.end_effector['name'] == 'rg2'
    assert 'table_01' in result.model.objects


def test_missing_yaml_status(tmp_path: Path):
    d = tmp_path / 'scene_b'; d.mkdir()
    result = open_existing_scene(d)
    assert result.status == 'YAML_MISSING'


def test_invalid_yaml_status(tmp_path: Path):
    d = tmp_path / 'scene_c'; d.mkdir()
    (d / 'environment.yaml').write_text('robot: [bad')
    result = open_existing_scene(d)
    assert result.status in {'YAML_INVALID_REPAIRABLE', 'YAML_INVALID_BLOCKED'}
