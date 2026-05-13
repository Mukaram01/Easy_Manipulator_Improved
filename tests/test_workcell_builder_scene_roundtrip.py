from pathlib import Path
import yaml
from tests.workcell_scene_backend import open_existing_scene, save_scene


def test_roundtrip_save_creates_backup_and_preserves_metadata(tmp_path: Path):
    d = tmp_path / 'scene_rt'; d.mkdir()
    (d / 'environment.yaml').write_text('''robot:\n  name: ur5\nend_effector:\n  name: gripper\n  origin: {rpy: [-1.5708, -1.5708, 0]}\nobjects:\n  box_1: {links: []}\nfake_hardware_first: true\nruntime_execution_enabled: false\nselected_template: pick_place\n''')
    opened = open_existing_scene(d)
    backup = save_scene(d, opened.model)
    assert backup.exists()
    root = yaml.safe_load((d / 'environment.yaml').read_text())
    assert root['end_effector']['origin']['rpy'] == [-1.5708, -1.5708, 0]
    assert 'box_1' in root['objects']
    assert root['fake_hardware_first'] is True
    assert root['runtime_execution_enabled'] is False
    assert root['selected_template'] == 'pick_place'
