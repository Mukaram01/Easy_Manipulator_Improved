from pathlib import Path
from tests.workcell_scene_backend import regenerate_scene


def test_regenerate_existing_scene_creates_generated_outputs(tmp_path: Path):
    d = tmp_path / 'scene_gen'; d.mkdir()
    (d / 'environment.yaml').write_text('robot: {name: ur5}\n')
    launch_cmd = regenerate_scene(d)
    assert (d / 'launch' / 'demo.launch.py').exists()
    assert (d / 'launch' / 'demo.rviz').exists()
    assert (d / 'urdf' / 'scene.urdf.xacro').exists()
    assert 'scene_gen' in launch_cmd
