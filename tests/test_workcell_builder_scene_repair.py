from pathlib import Path
from scripts import workcell_builder_gui_workflow as wf


def test_repair_scene_creates_environment_yaml(tmp_path):
    scene = tmp_path / 'broken_scene'
    scene.mkdir()
    (scene / 'package.xml').write_text('<package></package>', encoding='utf-8')
    out = wf.repair_scene_yaml(scene)
    assert out['ok']
    assert (scene / 'environment.yaml').exists()
