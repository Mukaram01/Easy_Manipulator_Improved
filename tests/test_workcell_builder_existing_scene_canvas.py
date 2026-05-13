from pathlib import Path
from tests.workcell_scene_backend import open_existing_scene, canvas_items


def test_existing_scene_canvas_items_from_yaml(tmp_path: Path):
    d = tmp_path / 'scene_canvas'; d.mkdir()
    (d / 'environment.yaml').write_text('''robot:\n  name: ur5\nobjects:\n  table_01: {}\n  bin_01: {}\n  part_01: {}\n''')
    opened = open_existing_scene(d)
    items = canvas_items(opened.model)
    types = {i['type'] for i in items}
    assert 'robot' in types
    assert 'table' in types
    assert 'bin' in types
    assert 'object' in types
