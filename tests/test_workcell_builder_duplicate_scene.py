from pathlib import Path
from tests.workcell_scene_backend import duplicate_scene


def test_duplicate_scene_creates_new_folder_and_keeps_original(tmp_path: Path):
    src = tmp_path / 'source_scene'; src.mkdir()
    (src / 'environment.yaml').write_text('name: source_scene\n')
    dst = duplicate_scene(src, 'source_scene_copy')
    assert dst.exists()
    assert (dst / 'environment.yaml').exists()
    assert (src / 'environment.yaml').read_text() == 'name: source_scene\n'
