from pathlib import Path

def test_repo_has_default_scenes_and_assets():
    root = Path(__file__).resolve().parents[1]
    assert (root / 'scenes').is_dir()
    assert (root / 'assets').is_dir()
