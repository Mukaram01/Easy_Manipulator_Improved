from pathlib import Path
from scripts.workcell_builder_gui_catalog import generate

def test_gui_catalog_export(tmp_path: Path):
    payload = generate(Path('.').resolve())
    assert 'robots' in payload
    out = tmp_path / 'workcell_builder_gui_catalog.json'
    out.write_text(__import__('json').dumps(payload))
    assert out.exists()
