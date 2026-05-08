from __future__ import annotations
import json, tempfile
from pathlib import Path
from scripts.export_builder_scene_to_cell_definition import export_scene

def test_canonical_yaml_contains_layout_and_fake_hw() -> None:
    with tempfile.TemporaryDirectory() as d:
        s=Path(d)/'scene'; s.mkdir(); o=s/'gen'; o.mkdir()
        (s/'environment.yaml').write_text('robot: {name: ur5}\nend_effector: {name: suction}\nobjects: {}\n')
        (s/'workcell_builder_metadata.yaml').write_text('{}')
        export_scene(s,o,validate=False)
        cell=(o/'cell_definition.yaml').read_text()
        assert 'layout: generated/environment_layout.yaml' in cell
        assert 'fake_hardware_first: true' in cell
        selected=json.loads((o/'selected_assets.json').read_text())
        assert selected['fake_hardware_default'] is True
