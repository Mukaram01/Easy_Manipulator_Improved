from __future__ import annotations
import json, subprocess, tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

def _scene(root: Path, meta: str) -> None:
    (root / 'environment.yaml').write_text('robot: {name: ur5}\nend_effector: {name: robotiq_2f}\nobjects:\n  bin1:\n    filepath: meshes/bin1.stl\n    dimensions: [0.3,0.2,0.1]\n', encoding='utf-8')
    (root / 'workcell_builder_metadata.yaml').write_text(meta, encoding='utf-8')

def test_pipeline_exports_core_files() -> None:
    with tempfile.TemporaryDirectory() as d:
        s=Path(d)/'scene'; s.mkdir()
        _scene(s, '{"robot":{"capability_id":"ur5"},"end_effector":{"capability_id":"robotiq_2f_85"},"task_template":{"selected":"pick_place"}}')
        subprocess.run(['python3', str(REPO_ROOT/'scripts/export_builder_scene_to_cell_definition.py'), str(s), '--output-dir', str(s/'generated')], check=True)
        for name in ['cell_definition.yaml','environment_layout.yaml','selected_assets.json','compatibility_report.json','builder_export_summary.json']:
            assert (s/'generated'/name).is_file()

def test_preview_only_generates_warn_compatibility() -> None:
    with tempfile.TemporaryDirectory() as d:
        s=Path(d)/'scene'; s.mkdir()
        _scene(s, '{"robot":{"capability_id":"generic_delta_900","preview_only":true},"end_effector":{"capability_id":"suction"}}')
        subprocess.run(['python3', str(REPO_ROOT/'scripts/export_builder_scene_to_cell_definition.py'), str(s), '--output-dir', str(s/'generated')], check=True)
        report=json.loads((s/'generated'/'compatibility_report.json').read_text())
        assert report['status']=='WARN'
        assert report['runtime_supported'] is False
