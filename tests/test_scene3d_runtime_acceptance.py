import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _run_script(out_json: Path, out_md: Path, scene: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [
            'python3',
            str(ROOT / 'scripts' / 'validate_scene3d_runtime_acceptance.py'),
            '--scene',
            scene,
            '--json',
            str(out_json),
            '--markdown',
            str(out_md),
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def test_runtime_acceptance_script_emits_structured_runtime_counts(tmp_path):
    out_json = tmp_path / 'scene3d_runtime_acceptance.json'
    out_md = tmp_path / 'scene3d_runtime_acceptance.md'
    result = _run_script(out_json, out_md, 'ur5_2f_test')
    assert out_json.exists() and out_md.exists()
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert payload['schema'] == 'workcell_studio_scene3d_runtime_acceptance/v1'
    assert 'blockers' in payload
    assert payload['scenes']

    scene_payload = payload['scenes'][0]
    counts = scene_payload['counts']
    assert 'editable_layout_count' in counts
    assert 'primitive_fallback_count' in counts
    assert 'mesh_preview_count' in counts
    assert 'locked_generated_urdf_visual_count' in counts
    assert 'overlay_count' in counts
    assert 'input_items_count' in counts
    assert 'visible_after_default_filters' in counts
    assert 'hidden_by_filters_count' in counts
    assert scene_payload['layer_summary']['default_visibility_contract']['visible_after_default_filters'] == counts['visible_after_default_filters']
    assert result.returncode == (0 if payload['pass'] else 1)


def test_runtime_acceptance_script_nonzero_for_missing_scene(tmp_path):
    out_json = tmp_path / 'scene3d_runtime_acceptance_missing.json'
    out_md = tmp_path / 'scene3d_runtime_acceptance_missing.md'
    result = _run_script(out_json, out_md, 'scene_that_does_not_exist')
    assert result.returncode != 0
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert payload['pass'] is False
    assert payload['blockers']
