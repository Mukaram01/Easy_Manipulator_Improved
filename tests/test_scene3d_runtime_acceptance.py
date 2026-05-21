import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_runtime_acceptance_script_emits_structured_runtime_payload(tmp_path):
    out_json = tmp_path / 'scene3d_runtime_acceptance.json'
    out_md = tmp_path / 'scene3d_runtime_acceptance.md'
    subprocess.run([
        'python3',
        str(ROOT / 'scripts' / 'validate_scene3d_runtime_acceptance.py'),
        '--json',
        str(out_json),
        '--markdown',
        str(out_md),
    ], check=True)

    assert out_json.exists() and out_md.exists()
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert payload['schema'] == 'workcell_studio_scene3d_runtime_acceptance/v1'
    assert 'blockers' in payload
    assert isinstance(payload['scenes'], list) and payload['scenes']

    scene = payload['scenes'][0]
    assert 'counts' in scene
    assert 'visibility_contract' in scene
    assert 'layers' in scene
    assert 'sources' in scene

    for field in (
        'editable_layout_count',
        'primitive_fallback_count',
        'mesh_preview_count',
        'locked_generated_urdf_visual_count',
        'overlay_count',
    ):
        assert field in scene['counts']
        assert isinstance(scene['counts'][field], int)

    visibility = scene['visibility_contract']
    assert set(visibility) == {
        'input_items_count',
        'visible_after_default_filters',
        'hidden_by_filters_count',
    }
    assert visibility['visible_after_default_filters'] > 0
    assert visibility['input_items_count'] >= visibility['visible_after_default_filters']
    assert visibility['hidden_by_filters_count'] == (
        visibility['input_items_count'] - visibility['visible_after_default_filters']
    )

    expected_scene_pass = visibility['visible_after_default_filters'] > 0 and not scene['blockers']
    assert scene['pass'] == expected_scene_pass


def test_runtime_acceptance_script_fails_for_missing_scene(tmp_path):
    out_json = tmp_path / 'missing_scene3d_runtime_acceptance.json'
    out_md = tmp_path / 'missing_scene3d_runtime_acceptance.md'
    proc = subprocess.run([
        'python3',
        str(ROOT / 'scripts' / 'validate_scene3d_runtime_acceptance.py'),
        '--scene',
        'definitely_missing_scene',
        '--json',
        str(out_json),
        '--markdown',
        str(out_md),
    ], check=False)
    assert proc.returncode != 0

    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert payload['pass'] is False
    assert payload['blockers']
