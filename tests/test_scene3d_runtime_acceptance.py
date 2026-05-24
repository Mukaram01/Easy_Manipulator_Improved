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
    ], check=False)

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


def test_runtime_acceptance_preview_warn_done_semantics_for_fixture_scenes(tmp_path):
    for scene_name in ('ur5_2f_test', 'ur5_2f_sorting_test'):
        out_json = tmp_path / f'{scene_name}_runtime_acceptance.json'
        subprocess.run([
            'python3',
            str(ROOT / 'scripts' / 'validate_scene3d_runtime_acceptance.py'),
            '--scene',
            scene_name,
            '--json',
            str(out_json),
        ], check=False)
        payload = json.loads(out_json.read_text(encoding='utf-8'))
        scene = payload['scenes'][0]
        assert scene['counts']['editable_layout_count'] > 0
        # Preview can still be considered done for visuals even with warning/blocker metadata.
        assert scene['visibility_contract']['visible_after_default_filters'] > 0


def test_runtime_acceptance_accepts_pass_status_and_hierarchy_counter_aliases(tmp_path):
    smoke = tmp_path / "smoke.json"
    smoke.write_text(json.dumps({
        "schema": "workcell_studio_scene3d_gui_smoke/v1",
        "status": "PASS",
        "counters": {
            "viewport_received_count": 10,
            "render_cache_count": 10,
            "rendered_count": 10,
            "selectable_count": 10,
            "hierarchy_rows_count": 10,
            "mesh_rendered_count": 8
        }
    }), encoding="utf-8")
    out_json = tmp_path / "acceptance.json"
    subprocess.run([
        "python3",
        str(ROOT / "scripts" / "validate_scene3d_runtime_acceptance.py"),
        "--scene", "ur5_2f_test",
        "--smoke-json", str(smoke),
        "--json", str(out_json),
    ], check=False)
    payload = json.loads(out_json.read_text(encoding="utf-8"))
    scene = payload["scenes"][0]
    assert "runtime smoke evidence status is not passing" not in "\n".join(scene["blockers"])
    assert scene["counts"]["hierarchy_rows_count"] == 10


def test_runtime_acceptance_reads_hierarchy_child_row_count_alias(tmp_path):
    smoke = tmp_path / "smoke_alias.json"
    smoke.write_text(json.dumps({
        "schema": "workcell_studio_scene3d_gui_smoke/v1",
        "status": "OK",
        "counters": {
            "viewport_received_count": 4,
            "render_cache_count": 4,
            "rendered_count": 4,
            "selectable_count": 4,
            "hierarchy_child_row_count": 7
        }
    }), encoding="utf-8")
    out_json = tmp_path / "acceptance_alias.json"
    subprocess.run([
        "python3",
        str(ROOT / "scripts" / "validate_scene3d_runtime_acceptance.py"),
        "--scene", "ur5_2f_test",
        "--smoke-json", str(smoke),
        "--json", str(out_json),
    ], check=False)
    payload = json.loads(out_json.read_text(encoding="utf-8"))
    assert payload["scenes"][0]["counts"]["hierarchy_rows_count"] == 7
