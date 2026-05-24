import json
import subprocess
from pathlib import Path

import scripts.validate_scene3d_runtime_acceptance as runtime

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


def _write_scene_fixture(tmp_path: Path, smoke_payload: dict) -> tuple[Path, Path, Path, Path, Path, Path]:
    repo = tmp_path / "repo"
    scene_root = repo / "scenes"
    scene = "demo_scene"
    sdir = scene_root / scene
    (sdir / "layout").mkdir(parents=True, exist_ok=True)
    (sdir / "generated").mkdir(parents=True, exist_ok=True)
    (repo / "cpp").mkdir(parents=True, exist_ok=True)

    (sdir / "layout/workcell_studio_layout.yaml").write_text("items:\n  - id: a\n", encoding="utf-8")
    (sdir / "generated/scene_visual_mesh_index.json").write_text(json.dumps({"safe_for_preview": True, "visual_items": [{"id": "m1"}], "items": []}), encoding="utf-8")
    (sdir / "layout/workcell_studio_layout.generated.yaml").write_text("items: []\n", encoding="utf-8")

    main_path = repo / "cpp/mainwindow.cpp"
    main_path.write_text("select_cb\napply_scene_selection(\nupdate_scene_builder_inspector_for_selected_item\n", encoding="utf-8")
    preview_path = repo / "cpp/scene_preview_widget.cpp"
    preview_path.write_text("select_cb\n", encoding="utf-8")
    viewport_path = repo / "cpp/scene3d_viewport_widget.cpp"
    viewport_path.write_text("draw_ground_grid_pass\ndraw_world_axes_pass\nmouseMoveEvent\nwheelEvent\npan_mode\ntransform_changed_cb\nScene3D runtime render: received=\n", encoding="utf-8")

    smoke = sdir / "generated/scene3d_gui_smoke.json"
    smoke.write_text(json.dumps(smoke_payload), encoding="utf-8")
    return repo, scene_root, main_path, preview_path, viewport_path, scene


def test_runtime_acceptance_default_filter_default_pass_from_runtime_smoke(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(
        tmp_path,
        {
            "schema": "workcell_studio_scene3d_gui_smoke/v1",
            "status": "PASS",
            "counters": {
                "viewport_received_count": 3,
                "render_cache_count": 3,
                "rendered_count": 3,
                "selectable_count": 3,
                "hierarchy_rows_count": 3,
                "mesh_rendered_count": 3,
                "assembled_preview_item_count": 2,
                "filtered_visible_candidate_count": 2,
            },
        },
    )
    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, None)
    assert result["default_filter_visibility_evidence"]["pass_mode"] == "default_pass"
    assert result["secondary_checks"]["layer_toggles_not_default_hide_all"] is True
    assert "scene3d_default_filter_hid_all_renderable_candidates" not in result["blockers"]


def test_runtime_acceptance_default_filter_fallback_pass_requires_warning(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(
        tmp_path,
        {
            "schema": "workcell_studio_scene3d_gui_smoke/v1",
            "status": "PASS",
            "counters": {
                "viewport_received_count": 3,
                "render_cache_count": 3,
                "rendered_count": 3,
                "selectable_count": 3,
                "hierarchy_rows_count": 3,
                "mesh_rendered_count": 3,
                "assembled_preview_item_count": 2,
                "filtered_visible_candidate_count": 1,
            },
            "warnings": ["default_filter_fallback_kept_renderable_items_visible"],
        },
    )
    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, None)
    assert result["default_filter_visibility_evidence"]["pass_mode"] == "fallback_pass"
    assert result["secondary_checks"]["layer_toggles_not_default_hide_all"] is True


def test_runtime_acceptance_default_filter_zero_visible_is_blocker(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(
        tmp_path,
        {
            "schema": "workcell_studio_scene3d_gui_smoke/v1",
            "status": "PASS",
            "counters": {
                "viewport_received_count": 3,
                "render_cache_count": 3,
                "rendered_count": 3,
                "selectable_count": 3,
                "hierarchy_rows_count": 3,
                "mesh_rendered_count": 3,
                "assembled_preview_item_count": 2,
                "filtered_visible_candidate_count": 0,
            },
        },
    )
    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, None)
    assert result["default_filter_visibility_evidence"]["pass_mode"] == "failed"
    assert "scene3d_default_filter_hid_all_renderable_candidates" in result["blockers"]
    assert result["secondary_checks"]["layer_toggles_not_default_hide_all"] is False

def test_runtime_acceptance_markdown_handles_missing_secondary_checks(tmp_path):
    out_json = tmp_path / 'acceptance.json'
    out_md = tmp_path / 'acceptance.md'
    subprocess.run([
        'python3',
        str(ROOT / 'scripts' / 'validate_scene3d_runtime_acceptance.py'),
        '--scene', 'definitely_missing_scene',
        '--json', str(out_json),
        '--markdown', str(out_md),
    ], check=False)
    assert out_md.exists()
    text = out_md.read_text(encoding='utf-8')
    assert 'missing_secondary_checks' in text
