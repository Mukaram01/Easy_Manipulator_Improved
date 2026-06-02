import json
import subprocess
from pathlib import Path

import scripts.validate_scene3d_runtime_acceptance as runtime

ROOT = Path(__file__).resolve().parents[1]
SYNTHETIC_SCENE = "synthetic_scene3d_acceptance"


def _write_scene_fixture(
    tmp_path: Path,
    smoke_payload: dict,
    *,
    scene: str = SYNTHETIC_SCENE,
    editable_items: int = 2,
    mesh_items: int = 1,
    locked_items: int = 1,
    overlay_items: int = 1,
    primitive_fallback_count: int = 0,
) -> tuple[Path, Path, Path, Path, Path, str]:
    repo = tmp_path / "repo"
    scene_root = repo / "scenes"
    sdir = scene_root / scene
    (sdir / "layout").mkdir(parents=True, exist_ok=True)
    (sdir / "generated").mkdir(parents=True, exist_ok=True)
    (sdir / "config").mkdir(parents=True, exist_ok=True)
    (repo / "workcell_builder/workcell_builder/gui").mkdir(parents=True, exist_ok=True)

    # Strong synthetic scene identity for CLI discovery without relying on any real scene names.
    (sdir / "package.xml").write_text(
        "<package format=\"3\"><name>synthetic_scene3d_acceptance</name><version>0.0.0</version>"
        "<description>synthetic Scene3D acceptance fixture</description>"
        "<maintainer email=\"test@example.com\">test</maintainer><license>Apache-2.0</license></package>\n",
        encoding="utf-8",
    )
    (sdir / "scene_manifest.yaml").write_text(
        "schema: workcell_studio_scene_manifest/v1\nscene: synthetic_scene3d_acceptance\n",
        encoding="utf-8",
    )

    layout_entries = "".join(
        f"  - id: editable_{i}\n    type: box\n    source: editable_layout\n    selectable: true\n    editable: true\n"
        for i in range(editable_items)
    )
    (sdir / "layout/workcell_studio_layout.yaml").write_text(
        ("items:\n" + layout_entries) if layout_entries else "items: []\n",
        encoding="utf-8",
    )

    visual_items = [{"id": f"mesh_preview_{i}", "mesh": f"package://fixture/mesh_{i}.stl"} for i in range(mesh_items)]
    mesh_index_items = [
        {"id": item["id"], "source_layer": "mesh_preview", "renderable": True, "selectable": True}
        for item in visual_items
    ]
    mesh_index_items.extend(
        {"id": f"locked_preview_{i}", "source_layer": "locked_generated_urdf_visual", "renderable": True, "editable": False}
        for i in range(locked_items)
    )
    mesh_index_items.extend(
        {"id": f"overlay_helper_{i}", "source_layer": "overlay", "renderable": False, "helper": True}
        for i in range(overlay_items)
    )
    (sdir / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps(
            {
                "safe_for_preview": True,
                "visual_items": visual_items,
                "unresolved_placeholder_count": primitive_fallback_count,
                "items": mesh_index_items,
            }
        ),
        encoding="utf-8",
    )

    generated_entries = "".join(
        f"  - id: locked_preview_{i}\n    source: locked_generated_urdf_visual\n    locked: true\n    editable: false\n"
        for i in range(locked_items)
    )
    (sdir / "layout/workcell_studio_layout.generated.yaml").write_text(
        ("items:\n" + generated_entries) if generated_entries else "items: []\n",
        encoding="utf-8",
    )
    overlays = [{"id": f"overlay_helper_{i}", "kind": "helper", "source": "diagnostic_overlay"} for i in range(overlay_items)]
    (sdir / "generated/scene_preview_metadata.json").write_text(json.dumps({"overlays": overlays}), encoding="utf-8")

    gui_dir = repo / "workcell_builder/workcell_builder/gui"
    main_path = gui_dir / "mainwindow.cpp"
    main_path.write_text("select_cb\napply_scene_selection(\nupdate_scene_builder_inspector_for_selected_item\n", encoding="utf-8")
    preview_path = gui_dir / "scene_preview_widget.cpp"
    preview_path.write_text("select_cb\n", encoding="utf-8")
    viewport_path = gui_dir / "scene3d_viewport_widget.cpp"
    viewport_path.write_text(
        "draw_ground_grid_pass\ndraw_world_axes_pass\nmouseMoveEvent\nwheelEvent\npan_mode\n"
        "transform_changed_cb\nScene3D runtime render: received=\n",
        encoding="utf-8",
    )

    smoke = sdir / "generated/scene3d_gui_smoke.json"
    smoke.write_text(json.dumps(smoke_payload), encoding="utf-8")
    return repo, scene_root, main_path, preview_path, viewport_path, scene


def _passing_smoke(**counter_overrides: int) -> dict:
    counters = {
        "viewport_received_count": 3,
        "render_cache_count": 3,
        "rendered_count": 3,
        "selectable_count": 3,
        "editable_physical_item_count": 2,
        "selectable_physical_item_count": 3,
        "hierarchy_rows_count": 4,
        "mesh_rendered_count": 1,
        "assembled_preview_item_count": 4,
        "filtered_visible_candidate_count": 3,
    }
    counters.update(counter_overrides)
    return {
        "schema": "workcell_studio_scene3d_gui_smoke/v1",
        "status": "PASS",
        "counters": counters,
    }


def test_runtime_acceptance_script_emits_structured_runtime_payload(tmp_path):
    repo, _, _, _, _, scene = _write_scene_fixture(tmp_path, _passing_smoke())
    out_json = tmp_path / 'scene3d_runtime_acceptance.json'
    out_md = tmp_path / 'scene3d_runtime_acceptance.md'
    subprocess.run([
        'python3',
        str(ROOT / 'scripts' / 'validate_scene3d_runtime_acceptance.py'),
        '--repo-root',
        str(repo),
        '--scene',
        scene,
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

    scene_result = payload['scenes'][0]
    assert 'counts' in scene_result
    assert 'visibility_contract' in scene_result
    assert 'layers' in scene_result
    assert 'sources' in scene_result

    for field in (
        'editable_layout_count',
        'primitive_fallback_count',
        'mesh_preview_count',
        'locked_generated_urdf_visual_count',
        'overlay_count',
    ):
        assert field in scene_result['counts']
        assert isinstance(scene_result['counts'][field], int)

    visibility = scene_result['visibility_contract']
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

    expected_scene_pass = visibility['visible_after_default_filters'] > 0 and not scene_result['blockers']
    assert scene_result['pass'] == expected_scene_pass


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


def test_runtime_acceptance_preview_warn_done_semantics_for_synthetic_scene(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(
        tmp_path,
        _passing_smoke(),
        editable_items=2,
        mesh_items=1,
        locked_items=2,
        overlay_items=2,
    )
    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, None)

    assert result['counts']['editable_layout_count'] == 2
    assert result['counts']['mesh_preview_count'] == 1
    assert result['counts']['locked_generated_urdf_visual_count'] == 2
    assert result['counts']['overlay_count'] == 2
    # Preview can still be considered done for editable/mesh visuals even with diagnostic-only layers present.
    assert result['visibility_contract']['visible_after_default_filters'] == 3
    assert result['visibility_contract']['hidden_by_filters_count'] == 4


def test_runtime_acceptance_accepts_pass_status_and_hierarchy_counter_aliases(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(tmp_path, _passing_smoke())
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
    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, str(smoke))
    assert "runtime smoke evidence status is not passing" not in "\n".join(result["blockers"])
    assert result["counts"]["hierarchy_rows_count"] == 10


def test_runtime_acceptance_reads_hierarchy_child_row_count_alias(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(tmp_path, _passing_smoke())
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
    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, str(smoke))
    assert result["counts"]["hierarchy_rows_count"] == 7


def test_runtime_acceptance_counts_editable_locked_and_overlay_layers_independently(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(
        tmp_path,
        _passing_smoke(
            viewport_received_count=5,
            render_cache_count=5,
            rendered_count=3,
            selectable_count=3,
            hierarchy_rows_count=5,
            mesh_rendered_count=1,
            assembled_preview_item_count=5,
            filtered_visible_candidate_count=3,
        ),
        editable_items=2,
        mesh_items=1,
        locked_items=2,
        overlay_items=2,
    )

    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, None)

    assert result["counts"]["editable_layout_count"] == 2
    assert result["counts"]["selectable_count"] == 3
    assert result["visibility_contract"]["visible_after_default_filters"] == 3
    assert result["layers"]["editable_layout_visible"] == 2

    assert result["counts"]["locked_generated_urdf_visual_count"] == 2
    assert result["layers"]["locked_generated_urdf_visual_hidden_default"] == 2
    assert result["visibility_contract"]["hidden_by_filters_count"] == 4

    assert result["counts"]["overlay_count"] == 2
    assert result["source_layer_counts"]["overlay"] == 2
    assert result["default_filter_visibility_evidence"]["pass_mode"] == "default_pass"
    assert "scene3d_default_filter_hid_all_renderable_candidates" not in result["blockers"]


def test_runtime_acceptance_overlay_helpers_do_not_satisfy_render_success(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(
        tmp_path,
        _passing_smoke(
            viewport_received_count=1,
            render_cache_count=1,
            rendered_count=1,
            selectable_count=0,
            hierarchy_rows_count=1,
            mesh_rendered_count=0,
            assembled_preview_item_count=1,
            filtered_visible_candidate_count=0,
        ),
        editable_items=0,
        mesh_items=0,
        locked_items=0,
        overlay_items=2,
    )

    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, None)

    assert result["counts"]["overlay_count"] == 2
    assert result["visibility_contract"]["visible_after_default_filters"] == 0
    assert "no visible scene items after default filters" in result["blockers"]
    assert result["pass"] is False


def test_runtime_acceptance_default_filter_default_pass_from_runtime_smoke(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(
        tmp_path,
        _passing_smoke(
            viewport_received_count=3,
            render_cache_count=3,
            rendered_count=3,
            selectable_count=3,
            hierarchy_rows_count=3,
            mesh_rendered_count=3,
            assembled_preview_item_count=2,
            filtered_visible_candidate_count=2,
        ),
    )
    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, None)
    assert result["default_filter_visibility_evidence"]["pass_mode"] == "default_pass"
    assert result["secondary_checks"]["layer_toggles_not_default_hide_all"] is True
    assert "scene3d_default_filter_hid_all_renderable_candidates" not in result["blockers"]


def test_runtime_acceptance_default_filter_fallback_pass_requires_warning(tmp_path):
    payload = _passing_smoke(
        viewport_received_count=3,
        render_cache_count=3,
        rendered_count=3,
        selectable_count=3,
        hierarchy_rows_count=3,
        mesh_rendered_count=3,
        assembled_preview_item_count=2,
        filtered_visible_candidate_count=1,
    )
    payload["warnings"] = ["default_filter_fallback_kept_renderable_items_visible"]
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(tmp_path, payload)
    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, None)
    assert result["default_filter_visibility_evidence"]["pass_mode"] == "fallback_pass"
    assert result["secondary_checks"]["layer_toggles_not_default_hide_all"] is True


def test_runtime_acceptance_default_filter_zero_visible_is_blocker(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(
        tmp_path,
        _passing_smoke(
            viewport_received_count=3,
            render_cache_count=3,
            rendered_count=3,
            selectable_count=3,
            hierarchy_rows_count=3,
            mesh_rendered_count=3,
            assembled_preview_item_count=2,
            filtered_visible_candidate_count=0,
        ),
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


def test_runtime_acceptance_editable_layout_fixture_is_selectable_and_editable(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(
        tmp_path,
        _passing_smoke(
            viewport_received_count=2,
            render_cache_count=2,
            rendered_count=2,
            selectable_count=2,
            hierarchy_rows_count=2,
            mesh_rendered_count=1,
            assembled_preview_item_count=2,
            filtered_visible_candidate_count=2,
        ),
        editable_items=1,
        mesh_items=1,
        locked_items=0,
        overlay_items=0,
    )

    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, None)

    assert result["counts"]["editable_layout_count"] == 1
    assert result["interaction_contract"]["editable_layout_selectable_count"] == 1
    assert result["interaction_contract"]["editable_layout_editable_count"] == 1
    assert result["counts"]["selectable_count"] > 0
    assert result["secondary_checks"]["selection_callback_wired"] is True
    assert result["secondary_checks"]["inspector_callback_wired"] is True
    assert result["pass"] is True


def test_runtime_acceptance_locked_generated_preview_is_visible_diagnosable_and_not_editable(tmp_path):
    repo, scene_root, main_path, preview_path, viewport_path, scene = _write_scene_fixture(
        tmp_path,
        _passing_smoke(
            viewport_received_count=2,
            render_cache_count=2,
            rendered_count=2,
            selectable_count=1,
            hierarchy_rows_count=2,
            mesh_rendered_count=1,
            assembled_preview_item_count=2,
            filtered_visible_candidate_count=1,
        ),
        editable_items=1,
        mesh_items=1,
        locked_items=1,
        overlay_items=0,
    )

    result = runtime.evaluate_scene(repo, scene_root, main_path, preview_path, viewport_path, scene, None)

    assert result["counts"]["locked_generated_urdf_visual_count"] == 1
    assert result["layers"]["locked_generated_urdf_visual_hidden_default"] == 1
    assert result["visibility_contract"]["hidden_by_filters_count"] >= 1
    assert result["interaction_contract"]["locked_generated_urdf_diagnosable_count"] == 1
    assert result["interaction_contract"]["locked_generated_urdf_editable_count"] == 0
    assert result["counts"]["locked_generated_urdf_editable_count"] == 0
    assert result["pass"] is True
