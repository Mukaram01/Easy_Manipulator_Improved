import json, os, subprocess, sys
from pathlib import Path


def test_wrapper_writes_fail_json_with_child_diagnostics(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / 'smoke.json'
    shot = tmp_path / 'shot.png'
    cmd = [
        'python3', str(repo / 'scripts/run_workcell_builder_scene3d_gui_smoke.py'),
        '--repo-root', str(repo), '--workspace-root', str(tmp_path), '--scene', 'ur5_2f_test',
        '--output', str(out), '--screenshot', str(shot), '--executable', '/bin/false', '--timeout-sec', '2',
    ]
    rc = subprocess.run(cmd, text=True, capture_output=True)
    assert rc.returncode != 0
    payload = json.loads(out.read_text(encoding='utf-8'))
    assert payload['status'] == 'FAIL'
    assert 'blockers' in payload and 'app_smoke_json_missing' in payload['blockers']


def test_wrapper_does_not_reuse_stale_success_json(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / 'smoke.json'
    shot = tmp_path / 'shot.png'
    out.write_text(json.dumps({'schema': 'workcell_studio_scene3d_gui_smoke/v1', 'status': 'PASS'}) + '\n', encoding='utf-8')
    shot.write_bytes(b'stale-png')
    cmd = [
        'python3', str(repo / 'scripts/run_workcell_builder_scene3d_gui_smoke.py'),
        '--repo-root', str(repo), '--workspace-root', str(tmp_path), '--scene', 'ur5_2f_test',
        '--output', str(out), '--screenshot', str(shot), '--executable', '/bin/false', '--timeout-sec', '2',
    ]
    rc = subprocess.run(cmd, text=True, capture_output=True)
    assert rc.returncode != 0
    payload = json.loads(out.read_text(encoding='utf-8'))
    assert payload['status'] == 'FAIL'
    assert 'app_smoke_json_missing' in payload['blockers']
    assert payload.get('screenshot_available') is False


def test_missing_executable_is_blocked_with_static_non_runtime_evidence(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / 'smoke.json'
    shot = tmp_path / 'shot.png'
    cmd = [
        sys.executable, str(repo / 'scripts/run_workcell_builder_scene3d_gui_smoke.py'),
        '--repo-root', str(repo), '--workspace-root', str(tmp_path), '--scene', 'ur5_2f_test',
        '--output', str(out), '--screenshot', str(shot), '--timeout-sec', '2',
    ]
    env = os.environ.copy()
    env['PATH'] = str(tmp_path)
    rc = subprocess.run(cmd, text=True, capture_output=True, env=env)
    assert rc.returncode != 0
    assert 'status=BLOCKED smoke_status=MISSING_EXECUTABLE' in rc.stdout
    payload = json.loads(out.read_text(encoding='utf-8'))
    assert payload['status'] == 'BLOCKED'
    assert payload['smoke_status'] == 'MISSING_EXECUTABLE'
    assert 'workcell_builder_executable_missing' in payload['blockers']
    assert payload['searched_paths']
    assert 'build workcell_builder in a ROS Humble workspace' in payload['message']
    assert payload['static_scene3d_visual_evidence']['evidence_kind'] == 'non_runtime_static_headless_renderability'
    assert 'not GUI-render PASS evidence' in payload['static_scene3d_visual_evidence']['notes'][0]
    counts = payload['non_runtime_static_headless_renderability_counts']
    assert counts['runtime_available'] is False
    assert 'non-runtime evidence' in counts['note']


def test_main_cpp_contract_contains_scene3d_counter_keys():
    repo = Path(__file__).resolve().parents[1]
    main_cpp = (repo / 'workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')
    for key in [
        'viewport_received_count', 'render_cache_count', 'visible_count', 'rendered_count',
        'unique_visible_item_count', 'selected_scene_name', 'selected_item_id', 'log_line_count'
    ]:
        assert f'counters["{key}"]' in main_cpp


def test_smoke_fallback_render_preserves_counter_gate():
    repo = Path(__file__).resolve().parents[1]
    main_cpp = (repo / 'workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')
    viewport_cpp = (repo / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
    assert 'render_smoke_fallback_frame' in main_cpp
    assert 'rc_after_paint.viewport_received_count > 0' in main_cpp
    assert 'rc_after_paint.visible_count > 0' in main_cpp
    assert 'rc_after_paint.rendered_count <= 0' in main_cpp
    assert 'last_render_counters.rendered_count = rendered_item_count' in viewport_cpp
    assert 'return rendered_item_count > 0' in viewport_cpp
