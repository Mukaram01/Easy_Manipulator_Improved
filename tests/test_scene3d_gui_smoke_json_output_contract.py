import json, subprocess
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


def test_main_cpp_contract_contains_scene3d_counter_keys():
    repo = Path(__file__).resolve().parents[1]
    main_cpp = (repo / 'workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')
    for key in [
        'viewport_received_count', 'render_cache_count', 'visible_count', 'rendered_count',
        'unique_visible_item_count', 'selected_scene_name', 'selected_item_id', 'log_line_count'
    ]:
        assert f'counters["{key}"]' in main_cpp
