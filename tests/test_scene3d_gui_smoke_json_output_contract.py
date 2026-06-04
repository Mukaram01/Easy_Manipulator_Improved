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


def _write_fake_smoke_executable(path: Path, status: str, exit_code: int = 0) -> None:
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import json, pathlib, sys\n"
        "out = pathlib.Path(sys.argv[sys.argv.index('--smoke-output') + 1])\n"
        "payload = {\n"
        f"    'schema': 'workcell_studio_scene3d_gui_smoke/v1', 'status': {status!r},\n"
        "    'counters': {'rendered_count': 7},\n"
        "    'render_debug_counters': {'physical_mesh_items_rendered': 3},\n"
        "}\n"
        "out.parent.mkdir(parents=True, exist_ok=True)\n"
        "out.write_text(json.dumps(payload) + '\\n', encoding='utf-8')\n"
        "print('fake app stdout line')\n"
        "print('fake app stderr line', file=sys.stderr)\n"
        f"raise SystemExit({exit_code})\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def test_wrapper_enriches_app_json_without_replacing_app_counters(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / "smoke.json"
    exe = tmp_path / "fake_workcell_builder.py"
    _write_fake_smoke_executable(exe, "PASS")
    cmd = [
        "python3", str(repo / "scripts/run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root", str(repo), "--workspace-root", str(tmp_path), "--scene", "ur5_2f_test",
        "--output", str(out), "--executable", str(exe), "--timeout-sec", "2",
    ]
    proc = subprocess.run(cmd, text=True, capture_output=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["status"] == "PASS"
    assert payload["wrapper_status"] == "PASS"
    assert payload["resolved_executable"] == str(exe)
    assert isinstance(payload["searched_paths"], list)
    assert payload["repo_root"] == str(repo)
    assert payload["workspace_root"] == str(tmp_path)
    assert payload["scene_path"] is None
    assert "--scene3d-smoke" in payload["child_command"]
    assert payload["child_returncode"] == 0
    assert payload["timed_out"] is False
    assert payload["stdout_log_path"].endswith(".stdout.log")
    assert payload["stderr_log_path"].endswith(".stderr.log")
    assert "fake app stdout line" in payload["stdout_tail"]
    assert "fake app stderr line" in payload["stderr_tail"]
    assert payload["screenshot_path"] is None
    assert payload["screenshot_available"] is False
    assert payload["counters"] == {"rendered_count": 7}
    assert payload["render_debug_counters"] == {"physical_mesh_items_rendered": 3}


def test_wrapper_preserves_app_fail_status_when_child_returns_zero(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / "smoke.json"
    exe = tmp_path / "fake_workcell_builder.py"
    _write_fake_smoke_executable(exe, "FAIL")
    cmd = [
        "python3", str(repo / "scripts/run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root", str(repo), "--workspace-root", str(tmp_path), "--scene", "ur5_2f_test",
        "--output", str(out), "--executable", str(exe), "--timeout-sec", "2",
    ]
    proc = subprocess.run(cmd, text=True, capture_output=True)
    assert proc.returncode != 0
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["status"] == "FAIL"
    assert payload["wrapper_status"] == "PASS"
    assert payload["counters"] == {"rendered_count": 7}
    assert "status=FAIL" in proc.stdout
