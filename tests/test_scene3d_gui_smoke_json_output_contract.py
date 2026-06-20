import json, os, subprocess, sys

import pytest
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
    assert 'ros_humble_available' in payload
    assert payload.get('ros_humble_setup_path') == '/opt/ros/humble/setup.bash'
    if payload.get('ros_humble_available') is False:
        assert 'ros_humble_missing' in payload['blockers']
        assert 'ROS Humble is not available' in payload['blocker_messages']['ros_humble_missing']


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


def test_wrapper_records_ros_missing_when_explicit_executable_writes_pass(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    fake_builder = tmp_path / 'fake_builder'
    fake_builder.write_text(
        '#!/usr/bin/env bash\n'
        'out=""\n'
        'while [[ $# -gt 0 ]]; do\n'
        '  if [[ "$1" == "--smoke-output" ]]; then out="$2"; shift 2; else shift; fi\n'
        'done\n'
        'printf \'{"schema":"workcell_studio_scene3d_gui_smoke/v1","status":"PASS"}\\n\' > "$out"\n',
        encoding='utf-8',
    )
    fake_builder.chmod(0o755)
    out = tmp_path / 'smoke.json'
    cmd = [
        'python3', str(repo / 'scripts/run_workcell_builder_scene3d_gui_smoke.py'),
        '--repo-root', str(repo), '--workspace-root', str(tmp_path), '--scene', 'ur5_2f_test',
        '--output', str(out), '--executable', str(fake_builder), '--timeout-sec', '2',
    ]
    rc = subprocess.run(cmd, text=True, capture_output=True)
    payload = json.loads(out.read_text(encoding='utf-8'))
    if payload.get('ros_humble_available') is False:
        assert rc.returncode != 0
        assert payload['status'] == 'FAIL'
        assert 'ros_humble_missing' in payload['blockers']
        assert 'ROS Humble is not available' in payload['blocker_messages']['ros_humble_missing']
    else:
        assert rc.returncode == 0
        assert payload['status'] == 'PASS'
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
    env.pop('WORKCELL_BUILDER_EXECUTABLE', None)
    rc = subprocess.run(cmd, text=True, capture_output=True, env=env)
    assert rc.returncode != 0
    assert 'status=BLOCKED smoke_status=MISSING_EXECUTABLE' in rc.stdout
    payload = json.loads(out.read_text(encoding='utf-8'))
    assert payload['status'] == 'BLOCKED'
    assert payload['smoke_status'] == 'MISSING_EXECUTABLE'
    assert payload['status'] != 'PASS'
    assert payload['smoke_status'] != 'PASS'
    assert 'workcell_builder_executable_missing' in payload['blockers']
    expected_searched_paths = [
        str(tmp_path / 'install/workcell_builder/bin/workcell_builder'),
        str(tmp_path / 'install/workcell_builder/lib/workcell_builder/workcell_builder'),
        str(tmp_path / 'install/bin/workcell_builder'),
        '/home/user/workcell_ws/install/workcell_builder/bin/workcell_builder',
        '/home/user/workcell_ws/install/workcell_builder/lib/workcell_builder/workcell_builder',
    ]
    assert payload['searched_paths'] == expected_searched_paths
    assert payload['message'] == (
        'workcell_builder executable was not found; build workcell_builder in a ROS Humble '
        'workspace and source install/setup.bash, or set WORKCELL_BUILDER_EXECUTABLE'
    )
    assert payload['guidance'] == [
        'cd /home/user/workcell_ws',
        'source /opt/ros/humble/setup.bash',
        'colcon build --symlink-install --packages-select workcell_builder',
        'source install/setup.bash',
        'export WORKCELL_BUILDER_EXECUTABLE=/home/user/workcell_ws/install/workcell_builder/bin/workcell_builder',
    ]
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
    assert 'const bool physical_mesh_source = !overlay_helper && item_has_credible_mesh_handoff(it);' in viewport_cpp
    assert 'if (physical_mesh_source) ++mesh_rendered_count;' in viewport_cpp
    assert 'last_render_counters.mesh_rendered_count = mesh_rendered_count;' in viewport_cpp
    assert '++generated_fallback_count;' in viewport_cpp
    assert 'last_render_counters.generated_fallback_count = generated_fallback_count;' in viewport_cpp
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


def test_wrapper_adds_supplemental_runtime_visual_evidence(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / "smoke.json"
    shot = tmp_path / "shot.png"
    exe = tmp_path / "fake_workcell_builder_runtime_evidence.py"
    exe.write_text(
        "#!/usr/bin/env python3\n"
        "import json, pathlib, sys\n"
        "out = pathlib.Path(sys.argv[sys.argv.index('--smoke-output') + 1])\n"
        "shot = pathlib.Path(sys.argv[sys.argv.index('--smoke-screenshot') + 1])\n"
        "shot.write_bytes(b'fake-png')\n"
        "payload = {\n"
        "  'schema': 'workcell_studio_scene3d_gui_smoke/v1',\n"
        "  'status': 'PASS',\n"
        "  'counters': {\n"
        "    'rendered_count': 9,\n"
        "    'viewport_width': 1280,\n"
        "    'viewport_height': 720,\n"
        "    'camera_fit_target': 'robot',\n"
        "    'robot_mesh_rendered_count': 4,\n"
        "    'tool_mesh_rendered_count': 1,\n"
        "    'environment_rendered_count': 2,\n"
        "    'overlay_rendered_count': 3,\n"
        "    'camera_rendered_count': 1,\n"
        "    'generated_fallback_count': 5\n"
        "  },\n"
        "  'render_debug_counters': {'physical_mesh_items_rendered': 4},\n"
        "  'visible_items': [{'label': 'UR5 robot'}, {'display_name': 'Workbench'}, {'id': 'pick_zone_main'}]\n"
        "}\n"
        "out.write_text(json.dumps(payload) + '\\n', encoding='utf-8')\n",
        encoding="utf-8",
    )
    exe.chmod(0o755)
    cmd = [
        "python3", str(repo / "scripts/run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root", str(repo), "--workspace-root", str(tmp_path), "--scene", "ur5_2f_test",
        "--output", str(out), "--screenshot", str(shot), "--executable", str(exe), "--timeout-sec", "2",
    ]
    env = os.environ.copy()
    env["ROS_DISTRO"] = "humble"
    proc = subprocess.run(cmd, text=True, capture_output=True, env=env)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["screenshot_path"] == str(shot)
    assert payload["screenshot_available"] is True
    assert payload["viewport_size"] == {"width": 1280, "height": 720}
    assert payload["camera_fit_target"] == "robot"
    assert payload["visible_item_labels"] == ["UR5 robot", "Workbench", "pick_zone_main"]
    assert payload["runtime_render_class_counts"] == {
        "robot_mesh_rendered": 4,
        "tool_mesh_rendered": 1,
        "environment_rendered": 2,
        "zones_rendered": 3,
        "camera_rendered": 1,
        "semantic_fallback_rendered": 5,
    }
    assert payload["robot_mesh_rendered"] == 4


def test_static_headless_counts_remain_non_pass_evidence_when_runtime_unavailable(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / "smoke.json"
    shot = tmp_path / "shot.png"
    cmd = [
        sys.executable, str(repo / "scripts/run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root", str(repo), "--workspace-root", str(tmp_path), "--scene", "ur5_2f_test",
        "--output", str(out), "--screenshot", str(shot), "--timeout-sec", "2",
    ]
    env = os.environ.copy()
    env["PATH"] = str(tmp_path)
    env.pop("WORKCELL_BUILDER_EXECUTABLE", None)
    proc = subprocess.run(cmd, text=True, capture_output=True, env=env)
    assert proc.returncode != 0
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["status"] == "BLOCKED"
    assert payload["runtime_available"] is False
    assert payload["screenshot_path"] == str(shot)
    assert payload["screenshot_available"] is False
    assert payload["visible_item_labels"] == []
    assert "runtime_render_class_counts" not in payload
    assert payload["non_runtime_static_headless_renderability_counts"]["runtime_available"] is False


def test_baked_world_visual_pose_counter_suppresses_zero_transform_warnings():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    payload = {
        "runtime_available": True,
        "counters": {
            "rendered_count": 1,
            "locked_generated_urdf_visual_count": 4,
            "transform_chain_applied_count": 0,
            "visual_origin_applied_count": 0,
        },
        "runtime_scene3d_diagnostics": {
            "baked_world_visual_pose_count": 4,
        },
    }

    smoke._apply_runtime_transform_counter_mapping(payload)

    assert payload["locked_generated_urdf_visual_count"] == 4
    assert payload["transform_chain_applied_count"] == 0
    assert payload["visual_origin_applied_count"] == 0
    assert payload["runtime_baked_world_visual_pose_applied_count"] == 4
    assert "runtime_transform_chain_applied_count_zero_with_generated_visuals" not in payload.get("warnings", [])
    assert "runtime_visual_origin_applied_count_zero_with_generated_visuals" not in payload.get("warnings", [])


def test_zero_transform_warnings_preserved_when_all_transform_evidence_zero():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    payload = {
        "runtime_available": True,
        "counters": {
            "rendered_count": 1,
            "locked_generated_urdf_visual_count": 4,
            "transform_chain_applied_count": 0,
            "visual_origin_applied_count": 0,
        },
    }

    smoke._apply_runtime_transform_counter_mapping(payload)

    assert payload["runtime_baked_world_visual_pose_applied_count"] == 0
    assert "runtime_transform_chain_applied_count_zero_with_generated_visuals" in payload["warnings"]
    assert "runtime_visual_origin_applied_count_zero_with_generated_visuals" in payload["warnings"]


def test_ur5_rendered_mesh_adjacency_prefers_final_draw_bboxes():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    def item(link: str, xmin: float, xmax: float) -> dict:
        return {
            "id": f"draw_{link}",
            "link": link,
            "mesh_path": f"meshes/{link}.dae",
            "final_draw_bbox": {"min": [xmin, 0.0, 0.0], "max": [xmax, 0.1, 0.1]},
        }

    payload = {
        "status": "PASS",
        "final_draw_visual_items": [
            item("base_link", 0.0, 0.1),
            item("shoulder_link", 0.1, 0.2),
            item("upper_arm_link", 0.2, 0.3),
            item("forearm_link", 0.3, 0.4),
            item("wrist_1_link", 0.4, 0.5),
            item("wrist_2_link", 0.5, 0.6),
            item("wrist_3_link", 0.6, 0.7),
            item("robotiq_arg2f_base_link", 0.7, 0.8),
        ],
    }

    smoke._apply_ur5_rendered_mesh_adjacency(payload, repo_root=Path(__file__).resolve().parents[1], scene_name="ur5_2f_test", index_data={})

    assert payload["rendered_mesh_adjacency_source"] == "final_draw_visual_items"
    assert payload["rendered_mesh_adjacency_status"] == "PASS"
    assert "rendered_mesh_adjacency_used_index_fallback" not in payload.get("warnings", [])
    checked = payload["rendered_mesh_adjacency_checked_pairs"]
    assert checked[-1]["parent"] == "wrist_3_link"
    assert checked[-1]["child"] == "robotiq_base"
    assert checked[-1]["parent_item_id"] == "draw_wrist_3_link"
    assert checked[-1]["child_item_id"] == "draw_robotiq_arg2f_base_link"
    assert checked[-1]["parent_bbox_min"] == [0.6, 0.0, 0.0]
    assert checked[-1]["parent_bbox_max"] == [0.7, 0.1, 0.1]
    assert checked[-1]["child_bbox_min"] == [0.7, 0.0, 0.0]
    assert checked[-1]["child_bbox_max"] == [0.8, 0.1, 0.1]
    assert checked[-1]["separation_m"] == 0.0
    assert checked[-1]["limit_m"] == smoke.RENDERED_MESH_ADJACENCY_MAX_SEPARATION_M
    assert checked[-1]["threshold_m"] == smoke.RENDERED_MESH_ADJACENCY_MAX_SEPARATION_M
    assert checked[-1]["parent_link"] == "wrist_3_link"
    assert checked[-1]["child_link"] == "robotiq_arg2f_base_link"
    assert checked[-1]["passed"] is True
    assert checked[-1]["ok"] is True


def test_ur5_rendered_mesh_adjacency_keeps_upper_arm_forearm_gap_strict():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    def item(link: str, xmin: float, xmax: float) -> dict:
        return {
            "id": f"draw_{link}",
            "link": link,
            "mesh_path": f"meshes/{link}.dae",
            "final_draw_bbox": {"min": [xmin, 0.0, 0.0], "max": [xmax, 0.1, 0.1]},
        }

    payload = {
        "status": "PASS",
        "final_draw_visual_items": [
            item("base_link", 0.0, 0.1),
            item("shoulder_link", 0.1, 0.2),
            item("upper_arm_link", 0.2, 0.3),
            item("forearm_link", 0.602, 0.702),
            item("wrist_1_link", 0.702, 0.802),
            item("wrist_2_link", 0.802, 0.902),
            item("wrist_3_link", 0.902, 1.002),
            item("robotiq_arg2f_base_link", 1.002, 1.102),
        ],
    }

    smoke._apply_ur5_rendered_mesh_adjacency(
        payload, repo_root=Path(__file__).resolve().parents[1], scene_name="ur5_2f_test", index_data={}
    )

    checked = payload["rendered_mesh_adjacency_checked_pairs"]
    upper_forearm = next(pair for pair in checked if pair["parent"] == "upper_arm_link" and pair["child"] == "forearm_link")
    assert payload["rendered_mesh_adjacency_status"] == "FAIL"
    assert upper_forearm["separation_m"] == pytest.approx(0.302)
    assert upper_forearm["limit_m"] == smoke.RENDERED_MESH_ADJACENCY_MAX_SEPARATION_M
    assert upper_forearm["ok"] is False
    assert any("upper_arm_link->forearm_link" in error and "0.302 m" in error for error in payload["rendered_mesh_adjacency_errors"])


def test_ur5_rendered_mesh_adjacency_final_draw_nonfinite_fails_without_index_fallback():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    payload = {
        "status": "PASS",
        "final_draw_visual_items": [
            {"id": "base", "link": "base_link", "final_draw_bbox": {"min": [0, 0, 0], "max": [1, 1, 1]}},
            {"id": "shoulder", "link_name": "shoulder_link", "final_draw_bbox": {"min": [float("nan"), 0, 0], "max": [1, 1, 1]}},
        ],
    }

    smoke._apply_ur5_rendered_mesh_adjacency(payload, repo_root=Path(__file__).resolve().parents[1], scene_name="ur5_2f_test", index_data={"items": []})

    assert payload["rendered_mesh_adjacency_source"] == "final_draw_visual_items"
    assert payload["rendered_mesh_adjacency_status"] == "FAIL"
    assert "scene3d_rendered_mesh_adjacency_failed" in payload["warnings"]
    assert "rendered_mesh_adjacency_used_index_fallback" not in payload["warnings"]
    assert any(pair["parent"] == "base_link" and pair["child"] == "shoulder_link" and pair["ok"] is False for pair in payload["rendered_mesh_adjacency_checked_pairs"])


def test_ur5_rendered_mesh_adjacency_marks_index_fallback_warning_when_final_draw_missing():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    payload = {"status": "PASS"}
    smoke._apply_ur5_rendered_mesh_adjacency(payload, repo_root=Path(__file__).resolve().parents[1], scene_name="ur5_2f_test", index_data={"items": []})

    assert payload["rendered_mesh_adjacency_source"] == "visual_index_fallback"
    assert "rendered_mesh_adjacency_used_index_fallback" in payload["warnings"]
