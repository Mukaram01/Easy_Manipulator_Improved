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
        "  'rendered_ur5_link_count': 6,\n"
        "  'missing_required_visible_ur5_links': [],\n"
        "  'visual_quality_status': 'PASS',\n"
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


def test_baked_world_visual_transform_counter_suppresses_zero_transform_warnings():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    payload = {
        "runtime_available": True,
        "counters": {
            "rendered_count": 1,
            "locked_generated_urdf_visual_count": 4,
            "transform_chain_applied_count": 0,
            "visual_origin_applied_count": 0,
            "baked_world_visual_transform_count": 4,
        },
        "runtime_scene3d_diagnostics": {
            "baked_world_visual_pose_count": 4,
        },
    }

    smoke._apply_runtime_transform_counter_mapping(payload)

    assert payload["locked_generated_urdf_visual_count"] == 4
    assert payload["transform_chain_applied_count"] == 0
    assert payload["visual_origin_applied_count"] == 0
    assert payload["baked_world_visual_transform_count"] == 4
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

    assert payload["baked_world_visual_transform_count"] == 0
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


def test_ur5_rendered_mesh_adjacency_ignores_legacy_rendered_mesh_bbox_aliases():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    def item(link: str, xmin: float, xmax: float) -> dict:
        return {
            "id": f"draw_{link}",
            "link": link,
            "mesh_path": f"meshes/{link}.dae",
            "final_draw_world_pose": [100.0 + xmin, 0.0, 0.0],
            "bounds": {"min": [100.0 + xmin, 0.0, 0.0], "max": [100.0 + xmax, 0.1, 0.1]},
            "final_rendered_mesh_bbox": {"min": [xmin, 0.0, 0.0], "max": [xmax, 0.1, 0.1]},
        }

    payload = {
        "status": "PASS",
        "final_draw_visual_items": [
            item("base_link_inertia", 0.0, 0.1),
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

    assert payload["rendered_mesh_adjacency_status"] == "FAIL"
    checked = payload["rendered_mesh_adjacency_checked_pairs"]
    assert checked[0]["parent_bbox_min"] is None
    assert all(pair["ok"] is False for pair in checked)
    assert any("final_draw_bbox" in error for error in payload["rendered_mesh_adjacency_errors"])



def test_ur5_rendered_mesh_adjacency_fails_when_final_draw_bboxes_far_despite_plausible_visual_index():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    chain = [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "gripper_base_link",
    ]

    def index_row(link: str, i: int) -> dict:
        return {
            "source_row_index": i,
            "link": link,
            "link_name": link,
            "baked_world_visual_pose": {"xyz": [i * 0.05, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "visual_origin": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "link_world_pose": {"xyz": [i * 0.05, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
        }

    def final_row(link: str, i: int) -> dict:
        x = i * 2.0
        return {
            "id": f"draw_{link}",
            "source_row_index": i,
            "link": link,
            "link_name": link,
            "canonical_link_name": link,
            "final_draw_status": "ok",
            "final_draw_bbox": {"min": [x, 0.0, 0.0], "max": [x + 0.1, 0.1, 0.1]},
            "final_draw_bbox_center": [x + 0.05, 0.05, 0.05],
        }

    payload = {
        "status": "PASS",
        "visual_index": {"items": [index_row(link, i) for i, link in enumerate(chain)]},
        "scene_visual_mesh_index": {"items": [index_row(link, i) for i, link in enumerate(chain)]},
        "final_draw_visual_items": [final_row(link, i) for i, link in enumerate(chain)],
    }

    smoke._apply_ur5_rendered_mesh_adjacency(
        payload, repo_root=Path(__file__).resolve().parents[1], scene_name="ur5_2f_test", index_data=payload["visual_index"]
    )

    assert payload["rendered_mesh_adjacency_source"] == "final_draw_visual_items"
    assert payload["rendered_mesh_adjacency_status"] == "FAIL"
    assert payload["generated_robot_topology_diagnostics"]["source"] == "final_draw_visual_items"
    first_pair = payload["rendered_mesh_adjacency_checked_pairs"][0]
    assert first_pair["parent"] == "base_link"
    assert first_pair["child"] == "shoulder_link"
    assert first_pair["separation_m"] == pytest.approx(1.9)
    assert first_pair["ok"] is False
    assert any("final draw bbox adjacency" in error for error in payload["rendered_mesh_adjacency_errors"])
    assert "rendered_mesh_adjacency_used_index_fallback" not in payload.get("warnings", [])

def test_ur5_rendered_mesh_adjacency_rejects_metadata_only_nonfinite_bboxes():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    chain = [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "gripper_base_link",
    ]

    def item(link: str, parent: str | None) -> dict:
        return {
            "id": f"generated_urdf::{link}::visual_0",
            "item_id": f"generated_urdf::{link}",
            "link": link,
            "link_name": link,
            "canonical_link_name": link,
            "parent_link": parent,
            "immediate_parent_link": parent,
            "link_chain": chain[: chain.index(link) + 1],
            "final_draw_status": "ok",
            "final_draw_bbox": {"min": [float("nan"), 0.0, 0.0], "max": [1.0, 1.0, 1.0]},
        }

    payload = {
        "status": "PASS",
        "final_draw_visual_items": [
            item("base_link_inertia", None),
            item("shoulder_link", "base_link_inertia"),
            item("upper_arm_link", "shoulder_link"),
            item("forearm_link", "upper_arm_link"),
            item("wrist_1_link", "forearm_link"),
            item("wrist_2_link", "wrist_1_link"),
            item("wrist_3_link", "wrist_2_link"),
            item("gripper_base_link", "wrist_3_link"),
        ],
    }

    smoke._apply_ur5_rendered_mesh_adjacency(payload, repo_root=Path(__file__).resolve().parents[1], scene_name="ur5_2f_test", index_data={})

    assert payload["rendered_mesh_adjacency_status"] == "FAIL"
    assert "scene3d_rendered_mesh_adjacency_failed" in payload.get("warnings", [])
    checked = payload["rendered_mesh_adjacency_checked_pairs"]
    assert len(checked) == 7
    arm_pairs = [pair for pair in checked if pair["child"] != "robotiq_base"]
    assert all(pair["ok"] is False for pair in arm_pairs)
    assert all(pair.get("evidence") != "stable_metadata" for pair in arm_pairs)
    assert checked[-1]["child"] == "robotiq_base"
    assert checked[-1]["child_item_id"] == "generated_urdf::gripper_base_link"


def test_reported_ur5_2f_smoke_json_generated_urdf_contract_passes():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    chain = [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "gripper_base_link",
    ]

    def generated_visual_row(link: str, parent: str | None) -> dict:
        return {
            "id": f"generated_urdf::{link}::visual_0",
            "item_id": f"generated_urdf::{link}",
            "link": link,
            "link_name": link,
            "canonical_link_name": link,
            "parent_link": parent,
            "immediate_parent_link": parent,
            "link_chain": chain[: chain.index(link) + 1],
            "final_draw_status": "ok",
            "final_draw_bbox": {"min": [float(chain.index(link)) * 0.1, 0.0, 0.0], "max": [float(chain.index(link)) * 0.1 + 0.1, 0.1, 0.1]},
            "generated_urdf_visual": True,
            "rendered": True,
        }

    generated_rows = [
        generated_visual_row("base_link_inertia", None),
        generated_visual_row("shoulder_link", "base_link_inertia"),
        generated_visual_row("upper_arm_link", "shoulder_link"),
        generated_visual_row("forearm_link", "upper_arm_link"),
        generated_visual_row("wrist_1_link", "forearm_link"),
        generated_visual_row("wrist_2_link", "wrist_1_link"),
        generated_visual_row("wrist_3_link", "wrist_2_link"),
        generated_visual_row("gripper_base_link", "wrist_3_link"),
    ]
    payload = {
        "schema": "workcell_studio_scene3d_gui_smoke/v1",
        "status": "PASS",
        "app_status": "PASS",
        "runtime_available": True,
        "warnings": ["scene3d_optional_ui_metadata_missing"],
        "counters": {
            "visible_count": 32,
            "rendered_count": 31,
            "locked_generated_urdf_visual_count": len(generated_rows),
            "transform_chain_applied_count": 0,
            "visual_origin_applied_count": 0,
            "physical_mesh_items_rendered": 31,
        },
        "render_debug_counters": {
            "physical_mesh_items_rendered": 31,
            "generated_urdf_visual_count": len(generated_rows),
            "baked_world_visual_transform_count": 18,
        },
        "runtime_scene3d_diagnostics": {
            "baked_world_visual_pose_count": 18,
        },
        "final_draw_visual_items": generated_rows,
    }

    smoke._add_smoke_report_supplemental_evidence(payload, screenshot_path=None, screenshot_available=None)
    smoke._apply_ur5_rendered_mesh_adjacency(
        payload,
        repo_root=Path(__file__).resolve().parents[1],
        scene_name="ur5_2f_test",
        index_data={},
    )

    assert payload["status"] == "PASS"
    assert payload["app_status"] == "PASS"
    assert payload["counters"]["visible_count"] == 32
    assert payload["counters"]["rendered_count"] == 31
    assert payload["locked_generated_urdf_visual_count"] == len(generated_rows)
    assert {row["link"] for row in payload["final_draw_visual_items"]} >= {
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    }
    assert payload["transform_chain_applied_count"] == 0
    assert payload["visual_origin_applied_count"] == 0
    assert payload["baked_world_visual_transform_count"] == 18
    assert payload["runtime_baked_world_visual_pose_applied_count"] == 18
    assert "runtime_transform_chain_applied_count_zero_with_generated_visuals" not in payload.get("warnings", [])
    assert "runtime_visual_origin_applied_count_zero_with_generated_visuals" not in payload.get("warnings", [])
    assert payload["rendered_mesh_adjacency_status"] == "PASS"
    assert payload["generated_robot_topology_diagnostics"]["status"] == "PASS"
    assert payload["generated_robot_topology_diagnostics"]["source"] == "final_draw_visual_items"
    assert "scene3d_rendered_mesh_adjacency_failed" not in payload.get("warnings", [])
    assert payload["warnings"] == ["scene3d_optional_ui_metadata_missing"]
    assert all(pair["separation_m"] == pytest.approx(0.0) for pair in payload["rendered_mesh_adjacency_checked_pairs"])



def test_ur5_rendered_mesh_adjacency_missing_robotiq_base_fails_payload_status():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    def item(link: str, xmin: float, xmax: float) -> dict:
        return {
            "id": f"draw_{link}",
            "link": link,
            "link_name": link,
            "canonical_link_name": link,
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
        ],
    }

    smoke._apply_ur5_rendered_mesh_adjacency(
        payload, repo_root=Path(__file__).resolve().parents[1], scene_name="ur5_2f_test", index_data={}
    )

    assert payload["rendered_mesh_adjacency_status"] == "FAIL"
    assert payload["status"] != "PASS"
    topology = payload["generated_robot_topology_diagnostics"]
    assert topology["status"] == "FAIL"
    assert topology["source"] == "final_draw_visual_items"
    robotiq_pair = topology["checked_pairs"][-1]
    assert robotiq_pair["child"] == "robotiq_base"
    assert robotiq_pair["ok"] is False
    assert robotiq_pair["separation_m"] is None
    assert robotiq_pair["distance_m"] is None
    assert any("Robotiq base" in error for error in topology["errors"])


def test_ur5_rendered_mesh_adjacency_far_robotiq_base_fails_despite_stable_metadata():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    chain = [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "gripper_base_link",
    ]

    def item(link: str, xmin: float, xmax: float, parent: str | None = None) -> dict:
        return {
            "id": f"draw_{link}",
            "link": link,
            "link_name": link,
            "canonical_link_name": link,
            "parent_link": parent,
            "immediate_parent_link": parent,
            "link_chain": chain[: chain.index(link) + 1] if link in chain else [],
            "final_draw_bbox": {"min": [xmin, 0.0, 0.0], "max": [xmax, 0.1, 0.1]},
        }

    payload = {
        "status": "PASS",
        "final_draw_visual_items": [
            item("base_link_inertia", 0.0, 0.1),
            item("shoulder_link", 0.1, 0.2, "base_link_inertia"),
            item("upper_arm_link", 0.2, 0.3, "shoulder_link"),
            item("forearm_link", 0.3, 0.4, "upper_arm_link"),
            item("wrist_1_link", 0.4, 0.5, "forearm_link"),
            item("wrist_2_link", 0.5, 0.6, "wrist_1_link"),
            item("wrist_3_link", 0.6, 0.7, "wrist_2_link"),
            item("gripper_base_link", 1.5, 1.6, "wrist_3_link"),
        ],
    }

    smoke._apply_ur5_rendered_mesh_adjacency(
        payload, repo_root=Path(__file__).resolve().parents[1], scene_name="ur5_2f_test", index_data={}
    )

    assert payload["rendered_mesh_adjacency_status"] == "FAIL"
    assert payload["status"] != "PASS"
    topology = payload["generated_robot_topology_diagnostics"]
    robotiq_pair = topology["checked_pairs"][-1]
    assert robotiq_pair["child"] == "robotiq_base"
    assert robotiq_pair["stable_metadata_adjacency"] is True
    assert robotiq_pair["separation_m"] == pytest.approx(0.8)
    assert robotiq_pair["distance_m"] == pytest.approx(0.8)
    assert robotiq_pair["limit_m"] <= smoke.RENDERED_MESH_ADJACENCY_MAX_SEPARATION_M
    assert robotiq_pair["ok"] is False
    assert any("Robotiq base final draw bbox separated" in error for error in topology["errors"])


def test_ur5_rendered_mesh_adjacency_exploded_robotiq_finger_fails():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    def item(link: str, xmin: float, xmax: float) -> dict:
        return {
            "id": f"draw_{link}",
            "link": link,
            "link_name": link,
            "canonical_link_name": link,
            "final_draw_status": "ok",
            "final_draw_bbox": {"min": [xmin, 0.0, 0.0], "max": [xmax, 0.1, 0.1]},
        }

    payload = {
        "status": "PASS",
        "final_draw_visual_items": [
            item("base_link_inertia", 0.0, 0.1),
            item("shoulder_link", 0.1, 0.2),
            item("upper_arm_link", 0.2, 0.3),
            item("forearm_link", 0.3, 0.4),
            item("wrist_1_link", 0.4, 0.5),
            item("wrist_2_link", 0.5, 0.6),
            item("wrist_3_link", 0.6, 0.7),
            item("tool0", 0.7, 0.75),
            item("robotiq_85_base_link", 0.75, 0.85),
            item("gripper_finger1_knuckle_link", 1.30, 1.40),
            item("gripper_finger1_finger_tip_link", 1.40, 1.50),
        ],
    }

    smoke._apply_ur5_rendered_mesh_adjacency(
        payload, repo_root=Path(__file__).resolve().parents[1], scene_name="ur5_2f_test", index_data={}
    )

    assert payload["rendered_mesh_adjacency_status"] == "FAIL"
    assert payload["status"] != "PASS"
    topology = payload["generated_robot_topology_diagnostics"]
    assert topology["status"] == "FAIL"
    assert topology["source"] == "final_draw_visual_items"
    checked = topology["checked_pairs"]
    wrist_tool = next(pair for pair in checked if pair["parent"] == "wrist_3_link" and pair["child"] == "tool0")
    tool_base = next(pair for pair in checked if pair["parent"] == "tool0" and pair["child"] == "robotiq_base")
    exploded_finger = next(pair for pair in checked if pair["parent"] == "robotiq_base" and pair["child"] == "gripper_finger1_knuckle_link")
    finger_tip = next(pair for pair in checked if pair["parent"] == "gripper_finger1_knuckle_link" and pair["child"] == "gripper_finger1_finger_tip_link")
    assert wrist_tool["separation_m"] == pytest.approx(0.0)
    assert tool_base["separation_m"] == pytest.approx(0.0)
    assert exploded_finger["limit_m"] == pytest.approx(0.25)
    assert exploded_finger["separation_m"] == pytest.approx(0.45)
    assert exploded_finger["ok"] is False
    assert finger_tip["separation_m"] == pytest.approx(0.0)
    assert finger_tip["ok"] is True
    assert any("robotiq_base->gripper_finger1_knuckle_link" in error for error in topology["errors"])


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


def test_ur5_rendered_mesh_adjacency_rejects_index_fallback_when_final_draw_missing():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    payload = {"status": "PASS"}
    index_items = [
        {"link": "base_link"},
        {"link": "shoulder_link"},
        {"link": "upper_arm_link"},
    ]
    smoke._apply_ur5_rendered_mesh_adjacency(
        payload,
        repo_root=Path(__file__).resolve().parents[1],
        scene_name="ur5_2f_test",
        index_data={"items": index_items},
    )

    assert payload["rendered_mesh_adjacency_source"] == "missing_final_draw_diagnostics"
    assert payload["rendered_mesh_adjacency_status"] == "FAIL"
    assert payload["rendered_mesh_adjacency_checked_pairs"] == []
    assert payload["rendered_mesh_adjacency_visual_index_supplemental_count"] == len(index_items)
    assert "rendered_mesh_adjacency_used_index_fallback" not in payload.get("warnings", [])
    assert "scene3d_rendered_mesh_adjacency_failed" in payload["warnings"]
    assert payload["rendered_mesh_adjacency_errors"] == [
        "Final Scene3D final_draw_visual_items diagnostics are missing; visual-index metadata cannot prove UR5 arm visibility"
    ]



def test_urdf_flattened_ur5_visual_rows_normalize_link_identity_and_audit_counts():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    expected_meshes = {
        "base_link_inertia": "base.dae",
        "shoulder_link": "shoulder.dae",
        "upper_arm_link": "upperarm.dae",
        "forearm_link": "forearm.dae",
        "wrist_1_link": "wrist1.dae",
        "wrist_2_link": "wrist2.dae",
        "wrist_3_link": "wrist3.dae",
    }

    def raw_generated_row(link: str, mesh_name: str) -> dict:
        package_uri = f"package://ur_description/meshes/ur5/visual/{mesh_name}"
        return {
            "link": link,
            "source": "urdf_flattened",
            "geometry_type": "mesh",
            "package_uri": package_uri,
            "mesh_uri": package_uri,
            "source_path": package_uri,
            "resolved_source_path": f"/opt/ros/humble/share/ur_description/meshes/ur5/visual/{mesh_name}",
            "baked_world_visual_pose": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "render_expected": True,
        }

    raw_ur5_rows = [raw_generated_row(link, mesh_name) for link, mesh_name in expected_meshes.items()]
    assert all("link_name" not in row for row in raw_ur5_rows)

    def final_ingested_row(row: dict, index: int) -> dict:
        return {
            **row,
            "id": f"generated_urdf::{row['link']}::visual_{index}",
            "item_id": f"generated_urdf::{row['link']}",
            "link_name": row["link"],
            "canonical_link_name": row["link"],
            "source_layer": "locked_generated_urdf_visual",
            "category": "robot",
            "role": "robot",
            "robot_model": "ur5",
            "has_mesh_metadata": True,
            "mesh_source": row["package_uri"],
            "mesh_path": row["resolved_source_path"],
            "final_draw_status": "ok",
            "final_draw_bbox": {"min": [float(index), 0.0, 0.0], "max": [float(index) + 0.1, 0.1, 0.1]},
            "visible": True,
            "rendered": True,
        }

    final_rows = [final_ingested_row(row, index) for index, row in enumerate(raw_ur5_rows)]
    final_rows.extend([
        {
            "id": "generated_urdf::gripper_base_link::visual_0",
            "link": "gripper_base_link",
            "link_name": "gripper_base_link",
            "source_layer": "locked_generated_urdf_visual",
            "category": "tool",
            "role": "gripper",
            "has_mesh_metadata": True,
            "mesh_source": "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae",
            "final_draw_status": "ok",
            "final_draw_bbox": {"min": [8.0, 0.0, 0.0], "max": [8.1, 0.1, 0.1]},
            "visible": True,
            "rendered": True,
        },
        {"id": "layout_table", "display_name": "table", "category": "environment", "geometry_type": "box", "visible": True, "rendered": True},
        {"id": "camera_sensor", "display_name": "camera", "category": "camera", "geometry_type": "box", "visible": True, "rendered": True},
    ])
    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "visual_index": {"items": raw_ur5_rows},
        "final_draw_visual_items": final_rows,
        "camera_fit_target": "product_physical_initial_fit_robot_included",
    }

    smoke._apply_ur5_final_viewport_payload_contract(payload)

    for raw_row, final_row in zip(raw_ur5_rows, final_rows):
        assert final_row["link_name"] == raw_row["link"]
        assert final_row["link"] == raw_row["link"]
        assert final_row["source_layer"] == "locked_generated_urdf_visual"
        assert final_row["category"] == "robot"
        assert final_row["robot_model"] == "ur5"
        assert final_row["package_uri"] == final_row["mesh_uri"] == final_row["source_path"]
        assert final_row["mesh_source"] == final_row["package_uri"]

    assert final_rows[7]["category"] == "tool"
    assert payload["rendered_ur5_link_count"] >= 6
    assert payload["missing_required_visible_ur5_links"] == []
    assert payload["table_visible_in_final_viewport"] is True
    assert payload["camera_visible_in_final_viewport"] is True
    assert "ur5_final_viewport_links_missing" not in payload.get("blockers", [])


def test_ur5_candidate_drop_diagnostics_populates_exact_first_rejection_stage():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    payload = {
        "scene": "ur5_2f_test",
        "ur5_final_draw_candidate_diagnostics": [
            {
                "id": "urdf_visual_missing_metadata",
                "source_row_index": 4,
                "link": "shoulder_link",
                "link_name": "shoulder_link",
                "canonical_link_name": "shoulder_link",
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "generated_urdf_visual",
                "role": "robot",
                "category": "robot",
                "mesh_path": "",
                "source_path": "",
                "package_uri": "package://ur_description/meshes/ur5/visual/shoulder.dae",
                "first_rejection_stage": "missing_mesh_metadata",
            },
            {
                "id": "urdf_visual_missing_cache",
                "source_row_index": 5,
                "link": "upper_arm_link",
                "link_name": "upper_arm_link",
                "canonical_link_name": "upper_arm_link",
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "generated_urdf_visual",
                "role": "robot",
                "category": "robot",
                "mesh_path": "package://ur_description/meshes/ur5/visual/upperarm.dae",
                "source_path": "",
                "package_uri": "package://ur_description/meshes/ur5/visual/upperarm.dae",
                "first_rejection_stage": "missing_mesh_cache",
            },
            {
                "id": "urdf_visual_ok",
                "source_row_index": 6,
                "link": "forearm_link",
                "link_name": "forearm_link",
                "canonical_link_name": "forearm_link",
                "source_layer": "locked_generated_urdf_visual",
                "active_visual_source": "generated_urdf_visual",
                "role": "robot",
                "category": "robot",
                "mesh_path": "package://ur_description/meshes/ur5/visual/forearm.dae",
                "source_path": "",
                "package_uri": "package://ur_description/meshes/ur5/visual/forearm.dae",
                "first_rejection_stage": "accepted",
            },
        ],
    }

    smoke._apply_ur5_candidate_drop_diagnostics(payload)

    assert payload["dropped_ur5_row_ids"] == ["urdf_visual_missing_metadata", "urdf_visual_missing_cache"]
    assert payload["dropped_ur5_first_stage"] == {
        "urdf_visual_missing_metadata": "missing_mesh_metadata",
        "urdf_visual_missing_cache": "missing_mesh_cache",
    }


def test_ur5_final_viewport_payload_requires_visible_links_table_and_camera():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    required = [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]
    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "counters": {"generated_fallback_count": 0},
        "final_draw_visual_items": [
            {
                "item_id": f"generated_urdf::{link}",
                "link": link,
                "link_name": link,
                "source_layer": "locked_generated_urdf_visual",
                "has_mesh_metadata": True,
                "mesh_source": f"package://ur_description/meshes/ur5/visual/{link}.dae",
                "final_draw_status": "ok",
                "final_draw_bbox": {"min": [0, 0, 0], "max": [1, 1, 1]},
                "visible": True,
                "rendered": True,
            }
            for link in required
        ]
        + [
            {
                "item_id": "generated_urdf::gripper_base_link",
                "link": "gripper_base_link",
                "source_layer": "locked_generated_urdf_visual",
                "has_mesh_metadata": True,
                "mesh_source": "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae",
                "package_uri": "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae",
                "final_draw_status": "ok",
                "final_draw_bbox": {"min": [1, 0, 0], "max": [2, 1, 1]},
                "visible": True,
                "rendered": True,
            },
            {"id": "layout_table", "display_name": "table", "visible": True, "rendered": True, "geometry_type": "box"},
            {"id": "camera_sensor", "display_name": "camera", "visible": True, "rendered": True, "geometry_type": "box"},
        ],
        "camera_fit_target": "product_physical_initial_fit_robot_included",
    }

    smoke._apply_ur5_final_viewport_payload_contract(payload)

    assert payload["status"] == "PASS"
    assert payload["rendered_ur5_link_count"] >= 6
    assert payload["missing_required_visible_ur5_links"] == []
    assert payload["table_visible_in_final_viewport"] is True
    assert payload["camera_visible_in_final_viewport"] is True
    assert payload["rviz_parity_robot_layer"] is True
    assert payload["ur5_mesh_renderables_count"] >= 7
    assert payload["robotiq_mesh_renderables_count"] >= 1
    assert payload["camera_fit_includes_robot"] is True
    assert "ur5_final_viewport_links_missing" not in payload.get("blockers", [])
    assert "stale_retained_visual_rows_missing_warning" not in payload.get("blockers", [])


def test_ur5_final_viewport_payload_rejects_metadata_or_index_only_names():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "visual_index": {
            "items": [
                {"link": "base_link_inertia"},
                {"link": "shoulder_link"},
                {"link": "upper_arm_link"},
                {"link": "forearm_link"},
                {"link": "wrist_1_link"},
                {"link": "wrist_2_link"},
                {"link": "wrist_3_link"},
            ]
        },
        "metadata": {"robot_links": ["base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]},
        "visible_items": [
            {"id": "layout_table", "display_name": "table", "visible": True, "rendered": True, "geometry_type": "box"},
            {"id": "camera_sensor", "display_name": "camera", "visible": True, "rendered": True, "geometry_type": "box"},
        ],
    }

    smoke._apply_ur5_final_viewport_payload_contract(payload)

    assert payload["status"] == "FAIL"
    assert payload["rendered_ur5_link_count"] == 0
    assert payload["missing_required_visible_ur5_links"] == [
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]
    assert "ur5_final_viewport_links_missing" in payload["blockers"]
    assert "rendered_ur5_link_count_below_7" in payload["blockers"]


def test_ur5_final_viewport_payload_reports_index_to_final_draw_drop():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    required = [
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]
    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "visual_index": {
            "items": [
                {
                    "id": f"urdf_visual_{index}",
                    "source_row_index": index,
                    "link": link,
                    "link_name": link,
                    "canonical_link_name": link,
                    "mesh_uri": f"package://ur_description/meshes/ur5/visual/{link}.dae",
                }
                for index, link in enumerate(required)
            ]
        },
        "visual_ingestion_diagnostics": {
            "generated_urdf_visual_row_diagnostics": [
                {
                    "id": f"urdf_visual_{index}",
                    "source_row_index": index,
                    "link": link,
                    "mesh_uri": f"package://ur_description/meshes/ur5/visual/{link}.dae",
                }
                for index, link in enumerate(required)
            ]
        },
        "final_draw_visual_items": [
            {"id": "layout_table", "display_name": "table", "visible": True, "rendered": True, "geometry_type": "box"},
            {"id": "camera_sensor", "display_name": "camera", "visible": True, "rendered": True, "geometry_type": "box"},
        ],
    }

    smoke._apply_ur5_final_viewport_payload_contract(payload)

    assert payload["indexed_ur5_row_count"] == 6
    assert payload["ingested_ur5_row_count"] == 6
    assert payload["ingested_ur5_row_count_source"] == "visual_ingestion_diagnostics.generated_urdf_visual_row_diagnostics"
    assert payload["final_draw_ur5_row_count_before_filtering"] == 0
    assert payload["final_draw_ur5_row_count"] == 0
    assert payload["final_draw_ur5_row_count_after_filtering"] == 0
    assert payload["dropped_ur5_row_ids"] == [str(index) for index in range(6)]
    assert payload["dropped_ur5_first_stage"] == "final_draw_export"
    assert payload["status"] == "FAIL"


def test_ur5_final_viewport_payload_counts_only_final_render_identity_fields():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    required = [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]
    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "rendered_ur5_link_count": 7,
        "final_draw_visual_items": [
            {
                "item_id": f"generated_urdf::{link}",
                "link_chain": [link],
                "metadata": {"canonical_link_name": link},
                "source_layer": "locked_generated_urdf_visual",
                "has_mesh_metadata": True,
                "mesh_source": f"package://ur_description/meshes/ur5/visual/{link}.dae",
                "final_draw_status": "ok",
                "final_draw_bbox": {"min": [0, 0, 0], "max": [1, 1, 1]},
                "visible": True,
                "rendered": True,
            }
            for link in required[:3]
        ]
        + [
            {
                "link": "forearm_link",
                "source_layer": "layout_preview",
                "has_mesh_metadata": True,
                "final_draw_status": "ok",
                "final_draw_bbox": {"min": [0, 0, 0], "max": [1, 1, 1]},
                "visible": True,
                "rendered": True,
            },
            {
                "link": "wrist_1_link",
                "source_layer": "locked_generated_urdf_visual",
                "has_mesh_metadata": True,
                "final_draw_status": "skipped",
                "final_draw_bbox": {"min": [0, 0, 0], "max": [1, 1, 1]},
                "visible": True,
                "rendered": True,
            },
            {
                "canonical_link_name": "wrist_2_link",
                "source_layer": "generated_urdf_visual",
                "has_mesh_metadata": True,
                "final_draw_status": "ok",
                "final_draw_bbox": {"min": [0, 0, 0], "max": [1, 1, 1]},
                "visible": True,
                "rendered": True,
            },
            {"link_name": "wrist_3_link", "source_layer": "locked_generated_urdf_visual", "final_draw_status": "ok", "visible": False, "rendered": True},
            {"id": "layout_table", "display_name": "table", "visible": True, "rendered": True, "geometry_type": "box"},
            {"id": "camera_sensor", "display_name": "camera", "visible": True, "rendered": True, "geometry_type": "box"},
        ],
    }

    smoke._apply_ur5_final_viewport_payload_contract(payload)

    assert payload["rendered_ur5_link_count"] == 1
    assert payload["missing_required_visible_ur5_links"] == [
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_3_link",
    ]
    assert "ur5_final_viewport_links_missing" in payload["blockers"]


def test_ur5_final_viewport_payload_fails_stale_retained_rows_warning_when_links_visible():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    required = [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]
    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "warnings": ["Preview warning: ur5_2f_test retained visual rows missing after loader filtering"],
        "final_draw_visual_items": [
            {
                "link": link,
                "source_layer": "locked_generated_urdf_visual",
                "has_mesh_metadata": True,
                "mesh_source": f"package://ur_description/meshes/ur5/visual/{link}.dae",
                "final_draw_status": "ok",
                "final_draw_bbox": {"min": [0, 0, 0], "max": [1, 1, 1]},
            }
            for link in required
        ]
        + [
            {
                "link": "gripper_base_link",
                "source_layer": "locked_generated_urdf_visual",
                "has_mesh_metadata": True,
                "mesh_source": "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae",
                "final_draw_status": "ok",
                "final_draw_bbox": {"min": [1, 0, 0], "max": [2, 1, 1]},
            },
            {"id": "table", "display_name": "table", "geometry_type": "box"},
            {"id": "camera", "display_name": "camera", "geometry_type": "box"},
        ],
        "camera_fit_target": "product_physical_initial_fit_robot_included",
    }

    smoke._apply_ur5_final_viewport_payload_contract(payload)

    assert payload["missing_required_visible_ur5_links"] == []
    assert "stale_retained_visual_rows_missing_warning" in payload["blockers"]
    assert payload["status"] == "FAIL"


def test_generated_robot_fallback_required_when_generated_mesh_rows_are_not_renderable():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "counters": {"generated_fallback_count": 3},
        "final_draw_visual_items": [
            {
                "link": "base_link_inertia",
                "link_name": "base_link_inertia",
                "source_layer": "locked_generated_urdf_visual",
                "has_mesh_metadata": True,
                "mesh_source": "package://ur_description/meshes/visual/base.dae",
                "final_draw_status": "missing_mesh_cache",
            },
            {"id": "table", "display_name": "table", "geometry_type": "box"},
            {"id": "camera", "display_name": "camera", "geometry_type": "box"},
        ],
    }

    smoke._apply_ur5_final_viewport_payload_contract(payload)

    assert payload["generated_robot_fallback_required"] is True
    assert "generated_robot_fallback_not_activated" not in payload.get("blockers", [])


def test_generated_robot_fallback_blocks_when_generated_mesh_rows_missing_renderables_without_fallback():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "counters": {"generated_fallback_count": 0},
        "final_draw_visual_items": [
            {
                "link": "base_link_inertia",
                "source_layer": "locked_generated_urdf_visual",
                "has_mesh_metadata": True,
                "mesh_source": "package://ur_description/meshes/visual/base.dae",
                "final_draw_status": "missing_mesh_cache",
            },
            {"id": "table", "display_name": "table", "geometry_type": "box"},
            {"id": "camera", "display_name": "camera", "geometry_type": "box"},
        ],
    }

    smoke._apply_ur5_final_viewport_payload_contract(payload)

    assert payload["generated_robot_fallback_required"] is True
    assert "generated_robot_fallback_not_activated" in payload["blockers"]
    assert payload["status"] == "FAIL"


def test_generated_urdf_visual_first_drop_smoke_stage_marks_final_draw_audit_only():
    from scripts.run_workcell_builder_scene3d_gui_smoke import _apply_generated_urdf_visual_first_drop_smoke_stage

    payload = {
        "visual_ingestion_diagnostics": {
            "generated_urdf_visual_row_diagnostics": [
                {"id": "generated_urdf::base_link::visual_0::0", "visual_name": "visual_0"},
                {"id": "generated_urdf::shoulder_link::visual_1::1", "visual_name": "visual_1", "first_drop_stage": "visual_index_loop_skip"},
            ]
        },
        "final_draw_visual_items": [
            {"item_id": "generated_urdf::base_link::visual_0::0", "final_draw_status": "ok"},
            {"item_id": "generated_urdf::shoulder_link::visual_1::1", "final_draw_status": "ok"},
        ],
    }

    _apply_generated_urdf_visual_first_drop_smoke_stage(payload)

    rows = payload["visual_ingestion_diagnostics"]["generated_urdf_visual_row_diagnostics"]
    assert rows[0]["first_drop_stage"] == "smoke_output_or_audit_only"
    assert rows[1]["first_drop_stage"] == "visual_index_loop_skip"


def test_ur5_2f_real_visual_mesh_index_preserves_visual_number_stages_and_final_ur5_identity():
    import re
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    repo = Path(__file__).resolve().parents[1]
    fixture_path = repo / "scenes/ur5_2f_test/generated/scene_visual_mesh_index.json"
    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    source_rows = fixture["visual_items"]
    expected_visual_numbers = list(range(18))
    source_visual_numbers = sorted(
        int(match.group(1))
        for row in source_rows
        for match in [re.search(r"urdf_visual_(\d+)_", str(row.get("id") or ""))]
        if match
    )
    assert source_visual_numbers == expected_visual_numbers

    required_ur5_links = list(smoke.REQUIRED_UR5_FINAL_VIEWPORT_LINKS)

    def final_row(source_row: dict, visual_number: int) -> dict:
        link = source_row["link"]
        row = {
            "id": f"generated_urdf::{link}::visual_{visual_number}",
            "item_id": f"generated_urdf::{link}::visual_{visual_number}",
            "source_layer": "locked_generated_urdf_visual",
            "has_mesh_metadata": True,
            "mesh_source": source_row.get("package_uri") or source_row.get("mesh_uri") or f"package://fixture/{link}.dae",
            "final_draw_status": "ok",
            "final_draw_bbox": {"min": [float(visual_number), 0.0, 0.0], "max": [float(visual_number) + 0.1, 0.1, 0.1]},
            "visible": True,
            "rendered": True,
        }
        if link in required_ur5_links:
            # Countable UR5 rows must be actual final draw identity rows.
            row["canonical_link_name"] = link
            row["link"] = link
            row["link_name"] = link
        else:
            # Gripper/table/camera rows may carry UR5 chain metadata from the handoff,
            # but must not count as rendered UR5 links unless a final draw identity
            # field is itself a required UR5 link.
            row["canonical_link_name"] = link
            row["link"] = link
            row["link_name"] = link
            row["link_chain"] = required_ur5_links
            row["metadata"] = {"link_chain": required_ur5_links, "canonical_link_name": required_ur5_links[0]}
        return row

    final_rows = [final_row(row, number) for number, row in zip(source_visual_numbers, source_rows)]
    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "runtime_available": True,
        "filter_diagnostics": {
            "generated_urdf_visual_numbers_after_ingest": expected_visual_numbers,
            "generated_urdf_visual_numbers_after_suppression": expected_visual_numbers,
            "generated_urdf_visual_numbers_after_filter": expected_visual_numbers,
        },
        "final_draw_visual_items": final_rows,
        "camera_fit_target": "product_physical_initial_fit_robot_included",
    }

    smoke._add_smoke_report_supplemental_evidence(payload, screenshot_path=None, screenshot_available=False)
    smoke._apply_ur5_final_viewport_payload_contract(payload)

    assert payload["generated_urdf_visual_numbers_after_ingest"] == expected_visual_numbers
    assert payload["generated_urdf_visual_numbers_after_suppression"] == expected_visual_numbers
    assert payload["generated_urdf_visual_numbers_after_filter"] == expected_visual_numbers
    assert payload["after_ingest"] == expected_visual_numbers
    assert payload["after_suppression"] == expected_visual_numbers
    assert payload["after_filter"] == expected_visual_numbers

    final_ids = {str(row.get("id") or row.get("item_id") or "") for row in final_rows}
    assert "generated_urdf::base_link_inertia::visual_0" in final_ids
    for link in required_ur5_links[1:]:
        assert any(
            row.get("canonical_link_name") == link
            and row.get("link") == link
            and row.get("link_name") == link
            and row.get("final_draw_status") == "ok"
            for row in final_rows
        ), f"missing canonical final draw row for {link}"

    assert payload["rendered_ur5_link_count"] == len(required_ur5_links) - 1
    gripper_rows = [row for row in final_rows if str(row.get("link") or "").startswith("gripper_")]
    assert gripper_rows
    assert all(row.get("link_chain") == required_ur5_links for row in gripper_rows)
    assert payload["rendered_ur5_link_count"] == len(required_ur5_links) - 1
    assert not any(
        f"::visual_{visual_number}" in final_id and "::dedupe_1" in final_id
        for visual_number in range(9, 18)
        for final_id in final_ids
    )


def test_wrapper_promotes_app_json_present_when_complete_pass_evidence(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / "smoke.json"
    shot = tmp_path / "shot.png"
    exe = tmp_path / "fake_workcell_builder_complete_evidence.py"
    exe.write_text(
        "#!/usr/bin/env python3\n"
        "import json, pathlib, sys\n"
        "out = pathlib.Path(sys.argv[sys.argv.index('--smoke-output') + 1])\n"
        "shot = pathlib.Path(sys.argv[sys.argv.index('--smoke-screenshot') + 1])\n"
        "shot.write_bytes(b'fake-png')\n"
        "payload = {\n"
        "  'schema': 'workcell_studio_scene3d_gui_smoke/v1',\n"
        "  'status': 'PASS',\n"
        "  'app_status': 'PASS',\n"
        "  'runtime_available': True,\n"
        "  'screenshot_available': True,\n"
        "  'screenshot_path': str(shot),\n"
        "  'visual_quality_status': 'PASS',\n"
        "  'visual_diagnostics': {'status': 'PASS'},\n"
        "  'render_debug_counters': {'physical_mesh_items_rendered': 6},\n"
        "  'rendered_ur5_link_count': 6,\n"
        "  'missing_required_visible_ur5_links': [],\n"
        "  'counters': {'rendered_count': 12}\n"
        "}\n"
        "out.write_text(json.dumps(payload) + '\\n', encoding='utf-8')\n",
        encoding="utf-8",
    )
    exe.chmod(0o755)
    cmd = [
        sys.executable, str(repo / "scripts/run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root", str(repo), "--workspace-root", str(tmp_path), "--scene", "ur5_2f_test",
        "--output", str(out), "--screenshot", str(shot), "--executable", str(exe), "--timeout-sec", "2",
    ]
    proc = subprocess.run(cmd, text=True, capture_output=True, env={**os.environ, "ROS_DISTRO": "humble"})
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "status=PASS smoke_status=APP_JSON_PRESENT wrapper_status=PASS app_status=PASS returncode=0 timed_out=False" in proc.stdout
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["status"] == "PASS"
    assert payload["wrapper_status"] == "PASS"
    assert payload["runtime_available"] is True
    assert payload["screenshot_available"] is True
    assert payload["rendered_ur5_link_count"] >= 6
    assert payload["missing_required_visible_ur5_links"] == []
    assert payload["visual_quality_status"] == "PASS"


def test_fresh_real_xacro_urdf_visual_rows_are_counted_from_visual_index_identity():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    required_links = [
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]
    visual_items = [
        {"id": "urdf_visual_0", "source_row_index": 0, "link": "base_link_inertia", "visual_name": "visual_0"},
        *[
            {"id": f"urdf_visual_{index}", "source_row_index": index, "link": link, "visual_name": f"visual_{index}"}
            for index, link in enumerate(required_links, start=1)
        ],
        {"id": "urdf_visual_7", "source_row_index": 7, "link": "gripper_base_link", "visual_name": "visual_7"},
        {"id": "urdf_visual_8", "source_row_index": 8, "link": "left_inner_finger", "visual_name": "visual_8"},
    ]
    final_rows = [
        {
            "id": row["id"],
            "item_id": row["id"],
            "source_row_index": row["source_row_index"],
            "source_layer": "locked_generated_urdf_visual",
            "final_draw_status": "ok",
            "has_mesh_metadata": True,
            "mesh_source": "package://ur_description/meshes/ur5/visual/placeholder.dae",
            "final_draw_bbox": {"min": [float(index), 0.0, 0.0], "max": [float(index) + 0.1, 0.1, 0.1]},
            "visible": True,
            "rendered": True,
        }
        for index, row in enumerate(visual_items)
    ]
    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "visual_index": {"visual_items": visual_items},
        "final_draw_visual_items": final_rows,
        "camera_fit_target": "product_physical_initial_fit_robot_included",
    }

    smoke._apply_ur5_final_viewport_payload_contract(payload)

    assert payload["rendered_ur5_link_count"] == 6
    assert payload["missing_required_visible_ur5_links"] == []
    assert payload["ur5_mesh_renderables_count"] >= 6
    assert payload["robotiq_mesh_renderables_count"] > 0
    assert "ur5_mesh_renderables_count_below_required_links" not in payload.get("blockers", [])
    assert "ur5_final_viewport_links_missing" not in payload.get("blockers", [])
    assert "rendered_ur5_link_count_below_6" not in payload.get("blockers", [])


def test_old_generated_urdf_row_identities_still_count_without_visual_index():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    required_links = [
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ]
    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "final_draw_visual_items": [
            {
                "id": f"generated_urdf::{link}::visual_{index}::{index}",
                "item_id": f"generated_urdf::{link}::visual_{index}::{index}",
                "source_row_index": index,
                "link": link,
                "link_name": link,
                "canonical_link_name": link,
                "source_layer": "locked_generated_urdf_visual",
                "final_draw_status": "ok",
                "has_mesh_metadata": True,
                "mesh_source": "package://ur_description/meshes/ur5/visual/placeholder.dae",
                "final_draw_bbox": {"min": [float(index), 0.0, 0.0], "max": [float(index) + 0.1, 0.1, 0.1]},
                "visible": True,
                "rendered": True,
            }
            for index, link in enumerate(required_links, start=1)
        ],
        "camera_fit_target": "product_physical_initial_fit_robot_included",
    }

    smoke._apply_ur5_final_viewport_payload_contract(payload)

    assert payload["rendered_ur5_link_count"] == 6
    assert payload["missing_required_visible_ur5_links"] == []
    assert payload["ur5_mesh_renderables_count"] >= 6


def test_fresh_visual_index_ur5_and_robotiq_rows_survive_final_renderable_contract():
    import scripts.run_workcell_builder_scene3d_gui_smoke as smoke

    ur5_rows = [
        ("base_link_inertia", "base.dae"),
        ("shoulder_link", "shoulder.dae"),
        ("upper_arm_link", "upperarm.dae"),
        ("forearm_link", "forearm.dae"),
        ("wrist_1_link", "wrist1.dae"),
        ("wrist_2_link", "wrist2.dae"),
        ("wrist_3_link", "wrist3.dae"),
    ]
    robotiq_rows = [
        ("robotiq_85_base_link", "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae"),
        ("left_inner_finger", "package://robotiq_85_description/meshes/visual/inner_finger.dae"),
    ]
    visual_items = [
        {
            "id": f"urdf_visual_{index}",
            "item_id": f"urdf_visual_{index}",
            "source_row_index": index,
            "link": link,
            "link_name": link,
            "visual_name": f"visual_{index}",
            "package_uri": f"package://ur_description/meshes/ur5/visual/{mesh}",
            "source_layer": "locked_generated_urdf_visual",
            "active_visual_source": "generated_urdf_visual",
        }
        for index, (link, mesh) in enumerate(ur5_rows)
    ] + [
        {
            "id": f"urdf_visual_{index}",
            "item_id": f"urdf_visual_{index}",
            "source_row_index": index,
            "link": link,
            "link_name": link,
            "visual_name": f"visual_{index}",
            "package_uri": package_uri,
            "source_layer": "locked_generated_urdf_visual",
            "active_visual_source": "generated_urdf_visual",
        }
        for index, (link, package_uri) in enumerate(robotiq_rows, start=len(ur5_rows))
    ]
    final_rows = [
        {
            "id": row["id"],
            "item_id": row["item_id"],
            "source_row_index": row["source_row_index"],
            "source_layer": row["source_layer"],
            "active_visual_source": row["active_visual_source"],
            "role": "generated_robot_visual",
            "category": "URDF Visual",
            "final_draw_status": "ok",
            "has_mesh_metadata": True,
            "mesh_source": row["package_uri"],
            "final_draw_bbox": {
                "min": [float(row["source_row_index"]), 0.0, 0.0],
                "max": [float(row["source_row_index"]) + 0.1, 0.1, 0.1],
            },
            "visible": True,
            "rendered": True,
        }
        for row in visual_items
    ]
    payload = {
        "scene": "ur5_2f_test",
        "status": "PASS",
        "visual_index": {"visual_items": visual_items},
        "final_draw_visual_items": final_rows,
        "camera_fit_target": "product_physical_initial_fit_robot_included",
    }

    enriched_rows = [smoke._enrich_final_row_from_visual_index(row, payload) for row in final_rows]
    assert {row["canonical_link_name"] for row in enriched_rows if row["mesh_source"].startswith("package://ur_description/")} >= {
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    }
    assert any("robotiq_85_description" in row["mesh_source"] for row in enriched_rows)
    assert all(row["source_layer"] in smoke.UR5_FINAL_RENDER_SOURCE_LAYERS for row in enriched_rows)
    assert not any(
        str(row.get("role", "")).lower() in {"overlay", "helper"}
        or str(row.get("category", "")).lower() in {"overlay", "helper", "semantic-only primitive"}
        for row in enriched_rows
        if row["mesh_source"].startswith("package://ur_description/")
    )

    smoke._apply_ur5_final_viewport_payload_contract(payload)

    assert payload["rendered_ur5_link_count"] == len(smoke.REQUIRED_UR5_FINAL_VIEWPORT_LINKS)
    assert payload["missing_required_visible_ur5_links"] == []
    assert payload["ur5_mesh_renderables_count"] >= len(smoke.REQUIRED_UR5_FINAL_VIEWPORT_LINKS)
    assert payload["robotiq_mesh_renderables_count"] >= 1
    assert "ur5_mesh_renderables_count_below_required_links" not in payload.get("blockers", [])
    assert "ur5_final_viewport_links_missing" not in payload.get("blockers", [])


def test_scene3d_final_draw_export_uses_generated_urdf_renderable_assembly():
    source = (Path(__file__).resolve().parents[1] / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(
        encoding="utf-8"
    )
    assembly_body = source.split("std::vector<const ScenePreviewWidget::PreviewItem *> build_final_generated_urdf_robot_renderables", 1)[1]
    assembly_body = assembly_body.split("bool is_critical_label_role", 1)[0]
    export_body = source.split("QJsonArray Scene3DViewportWidget::final_draw_visual_items_export", 1)[1]
    export_body = export_body.split("void Scene3DViewportWidget::emit_render_debug_counters", 1)[0]

    assert "const bool generated_or_locked = is_generated_urdf_visual_item(item) || is_locked_urdf_item(item);" in assembly_body
    assert "const bool overlay_helper = !generated_or_locked && (is_overlay_only_item(item) || is_overlay_visual_role(role));" in assembly_body
    assert "if (generated_or_locked && generated_urdf_item_has_renderable_geometry(item))" in assembly_body
    assert "generated_robot_items.push_back(&item);" in assembly_body
    assert "ordered_items.insert(ordered_items.end(), generated_robot_items.begin(), generated_robot_items.end());" in assembly_body
    assert "build_final_generated_urdf_robot_renderables(items, show_safety);" in export_body
    assert 'row["canonical_link"] = canonical_link_name;' in export_body
    assert 'row["canonical_link_name"] = canonical_link_name;' in export_body
    assert 'row["frame_id"] = !item.frame_id.trimmed().isEmpty() ? item.frame_id.trimmed() : canonical_link_name;' in export_body
    assert 'row["visual_index_link"] = !item.visual_index_link.trimmed().isEmpty() ? item.visual_index_link.trimmed() : link_name;' in export_body
    assert 'row["mesh_uri"] = !item.visual_index_mesh_uri.trimmed().isEmpty() ? item.visual_index_mesh_uri.trimmed() : mesh_source;' in export_body
    assert 'row["package_uri"] = !item.visual_index_package_uri.trimmed().isEmpty() ? item.visual_index_package_uri.trimmed() : item.package_uri;' in export_body
    assert 'const QMatrix4x4 baked_transform = authoritative_world_visual_transform(item);' in export_body
    assert 'const QMatrix4x4 viewport_root_transform = viewport_world_visual_transform(item);' in export_body
    assert 'const QMatrix4x4 final_transform = final_mesh_transform_matrix(item);' in export_body
    assert 'row["baked_world_visual_matrix"] = scene3d_matrix_to_json(baked_transform);' in export_body
    assert 'row["viewport_world_visual_matrix"] = scene3d_matrix_to_json(viewport_root_transform);' in export_body
    assert 'row["final_draw_model_matrix"] = scene3d_matrix_to_json(final_transform);' in export_body
    assert 'scene3d_final_draw_bbox_for_mesh(cache.mesh, final_transform, final_min, final_max)' in export_body
