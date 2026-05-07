from __future__ import annotations
import json, subprocess, sys
from pathlib import Path


def test_generate_and_validate_readiness_pack(tmp_path: Path):
    out = tmp_path / "pack"
    cmd = [sys.executable, "scripts/workcell_studio.py", "generate-readiness-pack", "--scene-package", "scenes/ur5_2f_test", "--output-dir", str(out), "--project-name", "demo", "--validate", "--smoke-dry-run", "--force", "--json"]
    run = subprocess.run(cmd, capture_output=True, text=True, check=False)
    assert run.returncode in (0,1)
    manifest = out / "readiness_pack_manifest.json"
    assert manifest.exists()
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    assert payload["schema"] == "workcell_studio_readiness_pack/v1"
    assert payload["safety"]["motion_command_sent"] is False
    assert "perception_bridge" in payload
    assert payload["perception_bridge"]["status"] in ("bridge_preview_ready", "bridge_preview_partial", "bridge_preview_blocked")
    assert payload["plan_preview_handoff"]["status"] in ("plan_preview_ready", "plan_preview_partial", "plan_preview_blocked")
    if "emd_bridge_payload_preview" in payload["artifacts"]:
        assert Path(payload["artifacts"]["emd_bridge_payload_preview"]).exists()
    vrun = subprocess.run([sys.executable, "scripts/workcell_studio.py", "validate-readiness-pack", "--manifest", str(manifest), "--json"], capture_output=True, text=True, check=False)
    assert vrun.returncode == 0


def test_unsafe_manifest_fails(tmp_path: Path):
    manifest = tmp_path / "m.json"
    manifest.write_text(json.dumps({"schema":"workcell_studio_readiness_pack/v1","safety":{"motion_command_sent":True},"results":{"final_readiness":"FAIL"},"artifacts":{}}), encoding="utf-8")
    run = subprocess.run([sys.executable, "scripts/validate_workcell_studio_readiness_pack.py", str(manifest), "--json"], capture_output=True, text=True, check=False)
    assert run.returncode != 0


def test_no_dashboard_option_and_generate_dashboard_cli(tmp_path: Path):
    out = tmp_path / 'pack2'
    run = subprocess.run([sys.executable,'scripts/workcell_studio.py','generate-readiness-pack','--scene-package','scenes/ur5_2f_test','--output-dir',str(out),'--project-name','demo','--no-dashboard','--force','--json'], capture_output=True, text=True, check=False)
    assert run.returncode in (0,1)
    assert not (out/'readiness_dashboard.html').exists()
    rerun = subprocess.run([sys.executable,'scripts/workcell_studio.py','generate-readiness-dashboard','--manifest',str(out/'readiness_pack_manifest.json'),'--output',str(out/'readiness_dashboard_regenerated.html'),'--json'], capture_output=True, text=True, check=False)
    assert rerun.returncode == 0


def test_pack_with_task_intent_includes_task_flow_summary(tmp_path: Path):
    import shutil
    scene = tmp_path / 'scene'
    shutil.copytree('scenes/ur5_2f_test', scene)
    gen = scene / 'generated'
    gen.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable,'scripts/create_or_update_builder_task_intent.py','--scene-package',str(scene),'--task-id','sorting_task_001','--task-type','pick_place','--pick-source','pick_zone_main','--place-target','bin_red','--grasp-strategy','finger_pinch_basic','--approach-axis','z_down','--approach-distance-m','0.12','--retreat-axis','z_up','--retreat-distance-m','0.10','--release-strategy','tool_release','--output',str(gen/'workcell_builder_task_intent.yaml'),'--validate','--json'],check=True)
    out = tmp_path / 'pack_task'
    run = subprocess.run([sys.executable, 'scripts/workcell_studio.py', 'generate-readiness-pack', '--scene-package', str(scene), '--output-dir', str(out), '--project-name', 'demo', '--validate', '--smoke-dry-run', '--force', '--json'], capture_output=True, text=True, check=False)
    assert run.returncode in (0,1)
    manifest = json.loads((out/'readiness_pack_manifest.json').read_text(encoding='utf-8'))
    tf = Path(manifest['artifacts']['task_flow_summary'])
    assert tf.exists()
    assert manifest['results']['task_flow_status'] in ('PASS','WARN')


def test_pack_static_preview_receives_task_flow(tmp_path: Path):
    import shutil
    scene = tmp_path / 'scene_preview'
    shutil.copytree('scenes/ur5_2f_test', scene)
    gen = scene / 'generated'
    gen.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable,'scripts/create_or_update_builder_task_intent.py','--scene-package',str(scene),'--task-id','sorting_task_001','--task-type','pick_place','--pick-source','pick_zone_main','--place-target','bin_red','--grasp-strategy','finger_pinch_basic','--approach-axis','z_down','--approach-distance-m','0.12','--retreat-axis','z_up','--retreat-distance-m','0.10','--release-strategy','tool_release','--output',str(gen/'workcell_builder_task_intent.yaml'),'--validate','--json'],check=True)
    out = tmp_path / 'pack_preview'
    subprocess.run([sys.executable, 'scripts/workcell_studio.py', 'generate-readiness-pack', '--scene-package', str(scene), '--output-dir', str(out), '--project-name', 'demo', '--validate', '--smoke-dry-run', '--force', '--json'], check=False)
    summary = json.loads((out/'preview'/'static_preview_summary.json').read_text(encoding='utf-8'))
    tf = summary.get('task_flow_summary', {})
    assert tf.get('status') in ('PASS', 'WARN')
    assert tf.get('pick_source_id') == 'pick_zone_main'
    assert tf.get('place_target_id') == 'bin_red'
    assert tf.get('grasp_strategy') == 'finger_pinch_basic'
    assert tf.get('release_strategy') == 'tool_release'


def test_pack_preserves_authored_targets_and_resolves_coordinates(tmp_path: Path):
    import shutil
    import yaml
    scene = tmp_path / 'scene_authored_targets'
    shutil.copytree('scenes/ur5_2f_test', scene)
    gen = scene / 'generated'
    gen.mkdir(parents=True, exist_ok=True)
    layout = gen / 'environment_layout.yaml'

    subprocess.run([sys.executable,'scripts/create_or_update_environment_target.py','--environment-layout',str(layout),'--target-id','pick_zone_main','--target-type','pick_zone','--label','Main pick zone','--frame','world','--xyz','0.45','0.00','0.08','--rpy','0','0','0','--size','0.30','0.20','0.10','--output',str(layout),'--json'], check=True)
    subprocess.run([sys.executable,'scripts/create_or_update_environment_target.py','--environment-layout',str(layout),'--target-id','bin_red','--target-type','place_target','--label','Red bin','--frame','world','--xyz','0.35','0.35','0.10','--rpy','0','0','0','--size','0.20','0.20','0.15','--output',str(layout),'--json'], check=True)
    subprocess.run([sys.executable,'scripts/create_or_update_builder_task_intent.py','--scene-package',str(scene),'--task-id','sorting_task_001','--task-type','pick_place','--pick-source','pick_zone_main','--place-target','bin_red','--grasp-strategy','finger_pinch_basic','--approach-axis','z_down','--approach-distance-m','0.12','--retreat-axis','z_up','--retreat-distance-m','0.10','--release-strategy','tool_release','--output',str(gen/'workcell_builder_task_intent.yaml'),'--validate','--json'],check=True)

    out = tmp_path / 'pack_authored_targets'
    run = subprocess.run([sys.executable, 'scripts/workcell_studio.py', 'generate-readiness-pack', '--scene-package', str(scene), '--output-dir', str(out), '--project-name', 'demo', '--validate', '--smoke-dry-run', '--force', '--json'], capture_output=True, text=True, check=False)
    assert run.returncode in (0,1)

    exported_layout = yaml.safe_load((out/'exported'/'environment_layout.yaml').read_text(encoding='utf-8'))
    zone_ids = {z.get('id') for z in (exported_layout.get('zones') or []) if isinstance(z, dict)}
    target_ids = {z.get('id') for z in (exported_layout.get('targets') or []) if isinstance(z, dict)}
    assert 'pick_zone_main' in zone_ids and 'bin_red' in zone_ids
    assert 'pick_zone_main' in target_ids and 'bin_red' in target_ids

    tf = json.loads((out/'task'/'task_flow_summary.json').read_text(encoding='utf-8'))
    vr = tf.get('visual_resolution', {})
    assert vr.get('pick_coordinates_resolved') is True
    assert vr.get('place_coordinates_resolved') is True
    assert vr.get('approximate_coordinates_used') is False
    assert 'Task flow present but exact pick/place coordinates could not be resolved.' not in (tf.get('warnings') or [])

    preview = json.loads((out/'preview'/'static_preview_summary.json').read_text(encoding='utf-8'))
    pvr = (preview.get('task_flow_summary') or {}).get('visual_resolution', {})
    assert pvr.get('pick_coordinates_resolved') is True
    assert pvr.get('place_coordinates_resolved') is True
    assert pvr.get('approximate_coordinates_used') is False
    preview_warnings = json.dumps(preview)
    assert 'exact pick/place coordinates could not be resolved' not in preview_warnings

    dashboard = (out/'readiness_dashboard.html').read_text(encoding='utf-8')
    assert 'pick_zone_main' in dashboard
    assert 'bin_red' in dashboard

def test_pack_visual_markers_and_safety_banner(tmp_path: Path):
    out = tmp_path / "pack_visual"
    run = subprocess.run([sys.executable, "scripts/workcell_studio.py", "generate-readiness-pack", "--scene-package", "scenes/ur5_2f_test", "--output-dir", str(out), "--project-name", "demo", "--validate", "--smoke-dry-run", "--force", "--json"], capture_output=True, text=True, check=False)
    assert run.returncode in (0, 1)
    manifest = json.loads((out / "readiness_pack_manifest.json").read_text(encoding="utf-8"))
    marker_path = Path(manifest["artifacts"]["static_preview"]["markers"])
    assert marker_path.exists()
    markers = json.loads(marker_path.read_text(encoding="utf-8"))
    assert markers.get("task_flow_marker_count", 0) > 0
    names = {m.get("name") for m in markers.get("markers", []) if isinstance(m, dict)}
    assert "pick_zone_source" in names and "place_zone_target" in names
    preview_summary = json.loads((out / "preview" / "static_preview_summary.json").read_text(encoding="utf-8"))
    assert preview_summary.get("safety_banner_present") is True
    dashboard = (out / "readiness_dashboard.html").read_text(encoding="utf-8")
    assert "Perception → Task Bridge Preview" in dashboard
    assert "Bridge Payload → RViz/MoveIt Plan Preview" in dashboard
