#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, shutil, subprocess, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
from workcell_studio_layout_source import resolve_saved_layout_path

def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}

def _run_json(cmd:list[str], label:str)->tuple[int,dict[str,Any]]:
    p=subprocess.run(cmd,capture_output=True,text=True,check=False)
    try:data=json.loads(p.stdout) if p.stdout.strip() else {}
    except Exception:data={"result":"FAIL","errors":[f"{label} non-json",p.stdout.strip(),p.stderr.strip()]}
    data.setdefault("stdout",p.stdout.strip());data.setdefault("stderr",p.stderr.strip());data.setdefault("returncode",p.returncode)
    return p.returncode,data



def _write_perception_profile(scene: Path) -> Path:
    generated = scene / 'generated'
    generated.mkdir(parents=True, exist_ok=True)
    profile_path = generated / 'perception_profile.yaml'
    snapshot = Path('tests/fixtures/perception/detected_objects_snapshot_golden.yaml').resolve()
    profile = {
        'schema': 'workcell_perception_profile/v1',
        'sensor': {'type': 'realsense_d435i', 'camera_frame': 'camera_color_optical_frame'},
        'topics': {
            'rgb': '/camera/color/image_raw',
            'depth': '/camera/aligned_depth_to_color/image_raw',
            'camera_info': '/camera/color/camera_info',
            'point_cloud': '/camera/depth/color/points',
            'epd_localization_output': '/epd/localisation/detected_objects',
            'epd_tracking_output': '/epd/tracking/detected_objects',
        },
        'expected_snapshot_path': str(snapshot),
        'frames': {'object_frame': 'world', 'scene_frame': 'world'},
        'qos_notes': 'Use sensor-data QoS for camera topics when available; use reliable QoS for EPD outputs where supported.',
        'safety_mode': {
            'perception_only': True,
            'no_robot_motion': True,
            'no_runtime_execution': True,
            'fake_hardware_default': True,
        },
    }
    import yaml  # type: ignore
    profile_path.write_text(yaml.safe_dump(profile, sort_keys=False), encoding='utf-8')
    return profile_path


def _generate_perception_readiness(profile_path: Path, snapshot_path: Path, output: Path) -> tuple[str, list[str], list[str]]:
    import yaml  # type: ignore
    prof = yaml.safe_load(profile_path.read_text(encoding='utf-8')) if profile_path.exists() else {}
    snap = yaml.safe_load(snapshot_path.read_text(encoding='utf-8')) if snapshot_path.exists() else {}
    warnings, blockers = [], []

    objects = snap.get('objects') if isinstance(snap, dict) and isinstance(snap.get('objects'), list) else []
    scene_frame = (prof.get('frames') or {}).get('scene_frame') if isinstance(prof, dict) else None
    frame_ok = False
    routeable = False
    in_pick_zone_hint = False
    selected_obj = None

    for idx, obj in enumerate(objects):
        if not isinstance(obj, dict):
            warnings.append(f"Skipping detected object index {idx}: object payload is not a mapping.")
            continue

        pose = obj.get('pose', {}) if isinstance(obj.get('pose'), dict) else {}
        xyz = pose.get('xyz', []) if isinstance(pose, dict) else []
        if not (isinstance(xyz, list) and len(xyz) >= 3):
            warnings.append(
                f"Skipping detected object '{obj.get('id', f'index_{idx}')}' due to incomplete pose.xyz vector (size={len(xyz) if isinstance(xyz, list) else 'n/a'})."
            )
            continue

        try:
            x = float(xyz[0])
            y = float(xyz[1])
        except (TypeError, ValueError):
            warnings.append(
                f"Skipping detected object '{obj.get('id', f'index_{idx}')}' due to non-numeric pose.xyz values."
            )
            continue

        selected_obj = obj
        frame_ok = obj.get('frame_id') == scene_frame
        if not frame_ok:
            warnings.append('Detected object frame does not match scene frame')

        label = obj.get('label')
        routeable = isinstance(label, str) and bool(label.strip())
        in_pick_zone_hint = 0.0 <= x <= 1.5 and -1.0 <= y <= 1.0
        break

    if selected_obj is None:
        blockers.append('No valid detected object with complete pose.xyz[0..2] available for readiness checks.')

    status = 'perception_replay_ready'
    if not routeable:
        blockers.append('Object label/class missing for routing check')
        status = 'perception_partial'
    report = {
        'schema': 'perception_readiness_report/v1',
        'status': status if not blockers else 'perception_partial',
        'profile_path': str(profile_path),
        'detected_object_snapshot': str(snapshot_path),
        'checks': {
            'frame_match': frame_ok,
            'label_routeable': routeable,
            'pose_inside_pick_zone_hint': bool(in_pick_zone_hint),
        },
        'safety': {'dry_run_only': True, 'motion_command_sent': False, 'runtime_execution_called': False, 'perception_only': True},
        'warnings': warnings,
        'blockers': blockers,
    }
    output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return report['status'], warnings, blockers
def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--scene-package',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--project-name',required=True)
    ap.add_argument('--validate',action='store_true');ap.add_argument('--prepare-rviz-preview',action='store_true');ap.add_argument('--smoke-dry-run',action='store_true')
    ap.add_argument('--smoke-execute',action='store_true');ap.add_argument('--smoke-timeout-s',type=int,default=20);ap.add_argument('--strict',action='store_true')
    ap.add_argument('--continue-on-error',action='store_true');ap.add_argument('--no-dashboard',action='store_true');ap.add_argument('--force',action='store_true');ap.add_argument('--json',action='store_true')
    a=ap.parse_args(); scene=a.scene_package.resolve(); out=a.output_dir.resolve()
    perception_profile_path = _write_perception_profile(scene)
    if out.exists() and a.force: shutil.rmtree(out)
    out.mkdir(parents=True,exist_ok=True)
    paths={"exported":out/'exported',"task":out/'task',"preview":out/'preview',"plan":out/'plan_preview',"smoke":out/'smoke_launch',"read":out/'planning_scene_readiness','logs':out/'logs'}
    [p.mkdir(parents=True,exist_ok=True) for p in paths.values()]
    art,results,summary,safety={}, {}, {"blockers":[],"warnings":[],"suggested_next_actions":[]}, {"motion_command_sent":False,"moveit_plan_service_called":False,"runtime_execution_called":False,"real_hardware_enabled":False,"runtime_io_applied":False,"smoke_execute_used":False}
    logs={}
    def step(name:str,cmd:list[str],hard:bool=False):
        rc,p=_run_json(cmd,name); logs[name]={"cmd":cmd,"payload":p};
        if rc!=0 and hard: summary['blockers'].append(f"{name} failed")
        return rc,p
    art['perception_profile']=str(perception_profile_path)
    snapshot_path = Path('tests/fixtures/perception/detected_objects_snapshot_golden.yaml').resolve()
    art['detected_object_snapshot']=str(snapshot_path)
    pr_status, pr_warnings, pr_blockers = _generate_perception_readiness(perception_profile_path, snapshot_path, scene/'generated'/'perception_readiness_report.json')
    art['perception_readiness_report']=str(scene/'generated'/'perception_readiness_report.json')
    results['perception_status']=pr_status
    bridge_payload = scene / 'generated' / 'emd_bridge_payload_preview.json'
    bridge_report = scene / 'generated' / 'perception_bridge_preview_report.json'
    rc,scene_v=step('validate_scene',[sys.executable,str(SCRIPT_DIR/'validate_builder_generated_scene.py'),str(scene),'--json'],hard=True)
    results['builder_scene_validation']='PASS' if scene_v.get('ok') else 'FAIL'
    rc,ppv=step('validate_perception_profile',[sys.executable,str(SCRIPT_DIR/'validate_perception_profile.py'),str(perception_profile_path),'--json'])
    results['perception_profile_validation']='PASS' if rc==0 else 'FAIL'
    gen=scene/'generated'; gen.mkdir(exist_ok=True)
    rc,exp=step('export_scene',[sys.executable,str(SCRIPT_DIR/'export_builder_scene_to_cell_definition.py'),str(scene),'--output-dir',str(paths['exported'])],hard=True)
    for f in ['cell_definition.yaml','environment_layout.yaml','builder_export_summary.json']:
        src=paths['exported']/f
        art[{'cell_definition.yaml':'cell_definition','environment_layout.yaml':'environment_layout','builder_export_summary.json':'builder_export_summary'}[f]]=str(src)
    saved_layout_path = resolve_saved_layout_path(scene)
    readiness_layout_path = saved_layout_path or (paths['exported'] / 'environment_layout.yaml')
    art['saved_layout'] = str(readiness_layout_path)
    ti=gen/'workcell_builder_task_intent.yaml'
    recipe=paths['task']/'task_recipe_from_builder_intent.yaml'
    if ti.exists():
        shutil.copy2(ti,paths['task']/ti.name); art['builder_task_intent']=str(paths['task']/ti.name)
        rc,tiv=step('validate_task_intent',[sys.executable,str(SCRIPT_DIR/'validate_builder_task_intent.py'),str(ti),'--scene-package',str(scene),'--json'])
        results['task_intent_status']='PASS' if rc==0 else 'WARN'
        rc,conv=step('convert_task_intent',[sys.executable,str(SCRIPT_DIR/'convert_builder_task_intent_to_task_recipe.py'),'--task-intent',str(ti),'--output',str(recipe),'--scene-package',str(scene),'--validate','--json'])
        if recipe.exists(): art['task_recipe']=str(recipe); results['task_recipe_status']='PASS' if rc==0 else 'WARN'
    else:
        results['task_intent_status']='WARN'; summary['warnings'].append('Missing task intent (physical scene only)')
    if ti.exists() and readiness_layout_path.exists():
        rc, pb = step('perception_bridge_preview', [
            sys.executable, str(SCRIPT_DIR / 'generate_perception_bridge_preview.py'),
            '--perception-profile', str(perception_profile_path),
            '--detected-objects', str(snapshot_path),
            '--task-intent', str(ti),
            '--environment-layout', str(readiness_layout_path),
            '--output-payload', str(bridge_payload),
            '--output-report', str(bridge_report),
            '--json',
        ])
        art['emd_bridge_payload_preview'] = str(bridge_payload)
        art['perception_bridge_preview_report'] = str(bridge_report)
        results['perception_bridge_status'] = 'PASS' if rc == 0 else 'WARN'
    else:
        results['perception_bridge_status'] = 'WARN'

    tf=paths['task']/'task_flow_summary.json'
    task_flow_cmd=None
    if recipe.exists():
        task_flow_cmd=[sys.executable,str(SCRIPT_DIR/'summarize_task_flow.py'),'--task-recipe',str(recipe),'--environment-layout',str(readiness_layout_path),'--output',str(tf),'--json']
    elif (paths['task']/ti.name).exists():
        task_flow_cmd=[sys.executable,str(SCRIPT_DIR/'summarize_task_flow.py'),'--task-intent',str(paths['task']/ti.name),'--environment-layout',str(readiness_layout_path),'--output',str(tf),'--json']
    if task_flow_cmd:
        rc,tfp=step('task_flow',task_flow_cmd)
        if tf.exists():
            art['task_flow_summary']=str(tf)
        results['task_flow_status']='PASS' if rc==0 else 'WARN'
    else:
        results['task_flow_status']='WARN'
    static_preview_cmd=[sys.executable,str(SCRIPT_DIR/'generate_workcell_static_preview.py'),'--cell-definition',str(paths['exported']/'cell_definition.yaml'),'--output-dir',str(paths['preview']),'--title',a.project_name,'--json']
    if readiness_layout_path.exists():
        static_preview_cmd += ['--environment-layout', str(readiness_layout_path)]
    copied_ti = paths['task']/ti.name
    if copied_ti.exists():
        static_preview_cmd += ['--task-intent', str(copied_ti)]
    if recipe.exists():
        static_preview_cmd += ['--task-recipe', str(recipe)]
    rc,sp=step('static_preview',static_preview_cmd)
    art['static_preview']={'svg':str(paths['preview']/'static_preview.svg'),'html':str(paths['preview']/'static_preview.html'),'summary':str(paths['preview']/'static_preview_summary.json'),'markers':str(paths['preview']/'visual_markers.json')}; results['static_preview_status']='PASS' if rc==0 else 'WARN'
    recipe=paths['task']/'task_recipe_from_builder_intent.yaml'
    req=paths['plan']/'offline_plan_preview_request.yaml'
    session=paths['plan']/'rviz_moveit_plan_preview_session.json'
    if recipe.exists():
        rc,rq=step('offline_req',[sys.executable,str(SCRIPT_DIR/'generate_offline_plan_preview_request.py'),'--task-recipe',str(recipe),'--output',str(req),'--validate','--json'])
        art['offline_plan_preview_request']=str(req); results['offline_plan_preview_status']='PASS' if rc==0 else 'WARN'
    else: results['offline_plan_preview_status']='WARN'
    if a.prepare_rviz_preview and req.exists():
        rc,rv=step('rviz_session',[sys.executable,str(SCRIPT_DIR/'generate_rviz_moveit_plan_preview_session.py'),'--scene-package',str(scene),'--plan-preview-request',str(req),'--output-dir',str(paths['plan']),'--allow-missing-launch','--json'])
        art['rviz_moveit_plan_preview_session']=str(session); results['rviz_preview_session_status']='PASS' if rc==0 else 'WARN'
    else: results['rviz_preview_session_status']='WARN'

    handoff_path = scene / 'generated' / 'bridge_payload_plan_preview_handoff.json'
    if bridge_payload.exists():
        rc, handoff = step('bridge_plan_preview_handoff', [
            sys.executable, str(SCRIPT_DIR / 'generate_bridge_payload_plan_preview_handoff.py'),
            '--bridge-payload', str(bridge_payload),
            '--rviz-session', str(session),
            '--scene-package', str(scene),
            '--output', str(handoff_path),
            '--json',
        ])
        art['bridge_payload_plan_preview_handoff'] = str(handoff_path)
        results['bridge_payload_plan_preview_handoff_status'] = 'PASS' if rc == 0 else 'WARN'
    else:
        results['bridge_payload_plan_preview_handoff_status'] = 'WARN'

    if a.smoke_execute: safety['smoke_execute_used']=True
    if a.smoke_dry_run or a.smoke_execute:
        cmd=[sys.executable,str(SCRIPT_DIR/'run_fake_hardware_smoke_launch.py'),'--session',str(session),'--output-dir',str(paths['smoke']),'--timeout-s',str(a.smoke_timeout_s),'--json', '--execute' if a.smoke_execute else '--dry-run']
        rc,sm=step('smoke',cmd); art['fake_hardware_smoke_launch_report']=str(paths['smoke']/'fake_hardware_smoke_launch_report.json'); results['smoke_launch_status']='PASS' if rc==0 else 'WARN'
    else: results['smoke_launch_status']='SKIPPED'
    rc,pr=step('planning_readiness',[sys.executable,str(SCRIPT_DIR/'check_planning_scene_readiness.py'),'--scene-package',str(scene),'--output-dir',str(paths['read']),'--cell-definition',str(paths['exported']/'cell_definition.yaml'),'--json']+(['--task-recipe',str(recipe)] if recipe.exists() else [])+(['--plan-preview-request',str(req)] if req.exists() else [])+(['--plan-preview-session',str(session)] if session.exists() else [])+(['--strict'] if a.strict else []))
    art['planning_scene_readiness_report']=str(paths['read']/'planning_scene_readiness_report.json'); results['planning_scene_readiness']='PASS' if rc==0 else 'WARN'
    build_launch_readiness_report_path = scene / 'generated' / 'guided_generated_scene_build_readiness_report.json'
    build_launch_cmd = [
        sys.executable, str(SCRIPT_DIR / 'validate_guided_generated_scene_build_readiness.py'),
        '--repo-root', str(Path.cwd()),
        '--workspace-root', str(Path.cwd()),
        '--scene', str(scene),
        '--json',
    ]
    if not a.validate:
        build_launch_cmd.append('--skip-build')
    if not (a.smoke_dry_run or a.smoke_execute):
        build_launch_cmd.append('--skip-launch-smoke')
    build_launch_status = 'WARN'
    build_launch_warnings: list[str] = []
    build_launch_blockers: list[str] = []
    build_launch_payload: dict[str, Any] = {}
    try:
        rc, build_launch_payload = step('build_launch_readiness', build_launch_cmd)
        if isinstance(build_launch_payload, dict):
            build_launch_status = str(build_launch_payload.get('status') or build_launch_payload.get('result') or ('PASS' if rc == 0 else 'WARN')).upper()
            build_launch_warnings = build_launch_payload.get('warnings') if isinstance(build_launch_payload.get('warnings'), list) else []
            build_launch_blockers = build_launch_payload.get('blockers') if isinstance(build_launch_payload.get('blockers'), list) else []
        else:
            build_launch_status = 'FAIL'
            build_launch_blockers = ['Build/launch readiness validator returned a non-object payload.']
    except Exception as exc:
        build_launch_status = 'FAIL'
        build_launch_blockers = [f'Build/launch readiness invocation error: {exc}']
    results['build_launch_readiness'] = build_launch_status
    if build_launch_warnings:
        summary['warnings'].extend(str(w) for w in build_launch_warnings)
    if build_launch_blockers:
        summary['blockers'].extend(str(b) for b in build_launch_blockers)
    build_launch_artifacts = {
        'validator_report': str(build_launch_readiness_report_path),
        'planning_scene_readiness_report': str(paths['read'] / 'planning_scene_readiness_report.json'),
        'fake_hardware_smoke_launch_report': str(paths['smoke'] / 'fake_hardware_smoke_launch_report.json'),
    }
    build_launch_readiness_section = {
        'status': build_launch_status,
        'blockers': build_launch_blockers,
        'warnings': build_launch_warnings,
        'artifact_paths': build_launch_artifacts,
    }
    if pr_warnings:
        summary['warnings'].extend(pr_warnings)
    if pr_blockers:
        summary['blockers'].extend(pr_blockers)
    if any(safety.values()) and (safety['motion_command_sent'] or safety['moveit_plan_service_called'] or safety['runtime_execution_called'] or safety['real_hardware_enabled']):
        results['final_readiness']='FAIL';summary['blockers'].append('Unsafe flags detected')
    elif results['builder_scene_validation']=='FAIL': results['final_readiness']='FAIL'
    elif not ti.exists(): results['final_readiness']='WARN'; results['classification']='physical_scene_only'
    elif recipe.exists() and req.exists() and (paths['read']/'planning_scene_readiness_report.json').exists() and results['planning_scene_readiness']=='PASS': results['final_readiness']='PASS'; results['classification']='task_planning_ready'
    else: results['final_readiness']='WARN'; results['classification']='partial_task_pipeline'
    perception_bridge_report = _read_json(bridge_report) if bridge_report.exists() else {}
    perception_section={'perception_profile_path':str(perception_profile_path),'detected_object_snapshot_path':str(snapshot_path),'perception_readiness_report_path':str(scene/'generated'/'perception_readiness_report.json'),'status':pr_status,'topic_frame_checks':{'profile_valid':results.get('perception_profile_validation')=='PASS'},'safety_flags':{'real_hardware_enabled':False,'motion_command_sent':False,'runtime_execution_called':False,'perception_only':True}}
    perception_bridge_section = {
        'payload_preview_path': str(bridge_payload),
        'report_path': str(bridge_report),
        'status': (perception_bridge_report.get('status') if isinstance(perception_bridge_report, dict) and perception_bridge_report.get('status') else 'bridge_preview_blocked'),
        'warnings': (perception_bridge_report.get('warnings') if isinstance(perception_bridge_report, dict) else []),
        'blockers': (perception_bridge_report.get('blockers') if isinstance(perception_bridge_report, dict) else []),
        'safety_flags': {'real_hardware_enabled': False, 'motion_command_sent': False, 'runtime_execution_called': False, 'moveit_plan_service_called': False, 'preview_only': True},
    }

    handoff_report = _read_json(handoff_path) if handoff_path.exists() else {}
    plan_preview_handoff_section = {
        'handoff_path': str(handoff_path),
        'status': handoff_report.get('status', 'plan_preview_blocked'),
        'command': handoff_report.get('preview_command'),
        'warnings': handoff_report.get('warnings', []),
        'blockers': handoff_report.get('blockers', []),
        'safety_flags': handoff_report.get('safety_flags', {
            'fake_hardware_default': True,
            'real_hardware_enabled': False,
            'motion_command_sent': False,
            'runtime_execution_called': False,
            'moveit_plan_service_called': False,
        }),
    }

    manifest={'schema':'workcell_studio_readiness_pack/v1','source':{'scene_package':str(scene),'project_name':a.project_name,'created_at':datetime.now(timezone.utc).isoformat()},'artifacts':art,'results':results,'safety':safety,'summary':summary,'perception':perception_section,'perception_bridge':perception_bridge_section,'plan_preview_handoff':plan_preview_handoff_section, 'build_launch_readiness': build_launch_readiness_section}
    (out/'logs'/'command_outputs.json').write_text(json.dumps(logs,indent=2)+"\n",encoding='utf-8')
    (out/'readiness_pack_manifest.json').write_text(json.dumps(manifest,indent=2)+"\n",encoding='utf-8')

    (out/'next_commands.md').write_text(f"python3 scripts/workcell_studio.py validate-readiness-pack --manifest {out/'readiness_pack_manifest.json'} --json\npython3 scripts/workcell_studio.py generate-readiness-dashboard --manifest {out/'readiness_pack_manifest.json'} --output {out/'readiness_dashboard.html'} --json\n",encoding='utf-8')

    dashboard = out/'readiness_dashboard.html'
    if not a.no_dashboard:
        rc,db=step('dashboard',[sys.executable,str(SCRIPT_DIR/'generate_readiness_pack_dashboard.py'),'--manifest',str(out/'readiness_pack_manifest.json'),'--output',str(dashboard),'--json'])
        if dashboard.exists():
            art['readiness_dashboard']=str(dashboard)
            results['readiness_dashboard_status']='PASS' if rc==0 else 'WARN'
    manifest['artifacts']=art
    (out/'readiness_pack_manifest.json').write_text(json.dumps(manifest,indent=2)+"\n",encoding='utf-8')
    scene_readiness_path = scene / 'generated' / 'scene_package_readiness.json'
    scene_readiness = _read_json(scene_readiness_path)
    scene_readiness['build_launch_readiness'] = build_launch_readiness_section
    scene_readiness_path.write_text(json.dumps(scene_readiness, indent=2) + "\n", encoding='utf-8')
    (out/'readiness_pack_summary.md').write_text(f"# Workcell Studio Readiness Pack\n\n- Final readiness: **{results['final_readiness']}**\n- Classification: `{results.get('classification','unknown')}`\n- Dashboard: `{art.get('readiness_dashboard','(disabled)')}`\n",encoding='utf-8')
    if a.json: print(json.dumps({'result':results['final_readiness'],'manifest':str(out/'readiness_pack_manifest.json')},indent=2))
    return 0 if results['final_readiness']!='FAIL' or a.continue_on_error else 2
if __name__=='__main__': raise SystemExit(main())
