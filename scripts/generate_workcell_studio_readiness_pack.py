#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, shutil, subprocess, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
SCRIPT_DIR = Path(__file__).resolve().parent

def _run_json(cmd:list[str], label:str)->tuple[int,dict[str,Any]]:
    p=subprocess.run(cmd,capture_output=True,text=True,check=False)
    try:data=json.loads(p.stdout) if p.stdout.strip() else {}
    except Exception:data={"result":"FAIL","errors":[f"{label} non-json",p.stdout.strip(),p.stderr.strip()]}
    data.setdefault("stdout",p.stdout.strip());data.setdefault("stderr",p.stderr.strip());data.setdefault("returncode",p.returncode)
    return p.returncode,data

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--scene-package',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);ap.add_argument('--project-name',required=True)
    ap.add_argument('--validate',action='store_true');ap.add_argument('--prepare-rviz-preview',action='store_true');ap.add_argument('--smoke-dry-run',action='store_true')
    ap.add_argument('--smoke-execute',action='store_true');ap.add_argument('--smoke-timeout-s',type=int,default=20);ap.add_argument('--strict',action='store_true')
    ap.add_argument('--continue-on-error',action='store_true');ap.add_argument('--no-dashboard',action='store_true');ap.add_argument('--force',action='store_true');ap.add_argument('--json',action='store_true')
    a=ap.parse_args(); scene=a.scene_package.resolve(); out=a.output_dir.resolve()
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
    rc,scene_v=step('validate_scene',[sys.executable,str(SCRIPT_DIR/'validate_builder_generated_scene.py'),str(scene),'--json'],hard=True)
    results['builder_scene_validation']='PASS' if scene_v.get('ok') else 'FAIL'
    gen=scene/'generated'; gen.mkdir(exist_ok=True)
    rc,exp=step('export_scene',[sys.executable,str(SCRIPT_DIR/'export_builder_scene_to_cell_definition.py'),str(scene),'--output-dir',str(paths['exported'])],hard=True)
    for f in ['cell_definition.yaml','environment_layout.yaml','builder_export_summary.json']:
        src=paths['exported']/f
        art[{'cell_definition.yaml':'cell_definition','environment_layout.yaml':'environment_layout','builder_export_summary.json':'builder_export_summary'}[f]]=str(src)
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

    tf=paths['task']/'task_flow_summary.json'
    task_flow_cmd=None
    if recipe.exists():
        task_flow_cmd=[sys.executable,str(SCRIPT_DIR/'summarize_task_flow.py'),'--task-recipe',str(recipe),'--environment-layout',str(paths['exported']/'environment_layout.yaml'),'--output',str(tf),'--json']
    elif (paths['task']/ti.name).exists():
        task_flow_cmd=[sys.executable,str(SCRIPT_DIR/'summarize_task_flow.py'),'--task-intent',str(paths['task']/ti.name),'--environment-layout',str(paths['exported']/'environment_layout.yaml'),'--output',str(tf),'--json']
    if task_flow_cmd:
        rc,tfp=step('task_flow',task_flow_cmd)
        if tf.exists():
            art['task_flow_summary']=str(tf)
        results['task_flow_status']='PASS' if rc==0 else 'WARN'
    else:
        results['task_flow_status']='WARN'
    static_preview_cmd=[sys.executable,str(SCRIPT_DIR/'generate_workcell_static_preview.py'),'--cell-definition',str(paths['exported']/'cell_definition.yaml'),'--output-dir',str(paths['preview']),'--title',a.project_name,'--json']
    env_layout_path = paths['exported']/'environment_layout.yaml'
    if env_layout_path.exists():
        static_preview_cmd += ['--environment-layout', str(env_layout_path)]
    copied_ti = paths['task']/ti.name
    if copied_ti.exists():
        static_preview_cmd += ['--task-intent', str(copied_ti)]
    if recipe.exists():
        static_preview_cmd += ['--task-recipe', str(recipe)]
    rc,sp=step('static_preview',static_preview_cmd)
    art['static_preview']={'svg':str(paths['preview']/'static_preview.svg'),'html':str(paths['preview']/'static_preview.html'),'summary':str(paths['preview']/'static_preview_summary.json')}; results['static_preview_status']='PASS' if rc==0 else 'WARN'
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
    if a.smoke_execute: safety['smoke_execute_used']=True
    if a.smoke_dry_run or a.smoke_execute:
        cmd=[sys.executable,str(SCRIPT_DIR/'run_fake_hardware_smoke_launch.py'),'--session',str(session),'--output-dir',str(paths['smoke']),'--timeout-s',str(a.smoke_timeout_s),'--json', '--execute' if a.smoke_execute else '--dry-run']
        rc,sm=step('smoke',cmd); art['fake_hardware_smoke_launch_report']=str(paths['smoke']/'fake_hardware_smoke_launch_report.json'); results['smoke_launch_status']='PASS' if rc==0 else 'WARN'
    else: results['smoke_launch_status']='SKIPPED'
    rc,pr=step('planning_readiness',[sys.executable,str(SCRIPT_DIR/'check_planning_scene_readiness.py'),'--scene-package',str(scene),'--output-dir',str(paths['read']),'--cell-definition',str(paths['exported']/'cell_definition.yaml'),'--json']+(['--task-recipe',str(recipe)] if recipe.exists() else [])+(['--plan-preview-request',str(req)] if req.exists() else [])+(['--plan-preview-session',str(session)] if session.exists() else [])+(['--strict'] if a.strict else []))
    art['planning_scene_readiness_report']=str(paths['read']/'planning_scene_readiness_report.json'); results['planning_scene_readiness']='PASS' if rc==0 else 'WARN'
    if any(safety.values()) and (safety['motion_command_sent'] or safety['moveit_plan_service_called'] or safety['runtime_execution_called'] or safety['real_hardware_enabled']):
        results['final_readiness']='FAIL';summary['blockers'].append('Unsafe flags detected')
    elif results['builder_scene_validation']=='FAIL': results['final_readiness']='FAIL'
    elif not ti.exists(): results['final_readiness']='WARN'; results['classification']='physical_scene_only'
    elif recipe.exists() and req.exists() and (paths['read']/'planning_scene_readiness_report.json').exists() and results['planning_scene_readiness']=='PASS': results['final_readiness']='PASS'; results['classification']='task_planning_ready'
    else: results['final_readiness']='WARN'; results['classification']='partial_task_pipeline'
    manifest={'schema':'workcell_studio_readiness_pack/v1','source':{'scene_package':str(scene),'project_name':a.project_name,'created_at':datetime.now(timezone.utc).isoformat()},'artifacts':art,'results':results,'safety':safety,'summary':summary}
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
    (out/'readiness_pack_summary.md').write_text(f"# Workcell Studio Readiness Pack\n\n- Final readiness: **{results['final_readiness']}**\n- Classification: `{results.get('classification','unknown')}`\n- Dashboard: `{art.get('readiness_dashboard','(disabled)')}`\n",encoding='utf-8')
    if a.json: print(json.dumps({'result':results['final_readiness'],'manifest':str(out/'readiness_pack_manifest.json')},indent=2))
    return 0 if results['final_readiness']!='FAIL' or a.continue_on_error else 2
if __name__=='__main__': raise SystemExit(main())
