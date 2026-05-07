#!/usr/bin/env python3
from __future__ import annotations
import argparse,json
from pathlib import Path

def main()->int:
 p=argparse.ArgumentParser();p.add_argument('manifest',type=Path);p.add_argument('--strict',action='store_true');p.add_argument('--json',action='store_true');a=p.parse_args()
 m=json.loads(a.manifest.read_text(encoding='utf-8'))
 errs=[];warn=[]
 if m.get('schema')!='workcell_studio_readiness_pack/v1': errs.append('schema invalid')
 s=m.get('safety',{})
 for k in ['motion_command_sent','moveit_plan_service_called','runtime_execution_called','real_hardware_enabled','runtime_io_applied']:
  if s.get(k) is not False: errs.append(f'unsafe flag {k}')
 r=m.get('results',{})
 if 'final_readiness' not in r: errs.append('missing final_readiness')
 arts=m.get('artifacts',{})
 for k in ['cell_definition','environment_layout']:
  v=arts.get(k)
  if v and not Path(v).exists(): errs.append(f'missing artifact {k}')

 dashboard=arts.get('readiness_dashboard')
 if dashboard:
  dp=Path(dashboard)
  if not dp.exists(): errs.append('missing artifact readiness_dashboard')
  else:
   txt=dp.read_text(encoding='utf-8').lower()
   for needle in ['no robot motion','not a safety certificate','workcell studio']:
    if needle not in txt: errs.append(f'dashboard missing required wording: {needle}')
   if 'use_fake_hardware:=false' in txt: errs.append('dashboard contains unsafe marker use_fake_hardware:=false')
 elif a.strict:
  warn.append('no readiness_dashboard listed')

 if r.get('final_readiness')=='PASS':
  for k in ['task_recipe','offline_plan_preview_request','planning_scene_readiness_report']:
   if not arts.get(k) or not Path(arts[k]).exists(): errs.append(f'PASS requires {k}')

 task_recipe=arts.get('task_recipe')
 if task_recipe and Path(task_recipe).exists() and r.get('final_readiness')=='PASS':
  if not arts.get('task_flow_summary') or not Path(arts.get('task_flow_summary','')).exists():
   warn.append('task_flow_summary missing; dashboard must derive task flow from recipe')


 static_summary=arts.get('static_preview',{}).get('summary') if isinstance(arts.get('static_preview'),dict) else None
 tf_summary_path=arts.get('task_flow_summary')
 if task_recipe and Path(task_recipe).exists() and tf_summary_path and Path(tf_summary_path).exists() and static_summary and Path(static_summary).exists():
  try:
   sp=json.loads(Path(static_summary).read_text(encoding='utf-8'))
   sp_tf=sp.get('task_flow_summary',{}) if isinstance(sp.get('task_flow_summary'),dict) else {}
   if sp_tf.get('status')=='FAIL' and 'No task input' in (sp_tf.get('errors') or []):
    if r.get('final_readiness')=='PASS': errs.append('static preview task_flow_summary reports No task input despite task recipe')
    else: warn.append('static preview did not receive task flow input')
  except Exception:
   warn.append('failed to parse static preview summary')

 if dashboard and Path(dashboard).exists() and task_recipe and Path(task_recipe).exists():
  txt=Path(dashboard).read_text(encoding='utf-8')
  lower=txt.lower()
  if not any(x in txt for x in ['pick_zone_main','pick source']): errs.append('dashboard missing pick flow label/value')
  if not any(x in txt for x in ['finger_pinch_basic','grasp strategy']): errs.append('dashboard missing grasp flow label/value')
  if not any(x in txt for x in ['bin_red','place target']): errs.append('dashboard missing place flow label/value')
  if 'href="/tmp/' in lower or "href='/tmp/" in lower: errs.append('dashboard contains absolute /tmp href for internal artifacts')
 out={'result':'FAIL' if errs else ('WARN' if warn else 'PASS'),'errors':errs,'warnings':warn}
 if a.json: print(json.dumps(out,indent=2))
 else: print(out['result'])
 return 2 if errs else 0
if __name__=='__main__': raise SystemExit(main())
