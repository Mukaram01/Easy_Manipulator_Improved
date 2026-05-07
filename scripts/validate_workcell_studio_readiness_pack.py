#!/usr/bin/env python3
from __future__ import annotations
import argparse,json
from pathlib import Path

def main()->int:
 p=argparse.ArgumentParser();p.add_argument('manifest',type=Path);p.add_argument('--json',action='store_true');a=p.parse_args()
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
 if r.get('final_readiness')=='PASS':
  for k in ['task_recipe','offline_plan_preview_request','planning_scene_readiness_report']:
   if not arts.get(k) or not Path(arts[k]).exists(): errs.append(f'PASS requires {k}')
 out={'result':'FAIL' if errs else ('WARN' if warn else 'PASS'),'errors':errs,'warnings':warn}
 if a.json: print(json.dumps(out,indent=2))
 else: print(out['result'])
 return 2 if errs else 0
if __name__=='__main__': raise SystemExit(main())
