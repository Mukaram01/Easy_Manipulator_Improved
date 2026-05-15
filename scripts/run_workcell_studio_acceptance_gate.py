#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / 'scripts'


def _run_json(cmd:list[str])->tuple[int,dict[str,Any],str]:
    p=subprocess.run(cmd,capture_output=True,text=True,check=False)
    merged=(p.stdout+'\n'+p.stderr).strip()
    payload={}
    if p.stdout.strip():
        try: payload=json.loads(p.stdout)
        except Exception: payload={}
    return p.returncode,payload,merged

def _status_from_rc(rc:int)->str:
    return 'PASS' if rc==0 else 'BLOCKED'

def _tail(s:str,n:int=30)->list[str]:
    lines=[x for x in s.splitlines() if x.strip()]
    return lines[-n:]

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--mode',choices=['scratch','existing-scene','regression'],required=True)
    ap.add_argument('--workspace',type=Path)
    ap.add_argument('--scene-name',default='scratch_ur5_2f_acceptance')
    ap.add_argument('--scenes',nargs='*',default=[])
    ap.add_argument('--output-root',type=Path,default=Path('/tmp/workcell_studio_acceptance'))
    ap.add_argument('--json-out',type=Path)
    ap.add_argument('--run-smoke',action='store_true')
    ap.add_argument('--timeout-sec',type=int,default=30)
    a=ap.parse_args()

    out_root=a.output_root; out_root.mkdir(parents=True,exist_ok=True)
    json_out=a.json_out or (out_root/'workcell_studio_acceptance_gate.json')
    summary_out=out_root/'acceptance_summary.md'

    audit_results={k:{'status':'SKIPPED','report_path':'','details':{}} for k in ['scratch_acceptance','file_output','state_transition','error_messages','build_run_smoke','regression']}
    blockers=[]; warnings=[]; logs_tail={}
    report_paths={}
    scene_dir=''

    # always run error-message audit
    rc,payload,log=_run_json([sys.executable,str(SCRIPTS/'audit_new_cell_error_messages.py')])
    audit_results['error_messages']={'status':_status_from_rc(rc),'report_path':'stdout','details':payload}
    logs_tail['error_messages']=_tail(log)

    if a.mode in ('scratch','existing-scene'):
        if a.mode=='scratch':
            sa_json=out_root/'scratch_acceptance.json'
            rc,payload,log=_run_json([sys.executable,str(SCRIPTS/'generate_scratch_cell_acceptance.py'),'--scene-name',a.scene_name,'--output-root',str(out_root),'--json-out',str(sa_json)])
            audit_results['scratch_acceptance']={'status':_status_from_rc(rc),'report_path':str(sa_json),'details':payload}
            report_paths['scratch_acceptance']=str(sa_json); logs_tail['scratch_acceptance']=_tail(log)
            scene_dir=payload.get('scene_dir','') if payload else ''
        else:
            if not a.workspace or not a.scenes:
                blockers.append('existing-scene mode requires --workspace and --scenes')
            if a.scenes: a.scene_name=a.scenes[0]
            if a.workspace and a.scenes:
                for c in [a.workspace/'src'/'easy_manipulation_deployment'/'scenes'/a.scenes[0],a.workspace/'src'/'scenes'/a.scenes[0],REPO_ROOT/'scenes'/a.scenes[0]]:
                    if c.is_dir(): scene_dir=str(c); break
            audit_results['scratch_acceptance']={'status':'SKIPPED','report_path':'','details':{}}

        if scene_dir:
            fo_json=out_root/'file_output_audit.json'; st_json=out_root/'state_transition_audit.json'
            rc,payload,log=_run_json([sys.executable,str(SCRIPTS/'audit_new_cell_file_outputs.py'),'--scene-dir',scene_dir,'--scene-name',a.scene_name,'--json-out',str(fo_json)])
            audit_results['file_output']={'status':_status_from_rc(rc),'report_path':str(fo_json),'details':payload}; report_paths['file_output']=str(fo_json); logs_tail['file_output']=_tail(log)
            rc,payload,log=_run_json([sys.executable,str(SCRIPTS/'audit_new_cell_state_transitions.py'),'--scene-dir',scene_dir,'--scene-name',a.scene_name,'--json-out',str(st_json)])
            audit_results['state_transition']={'status':_status_from_rc(rc),'report_path':str(st_json),'details':payload}; report_paths['state_transition']=str(st_json); logs_tail['state_transition']=_tail(log)
        else:
            audit_results['file_output']['status']='BLOCKED'; audit_results['state_transition']['status']='BLOCKED'; blockers.append('Scene directory not resolved for file/state audits')

        if a.run_smoke:
            if not a.workspace: blockers.append('--run-smoke requires --workspace')
            sm_json=out_root/'build_run_smoke.json'
            cmd=[sys.executable,str(SCRIPTS/'smoke_test_scratch_cell_workspace.py'),'--workspace',str(a.workspace),'--scene-name',a.scene_name,'--timeout-sec',str(a.timeout_sec),'--json-out',str(sm_json)]
            rc,payload,log=_run_json(cmd)
            audit_results['build_run_smoke']={'status':_status_from_rc(rc),'report_path':str(sm_json),'details':payload}; report_paths['build_run_smoke']=str(sm_json); logs_tail['build_run_smoke']=_tail(log)
        else:
            warnings.append('Build/run smoke not executed (pass --run-smoke to enable).')

    if a.mode=='regression':
        if not a.workspace: blockers.append('regression mode requires --workspace')
        reg_json=out_root/'regression_audit.json'
        cmd=[sys.executable,str(SCRIPTS/'audit_workcell_studio_regressions.py'),'--workspace',str(a.workspace),'--json-out',str(reg_json)]
        if a.scenes: cmd += ['--scenes',*a.scenes]
        if a.run_smoke: cmd += ['--run-smoke']
        rc,payload,log=_run_json(cmd)
        audit_results['regression']={'status':_status_from_rc(rc),'report_path':str(reg_json),'details':payload}; report_paths['regression']=str(reg_json); logs_tail['regression']=_tail(log)

    required=['error_messages']
    if a.mode=='scratch': required += ['scratch_acceptance','file_output','state_transition'] + (['build_run_smoke'] if a.run_smoke else [])
    elif a.mode=='existing-scene': required += ['file_output','state_transition'] + (['build_run_smoke'] if a.run_smoke else [])
    elif a.mode=='regression': required += ['regression']

    for name in required:
        if audit_results[name]['status']=='BLOCKED': blockers.append(f'{name} blocked')

    overall='PASS'
    if blockers: overall='BLOCKED'
    elif warnings: overall='WARNINGS'

    launch_scene=a.scene_name if a.mode!='regression' else ((a.scenes[0] if a.scenes else 'ur5_2f_test'))
    launch_cmd=f'ros2 launch {launch_scene} demo.launch.py use_fake_hardware:=true launch_rviz:=true'
    if 'use_fake_hardware:=false' in launch_cmd: blockers.append('Unsafe hardware token detected in generated launch command')

    report={
      'mode':a.mode,
      'scene_name':a.scene_name if a.mode!='regression' else '',
      'checked_scenes':a.scenes if a.mode in ('existing-scene','regression') else [a.scene_name],
      'overall_status':overall,
      'audit_results':audit_results,
      'blockers':blockers,
      'warnings':warnings,
      'next_recommended_action':'Open Plan & Simulate with fake hardware.' if overall=='PASS' else 'Resolve blockers/warnings in listed audits and rerun acceptance gate.',
      'build_command':f'colcon build --symlink-install --packages-select {launch_scene}',
      'source_command':'source install/setup.bash',
      'launch_command':launch_cmd,
      'report_paths':report_paths,
      'logs_tail':logs_tail,
    }

    json_out.parent.mkdir(parents=True,exist_ok=True); json_out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    rows=[]
    for k,v in audit_results.items():
        color='🟩' if v['status']=='PASS' else ('🟨' if v['status'] in ('WARNINGS','SKIPPED') else '🟥')
        rows.append(f"| {k} | {v['status']} | {color} | {v.get('report_path','')} |")
    summary_lines=[
      '# Workcell Studio Acceptance Gate Summary',
      f"- Overall status: **{overall}**",
      f"- Mode: `{a.mode}`",
      f"- Launch command: `{launch_cmd}`",
      '',
      '| Audit | Status | Signal | Report |',
      '|---|---|---|---|',
      *rows,
      '',
      '## Blockers',
    ]
    summary_lines += [f'- {x}' for x in blockers] if blockers else ['- None']
    summary_lines += ['## Warnings']
    summary_lines += [f'- {x}' for x in warnings] if warnings else ['- None']
    summary_lines += [f"## Next action\n- {report['next_recommended_action']}", '## Related reports']
    summary_lines += [f'- `{k}`: `{p}`' for k,p in report_paths.items()] if report_paths else ['- None']
    summary='\n'.join(summary_lines)
    summary_out.write_text(summary+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
    print(f'\nSummary: {summary_out}')
    return 0 if overall!='BLOCKED' else 1

if __name__=='__main__':
    raise SystemExit(main())
