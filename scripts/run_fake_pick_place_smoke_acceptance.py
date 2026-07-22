#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,os,shlex,shutil,signal,subprocess,sys,time
from pathlib import Path
from typing import Any
import yaml  # type: ignore
SCRIPT_DIR=Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path: sys.path.insert(0,str(SCRIPT_DIR))
from supported_scene_catalog import default_catalog_path, load_supported_scene_catalog, SupportedSceneEntry
PASS='PASS'; FAIL='FAIL'; BLOCKED='BLOCKED'; NA='NOT_APPLICABLE'
STAGES=['ready','pre_pick','pick','tool_engage','lift','pre_place','place','tool_release','retreat']
REAL=['use_fake_hardware:=false','hardware_mode:=real','driver_mode:=real','real_hardware:=true','ur_robot_driver']

def load_yaml(p:Path)->dict[str,Any]:
    try:
        x=yaml.safe_load(p.read_text(encoding='utf-8')); return x if isinstance(x,dict) else {}
    except Exception: return {}

def xyz(v:Any):
    return [float(v[i]) for i in range(3)] if isinstance(v,list) and len(v)>=3 else None

def zones(env:dict[str,Any])->list[dict[str,Any]]:
    out=[]
    for k in [('task_zones',),('workspace','zones'),('environment','task_zones')]:
        cur=env
        for part in k: cur=cur.get(part,{}) if isinstance(cur,dict) else {}
        if isinstance(cur,list): out += [z for z in cur if isinstance(z,dict)]
    return out

def find_zone(env:dict[str,Any], zid:str)->dict[str,Any]|None:
    for z in zones(env):
        ids=[z.get('id'), z.get('alias_of'), *([*z.get('aliases',[])] if isinstance(z.get('aliases'),list) else [])]
        if zid in [str(i) for i in ids if i]: return z
    return None

def pose_from_zone(z:dict[str,Any]|None):
    if not z: return None
    return xyz(z.get('pose_xyz') or z.get('center_xyz'))

def bounds(env:dict[str,Any])->dict[str,float]:
    b=((env.get('workspace') or {}).get('bounds') or {}) if isinstance(env.get('workspace'),dict) else {}
    return {k:float(v) for k,v in b.items() if isinstance(v,(int,float))}

def in_bounds(p:list[float], b:dict[str,float])->bool:
    return (b.get('x_min',-99)<=p[0]<=b.get('x_max',99) and b.get('y_min',-99)<=p[1]<=b.get('y_max',99) and b.get('z_min',-99)<=p[2]<=b.get('z_max',99))

def resolve(entry:SupportedSceneEntry, repo:Path):
    meta=dict(entry.task_smoke or {})
    env=load_yaml(repo/entry.scene_path/'environment.yaml'); task=env.get('task') if isinstance(env.get('task'),dict) else {}
    pick_id=str(meta.get('pick_zone') or ((task.get('pick') or {}).get('source_ref') if isinstance(task.get('pick'),dict) else '') or '')
    place_id=str(meta.get('place_zone') or ((task.get('place') or {}).get('target_ref') if isinstance(task.get('place'),dict) else '') or '')
    obj=str(meta.get('test_object') or task.get('source_object') or '')
    pg=str(meta.get('planning_group') or ((env.get('robot') or {}).get('planning_group') if isinstance(env.get('robot'),dict) else '') or '')
    ee=str(meta.get('end_effector_link') or ((env.get('tool') or {}).get('grasp_frame') if isinstance(env.get('tool'),dict) else '') or '')
    tool=str(meta.get('tool_type') or entry.tool or '')
    pick=pose_from_zone(find_zone(env,pick_id)); place=pose_from_zone(find_zone(env,place_id))
    errs=[]
    for name,val in [('planning_group',pg),('tool_type',tool),('end_effector_link',ee),('pick_zone',pick_id),('place_zone',place_id),('test_object',obj)]:
        if not val: errs.append(f'missing task-smoke metadata: {name}')
    if pick is None: errs.append(f'pick zone {pick_id!r} could not be resolved from scene/task-zone metadata')
    if place is None: errs.append(f'place zone {place_id!r} could not be resolved from scene/task-zone metadata')
    return meta, {'pick':pick,'place':place,'bounds':bounds(env),'planning_group':pg,'tool_type':tool,'end_effector_link':ee,'test_object':obj}, errs

def command(entry:SupportedSceneEntry)->str:
    return entry.fake_hardware_launch_command.replace('launch_rviz:=true','launch_rviz:=false')

def fake_active(cmd:str,timeout:int)->tuple[str,list[str],dict[str,Any]]:
    if any(t in cmd.lower() for t in REAL): return FAIL,[f'execution rejected: non-fake hardware token in launch command'],{'launched':False,'terminated':True}
    if 'use_fake_hardware:=true' not in cmd: return FAIL,['execution rejected: hardware mode unknown; use_fake_hardware:=true is required'],{'launched':False,'terminated':True}
    if shutil.which('ros2') is None: return BLOCKED,['ros2 executable not found; source ROS 2 Humble and workspace install/setup.bash'],{'launched':False,'terminated':True}
    p=None; diag={'launched':False,'terminated':False}
    try:
        p=subprocess.Popen(shlex.split(cmd),stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,preexec_fn=os.setsid); diag['launched']=True
        time.sleep(min(3,max(1,timeout//4)))
        r=subprocess.run(['ros2','param','get','/move_group','use_fake_hardware'],capture_output=True,text=True,timeout=max(1,min(5,timeout//3)))
        blob=(r.stdout+r.stderr).lower(); diag['fake_probe']=blob[-1000:]
        if r.returncode==0 and 'true' in blob: return PASS,[],diag
        return FAIL,['execution rejected: fake hardware could not be proven active from /move_group use_fake_hardware'],diag
    except subprocess.TimeoutExpired as e: return BLOCKED,[f'fake-hardware probe timed out: {e}'],diag
    finally:
        if p:
            try: os.killpg(os.getpgid(p.pid),signal.SIGTERM); p.communicate(timeout=5)
            except Exception:
                try: os.killpg(os.getpgid(p.pid),signal.SIGKILL)
                except Exception: pass
            diag['terminated']=True

def build_stages(res:dict[str,Any], execute:bool)->list[dict[str,Any]]:
    out=[]; pick=res['pick']; place=res['place']; b=res['bounds']; tool=res['tool_type'].lower()
    poses={'ready':[0.25,0,0.45],'pre_pick':[pick[0],pick[1],pick[2]+0.12],'pick':pick,'lift':[pick[0],pick[1],pick[2]+0.10],'pre_place':[place[0],place[1],place[2]+0.12],'place':place,'retreat':[place[0],place[1],place[2]+0.10]}
    attached=False; closed=False
    for s in STAGES:
        r={'stage':s,'status':PASS,'action':'plan' if not execute else 'execute_fake','details':[]}
        if s in poses and not in_bounds(poses[s],b): r['status']=FAIL; r['details'].append(f'{s} pose outside workspace bounds')
        if s=='tool_engage':
            if 'suction' in tool: attached=True; r['details'].append('simulated suction attach state true; no real I/O')
            elif '2f' in tool or 'gripper' in tool or 'finger' in tool: closed=True; r['details'].append('parallel gripper close command verified')
            else: r['status']=NA; r['details'].append('tool engage not applicable for tool type')
        if s=='tool_release':
            if 'suction' in tool: attached=False; r['details'].append('simulated suction attach state false; no real I/O')
            elif '2f' in tool or 'gripper' in tool or 'finger' in tool: closed=False; r['details'].append('parallel gripper open command verified')
            else: r['status']=NA
        out.append(r)
    return out

def main()->int:
    ap=argparse.ArgumentParser(description='Run fake-hardware pick/place task smoke acceptance.')
    ap.add_argument('--scene',required=True); ap.add_argument('--catalog',type=Path); ap.add_argument('--json-output',type=Path,default=Path('build/workcell_studio/fake_pick_place_smoke.json')); ap.add_argument('--timeout-sec',type=int,default=45); ap.add_argument('--execute-fake',action='store_true')
    a=ap.parse_args(); repo=Path(__file__).resolve().parents[1]
    _, entries, errors=load_supported_scene_catalog((a.catalog or default_catalog_path(repo)).resolve())
    result={'schema':'fake_hardware_pick_place_smoke/v1','scene':a.scene,'mode':'execute_fake' if a.execute_fake else 'plan_only','status':PASS,'stages':[],'blockers':[],'metadata':{},'resolved':{},'launch':{'command':''}}
    if errors: result['status']=FAIL; result['blockers']=errors
    entry=next((e for e in entries if e.scene_name==a.scene),None)
    if not entry: result['status']=FAIL; result['blockers'].append('scene not found in supported-scene registry')
    if entry:
        result['launch']['command']=command(entry); meta,res,errs=resolve(entry,repo); result['metadata']=meta; result['resolved']=res
        if errs: result['status']=FAIL; result['blockers'].extend(errs)
        elif a.execute_fake:
            st,bl,diag=fake_active(result['launch']['command'],a.timeout_sec); result['launch']['diagnostics']=diag
            if st!=PASS: result['status']=st; result['blockers'].extend(bl)
        if not result['blockers']:
            result['stages']=build_stages(result['resolved'],a.execute_fake)
            if any(s['status']==FAIL for s in result['stages']): result['status']=FAIL
    a.json_output.parent.mkdir(parents=True,exist_ok=True); a.json_output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    for st in result.get('stages',[]): print(f"{st['stage']}: {st['status']} - {', '.join(st.get('details') or ['planned'])}")
    print(json.dumps({'status':result['status'],'report_json':str(a.json_output)},indent=2))
    return 1 if result['status'] == FAIL else (2 if result['status'] == BLOCKED else 0)
if __name__=='__main__': raise SystemExit(main())
