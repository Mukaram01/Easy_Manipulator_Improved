#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path

def _load(path: Path):
    return json.loads(path.read_text(encoding='utf-8'))

def main() -> int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--profile', required=True)
    ap.add_argument('--input', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--summary', required=True)
    args=ap.parse_args()
    profile=_load(Path(args.profile))
    detected=_load(Path(args.input))
    if detected.get('schema_version')!='detected_objects/v1':
        raise SystemExit('invalid detected_objects schema')
    objs=detected.get('objects',[])
    targets=[]
    for o in objs:
        pose=o.get('pose',{})
        targets.append({'id':o.get('id'),'label':o.get('label'),'confidence':o.get('confidence'),'position':pose.get('xyz',[0,0,0]),'rpy':pose.get('rpy',[0,0,0])})
    payload={'schema_version':'emd_grasp_bridge_payload/v1','source':'perception_replay','dry_run_only':True,'perception_provider':profile.get('perception',{}).get('provider','epd'),'targets':targets,'runtime_execution':{'auto_execute':False,'moveit_planning_called':False,'robot_motion_called':False}}
    Path(args.output).write_text(json.dumps(payload,indent=2)+'\n',encoding='utf-8')
    summary={'status':'ok','dry_run_only':True,'live_epd_launched':False,'object_count':len(targets),'note':'Config generated. Live EPD not launched automatically.'}
    Path(args.summary).write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'payload':args.output,'summary':args.summary,'count':len(targets)}))
    return 0

if __name__=='__main__':
    raise SystemExit(main())
