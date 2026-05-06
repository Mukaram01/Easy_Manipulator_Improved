#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from typing import Any
try:
    import yaml
except Exception:
    yaml=None

def _load(path: Path)->dict[str,Any]:
    if yaml is not None:
        d=yaml.safe_load(path.read_text(encoding='utf-8')); return d if isinstance(d,dict) else {}
    return json.loads(path.read_text(encoding='utf-8'))

def validate_request(payload: dict[str,Any])->dict[str,Any]:
    errs=[]; warns=[]
    if payload.get('schema')!='offline_plan_preview_request/v1': errs.append('schema mismatch')
    req=payload.get('request') if isinstance(payload.get('request'),dict) else {}
    pick=(req.get('pick') or {}); place=(req.get('place') or {}); tool=(req.get('tool') or {})
    if not pick.get('source_id'): errs.append('pick.source_id missing')
    if not place.get('target_id'): errs.append('place.target_id missing')
    if not tool.get('grasp_strategy'): errs.append('tool.grasp_strategy missing')
    wps=req.get('waypoints') if isinstance(req.get('waypoints'),list) else []
    ids={w.get('id') for w in wps if isinstance(w,dict)}
    for rid in ['pre_pick','pick','post_pick','pre_place','place','post_place']:
        if rid not in ids: errs.append(f'waypoint missing: {rid}')
    safety=payload.get('safety') if isinstance(payload.get('safety'),dict) else {}
    must={'metadata_only':True,'motion_started':False,'ros_launch_started':False,'moveit_service_called':False,'runtime_io_applied':False}
    for k,v in must.items():
        if safety.get(k)!=v: errs.append(f'safety.{k} must be {v}')
    for k in ['approach_distance_m','retreat_distance_m']:
        if pick.get(k) is not None and float(pick.get(k))<=0: errs.append(f'pick.{k} must be >0')
    if place.get('retreat_distance_m') is not None and float(place.get('retreat_distance_m'))<=0: errs.append('place.retreat_distance_m must be >0')
    return {'status':'FAIL' if errs else ('WARN' if warns else 'PASS'),'warnings':warns,'errors':errs}

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('request',type=Path); ap.add_argument('--json',action='store_true'); a=ap.parse_args()
    out=validate_request(_load(a.request));
    if a.json: print(json.dumps(out,indent=2))
    else: print(out['status'])
    return 1 if out['status']=='FAIL' else 0
if __name__=='__main__': raise SystemExit(main())
