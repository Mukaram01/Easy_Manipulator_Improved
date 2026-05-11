#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, re, sys
from pathlib import Path

PASS='WORKCELL_ASSET_CATALOG: PASS'
WARN='WORKCELL_ASSET_CATALOG: WARN'
FAIL='WORKCELL_ASSET_CATALOG: FAIL'
SAFE=re.compile(r'^[a-z0-9_]+$')
VALID_TOOL_TYPES={'finger','suction','custom','unknown'}
VALID_STATUS={'COMPATIBLE','COMPATIBLE_WITH_WARNINGS','UNKNOWN_COMPATIBILITY','INCOMPATIBLE'}
VALID_ENV_CATEGORIES={"Tables / Workbenches","Bins / Trays / Totes","Conveyors","Fixtures","Safety / Fencing","Camera Mounts","Robot Bases","Pick Objects"}
MAX_ASSET_BYTES=2*1024*1024
SUSPICIOUS_LICENSE={"proprietary","vendor_only","restricted","unknown_vendor"}


def _load_jsons(d: Path):
    out=[]
    for p in sorted(d.glob('*.json')):
        try: out.append((p,json.loads(p.read_text(encoding='utf-8'))))
        except Exception as e: out.append((p,{'__parse_error__':str(e)}))
    return out

def _req(obj, fields, errs, where):
    for f in fields:
        if f not in obj or obj.get(f) in ('',None,[]): errs.append(f'{where}: missing required field {f}')

def _safe(v:str, errs, where):
    if not SAFE.match(v): errs.append(f'{where}: unsafe name {v}')

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--repo-root',default='.')
    ap.add_argument('--strict',action='store_true')
    a=ap.parse_args()
    root=Path(a.repo_root).resolve()
    croot=root/'workcell_builder/workcell_builder/config'
    robots_d=croot/'compatibility_profiles/robots'; tools_d=croot/'compatibility_profiles/tools'; pairs_d=croot/'compatibility_profiles/pairs'; cams_d=croot/'camera_profiles'
    assets_d=croot/'asset_profiles/environment_assets.json'
    blockers=[]; warns=[]
    for d in [robots_d,tools_d,pairs_d,cams_d]:
        if not d.exists(): blockers.append(f'missing directory: {d.relative_to(root)}')
    robot_ids=set(); tool_ids=set(); cam_ids=set()
    for p,obj in _load_jsons(robots_d):
        if '__parse_error__' in obj: blockers.append(f'{p.name}: parse error'); continue
        _req(obj,['robot_id','label','description_package','moveit_config_package','base_link','planning_group','default_tool_mount_link','controller_family','supported_tool_types','approximate_reach_radius_m','base_exclusion_radius_m'],blockers,p.name)
        rid=obj.get('robot_id','');
        if rid in robot_ids: blockers.append(f'duplicate robot_id: {rid}')
        if rid: robot_ids.add(rid); _safe(rid,blockers,p.name)
        if not obj.get('planning_group'): blockers.append(f'{p.name}: missing planning_group')
        if not obj.get('default_tool_mount_link'): blockers.append(f'{p.name}: missing mount link')
        if obj.get('description_package','unknown') in {'unknown','none'}: warns.append(f'{p.name}: invalid package/name hint description_package')
    for p,obj in _load_jsons(tools_d):
        if '__parse_error__' in obj: blockers.append(f'{p.name}: parse error'); continue
        _req(obj,['tool_id','label','tool_type','description_package','mount_link','tcp_frame','tcp_xyz_rpy','controller_hint','grasp_strategy_default','release_strategy_default','requires_io'],blockers,p.name)
        tid=obj.get('tool_id','')
        if tid in tool_ids: blockers.append(f'duplicate tool_id: {tid}')
        if tid: tool_ids.add(tid); _safe(tid,blockers,p.name)
        if obj.get('tool_type') not in VALID_TOOL_TYPES: blockers.append(f'{p.name}: invalid tool_type')
        if not obj.get('tcp_frame'): blockers.append(f'{p.name}: missing TCP frame')
        if not obj.get('mount_link'): blockers.append(f'{p.name}: missing mount link')
    for p,obj in _load_jsons(pairs_d):
        if '__parse_error__' in obj: blockers.append(f'{p.name}: parse error'); continue
        _req(obj,['robot_id','tool_id','status'],blockers,p.name)
        if obj.get('status') not in VALID_STATUS: blockers.append(f'{p.name}: invalid compatibility status')
        if obj.get('robot_id') and obj['robot_id'] not in robot_ids: warns.append(f"{p.name}: unknown robot_id {obj['robot_id']}")
        if obj.get('tool_id') and obj['tool_id'] not in tool_ids: warns.append(f"{p.name}: unknown tool_id {obj['tool_id']}")
    for p,obj in _load_jsons(cams_d):
        if '__parse_error__' in obj: blockers.append(f'{p.name}: parse error'); continue
        if 'camera_id' not in obj and 'profile' in obj: obj['camera_id']=obj['profile']
        if 'frame_id' not in obj and 'camera_frame' in obj: obj['frame_id']=obj['camera_frame']
        if 'optical_frame_id' not in obj and 'optical_frame' in obj: obj['optical_frame_id']=obj['optical_frame']
        _req(obj,['camera_id','label','camera_type','frame_id','optical_frame_id','rgb_topic','depth_topic','camera_info_topic','pointcloud_topic','mount_type','perception_hint','epd_input_hint'],blockers,p.name)
        cid=obj.get('camera_id','')
        if cid in cam_ids: blockers.append(f'duplicate camera_id: {cid}')
        if cid: cam_ids.add(cid); _safe(cid,blockers,p.name)
        for f in ['rgb_topic','depth_topic','camera_info_topic','pointcloud_topic']:
            if not str(obj.get(f,'')).startswith('/'): warns.append(f'{p.name}: missing camera topics format for {f}')
    if assets_d.exists():
        try:
            arr=json.loads(assets_d.read_text(encoding='utf-8'))
            seen=set()
            if isinstance(arr,list):
                for i,aobj in enumerate(arr):
                    where=f'environment_assets[{i}]'
                    _req(aobj,['asset_id','label','category','asset_type','mesh_path','default_dimensions_m','default_pose','default_z_hint','license'],blockers,where)
                    aid=str(aobj.get('asset_id',''))
                    if aid in seen: blockers.append(f'{where}: duplicate asset_id {aid}')
                    seen.add(aid)
                    if aid: _safe(aid,blockers,where)
                    cat=aobj.get('category')
                    if cat not in VALID_ENV_CATEGORIES: blockers.append(f'{where}: invalid category {cat}')
                    dims=aobj.get('default_dimensions_m',[])
                    if not (isinstance(dims,list) and len(dims)==3 and all(isinstance(x,(int,float)) and x>0 and x<100 for x in dims)): blockers.append(f'{where}: invalid default_dimensions_m')
                    for key in ('mesh_path','urdf_path'):
                        v=aobj.get(key)
                        if not v: continue
                        if str(v).startswith('/'):
                            blockers.append(f'{where}: absolute path forbidden {key}')
                            continue
                        fp=root/str(v)
                        if not fp.exists(): blockers.append(f'{where}: missing path {v}')
                        elif fp.is_symlink(): blockers.append(f'{where}: symlink forbidden {v}')
                        elif fp.stat().st_size>MAX_ASSET_BYTES: blockers.append(f'{where}: huge file {v}')
                    lic=str(aobj.get('license','')).lower()
                    if lic in SUSPICIOUS_LICENSE: blockers.append(f'{where}: suspicious proprietary/vendor license label {lic}')
        except Exception: blockers.append('environment_assets.json parse error')

    if blockers: print(FAIL)
    elif warns or a.strict: print(WARN if warns else PASS)
    else: print(PASS)
    for b in blockers: print('BLOCKER:',b)
    for w in warns: print('WARNING:',w)
    return 1 if blockers or (a.strict and warns) else 0

if __name__=='__main__':
    raise SystemExit(main())
