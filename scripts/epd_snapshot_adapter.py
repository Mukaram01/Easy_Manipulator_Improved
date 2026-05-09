#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, math
from pathlib import Path
from typing import Any

def _load(path: Path)->dict[str,Any]:
    return json.loads(path.read_text(encoding='utf-8')) if path and path.exists() else {}

def _dist(a,b):
    return math.sqrt(sum((float(x)-float(y))**2 for x,y in zip(a,b)))

def _pick_area_position(environment:dict[str,Any], pick_ref:str)->list[float]|None:
    assets=environment.get('assets') or environment.get('objects') or environment.get('current_cell_assets') or []
    for a in assets:
        if a.get('id')==pick_ref or a.get('asset_id')==pick_ref or a.get('name')==pick_ref:
            p=a.get('pose',{})
            if 'xyz' in p: return p['xyz']
            return [p.get('x',0.0),p.get('y',0.0),p.get('z',0.0)]
    return None

def _approach_vector(grasp:dict[str,Any])->list[float]:
    axis=(grasp.get('grasp',{}) or grasp).get('approach_axis','z_down')
    m={'x_plus':[1,0,0],'x_minus':[-1,0,0],'y_plus':[0,1,0],'y_minus':[0,-1,0],'z_up':[0,0,1],'z_down':[0,0,-1]}
    return m.get(axis,[0,0,-1])

def main() -> int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--profile', required=True)
    ap.add_argument('--input', required=True)
    ap.add_argument('--task')
    ap.add_argument('--grasp')
    ap.add_argument('--environment')
    ap.add_argument('--output', required=True)
    ap.add_argument('--markers')
    ap.add_argument('--summary', required=True)
    ap.add_argument('--selected-summary')
    args=ap.parse_args()

    profile=_load(Path(args.profile))
    detected=_load(Path(args.input))
    task=_load(Path(args.task)).get('task',{}) if args.task else {}
    grasp=_load(Path(args.grasp)) if args.grasp else {}
    env=_load(Path(args.environment)) if args.environment else {}

    if detected.get('schema_version')!='detected_objects/v1':
        raise SystemExit('invalid detected_objects schema')

    mapping=((profile.get('perception',{}) or {}).get('object_mapping',{}))
    conf=float(mapping.get('confidence_threshold',0.5))
    allowed=set(mapping.get('allowed_labels') or [])
    policy=mapping.get('selection_policy','first_object')
    pick_area_ref=mapping.get('pick_area_ref') or ((task.get('pick') or {}).get('source_ref'))
    place_ref=mapping.get('place_target_ref') or ((task.get('place') or {}).get('target_ref'))
    class_map=mapping.get('class_to_task_target',{})

    objects=detected.get('objects',[])
    warnings=[]
    candidates=[]
    for o in objects:
        if allowed and o.get('label') not in allowed:
            warnings.append(f"label_not_allowed:{o.get('label')}")
            continue
        if float(o.get('confidence',0.0)) < conf:
            warnings.append(f"low_confidence:{o.get('id')}")
        candidates.append(o)
    if not candidates:
        candidates=objects[:]
    selected=candidates[0] if candidates else {}
    if policy=='nearest_pick_area' and len(candidates)>1:
        pick_pos=_pick_area_position(env,pick_area_ref) if pick_area_ref else None
        if pick_pos:
            selected=min(candidates,key=lambda o:_dist((o.get('pose') or {}).get('xyz',[0,0,0]),pick_pos))
        else:
            warnings.append('pick_area_missing_fallback_first_object')

    frame_id=detected.get('frame_id')
    camera_frame=((profile.get('perception',{}).get('camera',{}) or {}).get('frame_id'))
    if frame_id and camera_frame and frame_id!=camera_frame:
        warnings.append('frame_mismatch_review_tf')

    selected_summary={
        'selected_object': {
            'id': selected.get('id'), 'label': selected.get('label'), 'tracking_id': selected.get('tracking_id'),
            'confidence': selected.get('confidence'), 'pose': selected.get('pose',{}), 'dimensions_xyz': selected.get('dimensions_xyz',[]), 'frame_id': frame_id,
        },
        'mapping': {'pick_area_ref':pick_area_ref,'place_target_ref':place_ref,'task_target_ref':class_map.get(selected.get('label')) or mapping.get('default_pick_object_ref')},
        'status':'BLOCKED' if not place_ref else ('WARN' if warnings else 'READY'),
        'warnings':warnings,
    }

    payload={'schema_version':'emd_grasp_bridge_payload/v1','source':'perception_replay','dry_run_only':True,'perception_provider':profile.get('perception',{}).get('provider','epd'),'targets':[selected_summary['selected_object']],'task_intent':selected_summary['mapping'],'runtime_execution':{'auto_execute':False,'moveit_planning_called':False,'robot_motion_called':False}}
    Path(args.output).write_text(json.dumps(payload,indent=2)+'\n',encoding='utf-8')
    if args.selected_summary:
        Path(args.selected_summary).write_text(json.dumps(selected_summary,indent=2)+'\n',encoding='utf-8')

    markers={'schema_version':'perception_replay_markers/v1','frame_id':frame_id or camera_frame or 'world','detected_objects':[{'id':o.get('id'),'label':o.get('label'),'confidence':o.get('confidence'),'pose':o.get('pose',{}),'dimensions_xyz':o.get('dimensions_xyz',[])} for o in objects],'selected_target_id':selected.get('id'),'pick_area_ref':pick_area_ref,'place_target_ref':place_ref,'pick_to_place_arrow':{'from':(selected.get('pose') or {}).get('xyz',[0,0,0]),'to':[0,0,0],'enabled':bool(place_ref)},'grasp_approach_vector':_approach_vector(grasp),'status':selected_summary['status'],'warnings':warnings}
    if args.markers:
        Path(args.markers).write_text(json.dumps(markers,indent=2)+'\n',encoding='utf-8')

    summary={'status':selected_summary['status'],'dry_run_only':True,'live_epd_launched':False,'runtime_execution_called':False,'moveit_planning_called':False,'robot_motion_called':False,'selected_pick_target':selected_summary['selected_object'],'bridge_payload_preview_ready':True,'perception_replay_markers_ready':bool(args.markers),'warnings':warnings,'note':'Offline replay preview only. Not a safety certificate.'}
    Path(args.summary).write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'payload':args.output,'summary':args.summary,'selected':selected.get('id')}))
    return 0

if __name__=='__main__':
    raise SystemExit(main())
