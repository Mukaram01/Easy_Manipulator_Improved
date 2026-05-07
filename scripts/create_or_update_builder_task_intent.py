#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
from capability_registry import load_structured_data
from validate_builder_task_intent import validate as validate_intent
try:
    import yaml
except Exception:
    yaml = None

def _load(path: Path) -> dict[str, Any]:
    if yaml is not None:
        try:
            d = yaml.safe_load(path.read_text(encoding='utf-8'))
            return d if isinstance(d, dict) else {}
        except Exception:
            pass
    d, _ = load_structured_data(path)
    return d if isinstance(d, dict) else {}

def _dump(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if yaml is None:
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding='utf-8')
    else:
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')

def _default(scene_package: str) -> dict[str, Any]:
    return {"schema":"workcell_builder_task_intent/v1","scene_package":scene_package,"task":{"id":"default_builder_task","type":"pick_place","mode":"offline_preview"},"pick":{"source":{"type":"zone","id":"pick_zone_main"},"object_filter":{"class_id":"any","color":"any"}},"grasp":{"strategy_ref":"finger_pinch_basic","approach_axis":"z_down","approach_distance_m":0.1,"retreat_axis":"z_up","retreat_distance_m":0.1},"place":{"target":{"type":"destination","id":"bin_main"},"release_strategy":"tool_release","retreat_axis":"z_up","retreat_distance_m":0.1},"routing":{"rules":[]},"safety":{"metadata_only":True,"runtime_io_applied":False,"motion_started":False,"ros_launch_started":False}}

def _seed(scene_package: Path, output: Path | None) -> tuple[dict[str, Any], Path | None]:
    cands = [output] if output else []
    cands += [scene_package/"workcell_builder_task_intent.yaml", scene_package/"generated"/"workcell_builder_task_intent.yaml", REPO_ROOT/"workcell_builder/workcell_builder/templates/workcell_builder_task_intent_template.yaml"]
    for c in cands:
        if c and c.is_file():
            return _load(c), c
    return _default(scene_package.as_posix()), None

def main() -> int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--scene-package', required=True, type=Path)
    ap.add_argument('--task-id', required=True)
    ap.add_argument('--task-type', required=True)
    ap.add_argument('--pick-source', required=True)
    ap.add_argument('--place-target', required=True)
    ap.add_argument('--grasp-strategy', required=True)
    ap.add_argument('--approach-axis', default='z_down')
    ap.add_argument('--approach-distance-m', type=float, default=0.1)
    ap.add_argument('--retreat-axis', default='z_up')
    ap.add_argument('--retreat-distance-m', type=float, default=0.1)
    ap.add_argument('--release-strategy', default='tool_release')
    ap.add_argument('--object-class', default='any')
    ap.add_argument('--object-color', default='any')
    ap.add_argument('--output', type=Path)
    ap.add_argument('--validate', action='store_true')
    ap.add_argument('--json', action='store_true')
    a=ap.parse_args()
    if not a.scene_package.is_dir():
        print(json.dumps({'result':'FAIL','error':f'invalid scene package: {a.scene_package}'}, indent=2))
        return 2
    payload, _ = _seed(a.scene_package, a.output)
    out = a.output or (a.scene_package/'generated'/'workcell_builder_task_intent.yaml')
    payload['schema']='workcell_builder_task_intent/v1'; payload['scene_package']=a.scene_package.as_posix()
    payload.setdefault('task',{}).update({'id':a.task_id,'type':a.task_type})
    payload.setdefault('pick',{}).setdefault('source',{}).update({'id':a.pick_source})
    payload['pick'].setdefault('object_filter',{}).update({'class_id':a.object_class,'color':a.object_color})
    payload.setdefault('grasp',{}).update({'strategy_ref':a.grasp_strategy,'approach_axis':a.approach_axis,'approach_distance_m':a.approach_distance_m,'retreat_axis':a.retreat_axis,'retreat_distance_m':a.retreat_distance_m})
    payload.setdefault('place',{}).setdefault('target',{}).update({'id':a.place_target})
    payload['place'].update({'release_strategy':a.release_strategy,'retreat_axis':a.retreat_axis,'retreat_distance_m':a.retreat_distance_m})
    payload['safety']={'metadata_only':True,'runtime_io_applied':False,'motion_started':False,'ros_launch_started':False}
    _dump(out, payload)
    val={'status':'SKIP'}
    if a.validate:
        val = validate_intent(out, a.scene_package)
    summary={'result':'PASS','output_path':str(out),'validation':val,'pick_source':a.pick_source,'place_target':a.place_target,'grasp_strategy':a.grasp_strategy,'release_strategy':a.release_strategy,'safety':payload['safety']}
    print(json.dumps(summary, indent=2) if a.json else f'Wrote {out}')
    return 0 if (not a.validate or val.get('status')!='FAIL') else 1

if __name__=='__main__': raise SystemExit(main())
