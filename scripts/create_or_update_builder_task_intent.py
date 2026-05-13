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

def _readable_label(identifier: str) -> str:
    return " ".join(part.capitalize() for part in str(identifier).replace("-", "_").split("_") if part)

def _extract_label_map(payload: dict[str, Any]) -> dict[str, str]:
    label_map: dict[str, str] = {}
    for key in ("zones", "targets", "objects", "assets"):
        items = payload.get(key)
        if not isinstance(items, list):
            continue
        for item in items:
            if not isinstance(item, dict):
                continue
            item_id = item.get("id")
            label = item.get("label") or item.get("name")
            if item_id and isinstance(label, str) and label.strip():
                label_map[str(item_id)] = label.strip()
    meta = payload.get("metadata")
    if isinstance(meta, dict):
        labels = meta.get("labels")
        if isinstance(labels, dict):
            for key, value in labels.items():
                if key and isinstance(value, str) and value.strip():
                    label_map[str(key)] = value.strip()
    return label_map

def _resolve_scene_label(scene_package: Path, target_id: str) -> tuple[str, bool]:
    candidates = [
        scene_package / "generated" / "environment_layout.yaml",
        scene_package / "exported" / "environment_layout.yaml",
        scene_package / "workcell_builder_metadata.yaml",
        scene_package / "scene_metadata.yaml",
        scene_package / "environment.yaml",
    ]
    for candidate in candidates:
        if not candidate.is_file():
            continue
        payload = _load(candidate)
        label = _extract_label_map(payload).get(target_id)
        if label:
            return label, True
    return _readable_label(target_id), False

def _default(scene_package: str) -> dict[str, Any]:
    return {"schema":"workcell_builder_task_intent/v1","scene_package":scene_package,"task":{"id":"default_builder_task","type":"pick_place","mode":"offline_preview","template":"pick_place"},"task_template":{"id":"pick_place","scenario":"pick_place","runtime_status":"supported","notes":"Template metadata only; runtime remains existing pick/place and sorting flows."},"pick":{"source":{"type":"zone","id":"pick_zone_main"},"object_filter":{"class_id":"any","color":"any"}},"grasp":{"strategy_ref":"finger_pinch_basic","approach_axis":"z_down","approach_distance_m":0.1,"retreat_axis":"z_up","retreat_distance_m":0.1},"place":{"target":{"type":"bin","id":"bin_red"},"release_strategy":"tool_release","retreat_axis":"z_up","retreat_distance_m":0.1,"place_clearance_m":0.05},"routing":{"rules":[]},"safety":{"metadata_only":True,"runtime_io_applied":False,"motion_started":False,"ros_launch_started":False}}

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
    ap.add_argument('--task-template', default='pick_place', choices=['pick_place', 'sorting', 'inspection', 'machine_tending', 'conveyor_picking'])
    ap.add_argument('--pick-source', required=True, help='Pick source id (e.g. epd_detected_object or pick_zone_main)')
    ap.add_argument('--pick-source-type', choices=['epd_detected_object','epd_replay','fixed_object','pick_zone','perception'], default='epd_detected_object')
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
    payload.setdefault('task',{}).update({'id':a.task_id,'type':a.task_type, 'template': a.task_template})
    template_meta: dict[str, Any] = {
        'id': a.task_template,
        'scenario': a.task_template,
        'runtime_status': 'supported' if a.task_template in {'pick_place', 'sorting', 'conveyor_picking'} else 'preview_only',
    }
    if a.task_template == 'pick_place':
        template_meta['pick_place'] = {
            'pick_source': a.pick_source,
            'place_target': a.place_target,
            'object_class': a.object_class,
            'grasp_strategy_ref': a.grasp_strategy,
        }
    elif a.task_template == 'sorting':
        template_meta['sorting'] = {
            'source_area': a.pick_source,
            'destination_bins': [a.place_target],
            'classification_key': 'class_id' if a.object_class not in {'', 'any'} else 'manual',
            'fallback_bin': a.place_target,
        }
    elif a.task_template == 'inspection':
        template_meta['inspection'] = {
            'inspection_camera': 'TODO_camera',
            'object_pose_source': a.pick_source,
            'pass_fail_output_target': a.place_target,
            'warning': 'no full runtime yet',
        }
    elif a.task_template == 'machine_tending':
        template_meta['machine_tending'] = {
            'machine_pose': 'TODO_machine_pose',
            'load_pose': a.pick_source,
            'unload_pose': a.place_target,
            'door_open_close': 'placeholder_only',
            'warning': 'no full runtime yet',
        }
    elif a.task_template == 'conveyor_picking':
        template_meta['conveyor_picking'] = {'source_area': a.pick_source, 'drop_target': a.place_target}
    payload['task_template'] = template_meta
    source_type = {'epd_detected_object':'perception','epd_replay':'replay_object'}.get(a.pick_source_type, a.pick_source_type)
    payload.setdefault('pick',{}).setdefault('source',{}).update({'id':a.pick_source, 'type': source_type})
    pick_label, pick_resolved = _resolve_scene_label(a.scene_package, a.pick_source)
    payload['pick']['source']['label'] = pick_label
    payload['pick'].setdefault('object_filter',{}).update({'class_id':a.object_class,'color':a.object_color})
    payload.setdefault('grasp',{}).update({'strategy_ref':a.grasp_strategy,'approach_axis':a.approach_axis,'approach_distance_m':a.approach_distance_m,'retreat_axis':a.retreat_axis,'retreat_distance_m':a.retreat_distance_m})
    
    payload.setdefault('place',{}).setdefault('target',{}).update({'id':a.place_target})
    payload['place'].setdefault('target',{}).setdefault('type', 'place_target')
    place_label, place_resolved = _resolve_scene_label(a.scene_package, a.place_target)
    payload['place']['target']['label'] = place_label
    payload['place'].update({'release_strategy':a.release_strategy,'retreat_axis':a.retreat_axis,'retreat_distance_m':a.retreat_distance_m, 'place_clearance_m': 0.05})
    payload.setdefault('routing', {})['rules'] = [{
        'id': 'route_any_to_selected_place',
        'when': {'object_class': a.object_class, 'object_color': a.object_color},
        'place_target': a.place_target,
    }]
    payload['safety']={'metadata_only':True,'runtime_io_applied':False,'motion_started':False,'ros_launch_started':False}
    _dump(out, payload)
    val={'status':'SKIP'}
    if a.validate:
        val = validate_intent(out, a.scene_package)
    warnings: list[str] = []
    if not pick_resolved:
        warnings.append(f"pick source label metadata missing for '{a.pick_source}', derived readable label used")
    if not place_resolved:
        warnings.append(f"place target label metadata missing for '{a.place_target}', derived readable label used")
    summary={'result':'PASS','output_path':str(out),'validation':val,'pick_source':a.pick_source,'pick_source_type':source_type,'place_target':a.place_target,'grasp_strategy':a.grasp_strategy,'release_strategy':a.release_strategy,'safety':payload['safety'],'warnings':warnings}
    print(json.dumps(summary, indent=2) if a.json else f'Wrote {out}')
    return 0 if (not a.validate or val.get('status')!='FAIL') else 1

if __name__=='__main__': raise SystemExit(main())
