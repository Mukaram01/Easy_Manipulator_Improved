#!/usr/bin/env python3
from __future__ import annotations
import argparse,json
from pathlib import Path
from typing import Any
import sys
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
from capability_registry import load_structured_data
try:
    import yaml
except Exception:
    yaml=None

def _load(p: Path|None)->dict[str,Any]:
    if not p or not p.exists(): return {}
    if yaml is not None:
        try:
            d=yaml.safe_load(p.read_text())
            return d if isinstance(d,dict) else {}
        except Exception:
            pass
    d,_=load_structured_data(p)
    return d if isinstance(d,dict) else {}

def summarize(task_intent:Path|None=None, task_recipe:Path|None=None, scene_package:Path|None=None, environment_layout:Path|None=None)->dict[str,Any]:
    intent=_load(task_intent); recipe=_load(task_recipe); layout=_load(environment_layout)
    if recipe:
        bti=recipe.get('builder_task_intent',{}) if isinstance(recipe.get('builder_task_intent'),dict) else {}
        task=recipe.get('task',{}) if isinstance(recipe.get('task'),dict) else {}
        pick=((bti.get('pick',{}) or {}).get('source',{}) if isinstance(bti.get('pick'),dict) else {})
        place=((bti.get('place',{}) or {}).get('target',{}) if isinstance(bti.get('place'),dict) else {})
        grasp=recipe.get('grasp',{}) if isinstance(recipe.get('grasp'),dict) else {}
        pick_block=((bti.get('pick',{}) or {}) if isinstance(bti.get('pick'),dict) else {})
        grasp_block=((bti.get('grasp',{}) or {}) if isinstance(bti.get('grasp'),dict) else {})
        approach={'axis': grasp_block.get('approach_axis'), 'distance_m': grasp_block.get('approach_distance_m')}
        retreat={'axis': grasp_block.get('retreat_axis'), 'distance_m': grasp_block.get('retreat_distance_m')}
        release=((bti.get('place',{}) or {}).get('release_strategy') if isinstance(bti.get('place'),dict) else None)
        routing=task.get('rules',[]) if isinstance(task.get('rules'),list) else []
        readiness='task_recipe_generated'
    elif intent:
        task=intent.get('task',{}) if isinstance(intent.get('task'),dict) else {}
        pick=((intent.get('pick',{}) or {}).get('source',{}) if isinstance(intent.get('pick'),dict) else {})
        place=((intent.get('place',{}) or {}).get('target',{}) if isinstance(intent.get('place'),dict) else {})
        grasp=intent.get('grasp',{}) if isinstance(intent.get('grasp'),dict) else {}
        approach=(grasp.get('approach',{}) if isinstance(grasp.get('approach'),dict) else {})
        retreat=(grasp.get('retreat',{}) if isinstance(grasp.get('retreat'),dict) else {})
        release=((intent.get('place',{}) or {}).get('release_strategy') if isinstance(intent.get('place'),dict) else None)
        routing=((intent.get('routing',{}) or {}).get('rules',[]) if isinstance(intent.get('routing'),dict) else [])
        readiness='task_intent_ready_offline'
    elif scene_package:
        return {'status':'WARN','readiness_classification':'physical_scene_only','warnings':['Task intent/recipe missing; physical scene only.'],'errors':[],'missing_required_fields':[],'suggested_next_actions':['Create builder task intent to define pick/place/grasp.'],'safety':{'metadata_only':True,'runtime_io_applied':False,'motion_started':False,'ros_launch_started':False},'visual_resolution':{'pick_coordinates_resolved':False,'place_coordinates_resolved':False,'approximate_coordinates_used':True,'notes':['No task flow input.']}}
    else:
        return {'status':'FAIL','readiness_classification':'task_intent_incomplete','warnings':[],'errors':['No task input'],'missing_required_fields':['task_intent_or_task_recipe'],'suggested_next_actions':[]}
    missing=[]
    if not pick.get('id'): missing.append('pick.source.id')
    if not place.get('id'): missing.append('place.target.id')
    if not (grasp.get('strategy_ref') or grasp.get('inline_strategy')): missing.append('grasp.strategy_ref')
    if missing: readiness='task_intent_incomplete'
    zones={z.get('id'):z for z in (layout.get('zones') or []) if isinstance(z,dict) and z.get('id')}
    pick_res=bool(pick.get('id') in zones) if zones else False
    place_res=bool(place.get('id') in zones) if zones else False
    approx=not (pick_res and place_res)
    warnings=['Task flow present but exact pick/place coordinates could not be resolved.'] if approx else []
    return {'status':'FAIL' if missing else ('WARN' if warnings else 'PASS'),'readiness_classification':readiness,'task_id':task.get('id'),'task_type':task.get('type'),'pick_source_id':pick.get('id'),'place_target_id':place.get('id'),'grasp_strategy':grasp.get('strategy_ref') or grasp.get('inline_strategy'),'release_strategy':release,'approach_axis':approach.get('axis'),'approach_distance_m':approach.get('distance_m'),'retreat_axis':retreat.get('axis'),'retreat_distance_m':retreat.get('distance_m'),'routing_rules':routing,'routing_rule_count':len(routing),'missing_required_fields':missing,'suggested_next_actions':['Select a pick source zone.' if 'pick.source.id' in missing else '', 'Select a place target.' if 'place.target.id' in missing else '', 'Choose a grasp strategy.' if 'grasp.strategy_ref' in missing else '','Generate task recipe from task intent.' if not recipe else ''],'warnings':warnings,'errors':[] if not missing else [f'{m} is required' for m in missing],'safety':{'metadata_only':True,'runtime_io_applied':False,'motion_started':False,'ros_launch_started':False},'visual_resolution':{'pick_coordinates_resolved':pick_res,'place_coordinates_resolved':place_res,'approximate_coordinates_used':approx,'notes':['Used deterministic fallback coordinates for preview markers.'] if approx else []}}

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('--task-intent',type=Path); ap.add_argument('--task-recipe',type=Path); ap.add_argument('--scene-package',type=Path); ap.add_argument('--environment-layout',type=Path); ap.add_argument('--output',type=Path); ap.add_argument('--json',action='store_true'); a=ap.parse_args()
    out=summarize(a.task_intent,a.task_recipe,a.scene_package,a.environment_layout)
    if a.output:
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text(json.dumps(out, indent=2)+"\n", encoding='utf-8')
    print(json.dumps(out,indent=2) if a.json else out)
    return 1 if out.get('status')=='FAIL' else 0
if __name__=='__main__': raise SystemExit(main())
