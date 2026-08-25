#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from typing import Any
import sys
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
from capability_registry import load_structured_data
from workcell_studio_layout_source import CANONICAL_LAYOUT_REL
try:
    import yaml
except Exception:
    yaml=None

def _load(path: Path)->dict[str,Any]:
    if yaml is not None:
        try:
            data=yaml.safe_load(path.read_text(encoding="utf-8"))
            return data if isinstance(data,dict) else {}
        except Exception:
            pass
    data,_=load_structured_data(path)
    return data if isinstance(data,dict) else {}

def _exists_in_scene(scene: Path, token: str)->bool:
    for rel in [CANONICAL_LAYOUT_REL, "environment.yaml", "workcell_builder_metadata.yaml", "generated/environment_layout.yaml", "generated/cell_definition.yaml"]:
        p=scene/rel
        if p.is_file() and token in p.read_text(encoding='utf-8', errors='ignore'):
            return True
    return False

def _load_scene_layout_ids(scene: Path) -> set[str]:
    ids: set[str] = set()
    def collect(value: Any) -> None:
        if isinstance(value, dict):
            if value.get("id"):
                ids.add(str(value["id"]))
            for child in value.values():
                collect(child)
        elif isinstance(value, list):
            for child in value:
                collect(child)

    for rel in [CANONICAL_LAYOUT_REL, "environment.yaml", "workcell_builder_metadata.yaml", "generated/environment_layout.yaml", "environment_layout.yaml"]:
        p = scene / rel
        if not p.is_file():
            continue
        payload = _load(p)
        collect(payload)
    return ids

def validate(path: Path, scene_package: Path|None=None, grasp_dir: Path|None=None)->dict[str,Any]:
    errors=[]; warnings=[]
    payload=_load(path)
    if not payload:
        return {"status":"FAIL","errors":["Task intent YAML could not be loaded"],"warnings":[],"task_intent":{}}
    if payload.get('schema')!='workcell_builder_task_intent/v1': errors.append('schema must be workcell_builder_task_intent/v1')
    task=payload.get('task') if isinstance(payload.get('task'),dict) else {}
    pick_block = payload.get('pick') if isinstance(payload.get('pick'), dict) else {}
    place_block = payload.get('place') if isinstance(payload.get('place'), dict) else {}
    pick = pick_block.get('source') if isinstance(pick_block.get('source'), dict) else {}
    place = place_block.get('target') if isinstance(place_block.get('target'), dict) else {}
    grasp=payload.get('grasp') if isinstance(payload.get('grasp'),dict) else {}
    safety=payload.get('safety') if isinstance(payload.get('safety'),dict) else {}
    if not task.get('type'): errors.append('task.type is required')
    task_template = payload.get('task_template') if isinstance(payload.get('task_template'), dict) else {}
    template_id = str(task_template.get('id') or task.get('template') or '').strip()
    if template_id in {'inspection', 'machine_tending', 'conveyor_picking'}:
        warnings.append(f"task template '{template_id}' is preview-only; no full runtime yet")
    if not pick.get('id'): errors.append('Pick source is not selected.')
    if not pick.get('id'): errors.append('pick.source.id is required')
    if not place.get('id'):
        errors.append('Place target is not selected.')
        errors.append('place.target.id is required')
    strategy_ref=grasp.get('strategy_ref')
    if not strategy_ref and not grasp.get('inline_strategy'): errors.append('grasp.strategy_ref or grasp.inline_strategy is required')
    for key in ['approach_distance_m','retreat_distance_m']:
        if key in grasp:
            try:
                if float(grasp.get(key))<=0: errors.append(f'grasp.{key} must be positive')
            except Exception:
                errors.append(f'grasp.{key} must be numeric')
    expected={"metadata_only":True,"runtime_io_applied":False,"motion_started":False,"ros_launch_started":False}
    for k,v in expected.items():
        if safety.get(k)!=v: errors.append(f'safety.{k} must be {str(v).lower()}')
    resolved=None
    if strategy_ref and grasp_dir and grasp_dir.is_dir():
        for f in grasp_dir.glob('*.yaml'):
            txt=f.read_text(encoding='utf-8', errors='ignore')
            if strategy_ref in txt:
                resolved=str(f); break
        if not resolved: warnings.append(f"grasp strategy '{strategy_ref}' not found in catalog")
    if scene_package:
        if not (scene_package/'package.xml').is_file(): errors.append('scene package missing package.xml')
        if not (scene_package/'environment.yaml').is_file(): warnings.append('scene package missing environment.yaml')
        scene_ids = _load_scene_layout_ids(scene_package)
        if pick.get('id') and str(pick['id']) not in scene_ids and not _exists_in_scene(scene_package,str(pick['id'])):
            warnings.append(f"pick.source.id '{pick['id']}' could not be verified in scene metadata")
        if place.get('id') and str(place['id']) not in scene_ids and not _exists_in_scene(scene_package,str(place['id'])):
            warnings.append(f"place.target.id '{place['id']}' could not be verified in scene metadata")
        routing = payload.get('routing') if isinstance(payload.get('routing'), dict) else {}
        rules = routing.get('rules') if isinstance(routing.get('rules'), list) else []
        for rule in rules:
            if not isinstance(rule, dict):
                continue
            destination = rule.get('place_target') or rule.get('destination')
            if isinstance(destination, str) and destination and destination not in scene_ids:
                errors.append('Routing target does not exist.')
                errors.append(f"routing rule target '{destination}' does not exist in scene metadata")
        if pick.get('type')=='pick_zone' and pick.get('id') and str(pick['id']) not in scene_ids:
            errors.append('Pick zone is not defined.')
            errors.append(f"selected pick source '{pick['id']}' does not exist in scene metadata")
        if place.get('id') and str(place['id']) not in scene_ids:
            errors.append(f"selected place target '{place['id']}' does not exist in scene metadata")
    
    if isinstance(pick.get('type'), str) and pick.get('type') not in {'perception','pick_zone','fixed_object','replay_object','zone'}:
        errors.append('pick.source.type must be one of perception/pick_zone/fixed_object/replay_object')
    if isinstance(place.get('type'), str) and place.get('type') in {'TODO','todo'}:
        errors.append('place.target.type cannot be TODO')
    serialized = json.dumps(payload)
    placeholder_tokens = ['TODO pick zone label', 'TODO place target label', 'TODO_place_target_id', 'unset_destination', '"type": "TODO"', '"type": "todo"']
    if any(token in serialized for token in placeholder_tokens):
        errors.append('task intent contains TODO/unset placeholder values')
    missing_required_fields=[]
    if not task.get('type'): missing_required_fields.append('task.type')
    if not pick.get('id'): missing_required_fields.append('pick.source.id')
    if not place.get('id'): missing_required_fields.append('place.target.id')
    if not strategy_ref and not grasp.get('inline_strategy'): missing_required_fields.append('grasp.strategy_ref')
    suggested_next_actions=[]
    if 'pick.source.id' in missing_required_fields: suggested_next_actions.append('Select a pick source zone.')
    if 'place.target.id' in missing_required_fields: suggested_next_actions.append('Select a place target.')
    if 'grasp.strategy_ref' in missing_required_fields: suggested_next_actions.append('Choose a grasp strategy.')
    if not missing_required_fields: suggested_next_actions.append('Generate task recipe from task intent.')
    readiness_classification='task_intent_incomplete' if missing_required_fields else 'task_intent_ready_offline'
    status='FAIL' if errors else ('WARN' if warnings else 'PASS')
    return {"status":status,"errors":errors,"warnings":warnings,"resolved_grasp_strategy":resolved,"pick_source":pick,"place_target":place,"safety":safety,"task_intent":payload,"missing_required_fields":missing_required_fields,"suggested_next_actions":suggested_next_actions,"readiness_classification":readiness_classification}

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('task_intent', type=Path)
    ap.add_argument('--scene-package', type=Path)
    ap.add_argument('--capabilities-dir', type=Path)
    ap.add_argument('--grasp-strategies-dir', type=Path)
    ap.add_argument('--json', action='store_true')
    args=ap.parse_args()
    report=validate(args.task_intent,args.scene_package,args.grasp_strategies_dir)
    if args.json: print(json.dumps(report,indent=2))
    else:
        print(f"status: {report['status']}")
        for e in report['errors']: print(f"FAIL: {e}")
        for w in report['warnings']: print(f"WARN: {w}")
    return 1 if report['status']=='FAIL' else 0
if __name__=='__main__': raise SystemExit(main())
