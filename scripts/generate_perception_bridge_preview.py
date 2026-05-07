#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from typing import Any


def _load(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    txt = path.read_text(encoding='utf-8')
    if path.suffix.lower() == '.json':
        data = json.loads(txt)
        return data if isinstance(data, dict) else {}
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(txt)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--perception-profile', type=Path, required=True)
    ap.add_argument('--detected-objects', type=Path, required=True)
    ap.add_argument('--task-intent', type=Path, required=True)
    ap.add_argument('--environment-layout', type=Path)
    ap.add_argument('--output-payload', type=Path, required=True)
    ap.add_argument('--output-report', type=Path, required=True)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    prof = _load(a.perception_profile)
    snap = _load(a.detected_objects)
    intent = _load(a.task_intent)
    layout = _load(a.environment_layout) if a.environment_layout else {}

    warnings: list[str] = []
    blockers: list[str] = []

    objs = snap.get('objects') if isinstance(snap.get('objects'), list) else []
    obj = objs[0] if objs and isinstance(objs[0], dict) else {}

    label = obj.get('label') or obj.get('class')
    confidence = obj.get('confidence')
    pose = obj.get('pose') if isinstance(obj.get('pose'), dict) else {}
    xyz = pose.get('xyz') if isinstance(pose.get('xyz'), list) else []
    obj_frame = obj.get('frame_id') or snap.get('frame_id')
    scene_frame = ((prof.get('frames') or {}).get('scene_frame') if isinstance(prof.get('frames'), dict) else None)
    frame_match = bool(obj_frame and scene_frame and obj_frame == scene_frame)
    if not frame_match:
        warnings.append('Detected object frame does not match expected scene frame')

    if not label:
        blockers.append('Detected object label/class missing')
    if confidence is None:
        blockers.append('Detected object confidence missing')
    if len(xyz) != 3:
        blockers.append('Detected object pose.xyz missing or invalid')

    pick = intent.get('pick', {}) if isinstance(intent.get('pick'), dict) else {}
    place = intent.get('place', {}) if isinstance(intent.get('place'), dict) else {}
    grasp = intent.get('grasp', {}) if isinstance(intent.get('grasp'), dict) else {}

    pick_source = (pick.get('source') or {}).get('id') if isinstance(pick.get('source'), dict) else None
    place_target = (place.get('target') or {}).get('id') if isinstance(place.get('target'), dict) else None
    grasp_strategy = grasp.get('strategy_ref') or grasp.get('inline_strategy')
    release_strategy = place.get('release_strategy') if isinstance(place, dict) else None
    approach_distance = grasp.get('approach_distance_m')
    retreat_distance = grasp.get('retreat_distance_m')

    if not pick_source:
        blockers.append('pick.source.id missing from task intent')
    if not place_target:
        blockers.append('place.target.id missing from task intent')
    if not grasp_strategy:
        blockers.append('grasp strategy missing from task intent')
    if approach_distance is None or retreat_distance is None:
        blockers.append('approach/retreat distance metadata missing')
    if not release_strategy:
        blockers.append('release strategy missing')

    route_result = 'routed' if (label and pick_source and place_target) else 'unroutable'
    if route_result != 'routed':
        blockers.append('object label/class cannot be routed by task intent')

    status = 'bridge_preview_ready'
    if blockers:
        status = 'bridge_preview_blocked'
    elif warnings:
        status = 'bridge_preview_partial'

    payload = {
        'schema': 'emd_bridge_payload_preview/v1',
        'source': 'perception_replay',
        'preview_only': True,
        'dry_run': True,
        'no_robot_motion': True,
        'no_runtime_execution': True,
        'moveit_plan_service_called': False,
        'real_hardware_enabled': False,
        'motion_command_sent': False,
        'runtime_execution_called': False,
        'status': status,
        'selected_object': {
            'id': obj.get('id'), 'label': label, 'class': obj.get('class'), 'confidence': confidence,
            'frame_id': obj_frame, 'pick_pose': pose,
        },
        'task_bridge': {
            'routing_result': route_result,
            'pick_source': pick_source,
            'grasp_strategy': grasp_strategy,
            'approach': {'axis': grasp.get('approach_axis'), 'distance_m': approach_distance},
            'retreat': {'axis': grasp.get('retreat_axis'), 'distance_m': retreat_distance},
            'place_target': place_target,
            'release_strategy': release_strategy,
        },
        'safety_flags': {
            'dry_run': True, 'preview_only': True, 'no_robot_motion': True, 'no_runtime_execution': True,
            'moveit_plan_service_called': False, 'real_hardware_enabled': False,
        },
    }
    report = {
        'schema': 'perception_bridge_preview_report/v1',
        'status': status,
        'inputs': {
            'perception_profile': str(a.perception_profile),
            'detected_objects': str(a.detected_objects),
            'task_intent': str(a.task_intent),
            'environment_layout': str(a.environment_layout) if a.environment_layout else None,
        },
        'checks': {
            'label_class_confidence_pose_present': bool(label and confidence is not None and len(xyz) == 3),
            'frame_match': frame_match,
            'routable': route_result == 'routed',
            'grasp_selected': bool(grasp_strategy),
            'approach_retreat_present': bool(approach_distance is not None and retreat_distance is not None),
            'place_target_resolved': bool(place_target),
            'release_strategy_present': bool(release_strategy),
        },
        'warnings': warnings,
        'blockers': blockers,
        'output_payload': str(a.output_payload),
        'safety_flags': payload['safety_flags'],
    }

    a.output_payload.parent.mkdir(parents=True, exist_ok=True)
    a.output_report.parent.mkdir(parents=True, exist_ok=True)
    a.output_payload.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
    a.output_report.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')

    if a.json:
        print(json.dumps({'result': status, 'payload': str(a.output_payload), 'report': str(a.output_report)}, indent=2))
    return 0 if status != 'bridge_preview_blocked' else 1


if __name__ == '__main__':
    raise SystemExit(main())
