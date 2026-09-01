#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, math
from pathlib import Path
from typing import Any

NORMALIZED_SNAPSHOT_SCHEMA_VERSION = "workcell_perception_snapshot/v1"

def _finite_vec(values: Any, size: int, name: str, errors: list[str]) -> list[float] | None:
    if not isinstance(values, list) or len(values) != size:
        errors.append(f"{name} must be a {size}-value list")
        return None
    out=[]
    for v in values:
        try:
            f=float(v)
        except Exception:
            errors.append(f"{name} contains non-numeric value")
            return None
        if not math.isfinite(f):
            errors.append(f"{name} contains non-finite value")
            return None
        out.append(f)
    return out

def validate_normalized_snapshot(snapshot: dict[str, Any], *, expected_scene_id: str | None = None, expected_camera_id: str | None = None) -> list[str]:
    """Validate the versioned Workcell Studio normalized EPD snapshot contract."""
    errors: list[str] = []
    if snapshot.get("schema_version") != NORMALIZED_SNAPSHOT_SCHEMA_VERSION:
        errors.append(f"schema_version must be {NORMALIZED_SNAPSHOT_SCHEMA_VERSION}")
    scene_id = snapshot.get("scene_id")
    camera_id = snapshot.get("camera_id")
    timestamp = snapshot.get("timestamp")
    frame_id = snapshot.get("frame_id")
    if not scene_id: errors.append("scene_id is required")
    if not camera_id: errors.append("camera_id is required")
    if not timestamp: errors.append("timestamp is required")
    if not frame_id: errors.append("frame_id is required")
    if expected_scene_id and scene_id != expected_scene_id:
        errors.append(f"scene_id mismatch: expected {expected_scene_id}, got {scene_id}")
    if expected_camera_id and camera_id != expected_camera_id:
        errors.append(f"camera_id mismatch: expected {expected_camera_id}, got {camera_id}")
    objects = snapshot.get("objects")
    if not isinstance(objects, list):
        errors.append("objects must be a list")
        return errors
    seen: set[str] = set()
    for idx, obj in enumerate(objects):
        if not isinstance(obj, dict):
            errors.append(f"objects[{idx}] must be a mapping")
            continue
        oid = obj.get("object_id") or obj.get("track_id")
        if not oid:
            errors.append(f"objects[{idx}] requires object_id or track_id")
        elif str(oid) in seen:
            errors.append(f"duplicate object id: {oid}")
        else:
            seen.add(str(oid))
        if not obj.get("label"):
            errors.append(f"objects[{idx}].label is required")
        if obj.get("confidence") is not None:
            try:
                conf = float(obj.get("confidence"))
                if not math.isfinite(conf) or conf < 0.0 or conf > 1.0:
                    errors.append(f"objects[{idx}].confidence must be in [0, 1]")
            except Exception:
                errors.append(f"objects[{idx}].confidence must be numeric when present")
        pose = obj.get("pose") if isinstance(obj.get("pose"), dict) else None
        centroid = obj.get("centroid")
        if pose:
            _finite_vec(pose.get("position"), 3, f"objects[{idx}].pose.position", errors)
            q = _finite_vec(pose.get("orientation_xyzw"), 4, f"objects[{idx}].pose.orientation_xyzw", errors)
            if q is not None and abs(math.sqrt(sum(v*v for v in q)) - 1.0) > 1e-3:
                errors.append(f"objects[{idx}].pose.orientation_xyzw must be normalized")
            if pose.get("frame_id") and pose.get("frame_id") != frame_id:
                errors.append(f"objects[{idx}].pose.frame_id must match snapshot frame_id")
        elif centroid is not None:
            _finite_vec(centroid, 3, f"objects[{idx}].centroid", errors)
        else:
            errors.append(f"objects[{idx}] requires pose or centroid")
        attrs = obj.get("attributes", {})
        if attrs is not None and not isinstance(attrs, dict):
            errors.append(f"objects[{idx}].attributes must be a mapping when present")
        dimensions = obj.get("dimensions_xyz")
        if dimensions is not None:
            parsed_dimensions = _finite_vec(dimensions, 3, f"objects[{idx}].dimensions_xyz", errors)
            if parsed_dimensions is not None and any(value <= 0.0 for value in parsed_dimensions):
                errors.append(f"objects[{idx}].dimensions_xyz values must be positive")
        shape = obj.get("shape")
        if shape is not None and (not isinstance(shape, str) or not shape.strip()):
            errors.append(f"objects[{idx}].shape must be a non-empty string when present")
    return errors

def normalize_detected_objects_snapshot(detected: dict[str, Any], profile: dict[str, Any]) -> dict[str, Any]:
    perception = profile.get("perception", {}) if isinstance(profile.get("perception"), dict) else {}
    camera = perception.get("camera", {}) if isinstance(perception.get("camera"), dict) else {}
    scene_id = detected.get("scene_id") or perception.get("scene_id") or profile.get("scene_id")
    camera_id = detected.get("camera_id") or camera.get("camera_id") or camera.get("id") or detected.get("camera")
    frame_id = detected.get("frame_id") or camera.get("frame_id") or camera.get("optical_frame_id")
    source = detected.get("source") if isinstance(detected.get("source"), dict) else {}
    timestamp = (detected.get("timestamp") or detected.get("captured_at") or
                 source.get("source_stamp_ns") or source.get("captured_at"))
    out = {"schema_version": NORMALIZED_SNAPSHOT_SCHEMA_VERSION, "scene_id": scene_id, "camera_id": camera_id, "timestamp": timestamp, "frame_id": frame_id, "objects": []}
    for obj in detected.get("objects", []):
        if not isinstance(obj, dict):
            continue
        pose = obj.get("pose") if isinstance(obj.get("pose"), dict) else {}
        centroid = obj.get("centroid")
        item = {
            "object_id": str(obj.get("object_id") or obj.get("id") or obj.get("name") or obj.get("tracking_id") or obj.get("track_id") or ""),
            "track_id": obj.get("tracking_id") or obj.get("track_id"),
            "label": obj.get("label") or obj.get("class_label") or obj.get("class") or obj.get("class_id") or obj.get("name"),
            "confidence": obj.get("confidence"),
            "attributes": obj.get("attributes", {}),
        }
        dimensions = obj.get("dimensions_xyz") or obj.get("dimensions")
        if isinstance(dimensions, dict) and all(axis in dimensions for axis in ("x", "y", "z")):
            dimensions = [dimensions["x"], dimensions["y"], dimensions["z"]]
        if dimensions is not None:
            item["dimensions_xyz"] = dimensions
        shape = obj.get("shape") or (obj.get("attributes", {}) or {}).get("shape")
        if isinstance(shape, dict):
            shape = shape.get("type")
        if shape is not None:
            item["shape"] = shape
        pos = pose.get("position") or pose.get("xyz") or obj.get("position")
        quat = pose.get("orientation_xyzw") or pose.get("quaternion_xyzw")
        if pos is not None:
            item["pose"] = {"frame_id": pose.get("frame_id") or frame_id, "position": pos, "orientation_xyzw": quat or [0.0, 0.0, 0.0, 1.0]}
        elif isinstance(centroid, dict):
            item["centroid"] = [centroid.get("x"), centroid.get("y"), centroid.get("z")]
        elif isinstance(centroid, list):
            item["centroid"] = centroid
        out["objects"].append(item)
    return out

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

    normalized_snapshot = normalize_detected_objects_snapshot(detected, profile)
    expected_scene = (profile.get('perception', {}) or {}).get('scene_id') or profile.get('scene_id')
    expected_camera = (((profile.get('perception', {}) or {}).get('camera', {}) or {}).get('camera_id')
                       or (((profile.get('perception', {}) or {}).get('camera', {}) or {}).get('id')))
    snapshot_errors = validate_normalized_snapshot(normalized_snapshot, expected_scene_id=expected_scene, expected_camera_id=expected_camera)
    if snapshot_errors:
        raise SystemExit('invalid normalized perception snapshot: ' + '; '.join(snapshot_errors))

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
        if not (o.get('dimensions_xyz') or o.get('dimensions')):
            warnings.append(f"planning_scene_geometry_unavailable:{o.get('id') or o.get('object_id')}")
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
