#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, shutil
import subprocess, sys
from pathlib import Path
from typing import Any

from capture_epd_detected_objects import convert_epd_message_to_detected_objects, _write_output
from run_generated_cell_acceptance import run_acceptance
from replay_emd_bridge_payload import _load_payload, _validate, _send_runtime
from validate_detected_objects import _load_yaml_or_json


def _load_detected(path: Path) -> dict[str, Any]:
    try:
        data, _, _ = _load_yaml_or_json(path)
    except Exception:
        data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('detected_objects must be a mapping/object')
    return data


def _pick_selected_object(detected: dict[str, Any], acceptance: dict[str, Any]) -> dict[str, Any] | None:
    selected_name = acceptance.get("selected_object")
    objects = detected.get("objects") or []
    if not isinstance(objects, list):
        return None
    if selected_name:
        for obj in objects:
            if isinstance(obj, dict) and obj.get("name") == selected_name:
                return obj
    return objects[0] if objects else None


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description='Run one generated-cell cycle.')
    p.add_argument('--scene-package', required=True)
    p.add_argument('--task-recipe', type=Path, required=True)
    p.add_argument('--detected-objects', type=Path)
    p.add_argument('--capture-live', action='store_true')
    p.add_argument('--epd-topic', default='/easy_perception_deployment/epd_localize_output')
    p.add_argument('--output-dir', type=Path, default=Path('/tmp/mvp1'))
    p.add_argument('--epd-qos-reliability', choices=('auto','best_effort','reliable'), default='best_effort')
    p.add_argument('--epd-qos-depth', type=int, default=10)
    p.add_argument('--offline-fake-live', action='store_true')
    p.add_argument('--min-objects', type=int, default=1)
    p.add_argument('--capture-timeout', type=float, default=10)
    p.add_argument('--frame-fallback', default='camera_depth_optical_frame')
    p.add_argument('--target-frame', default='world')
    p.add_argument('--tf-timeout', type=float, default=2.0)
    p.add_argument('--require-transform', action='store_true', default=True)
    p.add_argument('--allow-untransformed', action='store_true')
    p.add_argument('--replay', action='store_true')
    p.add_argument('--no-replay', action='store_true')
    p.add_argument('--dry-run', action='store_true')
    p.add_argument('--strict', action='store_true')
    p.add_argument('--json', action='store_true')
    p.add_argument('--once', action='store_true')
    args = p.parse_args(argv)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    warnings: list[str] = []

    perception_source = "fixture"
    capture_status = 'offline_fixture'
    if args.capture_live:
        live_out = args.output_dir / 'live_detected_objects.yaml'
        if args.offline_fake_live:
            fake_detected = Path('/tmp/mvp1/fake_detected_objects.yaml')
            fake_path = Path('/tmp/mvp1/fake_epd_message.json')
            if fake_detected.exists():
                shutil.copyfile(fake_detected, live_out)
                capture_status = 'offline_fake_file'
            elif fake_path.exists():
                msg = json.loads(fake_path.read_text())
                payload, capture_warnings = convert_epd_message_to_detected_objects(msg, args.epd_topic, args.scene_package, args.frame_fallback)
                warnings.extend(capture_warnings)
                if len(payload.get('objects', [])) < max(1, args.min_objects):
                    raise SystemExit('FAIL: capture-live generated fewer than min-objects in offline fake mode')
                _write_output(payload, live_out, as_json=True)
                capture_status = 'offline_fake_message'
            else:
                raise SystemExit('FAIL: --offline-fake-live requested but no fake input found in /tmp/mvp1')
        else:
            capture_script = Path(__file__).resolve().parent / 'capture_epd_detected_objects.py'
            cap_cmd = [
                sys.executable, str(capture_script), '--topic', args.epd_topic, '--output', str(live_out),
                '--scene-package', args.scene_package, '--timeout', str(args.capture_timeout), '--min-objects', str(args.min_objects),
                '--frame-fallback', args.frame_fallback, '--qos-reliability', args.epd_qos_reliability, '--qos-depth', str(args.epd_qos_depth), '--once', '--json',
                '--target-frame', args.target_frame, '--tf-timeout', str(args.tf_timeout), '--require-transform'
            ]
            if args.allow_untransformed:
                cap_cmd.append('--allow-untransformed')
            cap = subprocess.run(cap_cmd, capture_output=True, text=True, check=False)
            if cap.returncode != 0:
                err = cap.stdout.strip() or cap.stderr.strip() or 'unknown error'
                raise SystemExit(f'FAIL: live EPD capture failed: {err}')
            capture_status = 'live_capture_ok'
            perception_source = 'live_epd'
        detected_input = live_out
    else:
        if not args.detected_objects:
            raise SystemExit('FAIL: --detected-objects is required when --capture-live is not set')
        if not args.detected_objects.exists():
            raise SystemExit(f'FAIL: detected_objects file not found: {args.detected_objects}')
        _load_detected(args.detected_objects)
        detected_input = args.detected_objects

    used = args.output_dir / 'detected_objects_used.yaml'
    shutil.copyfile(detected_input, used)
    detected_data = _load_detected(used)

    acceptance, rc = run_acceptance(args.scene_package, args.task_recipe, used, args.output_dir, strict=args.strict)
    payload_path = Path(acceptance['emd_bridge_payload_path'])
    replay_status = 'SKIPPED'
    replay_message = 'replay disabled'
    if args.no_replay:
        replay_status = 'SKIPPED'
    elif args.replay and not args.dry_run:
        payload = _load_payload(payload_path)
        w, e = _validate(payload, args.scene_package)
        warnings.extend(w)
        if e:
            replay_status = 'FAIL'
            replay_message = '; '.join(e)
            rc = 1
        else:
            ns = argparse.Namespace(ros_interface='service', service_name='grasp_requests', topic_name='grasp_tasks', frame_id='base_link')
            ok, msg = _send_runtime(payload, ns)
            replay_status = 'PASS' if ok else 'FAIL'
            replay_message = msg
            if not ok:
                rc = 1
    elif args.replay and args.dry_run:
        replay_status = 'SKIPPED'
        replay_message = 'dry-run enabled'

    detected_count = len(detected_data.get("objects", [])) if isinstance(detected_data.get("objects"), list) else 0
    if detected_count == 0:
        warnings.append("No objects detected in detected_objects_used payload")
    if detected_count < max(1, args.min_objects):
        warnings.append(f"Detected object count {detected_count} is below min-objects={max(1, args.min_objects)}")

    selected_obj = _pick_selected_object(detected_data, acceptance)
    selected_pose = (selected_obj or {}).get("pose") if isinstance(selected_obj, dict) else {}
    pose_frame = selected_pose.get("frame_id") if isinstance(selected_pose, dict) else None
    raw_pose = (selected_obj or {}).get("raw_pose") if isinstance(selected_obj, dict) else {}
    raw_pose_frame = raw_pose.get("frame_id") if isinstance(raw_pose, dict) else None
    transform_meta = (((detected_data.get("source") or {}).get("transform")) if isinstance(detected_data.get("source"), dict) else {}) or {}
    transform_status = transform_meta.get("status")
    transform_message = transform_meta.get("message")
    xyz = selected_pose.get("xyz") if isinstance(selected_pose, dict) else None
    conf = (selected_obj or {}).get("confidence") if isinstance(selected_obj, dict) else None
    dims = (selected_obj or {}).get("dimensions") if isinstance(selected_obj, dict) else None
    if selected_obj and not dims:
        warnings.append("Selected object missing dimensions")
    if selected_obj and conf is None:
        warnings.append("Selected object missing confidence")
    elif isinstance(conf, (int, float)) and conf < 0.5:
        warnings.append(f"Selected object has low confidence ({conf:.3f} < 0.500)")
    if pose_frame and pose_frame != args.target_frame:
        warnings.append(f"Selected object pose frame is '{pose_frame}', not target planning frame; transform validation not available")
    if transform_status == "WARN":
        warnings.append("Object pose transform is WARN; runtime replay is unsafe/not recommended")
    if isinstance(xyz, list) and len(xyz) >= 3 and isinstance(xyz[2], (int, float)) and xyz[2] < 0.0:
        warnings.append(f"Selected object z ({xyz[2]:.4f}) is below conservative table threshold (0.0)")

    report = {
        'status': acceptance['status'] if rc == 0 else 'FAIL',
        'scene_package': args.scene_package,
        'task_recipe': str(args.task_recipe),
        'perception_source': perception_source,
        'capture_status': capture_status,
        'epd_qos_reliability': args.epd_qos_reliability if args.capture_live else None,
        'epd_topic': args.epd_topic if args.capture_live else None,
        'detected_objects_used': str(used),
        'detected_object_count': detected_count,
        'selected_object': acceptance.get('selected_object'),
        'selected_object_class': (selected_obj or {}).get('class_id') if isinstance(selected_obj, dict) else None,
        'selected_object_label': (selected_obj or {}).get('name') if isinstance(selected_obj, dict) else None,
        'selected_object_confidence': conf,
        'object_pose_frame': pose_frame,
        'object_pose_frame_raw': raw_pose_frame,
        'object_pose_frame_normalized': pose_frame,
        'transform_status': transform_status,
        'transform_message': transform_message,
        'object_xyz': xyz,
        'chosen_destination': acceptance.get('destination_selected'),
        'destination_release_pose': acceptance.get('destination_release_pose'),
        'runtime_release_strategy': acceptance.get('runtime_release_strategy'),
        'payload_path': str(payload_path),
        'replay_status': replay_status,
        'replay_message': replay_message,
        'dry_run': bool(args.dry_run),
        'blockers': acceptance.get('blockers', []),
        'warnings': acceptance.get('warnings', []) + warnings,
        'acceptance': acceptance,
    }
    (args.output_dir / 'cycle_report.json').write_text(json.dumps(report, indent=2, sort_keys=True)+'\n')

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(f"{report['status']}: scene={args.scene_package} task={args.task_recipe}")
    return rc

if __name__ == '__main__':
    raise SystemExit(main())
