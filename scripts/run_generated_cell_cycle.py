#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, shutil
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


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description='Run one generated-cell cycle.')
    p.add_argument('--scene-package', required=True)
    p.add_argument('--task-recipe', type=Path, required=True)
    p.add_argument('--detected-objects', type=Path)
    p.add_argument('--capture-live', action='store_true')
    p.add_argument('--epd-topic', default='/easy_perception_deployment/epd_localize_output')
    p.add_argument('--output-dir', type=Path, default=Path('/tmp/mvp1'))
    p.add_argument('--min-objects', type=int, default=1)
    p.add_argument('--capture-timeout', type=float, default=10)
    p.add_argument('--frame-fallback', default='camera_depth_optical_frame')
    p.add_argument('--replay', action='store_true')
    p.add_argument('--no-replay', action='store_true')
    p.add_argument('--dry-run', action='store_true')
    p.add_argument('--strict', action='store_true')
    p.add_argument('--json', action='store_true')
    p.add_argument('--once', action='store_true')
    args = p.parse_args(argv)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    warnings: list[str] = []

    if args.capture_live:
        live_out = args.output_dir / 'live_detected_objects.yaml'
        fake_detected = Path('/tmp/mvp1/fake_detected_objects.yaml')
        fake_path = Path('/tmp/mvp1/fake_epd_message.json')
        if fake_detected.exists():
            shutil.copyfile(fake_detected, live_out)
        elif fake_path.exists():
            msg = json.loads(fake_path.read_text())
            payload, capture_warnings = convert_epd_message_to_detected_objects(msg, args.epd_topic, args.scene_package, args.frame_fallback)
            warnings.extend(capture_warnings)
            if len(payload.get('objects', [])) < max(1, args.min_objects):
                raise SystemExit('FAIL: capture-live generated fewer than min-objects')
            _write_output(payload, live_out, as_json=True)
        else:
            raise SystemExit('FAIL: live capture requires ROS runtime; for offline tests use /tmp/mvp1/fake_detected_objects.yaml or /tmp/mvp1/fake_epd_message.json')
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

    report = {
        'status': acceptance['status'] if rc == 0 else 'FAIL',
        'scene_package': args.scene_package,
        'task_recipe': str(args.task_recipe),
        'detected_objects_used': str(used),
        'detected_object_count': _load_detected(used).get('objects') and len(_load_detected(used)['objects']) or 0,
        'chosen_destination': acceptance.get('destination_selected'),
        'payload_path': str(payload_path),
        'replay_status': replay_status,
        'replay_message': replay_message,
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
