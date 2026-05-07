#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from typing import Any


def _load(path: Path) -> dict[str, Any]:
    import yaml  # type: ignore
    data = yaml.safe_load(path.read_text(encoding='utf-8')) if path.exists() else {}
    return data if isinstance(data, dict) else {}


def main() -> int:
    p = argparse.ArgumentParser(description='Validate generated perception profile for offline readiness flows')
    p.add_argument('profile', type=Path)
    p.add_argument('--json', action='store_true')
    a = p.parse_args()

    d = _load(a.profile)
    errors: list[str] = []
    warnings: list[str] = []
    blockers: list[str] = []

    def req(path: list[str], label: str) -> Any:
        cur: Any = d
        for k in path:
            if not isinstance(cur, dict) or k not in cur:
                errors.append(f'missing required field: {label}')
                return None
            cur = cur[k]
        return cur

    def req_str(path: list[str], label: str) -> str | None:
        v = req(path, label)
        if v is None:
            return None
        if not isinstance(v, str) or not v.strip():
            errors.append(f'{label} must be a non-empty string')
            return None
        return v

    req_str(['schema'], 'schema')
    sensor = req_str(['sensor', 'type'], 'sensor.type')
    if sensor and sensor != 'realsense_d435i':
        warnings.append(f"sensor.type is '{sensor}' (expected realsense_d435i for golden profile)")
    req_str(['sensor', 'camera_frame'], 'sensor.camera_frame')
    req_str(['topics', 'rgb'], 'topics.rgb')
    req_str(['topics', 'depth'], 'topics.depth')
    req_str(['topics', 'camera_info'], 'topics.camera_info')
    pc = d.get('topics', {}).get('point_cloud') if isinstance(d.get('topics'), dict) else None
    if pc is None or (isinstance(pc, str) and not pc.strip()):
        warnings.append('topics.point_cloud missing (optional for this profile)')
    elif not isinstance(pc, str):
        warnings.append('topics.point_cloud should be a string when provided')
    if not req_str(['topics', 'epd_localization_output'], 'topics.epd_localization_output'):
        blockers.append('Missing required EPD localization output topic')
    if not req_str(['topics', 'epd_tracking_output'], 'topics.epd_tracking_output'):
        blockers.append('Missing required EPD tracking output topic')

    req_str(['expected_snapshot_path'], 'expected_snapshot_path')
    req_str(['frames', 'object_frame'], 'frames.object_frame')
    req_str(['frames', 'scene_frame'], 'frames.scene_frame')

    safety = req(['safety_mode'], 'safety_mode')
    if isinstance(safety, dict):
        for k in ['perception_only', 'no_robot_motion', 'no_runtime_execution', 'fake_hardware_default']:
            if safety.get(k) is not True:
                errors.append(f'safety_mode.{k} must be true')

    status = 'PASS' if not errors and not blockers else 'FAIL'
    payload = {'status': status, 'errors': errors, 'warnings': warnings, 'blockers': blockers, 'profile': str(a.profile)}
    if a.json:
        print(json.dumps(payload, indent=2))
    return 0 if status == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
