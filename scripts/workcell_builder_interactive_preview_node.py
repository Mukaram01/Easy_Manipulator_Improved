#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import yaml

MESH_EXTENSIONS = {'.stl', '.dae', '.obj'}


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> list[float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def mesh_status(mesh_path: str) -> str:
    if not mesh_path:
        return 'skipped_empty_mesh_path'
    low = mesh_path.lower()
    if not any(low.endswith(ext) for ext in MESH_EXTENSIONS):
        return 'skipped_invalid_mesh_extension'
    if mesh_path.startswith('/'):
        return 'warning_absolute_external_path'
    return 'ok'


def load_preview_yaml(preview_dir: Path) -> dict[str, Any]:
    preview_yaml = preview_dir / 'placed_objects_preview.yaml'
    if not preview_yaml.exists():
        return {'scene_name': preview_dir.name, 'placed_objects': []}
    try:
        data = yaml.safe_load(preview_yaml.read_text(encoding='utf-8'))
    except Exception:
        return {'scene_name': preview_dir.name, 'placed_objects': []}
    if not isinstance(data, dict):
        return {'scene_name': preview_dir.name, 'placed_objects': []}
    if not isinstance(data.get('placed_objects', []), list):
        data['placed_objects'] = []
    return data


def write_feedback_yaml(preview_dir: Path, scene_name: str, objects: list[dict[str, Any]]) -> None:
    out = {
        'scene_name': scene_name,
        'updated_at': datetime.now(timezone.utc).isoformat(),
        'source': 'rviz_interactive_marker_preview',
        'safe_for_robot_motion': False,
        'objects': objects,
    }
    (preview_dir / 'placed_objects_feedback.yaml').write_text(yaml.safe_dump(out, sort_keys=False), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description='Workcell Builder interactive RViz preview node')
    parser.add_argument('--preview-dir', required=True)
    args = parser.parse_args()
    preview_dir = Path(args.preview_dir)
    data = load_preview_yaml(preview_dir)
    scene_name = str(data.get('scene_name', preview_dir.name))
    feedback_objects = []
    for obj in data.get('placed_objects', []):
        if not isinstance(obj, dict):
            continue
        name = str(obj.get('name', 'unnamed_object'))
        mesh = str(obj.get('mesh', ''))
        pose = obj.get('pose', [0, 0, 0, 0, 0, 0])
        status = mesh_status(mesh)
        feedback_objects.append({
            'name': name,
            'pose': {'xyz': list(pose[:3]) if isinstance(pose, list) else [0, 0, 0], 'rpy': list(pose[3:6]) if isinstance(pose, list) else [0, 0, 0]},
            'original_mesh': mesh,
            'status': 'edited_in_rviz_preview' if status == 'ok' else status,
        })
    write_feedback_yaml(preview_dir, scene_name, feedback_objects)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
