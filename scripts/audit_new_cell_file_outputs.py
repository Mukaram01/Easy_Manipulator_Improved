#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, re
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:
    yaml = None

REQUIRED_FILES = [
    'environment.yaml','environment_layout.yaml','config/workcell_builder_task_intent.yaml',
    'cell_definition.yaml','scene_manifest.yaml','package.xml','CMakeLists.txt','launch/demo.launch.py'
]
OPTIONAL_FILES = ['urdf/environment.urdf.xacro','README.md']


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    text = path.read_text(encoding='utf-8')
    if yaml is None:
        return {}
    try:
        data = yaml.safe_load(text)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _text(path: Path) -> str:
    return path.read_text(encoding='utf-8') if path.is_file() else ''


def _check_token(name: str, text: str, tokens: list[str], checks: list[dict[str, Any]], malformed: list[str]):
    ok = any(t.lower() in text.lower() for t in tokens)
    checks.append({'check': name, 'pass': ok, 'tokens': tokens})
    if not ok:
        malformed.append(name)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene-dir', required=True, type=Path)
    ap.add_argument('--scene-name', required=True)
    ap.add_argument('--json-out', required=True, type=Path)
    a = ap.parse_args()

    scene_dir = a.scene_dir
    report: dict[str, Any] = {
        'scene_name': a.scene_name,
        'scene_dir': str(scene_dir),
        'required_files': REQUIRED_FILES,
        'optional_files': OPTIONAL_FILES,
        'present_files': [],
        'missing_files': [],
        'malformed_files': [],
        'content_checks': [],
        'cross_reference_checks': [],
        'blockers': [],
        'warnings': [],
        'file_output_status': 'PASS',
    }

    for rel in REQUIRED_FILES + OPTIONAL_FILES:
        path = scene_dir / rel
        (report['present_files'] if path.exists() else report['missing_files']).append(rel) if rel in REQUIRED_FILES else None

    cell_text = _text(scene_dir / 'cell_definition.yaml')
    layout_text = _text(scene_dir / 'environment_layout.yaml')
    task_text = _text(scene_dir / 'config/workcell_builder_task_intent.yaml')
    launch_text = _text(scene_dir / 'launch/demo.launch.py')
    package_text = _text(scene_dir / 'package.xml')

    _check_token('cell_definition has ur5 token', cell_text, ['ur5'], report['content_checks'], report['malformed_files'])
    _check_token('cell_definition has robotiq token', cell_text, ['robotiq', 'robotiq 2f'], report['content_checks'], report['malformed_files'])
    _check_token('environment_layout has version marker', layout_text, ['environment_layout/v1', 'schema_version', 'version'], report['content_checks'], report['malformed_files'])
    _check_token('environment_layout has placed_assets list', layout_text, ['placed_assets', 'assets:'], report['content_checks'], report['malformed_files'])
    _check_token('task intent has pick_place', task_text, ['pick_place'], report['content_checks'], report['malformed_files'])
    _check_token('task intent has pick/source fields', task_text, ['pick:', 'source:'], report['content_checks'], report['malformed_files'])
    _check_token('task intent has place/target fields', task_text, ['place:', 'target:'], report['content_checks'], report['malformed_files'])
    _check_token('launch has use_fake_hardware', launch_text, ['use_fake_hardware'], report['content_checks'], report['malformed_files'])
    _check_token('launch has launch_rviz arg', launch_text, ['launch_rviz'], report['content_checks'], report['malformed_files'])

    pkg_name_match = re.search(r'<name>\s*([^<\s]+)\s*</name>', package_text)
    pkg_name = pkg_name_match.group(1).strip() if pkg_name_match else ''
    pkg_ok = pkg_name == a.scene_name
    report['cross_reference_checks'].append({'check': 'package.xml scene-name consistency', 'pass': pkg_ok, 'package_name': pkg_name})
    if not pkg_ok:
        report['blockers'].append(f'package.xml name mismatch: expected {a.scene_name}, found {pkg_name or "<missing>"}')

    layout = _load_yaml(scene_dir / 'environment_layout.yaml')
    task = _load_yaml(scene_dir / 'config/workcell_builder_task_intent.yaml')
    placed_assets = layout.get('placed_assets') or layout.get('assets') or []
    layout_ids = {str(item.get('id')) for item in placed_assets if isinstance(item, dict) and item.get('id')}
    pick_id = (((task.get('pick') or {}).get('source') or {}).get('id')) if isinstance(task, dict) else None
    place_id = (((task.get('place') or {}).get('target') or {}).get('id')) if isinstance(task, dict) else None
    for label, ref_id in [('pick source', pick_id), ('place target', place_id)]:
        if ref_id:
            ok = str(ref_id) in layout_ids
            report['cross_reference_checks'].append({'check': f'task pick/place cross-reference check exists ({label})', 'pass': ok, 'id': ref_id})
            if not ok:
                report['warnings'].append(f'{label} id not found in layout assets: {ref_id}')

    launch_cmd = f'ros2 launch {a.scene_name} demo.launch.py use_fake_hardware:=true launch_rviz:=true'
    report['cross_reference_checks'].append({'check': 'generated launch command points to scene package', 'pass': a.scene_name in launch_cmd, 'launch_command': launch_cmd})

    if report['missing_files'] or report['blockers']:
        report['file_output_status'] = 'BLOCKED'
    elif report['warnings'] or report['malformed_files']:
        report['file_output_status'] = 'WARNINGS'

    a.json_out.parent.mkdir(parents=True, exist_ok=True)
    a.json_out.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    return 0 if report['file_output_status'] == 'PASS' else 1

if __name__ == '__main__':
    raise SystemExit(main())
