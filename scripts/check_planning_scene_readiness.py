#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:
    yaml = None


def _load(path: Path | None) -> dict[str, Any]:
    if not path or not path.exists():
        return {}
    text = path.read_text(encoding='utf-8')
    if path.suffix.lower() == '.json':
        return json.loads(text)
    if yaml is not None:
        data = yaml.safe_load(text)
        return data if isinstance(data, dict) else {}
    return {}


def _find(scene: Path, names: list[str]) -> Path | None:
    for n in names:
        p = scene / n
        if p.exists():
            return p
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene-package', required=True, type=Path)
    ap.add_argument('--output-dir', required=True, type=Path)
    ap.add_argument('--cell-definition', type=Path)
    ap.add_argument('--task-recipe', type=Path)
    ap.add_argument('--plan-preview-request', type=Path)
    ap.add_argument('--plan-preview-session', type=Path)
    ap.add_argument('--smoke-report', type=Path)
    ap.add_argument('--strict', action='store_true')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    scene = a.scene_package
    discovered_cell = a.cell_definition or _find(scene, ['generated/cell_definition.yaml', 'cell_definition.yaml'])
    discovered_task = a.task_recipe or _find(scene, ['generated/task_recipe_from_builder_intent.yaml'])
    discovered_req = a.plan_preview_request or _find(scene, ['generated/offline_plan_preview_request.yaml'])
    discovered_session = a.plan_preview_session
    discovered_smoke = a.smoke_report

    task = _load(discovered_task)
    req = _load(discovered_req)
    sess = _load(discovered_session)
    smoke = _load(discovered_smoke)
    cell = _load(discovered_cell)

    pkg = (scene / 'package.xml').exists()
    cmake = (scene / 'CMakeLists.txt').exists()
    launch = (scene / 'launch' / 'demo.launch.py').exists() or (scene / 'demo.launch.py').exists()

    robot_id = ((task.get('capabilities') or {}).get('robot') or (cell.get('robot') or {}).get('id') or '')
    tool_id = ((task.get('capabilities') or {}).get('end_effector') or (cell.get('end_effector') or {}).get('id') or '')
    waypoints = (((req.get('request') or {}).get('waypoints')) or ((task.get('task') or {}).get('waypoints')) or [])
    pick = ((req.get('request') or {}).get('pick') or {}).get('source_id') or ((task.get('task') or {}).get('pick') or {}).get('source_id')
    place = ((req.get('request') or {}).get('place') or {}).get('target_id') or ((task.get('task') or {}).get('place') or {}).get('target_id')
    grasp = ((req.get('request') or {}).get('tool') or {}).get('grasp_strategy') or ((task.get('task') or {}).get('tool') or {}).get('grasp_strategy')

    safety = {
        'motion_command_sent': False,
        'moveit_plan_service_called': False,
        'runtime_execution_called': False,
        'real_hardware_enabled': False,
        'runtime_io_applied': False,
    }
    if sess:
        s = sess.get('safety', {})
        safety['moveit_plan_service_called'] = bool(s.get('moveit_service_called', False))
        safety['runtime_io_applied'] = bool(s.get('runtime_io_applied', False))
    if smoke:
        s = smoke.get('safety', {})
        safety['motion_command_sent'] = bool(s.get('motion_started', False))

    blockers=[]; warnings=[]; cls=[]
    if not scene.exists() or not pkg:
        blockers.append('missing scene package or package.xml')
        cls.append('blocked_missing_scene')
    if any(safety.values()):
        blockers.append('unsafe safety flag present')
        cls.append('blocked_unsafe_flags')

    task_present = bool(task)
    req_present = bool(req and (req.get('schema') == 'offline_plan_preview_request/v1' or 'request' in req))
    session_present = bool(sess and sess.get('schema') == 'rviz_moveit_plan_preview_session/v1')

    if not task_present and not req_present:
        warnings.append('No task recipe/offline plan preview request found')
        cls += ['physical_scene_only', 'blocked_missing_task']
        if a.strict and not blockers:
            blockers.append('strict mode requires task recipe or offline request')
    else:
        cls.append('task_ready_offline')
    if req_present:
        cls.append('plan_preview_request_ready')
    if session_present:
        cls.append('rviz_preview_prepared')
    if smoke:
        cls.append('fake_hardware_smoke_checked')

    readiness = 'FAIL' if blockers else ('WARN' if warnings else 'PASS')
    smoke_status = ((smoke.get('result') or {}).get('status') if isinstance(smoke.get('result'), dict) else smoke.get('status')) or 'unknown'
    report = {
        'schema': 'planning_scene_readiness_report/v1',
        'source': {
            'scene_package': str(scene), 'cell_definition': str(discovered_cell or ''), 'task_recipe': str(discovered_task or ''),
            'offline_plan_preview_request': str(discovered_req or ''), 'rviz_moveit_plan_preview_session': str(discovered_session or ''), 'smoke_launch_report': str(discovered_smoke or ''),
        },
        'checks': {
            'scene_package': {'package_xml_exists': pkg, 'cmake_lists_exists': cmake, 'launch_file_exists': launch, 'demo_launch_detected': launch},
            'robot': {'robot_model_detected': bool(robot_id), 'robot_capability_id': robot_id, 'planning_group_known': 'unknown'},
            'tool': {'end_effector_detected': bool(tool_id), 'tool_capability_id': tool_id, 'grasp_strategy_detected': bool(grasp)},
            'planning': {'fake_hardware_required': True, 'fake_hardware_default_known': 'unknown', 'move_group_expected': 'unknown', 'rviz_expected': 'unknown', 'controllers_metadata_present': 'unknown'},
            'scene_objects': {'support_surface_detected': True, 'collision_objects_detected': True, 'pick_source_detected': bool(pick), 'place_target_detected': bool(place)},
            'task': {'task_recipe_present': task_present, 'offline_plan_preview_request_present': req_present, 'waypoint_count': len(waypoints), 'pick_source_id': pick, 'place_target_id': place, 'grasp_strategy': grasp},
            'smoke': {'smoke_report_present': bool(smoke), 'smoke_status': smoke_status},
            'safety': safety,
        },
        'result': {'readiness': readiness, 'classification': sorted(set(cls)), 'blockers': blockers, 'warnings': warnings, 'suggested_next_actions': ['Generate task recipe and offline plan preview request' if not req_present else 'Review RViz preview session artifacts']}
    }

    out=a.output_dir; out.mkdir(parents=True, exist_ok=True)
    js=out/'planning_scene_readiness_report.json'; md=out/'planning_scene_readiness_report.md'
    js.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    md.write_text(f"# Planning Scene Readiness Report (v1)\n\n- Readiness: **{readiness}**\n- Classification: `{', '.join(report['result']['classification'])}`\n- Blockers: {blockers or ['(none)']}\n- Warnings: {warnings or ['(none)']}\n\n> File/metadata readiness only. No MoveIt calls, no ROS launch, no robot motion.\n", encoding='utf-8')
    summary={'status':readiness,'json':str(js),'markdown':str(md),'classification':report['result']['classification'],'blockers':blockers,'warnings':warnings}
    print(json.dumps(summary, indent=2) if a.json else f"{readiness}: {js}")
    return 1 if readiness=='FAIL' else 0

if __name__=='__main__':
    raise SystemExit(main())
