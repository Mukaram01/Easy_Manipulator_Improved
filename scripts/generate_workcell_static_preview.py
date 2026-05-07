#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, html, sys, subprocess
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:
    yaml = None

CANVAS_W, CANVAS_H = 900, 640

def load_yaml(path: Path) -> dict[str, Any]:
    if yaml is not None:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
        return data if isinstance(data, dict) else {}
    script_dir = Path(__file__).resolve().parent
    if str(script_dir) not in sys.path:
        sys.path.insert(0, str(script_dir))
    import validate_cell_definition as yaml_support
    loaded, _, _ = yaml_support.load_yaml(path)
    return loaded if isinstance(loaded, dict) else {}

def _xy(v: Any):
    if isinstance(v, list) and len(v) >= 2:
        return float(v[0]), float(v[1]), False
    return 0.0, 0.0, True

def _scale(x: float, y: float):
    return 450 + x*220, 320 - y*220


def _marker(name: str, marker_type: str, frame: str, pose_xyz: list[float], dims: list[float], label: str, category: str) -> dict[str, Any]:
    return {
        "name": name,
        "marker_type": marker_type,
        "frame": frame,
        "pose": {"xyz": [float(pose_xyz[0]), float(pose_xyz[1]), float(pose_xyz[2])], "rpy": [0.0, 0.0, 0.0]},
        "dimensions": {"x": float(dims[0]), "y": float(dims[1]), "z": float(dims[2])},
        "label": label,
        "category": category,
    }

def gen_preview(cell: dict[str, Any], env: dict[str, Any] | None, title: str, task_flow: dict[str, Any] | None=None) -> tuple[str, str, dict[str, Any], dict[str, Any]]:
    warnings: list[str] = []
    elements = []
    approx = 0
    robot = (cell.get('robot') or {})
    ee = (cell.get('end_effector') or {})
    task = (cell.get('task') or {})
    comm = (cell.get('commissioning') or {})
    markers: list[dict[str, Any]] = []

    # robot marker
    rx, ry = 0.0, 0.0
    sx, sy = _scale(rx, ry)
    elements.append(f"<circle cx='{sx:.1f}' cy='{sy:.1f}' r='26' fill='#0f766e'/><text x='{sx:.1f}' y='{sy+5:.1f}' text-anchor='middle' fill='white'>Robot</text>")
    markers.append(_marker("robot_base", "sphere", "world", [rx, ry, 0.0], [0.1, 0.1, 0.1], "Robot Base", "layout"))

    surfaces = ((cell.get('environment') or {}).get('support_surfaces') or [])
    for i,s in enumerate(surfaces[:3]):
        x,y,a = _xy((s.get('pose_xyz') if isinstance(s,dict) else None))
        if a:
            warnings.append(f"support_surface[{i}] missing pose_xyz; using approximate placement")
            approx +=1
            x,y=-0.2+0.3*i,0.25
        dim=(s.get('dimensions') if isinstance(s,dict) else None) or [0.8,0.6,0.05]
        w,h=float(dim[0]),float(dim[1])
        cx,cy=_scale(x,y)
        elements.append(f"<rect x='{cx-w*110:.1f}' y='{cy-h*110:.1f}' width='{w*220:.1f}' height='{h*220:.1f}' fill='none' stroke='#334155' stroke-width='2'/><text x='{cx:.1f}' y='{cy:.1f}' text-anchor='middle'>Table:{html.escape(str(s.get('id','surface')))}</text>")
        markers.append(_marker(f"table_{s.get('id','surface')}", "cube", "world", [x, y, float(dim[2]) * 0.5], [w, h, float(dim[2])], f"Table {s.get('id','surface')}", "layout"))

    dests = task.get('destinations') if isinstance(task.get('destinations'), list) else []
    for i,d in enumerate(dests[:6]):
        x,y,a = _xy((d.get('pose_xyz') if isinstance(d,dict) else None))
        if a:
            warnings.append(f"destination[{i}] missing pose_xyz; using approximate placement")
            approx +=1
            x,y=0.5, -0.4 + i*0.15
        cx,cy=_scale(x,y)
        elements.append(f"<rect x='{cx-34:.1f}' y='{cy-20:.1f}' width='68' height='40' fill='#bfdbfe' stroke='#1d4ed8'/><text x='{cx:.1f}' y='{cy+4:.1f}' text-anchor='middle'>Bin:{html.escape(str(d.get('id','dest')))}</text>")
        markers.append(_marker(f"place_zone_{d.get('id','dest')}", "cube", "world", [x, y, 0.05], [0.2, 0.2, 0.1], f"Place Zone {d.get('id','dest')}", "task-flow"))

    cam = cell.get('camera') if isinstance(cell.get('camera'), dict) else {}
    cx,cy=_scale(-0.55,0.55)
    elements.append(f"<polygon points='{cx},{cy-18} {cx-14},{cy+12} {cx+14},{cy+12}' fill='#a855f7'/><text x='{cx:.1f}' y='{cy+28:.1f}' text-anchor='middle'>Camera</text>")
    markers.append(_marker("camera_pose", "arrow", "world", [-0.55, 0.55, 0.4], [0.15, 0.03, 0.03], "Camera Pose", "layout"))

    if env and isinstance(env.get('zones'), list):
        for z in env['zones'][:4]:
            b=((z.get('bounds_xyz') or {}) if isinstance(z,dict) else {})
            mn,mx=b.get('min'),b.get('max')
            if isinstance(mn,list) and isinstance(mx,list) and len(mn)>=2 and len(mx)>=2:
                x1,y1=_scale(float(mn[0]),float(mn[1])); x2,y2=_scale(float(mx[0]),float(mx[1]))
                lx,minx=min(x1,x2),max(x1,x2); ty,by=min(y1,y2),max(y1,y2)
                elements.append(f"<rect x='{lx:.1f}' y='{ty:.1f}' width='{minx-lx:.1f}' height='{by-ty:.1f}' fill='none' stroke='#f59e0b' stroke-dasharray='4 3'/><text x='{lx+4:.1f}' y='{ty+14:.1f}' fill='#92400e'>{html.escape(str(z.get('id','zone')))}</text>")


    if task_flow:
        pick_id = task_flow.get('pick_source_id') or 'unknown'
        place_id = task_flow.get('place_target_id') or 'unknown'
        gx,gy=_scale(-0.7,-0.55); elements.append(f"<text x='{gx:.1f}' y='{gy:.1f}' fill='#111827'>Task Flow</text>")
        elements.append(f"<text x='{gx:.1f}' y='{gy+18:.1f}' fill='#1f2937'>Pick: {html.escape(str(pick_id))}</text>")
        elements.append(f"<text x='{gx:.1f}' y='{gy+36:.1f}' fill='#1f2937'>Place: {html.escape(str(place_id))}</text>")
        elements.append(f"<text x='{gx:.1f}' y='{gy+54:.1f}' fill='#1f2937'>Grasp: {html.escape(str(task_flow.get('grasp_strategy')))}</text>")
        elements.append(f"<text x='{gx:.1f}' y='{gy+72:.1f}' fill='#1f2937'>Release: {html.escape(str(task_flow.get('release_strategy')))}</text>")
        elements.append(f"<text x='{gx:.1f}' y='{gy+90:.1f}' fill='#1f2937'>Routes: {task_flow.get('routing_rule_count',0)}</text>")
        pick_xyz = (task_flow.get('pick_pose_xyz') or [0.35, -0.2, 0.08])
        place_xyz = (task_flow.get('place_pose_xyz') or [0.35, 0.35, 0.10])
        markers.extend([
            _marker("pick_zone_source", "cube", "world", pick_xyz, [0.2, 0.2, 0.1], f"Pick Zone {task_flow.get('pick_source_id','unknown')}", "task-flow"),
            _marker("grasp_point", "sphere", "world", pick_xyz, [0.04, 0.04, 0.04], f"Grasp {task_flow.get('grasp_strategy','unknown')}", "task-flow"),
            _marker("place_zone_target", "cube", "world", place_xyz, [0.2, 0.2, 0.1], f"Place Zone {task_flow.get('place_target_id','unknown')}", "task-flow"),
            _marker("release_point", "sphere", "world", place_xyz, [0.04, 0.04, 0.04], f"Release {task_flow.get('release_strategy','unknown')}", "task-flow"),
            _marker("approach_vector", "arrow", "world", [pick_xyz[0], pick_xyz[1], pick_xyz[2] + 0.12], [0.12, 0.02, 0.02], f"Approach {task_flow.get('approach_distance_m','?')}m", "task-flow"),
            _marker("retreat_vector", "arrow", "world", [pick_xyz[0], pick_xyz[1], pick_xyz[2] + 0.02], [0.10, 0.02, 0.02], f"Retreat {task_flow.get('retreat_distance_m','?')}m", "task-flow"),
            _marker("task_flow_pick_to_place", "arrow", "world", pick_xyz, [max(0.05, abs(float(place_xyz[0]) - float(pick_xyz[0]))), 0.02, 0.02], "Pick → Grasp → Place → Release", "task-flow"),
        ])

    builder_task_intent = cell.get('builder_task_intent') if isinstance(cell.get('builder_task_intent'), dict) else {}
    if builder_task_intent and (not builder_task_intent.get('pick') or not builder_task_intent.get('place')):
        warnings.append('Task intent is present but exact pick/place coordinates could not be resolved.')

    summary={
        'title': title,
        'robot': robot.get('model','unknown'),
        'end_effector': ee.get('id') or ee.get('type','unknown'),
        'task': task.get('type','unknown'),
        'runtime_mode': comm.get('runtime_mode','unknown'),
        'safety_status': {'fake_hardware_default': bool(comm.get('fake_hardware_default', True)), 'require_operator_review': bool(comm.get('require_operator_review', False))},
        'preview_only': comm.get('runtime_mode')=='preview_only' or bool(cell.get('preview_only', False)),
        'runtime_blocked': comm.get('runtime_mode')=='preview_only',
        'safety_banner_present': True,
        'warnings': warnings,
        'approximate_placements': approx,
        'task_flow_summary': task_flow or {},
        'builder_task_intent': {
            'pick': (builder_task_intent.get('pick') or {}),
            'grasp': (builder_task_intent.get('grasp') or {}),
            'place': (builder_task_intent.get('place') or {}),
        } if builder_task_intent else {},
    }
    svg=f"<svg xmlns='http://www.w3.org/2000/svg' width='{CANVAS_W}' height='{CANVAS_H}'><rect width='100%' height='100%' fill='#f8fafc'/><text x='20' y='30' font-size='20'>{html.escape(title)}</text><text x='20' y='55'>robot={html.escape(str(summary['robot']))}, ee={html.escape(str(summary['end_effector']))}, task={html.escape(str(summary['task']))}</text>{''.join(elements)}</svg>"
    task_flow_html = ''
    if task_flow:
        task_flow_html = f"<h2>Task Flow</h2><pre>{html.escape(json.dumps(task_flow, indent=2))}</pre>"
    html_doc=f"<!doctype html><html><body><h1>{html.escape(title)}</h1><p>Offline static preview approximation (not collision/safety validation).</p>{task_flow_html}{svg}<pre>{html.escape(json.dumps(summary, indent=2))}</pre></body></html>"
    marker_payload = {"schema": "workcell_visual_markers/v1", "markers": markers, "marker_count": len(markers), "task_flow_marker_count": len([m for m in markers if m.get('category') == 'task-flow'])}
    return svg, html_doc, summary, marker_payload


def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--cell-definition', required=True, type=Path)
    ap.add_argument('--environment-layout', type=Path)
    ap.add_argument('--output-dir', required=True, type=Path)
    ap.add_argument('--title', required=True)
    ap.add_argument('--task-intent', type=Path)
    ap.add_argument('--task-recipe', type=Path)
    ap.add_argument('--json', action='store_true')
    a=ap.parse_args()
    try:
        cell=load_yaml(a.cell_definition)
        if not cell or 'robot' not in cell:
            print('ERROR: cell definition structurally unusable', file=sys.stderr); return 2
        env=None
        if a.environment_layout and a.environment_layout.exists():
            env=load_yaml(a.environment_layout)
        elif a.environment_layout:
            env={}
        a.output_dir.mkdir(parents=True, exist_ok=True)
        task_flow=None
        if a.task_intent or a.task_recipe or a.environment_layout:
            cmd=[sys.executable, str(Path(__file__).resolve().parent/'summarize_task_flow.py'), '--json']
            if a.task_intent: cmd += ['--task-intent', str(a.task_intent)]
            if a.task_recipe: cmd += ['--task-recipe', str(a.task_recipe)]
            if a.environment_layout: cmd += ['--environment-layout', str(a.environment_layout)]
            run=subprocess.run(cmd,capture_output=True,text=True,check=False)
            try: task_flow=json.loads(run.stdout) if run.stdout.strip() else {}
            except Exception: task_flow={}
        svg, html_doc, summary, marker_payload=gen_preview(cell, env, a.title, task_flow)
        if a.environment_layout and not a.environment_layout.exists():
            summary.setdefault("warnings", []).append("environment_layout path missing; used approximate/default placement")
        (a.output_dir/'static_preview.svg').write_text(svg, encoding='utf-8')
        (a.output_dir/'static_preview.html').write_text(html_doc, encoding='utf-8')
        (a.output_dir/'static_preview_summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
        (a.output_dir/'visual_markers.json').write_text(json.dumps(marker_payload, indent=2)+'\n', encoding='utf-8')
        if a.json:
            print(json.dumps(summary, indent=2))
        return 0
    except Exception as exc:
        print(f'ERROR: failed to load/parse cell definition: {exc}', file=sys.stderr)
        return 2

if __name__=='__main__':
    raise SystemExit(main())
