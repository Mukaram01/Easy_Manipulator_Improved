#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, html
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}

def _safe(v: Any) -> str:
    return html.escape(str(v if v is not None else ''))

def _artifact_status(path_str: str) -> str:
    if not path_str:
        return 'MISSING'
    return 'OK' if Path(path_str).exists() else 'MISSING'

def _load_text(path_str: str) -> str:
    p = Path(path_str)
    return p.read_text(encoding='utf-8') if p.exists() else ''

def main() -> int:
    p=argparse.ArgumentParser()
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--title', default='Workcell Studio Readiness Dashboard')
    p.add_argument('--embed-static-preview', action='store_true')
    p.add_argument('--strict-links', action='store_true')
    p.add_argument('--json', action='store_true')
    a=p.parse_args()

    m=_read_json(a.manifest)
    arts=m.get('artifacts',{}) if isinstance(m.get('artifacts'),dict) else {}
    results=m.get('results',{}) if isinstance(m.get('results'),dict) else {}
    safety=m.get('safety',{}) if isinstance(m.get('safety'),dict) else {}
    summary=m.get('summary',{}) if isinstance(m.get('summary'),dict) else {}
    src=m.get('source',{}) if isinstance(m.get('source'),dict) else {}

    static_ref=arts.get('static_preview',{}) if isinstance(arts.get('static_preview'),dict) else {}
    static_svg=static_ref.get('svg','')
    static_html=static_ref.get('html','')
    static_summary= _read_json(Path(static_ref.get('summary',''))) if static_ref.get('summary') else {}

    task_flow = _read_json(Path(arts.get('task_flow_summary',''))) if arts.get('task_flow_summary') else {}
    next_commands_path = a.output.parent / 'next_commands.md'
    next_commands = next_commands_path.read_text(encoding='utf-8') if next_commands_path.exists() else ''

    art_rows=[
        ('builder export', arts.get('builder_export_summary','')),
        ('cell definition', arts.get('cell_definition','')),
        ('environment layout', arts.get('environment_layout','')),
        ('task intent', arts.get('builder_task_intent','')),
        ('task recipe', arts.get('task_recipe','')),
        ('task flow summary', arts.get('task_flow_summary','')),
        ('static preview', static_svg or static_html),
        ('offline plan preview request', arts.get('offline_plan_preview_request','')),
        ('RViz/MoveIt plan preview session', arts.get('rviz_moveit_plan_preview_session','')),
        ('smoke report', arts.get('fake_hardware_smoke_launch_report','')),
        ('planning scene readiness report', arts.get('planning_scene_readiness_report','')),
    ]

    preview_block=''
    if a.embed_static_preview and static_svg and Path(static_svg).exists():
        preview_block=f"<div class='panel'><h3>Embedded static preview (SVG)</h3><div class='svg'>{Path(static_svg).read_text(encoding='utf-8')}</div></div>"
    else:
        links=[]
        for label,path in [('SVG',static_svg),('HTML',static_html)]:
            if path:
                href=Path(path).as_posix()
                links.append(f"<li>{label}: <a href='{_safe(href)}'>{_safe(href)}</a></li>")
        preview_block=f"<div class='panel'><h3>Static preview links</h3><ul>{''.join(links) or '<li>Missing static preview artifacts</li>'}</ul></div>"

    tf = task_flow.get('summary',{}) if isinstance(task_flow.get('summary'),dict) else {}
    html_out=f"""<!doctype html><html><head><meta charset='utf-8'><title>{_safe(a.title)}</title>
<style>body{{font-family:Arial,sans-serif;margin:20px;background:#f8fafc}}.panel{{background:white;border:1px solid #ddd;border-radius:8px;padding:14px;margin-bottom:12px}}.badge{{padding:4px 8px;border-radius:10px;color:#fff}}.PASS{{background:#16803c}}.WARN{{background:#a97800}}.FAIL{{background:#a91d3a}}table{{width:100%;border-collapse:collapse}}th,td{{border:1px solid #ddd;padding:6px;text-align:left}}.warn{{background:#fff3cd}}</style>
</head><body>
<div class='panel'><h1>Workcell Studio</h1><h2>{_safe(src.get('project_name','unknown cell'))}</h2>
<p>Generated: {_safe(datetime.now(timezone.utc).isoformat())}</p>
<p>Final readiness: <span class='badge {_safe(results.get('final_readiness','WARN'))}'>{_safe(results.get('final_readiness','WARN'))}</span> Classification: <b>{_safe(results.get('classification','unknown'))}</b></p></div>
<div class='panel warn'><h3>Safety Banner</h3><ul><li>Offline/fake-hardware readiness review only</li><li>No robot motion commanded</li><li>No MoveIt planning service called</li><li>No real hardware enabled</li></ul><pre>{_safe(json.dumps(safety,indent=2))}</pre></div>
{preview_block}
<div class='panel'><h3>Task flow: pick → grasp → place → release</h3><ul>
<li>pick source: {_safe(tf.get('pick_source_id','missing'))}</li><li>grasp strategy: {_safe(tf.get('grasp_strategy','missing'))}</li><li>approach/retreat: {_safe(tf.get('approach','missing'))} / {_safe(tf.get('retreat','missing'))}</li><li>place target: {_safe(tf.get('place_target_id','missing'))}</li><li>release strategy: {_safe(tf.get('release_strategy','missing'))}</li><li>routing rules: {_safe(tf.get('routing_rules','missing'))}</li>
</ul></div>
<div class='panel'><h3>Artifact status</h3><table><tr><th>Artifact</th><th>Status</th><th>Path</th></tr>
{''.join([f"<tr><td>{_safe(n)}</td><td>{_safe(_artifact_status(pth))}</td><td>{_safe(pth)}</td></tr>" for n,pth in art_rows])}
</table></div>
<div class='panel'><h3>Blockers and warnings</h3><p>Blockers: {_safe(', '.join(summary.get('blockers',[])) or 'none')}</p><p>Warnings: {_safe(', '.join(summary.get('warnings',[])) or 'none')}</p><p>Suggested next actions: {_safe(', '.join(summary.get('suggested_next_actions',[])) or 'none')}</p></div>
<div class='panel'><h3>Generated commands / next commands</h3><p>Manual and guarded commands only.</p><pre>{_safe(next_commands or 'No next_commands.md found')}</pre></div>
<div class='panel'><h3>Footer</h3><p>This dashboard is not a safety certificate.</p><p>Validate on real hardware only through guarded commissioning procedures.</p></div>
</body></html>"""
    if a.strict_links and 'javascript:' in html_out.lower():
        raise SystemExit('strict-links violation')
    a.output.write_text(html_out, encoding='utf-8')
    if a.json:
        print(json.dumps({'result':'PASS','dashboard':str(a.output),'manifest':str(a.manifest)},indent=2))
    return 0

if __name__=='__main__':
    raise SystemExit(main())
