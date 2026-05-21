#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, re
from collections import Counter
from pathlib import Path
try:
    from scripts.scene_root_resolver import resolve_scene_root
except ModuleNotFoundError:
    from scene_root_resolver import resolve_scene_root

ROOT = Path(__file__).resolve().parents[1]
SCENES_ROOT = resolve_scene_root(ROOT)
PLACEHOLDER_RE = re.compile(r"\$\{[^}]+\}")


def _load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return None


def check_scene(scene_dir: Path) -> dict:
    scene = scene_dir.name
    layout_path = scene_dir / 'layout' / 'workcell_studio_layout.yaml'
    has_layout = layout_path.exists()
    idx = _load_json(scene_dir / 'generated' / 'scene_visual_mesh_index.json') or {}
    items = idx.get('items', []) if isinstance(idx, dict) else []

    layer_counts = Counter()
    visual_counts = Counter()
    unresolved = 0
    unsafe_reasons = 0
    locked_generated = 0
    editable_layout = 0
    blockers, fixes = [], []

    for i in items:
        layer = i.get('source_layer') or i.get('item_source') or ('primitive_fallback' if i.get('geometry_type') in {'box','cylinder','sphere'} else 'mesh_preview')
        visual = i.get('active_visual_source') or ('expanded_urdf_mesh' if i.get('extraction_mode') == 'xacro_expanded' else ('mesh' if i.get('mesh_path') else 'primitive'))
        layer_counts[layer] += 1
        visual_counts[visual] += 1
        item_id = str(i.get('id', ''))
        unresolved += 1 if PLACEHOLDER_RE.search(item_id) else 0
        if i.get('unsafe_visual_reason'):
            unsafe_reasons += 1
        if layer == 'locked_generated_urdf_visual':
            if i.get('editable') is not False:
                blockers.append(f"locked item editable: {item_id}")
            locked_generated += 1
        if layer == 'editable_layout':
            if i.get('editable') is not True:
                blockers.append(f"editable layout locked: {item_id}")
            editable_layout += 1

    primitive_count = layer_counts['primitive_fallback']
    mesh_count = layer_counts['mesh_preview']
    urdf_count = layer_counts['locked_generated_urdf_visual']

    if not has_layout:
        blockers.append('missing layout/workcell_studio_layout.yaml')
    if primitive_count == 0:
        blockers.append('primitive_fallback layer missing')
        fixes.append('retain primitive_fallback visuals even when mesh/urdf previews exist')
    if mesh_count == 0:
        fixes.append('if safe mesh index exists, map items into mesh_preview layer')
    if unresolved > 0 and bool(idx.get('safe_for_preview', False)):
        blockers.append('unresolved placeholders remain in IDs during safe preview')

    status = 'PASS'
    if blockers:
        status = 'FAIL'
    elif mesh_count == 0 or urdf_count == 0:
        status = 'WARN'

    return {
        'scene': scene,
        'editable_layout_count': editable_layout,
        'mesh_preview_count': mesh_count,
        'locked_generated_urdf_visual_count': urdf_count,
        'primitive_fallback_count': primitive_count,
        'overlay_count': layer_counts['overlay'],
        'unsafe_visual_reason_count': unsafe_reasons,
        'unresolved_placeholder_count': unresolved,
        'contract_status': status,
        'blockers': blockers,
        'suggested_fixes': fixes,
        'source_layer_counts': dict(layer_counts),
        'active_visual_source_counts': dict(visual_counts),
    }


def markdown(report: list[dict]) -> str:
    lines = ['# Scene3D Canvas Contract Report', '']
    for r in report:
        lines += [f"## {r['scene']} - {r['contract_status']}",
                  f"- editable_layout_count: {r['editable_layout_count']}",
                  f"- mesh_preview_count: {r['mesh_preview_count']}",
                  f"- locked_generated_urdf_visual_count: {r['locked_generated_urdf_visual_count']}",
                  f"- primitive_fallback_count: {r['primitive_fallback_count']}",
                  f"- overlay_count: {r['overlay_count']}",
                  f"- unsafe_visual_reason_count: {r['unsafe_visual_reason_count']}",
                  f"- unresolved_placeholder_count: {r['unresolved_placeholder_count']}"]
        if r['blockers']:
            lines.append('- blockers:'); lines += [f"  - {b}" for b in r['blockers']]
        if r['suggested_fixes']:
            lines.append('- suggested_fixes:'); lines += [f"  - {s}" for s in r['suggested_fixes']]
        lines.append('')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', action='append')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--json')
    ap.add_argument('--markdown')
    args = ap.parse_args()

    scenes = []
    if args.all:
        scenes = sorted([p for p in SCENES_ROOT.iterdir() if p.is_dir()]) if SCENES_ROOT.exists() else []
    elif args.scene:
        scenes = [SCENES_ROOT / s for s in args.scene]
    else:
        ap.error('use --all or --scene <name>')

    missing = [str(s) for s in scenes if not s.exists()]
    if missing:
        ap.error(f"requested scene path is missing: {', '.join(missing)}")

    report = [check_scene(s) for s in scenes]
    overall = 'PASS' if all(r['contract_status'] == 'PASS' for r in report) else ('FAIL' if any(r['contract_status']=='FAIL' for r in report) else 'WARN')
    payload = {'overall_status': overall, 'scenes': report}

    if args.json:
        Path(args.json).write_text(json.dumps(payload, indent=2), encoding='utf-8')
    if args.markdown:
        Path(args.markdown).write_text(markdown(report), encoding='utf-8')
    print(f"Scene3D Contract: {overall}")
    for r in report:
        print(f"Layers: editable_layout={r['editable_layout_count']} mesh_preview={r['mesh_preview_count']} locked_generated_urdf_visual={r['locked_generated_urdf_visual_count']} primitive_fallback={r['primitive_fallback_count']} overlay={r['overlay_count']}")
    raise SystemExit(1 if overall == 'FAIL' else 0)

if __name__ == '__main__':
    main()
