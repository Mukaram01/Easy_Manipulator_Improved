#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / 'scenes'
OUT = ROOT / 'build/workcell_studio/visual_mesh_index_regeneration_report.json'


def parse():
    p=argparse.ArgumentParser()
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--all', action='store_true')
    g.add_argument('--scene')
    p.add_argument('--portable', action='store_true')
    p.add_argument('--fail-on-unsafe', action='store_true')
    return p.parse_args()


def scene_list(args):
    if args.scene:
        return [SCENES / args.scene]
    return sorted([p for p in SCENES.iterdir() if p.is_dir()])


def summarize(scene: Path):
    idx = scene / 'generated/scene_visual_mesh_index.json'
    data = json.loads(idx.read_text()) if idx.exists() else {}
    items = data.get('visual_items', [])
    mesh_backed = sum(1 for i in items if i.get('geometry_type') == 'mesh' and i.get('resolved'))
    primitive = sum(1 for i in items if i.get('geometry_type') in ('box','cylinder','sphere') or i.get('item_source') == 'primitive_fallback')
    unresolved = sum(1 for i in items if i.get('geometry_type') == 'mesh' and not i.get('resolved'))
    stale_unsafe = int(bool(data.get('stale_index'))) + (1 if not data.get('safe_for_preview', True) else 0)
    previewable = mesh_backed + primitive
    status = 'PASS'
    if stale_unsafe or unresolved:
        status = 'WARN'
    if previewable <= 0:
        status = 'FAIL'
    return {
        'scene': scene.name,
        'status': status,
        'visual_item_count': len(items),
        'mesh_backed_count': mesh_backed,
        'primitive_fallback_count': primitive,
        'unresolved_count': unresolved,
        'stale_or_unsafe_count': stale_unsafe,
        'generated_index_path': str(idx.relative_to(ROOT)),
    }


def main():
    a=parse()
    rows=[]
    for scene in scene_list(a):
        cmd=['python3', str(ROOT/'scripts/extract_scene_urdf_visual_mesh_index.py'), '--scene', scene.name]
        subprocess.run(cmd, check=False)
        rows.append(summarize(scene))
    payload={'schema':'workcell_studio_visual_mesh_index_regeneration/v1','scenes':rows}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(payload, indent=2)+'\n')
    print(OUT)
    if a.fail_on_unsafe and any(r['status']!='PASS' for r in rows):
        return 1
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
