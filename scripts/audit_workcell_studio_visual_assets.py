#!/usr/bin/env python3
from __future__ import annotations
import json
from datetime import datetime, timezone
from pathlib import Path
import re
from collections import Counter, defaultdict
from workcell_visual_asset_resolver import resolve_mesh_uri, discover_package_map

ROOT = Path(__file__).resolve().parents[1]
OUT_JSON = ROOT / 'build/workcell_studio/visual_asset_inventory.json'
OUT_MD = ROOT / 'build/workcell_studio/visual_asset_inventory.md'
MESH_EXTS = {'.stl','.dae','.obj','.mesh'}
MESH_RE = re.compile(r"filename=[\"']([^\"']+)[\"']")


def main() -> int:
    assets_root = ROOT / 'assets'
    wb_assets_root = ROOT / 'workcell_builder/workcell_builder/assets'
    scenes_root = ROOT / 'scenes'
    roots = [assets_root, wb_assets_root, scenes_root]
    files = []
    for r in roots:
        if r.exists():
            files.extend([p for p in r.rglob('*') if p.is_file() and p.suffix.lower() in MESH_EXTS])
    by_ext = Counter(p.suffix.lower() for p in files)
    dup = defaultdict(list)
    for p in files:
        dup[p.name.lower()].append(str(p.relative_to(ROOT)))
    duplicates = {k:v for k,v in dup.items() if len(v)>1}

    package_map = discover_package_map(ROOT, [scenes_root])
    scenes = []
    missing_refs, unresolved_pkg = [], []
    coverage = {k: False for k in ['ur5','robotiq_2f','suction','airpick','table_workbench','conveyor','camera','bins_boxes_fixtures']}
    for f in files:
        s=str(f).lower()
        if 'ur5' in s or 'universal_robot' in s: coverage['ur5']=True
        if 'robotiq' in s and '2f' in s: coverage['robotiq_2f']=True
        if 'suction' in s: coverage['suction']=True
        if 'airpick' in s: coverage['airpick']=True
        if 'table' in s or 'workbench' in s: coverage['table_workbench']=True
        if 'conveyor' in s: coverage['conveyor']=True
        if 'camera' in s or 'realsense' in s: coverage['camera']=True
        if any(t in s for t in ['bin','box','fixture','tote']): coverage['bins_boxes_fixtures']=True

    for scene in sorted([p for p in scenes_root.iterdir() if p.is_dir()] if scenes_root.exists() else []):
        refs=[]
        for urdf in list(scene.rglob('*.urdf'))+list(scene.rglob('*.xacro')):
            txt = urdf.read_text(errors='ignore')
            refs.extend(MESH_RE.findall(txt))
        unresolved=0
        for ref in refs:
            res=resolve_mesh_uri(ref, repo_root=ROOT, scene_dir=scene, package_map=package_map)
            if not res.exists:
                unresolved += 1
                missing_refs.append({'scene':scene.name, **res.to_dict()})
                if ref.startswith('package://'):
                    unresolved_pkg.append(ref)
        scenes.append({'scene':scene.name, 'mesh_reference_count':len(refs), 'unresolved_reference_count':unresolved})

    status = 'PASS' if not missing_refs else 'WARN'
    payload = {
        'schema':'workcell_studio_visual_asset_inventory/v1',
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'repo_root': str(ROOT),
        'asset_roots_scanned':[str(x) for x in roots],
        'total_mesh_files': len(files),
        'mesh_files_by_extension': dict(by_ext),
        'packages_detected': sorted(package_map.keys()),
        'scenes': scenes,
        'missing_mesh_references': missing_refs,
        'unresolved_package_uris': sorted(set(unresolved_pkg)),
        'duplicate_asset_names': duplicates,
        'oversized_mesh_warnings': [str(p.relative_to(ROOT)) for p in files if p.stat().st_size > 15*1024*1024],
        'coverage_expectations': {k:('PASS' if v else 'WARN') for k,v in coverage.items()},
        'summary': {'status': status}
    }
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(payload, indent=2)+'\n')
    md = ["# Workcell Studio Visual Asset Inventory", "", f"Status: **{status}**", "", f"Total mesh files: {len(files)}", f"Unresolved mesh references: {len(missing_refs)}", "", "## Coverage"]
    for k,v in payload['coverage_expectations'].items(): md.append(f"- {k}: {v}")
    OUT_MD.write_text('\n'.join(md)+'\n')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
