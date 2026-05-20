#!/usr/bin/env python3
from __future__ import annotations
import json, math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / 'scenes'
BUILD = ROOT / 'build'
OUT = BUILD / 'workcell_studio_scene3d_visual_diagnostics.json'

def bounds_from_items(items):
    mins=[float('inf')]*3; maxs=[float('-inf')]*3; ok=False
    for it in items:
        pose=(it.get('pose') or {})
        xyz=pose.get('xyz') or [0,0,0]
        if not (isinstance(xyz,list) and len(xyz)==3):
            continue
        s=it.get('scale') or [1,1,1]
        if not (isinstance(s,list) and len(s)==3): s=[1,1,1]
        ext=[abs(float(v)) for v in s]
        lo=[float(xyz[i])-ext[i]*0.5 for i in range(3)]
        hi=[float(xyz[i])+ext[i]*0.5 for i in range(3)]
        mins=[min(mins[i], lo[i]) for i in range(3)]
        maxs=[max(maxs[i], hi[i]) for i in range(3)]
        ok=True
    return ({'min':mins,'max':maxs,'extents':[maxs[i]-mins[i] for i in range(3)]} if ok else {'min':[0,0,0],'max':[0,0,0],'extents':[0,0,0]})

def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    diagnostics={'scenes':[], 'warning_count':0, 'identical_position_warning_count':0,
                 'transform_resolved_count':0,'transform_partial_count':0,'transform_local_only_count':0,
                 'fallback_asset_item_count':0}
    hard_errors=[]
    for scene in sorted([p for p in SCENES.iterdir() if p.is_dir()]):
        idx=scene/'generated'/'scene_visual_mesh_index.json'
        if not idx.exists():
            hard_errors.append(f'missing index: {idx}')
            continue
        payload=json.loads(idx.read_text())
        items=payload.get('visual_items', [])
        mesh_items=[i for i in items if i.get('mesh_extension')]
        loaded=[i for i in mesh_items if i.get('resolved')]
        failed=[i for i in mesh_items if not i.get('resolved')]
        exts=[]
        for i in mesh_items:
            s=i.get('scale') or [1,1,1]
            if isinstance(s,list) and len(s)==3:
                exts.append(max(abs(float(s[0])),abs(float(s[1])),abs(float(s[2]))))
        largest=max(exts) if exts else 0.0
        smallest=min(exts) if exts else 0.0
        b=bounds_from_items(mesh_items or items)
        radius=0.5*math.sqrt(sum(float(v)*float(v) for v in b['extents']))
        warnings=[]
        transform_counts={'resolved':0,'partial':0,'local_only':0}
        for i in items:
            ts=i.get('transform_status','local_only')
            if ts not in transform_counts: ts='local_only'
            transform_counts[ts]+=1
            if 'fallback_asset_search' in (i.get('transform_source') or ''):
                diagnostics['fallback_asset_item_count'] += 1
        xyzs=[tuple((i.get('pose') or {}).get('xyz') or [0,0,0]) for i in mesh_items]
        shared=[]
        if xyzs and len(set(xyzs)) <= max(1, len(xyzs)//4):
            warnings.append('many mesh items share identical world positions')
            diagnostics['identical_position_warning_count'] += 1
            pos_to_ids={}
            for i in mesh_items:
                key=tuple((i.get('pose') or {}).get('xyz') or [0,0,0])
                pos_to_ids.setdefault(key,[]).append(i.get('id'))
            shared=[{'xyz':list(k),'visual_ids':v} for k,v in pos_to_ids.items() if len(v)>1]
        if largest>20.0 or (smallest>0 and smallest<0.001): warnings.append('extreme mesh scale detected')
        if radius>200 or (radius>0 and radius<0.001): warnings.append('scene bounds radius extreme')
        if largest>0 and smallest>0 and (largest/max(smallest,1e-12))>10000: warnings.append('fit view may be dominated by one bad mesh')
        if any((i.get('mesh_extension')=='.dae' and i.get('render_expected') and not i.get('resolved')) for i in mesh_items): warnings.append('dae parser or load produced non-renderable meshes')
        diagnostics['warning_count'] += len(warnings)
        diagnostics['scenes'].append({
            'scene': scene.name,
            'item_count': len(items),
            'mesh_item_count': len(mesh_items),
            'loaded_mesh_count': len(loaded),
            'failed_mesh_count': len(failed),
            'world_bounds': b,
            'largest_mesh': largest,
            'smallest_mesh': smallest,
            'fit_radius': radius,
            'warnings': warnings,
            'transform_resolved_count': transform_counts['resolved'],
            'transform_partial_count': transform_counts['partial'],
            'transform_local_only_count': transform_counts['local_only'],
            'shared_position_groups': shared,
        })
        diagnostics['transform_resolved_count'] += transform_counts['resolved']
        diagnostics['transform_partial_count'] += transform_counts['partial']
        diagnostics['transform_local_only_count'] += transform_counts['local_only']
    OUT.write_text(json.dumps(diagnostics, indent=2)+"\n")
    if hard_errors:
        print('\n'.join(hard_errors))
        raise SystemExit(1)
    print(f'Wrote {OUT} for {len(diagnostics["scenes"])} scenes; warnings={diagnostics["warning_count"]}')

if __name__=='__main__':
    main()
