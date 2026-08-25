#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
from typing import Any
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path: sys.path.insert(0, str(SCRIPT_DIR))
from capability_registry import load_structured_data
try:
    import yaml
except Exception:
    yaml=None

def _load(p: Path)->dict[str,Any]:
    if not p.exists(): return {}
    if yaml is not None:
        try:
            d=yaml.safe_load(p.read_text(encoding='utf-8')); return d if isinstance(d,dict) else {}
        except Exception: pass
    d,_=load_structured_data(p); return d if isinstance(d,dict) else {}

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('--scene-package',required=True,type=Path); ap.add_argument('--json',action='store_true'); a=ap.parse_args()
    if not a.scene_package.is_dir():
        print(json.dumps({'result':'FAIL','error':f'invalid scene package: {a.scene_package}'},indent=2)); return 2
    data={}; warnings=[]
    for rel in ['layout/workcell_studio_layout.yaml','generated/environment_layout.yaml','generated/cell_definition.yaml','environment_layout.yaml','cell_definition.yaml','environment.yaml','config/workcell_builder_task_intent.yaml','generated/workcell_builder_task_intent.yaml','workcell_builder_task_intent.yaml']:
        d=_load(a.scene_package/rel)
        if d: data[rel]=d
    zones=[]; bins=[]; objs=[]; surfaces=[]; cams=[]; zone_details=[]
    for d in data.values():
        for item in (d.get('items') or []):
            if not isinstance(item,dict) or not item.get('id'): continue
            role=str(item.get('role') or item.get('type') or '').lower()
            if role in {'pick','pick_zone'}:
                zones.append(item.get('id')); zone_details.append(item)
            elif role in {'place','place_zone','place_target','target_bin'}:
                zones.append(item.get('id')); zone_details.append(item)
        for z in (d.get('zones') or []):
            if isinstance(z,dict) and z.get('id'):
                zones.append(z.get('id')); zone_details.append(z)
        objs += [o.get('id') for o in (d.get('objects') or []) if isinstance(o,dict) and o.get('id')]
        surfaces += [s.get('id') for s in ((d.get('environment') or {}).get('support_surfaces') or []) if isinstance(s,dict) and s.get('id')]
        cam=((d.get('camera') or {}).get('id') if isinstance(d.get('camera'),dict) else None)
        if cam: cams.append(cam)
        task=(d.get('task') or {}) if isinstance(d.get('task'),dict) else {}
        bins += [x.get('id') for x in (task.get('destinations') or []) if isinstance(x,dict) and x.get('id')]
    pick_sources=sorted(set([z.get('id') for z in zone_details if str(z.get('type','')).lower() in {'pick','pick_zone'}] + [z for z in zones if 'pick' in z] + objs))
    place_targets=sorted(set([z.get('id') for z in zone_details if str(z.get('type','')).lower() in {'place','place_target','bin'}] + [z for z in zones if ('bin' in z or 'place' in z)] + bins))
    if not pick_sources:
        warnings.append('No explicit pick sources discovered; using fallback suggestion pick_zone_main.'); pick_sources=['pick_zone_main']
    if not place_targets:
        warnings.append('No explicit place targets discovered; using fallback suggestion bin_main.'); place_targets=['bin_main']
    def _coords(z):
        p=(z.get('pose') or {}) if isinstance(z,dict) else {}
        xyz=p.get('xyz') if isinstance(p.get('xyz'),list) else None
        return xyz
    out={'result':'PASS','scene_package':a.scene_package.as_posix(),'pick_sources':pick_sources,'place_targets':place_targets,'zone_targets':[{'id':z.get('id'),'type':z.get('type'),'label':z.get('label'),'xyz':_coords(z)} for z in zone_details if z.get('id')],'zones':sorted(set(zones)),'bins':sorted(set(bins)),'objects':sorted(set(objs)),'support_surfaces':sorted(set(surfaces)),'cameras':sorted(set(cams)),'warnings':warnings}
    print(json.dumps(out, indent=2) if a.json else out)
    return 0
if __name__=='__main__': raise SystemExit(main())
