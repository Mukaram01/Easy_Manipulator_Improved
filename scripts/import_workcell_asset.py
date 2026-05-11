#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, shutil
from pathlib import Path
from scripts.validate_workcell_asset_catalog import SAFE

VALID_EXT={'.stl','.urdf','.xacro'}

def sanitize(name:str)->str:
    return '_'.join(filter(None,[''.join(c.lower() if c.isalnum() else '_' for c in name).strip('_')])).replace('__','_')

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--source',required=True)
    ap.add_argument('--asset-id',required=True)
    ap.add_argument('--label',required=True)
    ap.add_argument('--category',default='Custom / Imported')
    ap.add_argument('--asset-type',required=True)
    ap.add_argument('--repo-root',default='.')
    ap.add_argument('--print-summary',action='store_true')
    a=ap.parse_args()
    root=Path(a.repo_root).resolve(); src=Path(a.source)
    if not src.exists(): raise SystemExit('source missing')
    if src.suffix.lower() not in VALID_EXT: raise SystemExit('unsupported file type')
    aid=sanitize(a.asset_id)
    if not SAFE.match(aid): raise SystemExit('unsafe asset id')
    dest_dir=root/'workcell_builder/workcell_builder/assets/imported'; dest_dir.mkdir(parents=True, exist_ok=True)
    dest=dest_dir/f'{aid}{src.suffix.lower()}'
    i=1
    while dest.exists(): dest=dest_dir/f'{aid}_{i}{src.suffix.lower()}'; i+=1
    shutil.copy2(src,dest)
    rel=dest.relative_to(root).as_posix()
    rec={"asset_id":aid,"label":a.label,"category":a.category,"asset_type":a.asset_type,"mesh_path":rel,"default_dimensions_m":[1,1,1],"default_pose":[0,0,0,0,0,0],"default_z_hint":0.0,"placement_notes":"Imported via CLI","license":"operator_provided","source_note":str(src),"imported_by_version":"workcell_builder_external_import_v1","tags":["imported"]}
    out=root/'workcell_builder/workcell_builder/config/asset_profiles/imported_environment_assets.json'
    arr=[]
    if out.exists():
        try: arr=json.loads(out.read_text())
        except Exception: arr=[]
    arr=[x for x in arr if x.get('asset_id')!=aid]; arr.append(rec)
    out.write_text(json.dumps(arr,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    if a.print_summary: print(json.dumps({"status":"ok","asset_id":aid,"path":rel},indent=2))
    return 0

if __name__=='__main__': raise SystemExit(main())
