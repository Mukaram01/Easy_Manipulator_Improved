#!/usr/bin/env python3
"""Import one mesh into this PC's imported-asset library (Git-ignored), as Workcell Builder's Import Mesh does."""
from __future__ import annotations
import argparse, hashlib, json, os, shutil, tempfile
from datetime import datetime, timezone
from pathlib import Path
from scripts.validate_workcell_asset_catalog import SAFE, IMPORTED_LIBRARY, IMPORTED_CATALOG, load_imported_catalog

VALID_EXT={'.stl','.obj','.dae'}

def sanitize(name:str)->str:
    return '_'.join(filter(None,[''.join(c.lower() if c.isalnum() else '_' for c in name).strip('_')])).replace('__','_')

def sha256(path:Path)->str:
    digest=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1<<20), b''): digest.update(block)
    return digest.hexdigest()

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--source',required=True)
    ap.add_argument('--asset-id',required=True)
    ap.add_argument('--label',required=True)
    ap.add_argument('--category',default='Custom / Imported')
    ap.add_argument('--asset-type',required=True)
    ap.add_argument('--license',default='not specified')
    ap.add_argument('--unit-scale-to-m',type=float,default=1.0)
    ap.add_argument('--repo-root',default='.')
    ap.add_argument('--print-summary',action='store_true')
    a=ap.parse_args()
    root=Path(a.repo_root).resolve(); src=Path(a.source)
    if src.is_symlink(): raise SystemExit('symlink not allowed')
    if not src.is_file(): raise SystemExit('source missing')
    if src.suffix.lower() not in VALID_EXT: raise SystemExit('unsupported file type')
    aid=sanitize(a.asset_id)
    if not SAFE.match(aid): raise SystemExit('unsafe asset id')
    dest_dir=root/IMPORTED_LIBRARY; dest_dir.mkdir(parents=True, exist_ok=True)
    if dest_dir.resolve()!=root/IMPORTED_LIBRARY: raise SystemExit(f'{IMPORTED_LIBRARY} must be a real folder inside the repository')
    try: _,arr=load_imported_catalog(root)
    except ValueError as e: raise SystemExit(f'imported-asset catalog is not valid JSON ({e}); nothing was imported')
    digest=sha256(src)
    same=[x for x in arr if x.get('source_sha256')==digest and x.get('unit_scale_to_m',1.0)==a.unit_scale_to_m
          and (root/str(x.get('mesh_path',''))).is_file()]
    if same:
        if a.print_summary: print(json.dumps({"status":"reused","asset_id":same[0]['asset_id'],"path":same[0]['mesh_path']},indent=2))
        return 0
    # Never overwrite an asset: a taken ID or file name gets the next free suffix.
    ids={x.get('asset_id') for x in arr}; ext=src.suffix.lower(); base=aid; n=2
    while base in ids or (dest_dir/f'{base}{ext}').exists(): base=f'{aid}_{n}'; n+=1
    dest=dest_dir/f'{base}{ext}'
    partial=dest_dir/f'.{base}{ext}.importing'
    shutil.copy2(src,partial)
    try:
        if sha256(partial)!=digest: raise SystemExit('source changed while it was copied')
        os.link(partial,dest)  # fails instead of replacing a file that appeared meanwhile
    finally:
        partial.unlink()
    rel=dest.relative_to(root).as_posix()
    arr.append({"asset_id":base,"label":a.label,"category":a.category,"asset_type":a.asset_type,"mesh_path":rel,
                "visual_type":ext[1:],"unit_scale_to_m":a.unit_scale_to_m,"default_dimensions_m":[1,1,1],
                "default_pose":[0,0,0,0,0,0],"default_z_hint":0.0,"placement_notes":"Imported via CLI",
                "license":a.license,"source_note":f"Imported via CLI from {src.name}","original_filename":src.name,
                "source_sha256":digest,"imported_at_utc":datetime.now(timezone.utc).isoformat(),
                "imported_by_version":"workcell_studio_mesh_import/v3","tags":["imported"]})
    fd,tmp=tempfile.mkstemp(dir=dest_dir,prefix='.catalog.',suffix='.json')
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as out: out.write(json.dumps(arr,indent=2,sort_keys=True)+'\n')
        os.replace(tmp,root/IMPORTED_CATALOG)
    except OSError:
        Path(tmp).unlink(missing_ok=True); dest.unlink()
        raise SystemExit('writing the imported-asset catalog failed; the copied file was removed')
    if a.print_summary: print(json.dumps({"status":"ok","asset_id":base,"path":rel},indent=2))
    return 0

if __name__=='__main__': raise SystemExit(main())
