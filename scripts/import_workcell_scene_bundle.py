#!/usr/bin/env python3
from __future__ import annotations
import argparse, zipfile, json, shutil, tempfile
from pathlib import Path, PurePosixPath

META_FILES={'environment.yaml','scene_manifest.yaml','cell_definition.yaml','environment_layout.yaml','task_recipe.yaml'}

def _unsafe(p:str)->bool:
    pp=PurePosixPath(p)
    return pp.is_absolute() or '..' in pp.parts or p.startswith('~')

def _next_name(root:Path, base:str)->str:
    candidate=f"{base}_imported"
    i=1
    out=root/candidate
    while out.exists():
        i+=1
        out=root/f"{base}_imported_{i}"
    return out.name

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--bundle',required=True)
    ap.add_argument('--target-scenes-dir',required=True)
    ap.add_argument('--scene-name')
    ap.add_argument('--validate',action='store_true')
    ap.add_argument('--print-summary',action='store_true')
    a=ap.parse_args()
    b=Path(a.bundle); target_root=Path(a.target_scenes_dir); target_root.mkdir(parents=True,exist_ok=True)
    staging=None
    if b.is_dir():
        staging=b
    else:
        td=Path(tempfile.mkdtemp(prefix='scene_bundle_'))
        with zipfile.ZipFile(b,'r') as zf:
            names=zf.namelist()
            if any(_unsafe(n) for n in names): raise SystemExit('unsafe zip path detected')
            zf.extractall(td)
        staging=td
    mf_path=staging/'manifest.json'
    if not mf_path.exists(): raise SystemExit('bundle missing manifest.json')
    mf=json.loads(mf_path.read_text(encoding='utf-8'))
    if 'bundle_format' not in mf: raise SystemExit('manifest missing bundle_format')
    has_meta=any((staging/f).exists() for f in META_FILES)
    if not has_meta: raise SystemExit('bundle missing required scene metadata files')
    scene_name=a.scene_name or mf.get('source_scene_name') or b.stem
    final_name=scene_name if not (target_root/scene_name).exists() else _next_name(target_root,scene_name)
    scene_dir=target_root/final_name
    shutil.copytree(staging, scene_dir)
    if a.print_summary:
        print('Portable Scene Bundle import summary:')
        print('-', f'Imported Scene Ready: {scene_dir}')
    return 0
if __name__=='__main__': raise SystemExit(main())
