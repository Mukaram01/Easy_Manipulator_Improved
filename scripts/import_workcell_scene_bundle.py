#!/usr/bin/env python3
from __future__ import annotations
import argparse, zipfile, json, shutil, subprocess
from pathlib import Path, PurePosixPath

def _unsafe(p:str)->bool:
    pp=PurePosixPath(p)
    return pp.is_absolute() or '..' in pp.parts or p.startswith('~')

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('--bundle',required=True); ap.add_argument('--target-scenes-dir',required=True); ap.add_argument('--scene-name'); ap.add_argument('--overwrite',action='store_true'); ap.add_argument('--validate',action='store_true'); ap.add_argument('--strict',action='store_true'); ap.add_argument('--dry-run',action='store_true'); ap.add_argument('--print-summary',action='store_true')
    a=ap.parse_args()
    b=Path(a.bundle); target_root=Path(a.target_scenes_dir); summary=[]
    with zipfile.ZipFile(b,'r') as zf:
        names=zf.namelist()
        if any(_unsafe(n) for n in names): raise SystemExit('unsafe zip path detected')
        mf=json.loads(zf.read('manifest.json').decode('utf-8'))
        if mf.get('bundle_format')!='workcell_bundle/v1': raise SystemExit('unsupported bundle format')
        scene_name=a.scene_name or mf.get('scene_name') or b.stem
        scene_dir=target_root/scene_name
        if scene_dir.exists() and not a.overwrite: raise SystemExit('target exists; use --overwrite')
        if not a.dry_run:
            if scene_dir.exists(): shutil.rmtree(scene_dir)
            scene_dir.mkdir(parents=True,exist_ok=True)
            for n in names:
                if n.endswith('/'): continue
                data=zf.read(n)
                out=scene_dir/n
                out.parent.mkdir(parents=True, exist_ok=True)
                out.write_bytes(data)
            env=scene_dir/'environment.yaml'
            if env.exists():
                t=env.read_text(encoding='utf-8')
                t=t.replace(' /',' ').replace('asset_stl: /','asset_stl: assets/imported/')
                env.write_text(t,encoding='utf-8')
        summary.append(f'Imported Scene Ready: {scene_dir}')
    if a.validate and not a.dry_run:
        subprocess.run(['python3','scripts/validate_workcell_scene.py','--scene-dir',str(scene_dir)], check=False)
    if a.print_summary:
        print('Portable Scene Bundle import summary:')
        for s in summary: print('-',s)
    return 0
if __name__=='__main__': raise SystemExit(main())
