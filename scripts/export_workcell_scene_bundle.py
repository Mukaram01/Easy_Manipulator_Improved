#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, zipfile, getpass, datetime, shutil
from pathlib import Path

CANDIDATE_FILES=[
 'environment.yaml','scene_manifest.yaml','cell_definition.yaml','environment_layout.yaml','task_recipe.yaml',
 'config/workcell_builder_task_intent.yaml','generated_launch_commands.md','workcell_studio_summary.md','workcell_studio_summary.json'
]

def copy_if_exists(scene:Path, bundle:Path, rel:str, exported:list[str]):
    src=scene/rel
    if src.exists():
        dst=bundle/rel
        dst.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(src,dst)
        exported.append(rel)

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--scene-dir',required=True)
    ap.add_argument('--output',required=True)
    ap.add_argument('--validate',action='store_true')
    ap.add_argument('--include-assets',action='store_true')
    a=ap.parse_args()
    scene=Path(a.scene_dir)
    if not scene.exists(): raise SystemExit('scene-dir missing')
    out_zip=Path(a.output)
    bundle_dir=out_zip.with_suffix('')
    bundle_dir.mkdir(parents=True,exist_ok=True)
    exported=[]
    for rel in CANDIDATE_FILES: copy_if_exists(scene,bundle_dir,rel,exported)
    for d in ('preview','validation','readiness','smoke'):
        src=scene/d
        if src.exists() and src.is_dir():
            dst=bundle_dir/d
            if dst.exists(): shutil.rmtree(dst)
            shutil.copytree(src,dst)
            exported.append(f'{d}/')
    manifest={
      'bundle_format':'workcell_studio_scene_bundle/v1',
      'generated_by':'Workcell Studio','source_scene_name':scene.name,
      'exported_at':datetime.datetime.utcnow().replace(microsecond=0).isoformat()+'Z',
      'exported_by':getpass.getuser(),
      'preview_only':True,'use_fake_hardware_default':True,'no_robot_motion':True,
      'file_manifest':sorted(exported)
    }
    (bundle_dir/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    with zipfile.ZipFile(out_zip,'w',compression=zipfile.ZIP_DEFLATED) as zf:
        for p in bundle_dir.rglob('*'):
            if p.is_file(): zf.write(p, arcname=str(p.relative_to(bundle_dir)))
    print(f'Exported Scene Bundle Folder: {bundle_dir}')
    print(f'Exported Scene Archive: {out_zip}')
    return 0
if __name__=='__main__': raise SystemExit(main())
