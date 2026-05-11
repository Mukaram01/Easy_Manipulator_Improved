#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, zipfile, getpass, datetime, subprocess
from pathlib import Path

OPTIONAL=['config/perception_metadata.json','workcell_studio_summary.json','workcell_studio_summary.md','preview/workcell_preview.svg','preview/workcell_preview.html']
REQUIRED=['environment.yaml','config/task_recipe.yaml']

def _extract_meshes(env_text:str):
    out=[]
    for ln in env_text.splitlines():
        if 'mesh:' in ln or 'asset_stl:' in ln:
            v=ln.split(':',1)[1].strip().strip('"\'')
            if v: out.append(v)
    return sorted(set(out))

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('--scene-dir',required=True); ap.add_argument('--output',required=True); ap.add_argument('--validate',action='store_true'); ap.add_argument('--include-assets',action='store_true')
    a=ap.parse_args(); scene=Path(a.scene_dir); out=Path(a.output)
    if not scene.exists(): raise SystemExit('scene-dir missing')
    env=(scene/'environment.yaml').read_text(encoding='utf-8')
    files=[]
    for f in REQUIRED+OPTIONAL:
        p=scene/f
        if p.exists(): files.append(f)
    mesh_refs=_extract_meshes(env)
    asset_manifest=[]
    staged=[]
    for ref in mesh_refs:
        if ref.startswith('/'):
            asset_manifest.append({'path':ref,'status':'unresolved_external_asset'})
            if a.include_assets and Path(ref).exists():
                bn=f'assets/external/{Path(ref).name}'; staged.append((Path(ref),bn)); asset_manifest[-1]['bundle_path']=bn
            continue
        cand=(scene/ref)
        if cand.exists(): bn=f'meshes/{cand.name}'; staged.append((cand,bn)); asset_manifest.append({'path':ref,'bundle_path':bn,'kind':'scene_mesh'})
        elif Path(ref).exists() and a.include_assets:
            src=Path(ref); bn=f'assets/imported/{src.name}'; staged.append((src,bn)); asset_manifest.append({'path':ref,'bundle_path':bn,'kind':'external_relative'})
        else:
            asset_manifest.append({'path':ref,'status':'missing'})
    manifest={
      'bundle_format':'workcell_bundle/v1','scene_schema_version':'workcell_scene/v1','scene_name':scene.name,
      'package_name':scene.name,'exported_at':datetime.datetime.utcnow().isoformat()+'Z','exported_by':getpass.getuser(),
      'source_repo_hint':str(Path.cwd()),'asset_manifest':asset_manifest,
      'file_manifest':files+[b for _,b in staged],'safety_flags':{'fake_hardware_first':True,'real_hardware_enabled':False,'runtime_execution_enabled':False,'motion_command_sent':False},
      'validation_status':'pending','warnings':[],'blockers':[]}
    with zipfile.ZipFile(out,'w',compression=zipfile.ZIP_DEFLATED) as zf:
        for rel in files: zf.write(scene/rel, arcname=rel)
        for src,bn in staged: zf.write(src, arcname=bn)
        zf.writestr('manifest.json', json.dumps(manifest,indent=2))
    if a.validate:
        subprocess.run(['python3','scripts/validate_workcell_scene_bundle.py','--bundle',str(out)], check=False)
    print(f'Exported Scene Archive: {out}')
    return 0
if __name__=='__main__': raise SystemExit(main())
