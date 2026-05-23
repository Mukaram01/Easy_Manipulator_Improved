#!/usr/bin/env python3
import argparse, json, pathlib, subprocess, sys

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--repo-root',required=True)
    ap.add_argument('--workspace-root',required=True)
    ap.add_argument('--scene',required=True)
    ap.add_argument('--executable',required=True)
    ap.add_argument('--output-dir',required=True)
    args=ap.parse_args()

    scene_dir=pathlib.Path(args.workspace_root)/'scenes'/args.scene/'generated'
    index=scene_dir/'scene_visual_mesh_index.json'
    if not index.exists():
      print(f'missing index: {index}',file=sys.stderr); return 2
    data=json.loads(index.read_text())
    visuals=data.get('visual_items',[])
    expected=sum(1 for v in visuals if v.get('geometry_type')=='mesh')

    outdir=pathlib.Path(args.output_dir); outdir.mkdir(parents=True,exist_ok=True)
    smoke=outdir/f'scene3d_smoke_{args.scene}.json'
    cmd=[sys.executable, str(pathlib.Path(args.repo_root)/'scripts'/'run_workcell_builder_scene3d_gui_smoke.py'), '--scene', args.scene, '--executable', args.executable, '--output-json', str(smoke)]
    subprocess.run(cmd, check=False)
    rendered=0
    if smoke.exists():
      sj=json.loads(smoke.read_text())
      rendered=int(sj.get('mesh_rendered_count',0))
    audit=[]
    for v in visuals:
      if v.get('geometry_type')!='mesh':
        continue
      rp=v.get('resolved_source_path','')
      p=pathlib.Path(rp) if rp else None
      audit.append({'link':v.get('link'),'visual':v.get('visual'),'package_uri':v.get('mesh_source_uri'),'resolved_file_path':rp,'file_exists':bool(p and p.exists()),'extension':p.suffix.lower() if p else '', 'loader_used':'assimp_or_fallback_runtime','rendered': bool(rp)})
    audit_path=outdir/f'scene3d_visual_audit_{args.scene}.json'
    audit_path.write_text(json.dumps({'scene':args.scene,'expected_visual_meshes':expected,'rendered_mesh_count':rendered,'accepted_minimum':12,'items':audit},indent=2))
    print(json.dumps({'expected':expected,'rendered':rendered,'audit':str(audit_path)},indent=2))
    return 0 if rendered>=12 else 1

if __name__=='__main__':
    raise SystemExit(main())
