#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / 'scenes'
OUT = ROOT / 'build/workcell_studio/visual_mesh_index_regeneration_report.json'
SUPPORTED_SCENES = ["ur5_2f_test","ur5_3f_test","ur10_2f_test","ur3_suction_test","ur5_airpick4_test","suction_test"]

def parse():
    p=argparse.ArgumentParser()
    g=p.add_mutually_exclusive_group(required=True); g.add_argument('--all',action='store_true'); g.add_argument('--scene')
    p.add_argument('--portable',action='store_true')
    p.add_argument('--prefer-xacro-expanded',action='store_true',default=True)
    p.add_argument('--fallback-best-effort',action='store_true',default=True)
    p.add_argument('--fail-on-unexpanded',action='store_true')
    p.add_argument('--xacro-arg',action='append',default=[])
    p.add_argument('--fail-on-unsafe',action='store_true')
    return p.parse_args()

def scene_list(a):
    if a.scene:
        return [SCENES/a.scene]
    return [SCENES/s for s in SUPPORTED_SCENES if (SCENES/s).exists()]

def summarize(scene):
    idx=scene/'generated/scene_visual_mesh_index.json'; data=json.loads(idx.read_text()) if idx.exists() else {}
    items=data.get('visual_items',[])
    mesh_backed=sum(1 for i in items if i.get('geometry_type')=='mesh')
    primitive=sum(1 for i in items if i.get('item_source')=='primitive_fallback' or i.get('geometry_type') in ('box','cylinder','sphere'))
    unresolved=data.get('unresolved_placeholder_count',0)
    safe=data.get('safe_for_preview',False)
    status='PASS' if safe else ('FAIL' if not items else 'WARN')
    stale_unsafe = int(bool(data.get('stale_index'))) + (1 if not safe else 0)
    return {'scene':scene.name,'extraction_mode':data.get('extraction_mode','unknown'),'xacro_available':data.get('xacro_available',False),'expanded_urdf_written':bool(data.get('source_expanded_urdf_path')),'safe_for_preview':safe,'fallback_reason':data.get('fallback_reason',''),'unresolved_placeholder_count':unresolved,'mesh_backed_count':mesh_backed,'skipped_count':sum(1 for i in items if i.get('render_skip_reason')),'primitive_fallback_count':primitive,'stale_index':data.get('stale_index',False),'status':status,'visual_item_count':len(items),'unresolved_count':unresolved,'stale_or_unsafe_count':stale_unsafe,'generated_index_path':str(idx.relative_to(ROOT))}

def main():
    a=parse(); rows=[]
    for s in scene_list(a):
        cmd=['python3',str(ROOT/'scripts/extract_scene_urdf_visual_mesh_index.py'),'--scene',s.name,'--prefer-xacro-expanded']
        for xa in a.xacro_arg: cmd += ['--xacro-arg', xa]
        if a.fail_on_unexpanded: cmd.append('--fail-on-unexpanded')
        subprocess.run(cmd,check=False)
        row=summarize(s); rows.append(row)
        print(f"{row['scene']}: visual_items={row['visual_item_count']} mesh={row['mesh_backed_count']} skipped={row['skipped_count']} safe={row['safe_for_preview']} fallback_reason={row['fallback_reason']}")
    payload={'schema':'workcell_studio_visual_mesh_index_regeneration/v2','scenes':rows,'summary':{'scene_count':len(rows)}}
    OUT.parent.mkdir(parents=True,exist_ok=True); OUT.write_text(json.dumps(payload,indent=2)+'\n'); print(OUT)
    if a.fail_on_unsafe and any(r['status']!='PASS' for r in rows): return 1
    if a.fail_on_unexpanded and any(r['extraction_mode']!='xacro_expanded' for r in rows): return 3
    return 0
if __name__=='__main__': raise SystemExit(main())
