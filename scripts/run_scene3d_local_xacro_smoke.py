#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, shutil, subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

def run(cmd):
    p=subprocess.run(cmd,capture_output=True,text=True)
    if p.stdout: print(p.stdout.strip())
    if p.returncode!=0 and p.stderr: print(p.stderr.strip())
    return p

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--scene'); ap.add_argument('--require-xacro',action='store_true'); a=ap.parse_args()
    x=shutil.which('xacro') is not None
    if a.require_xacro and not x:
        print('xacro executable unavailable (required)'); return 2
    cmd=['python3',str(ROOT/'scripts'/'extract_scene_urdf_visual_mesh_index.py'),'--prefer-xacro']
    cmd += ['--scene',a.scene] if a.scene else ['--all']
    if a.require_xacro: cmd.append('--require-xacro')
    p=run(cmd)
    if p.returncode!=0: return p.returncode
    p=run(['python3',str(ROOT/'scripts'/'validate_scene3d_visual_diagnostics.py')])
    if p.returncode!=0: return p.returncode
    rpt=json.loads((ROOT/'build'/'workcell_studio_urdf_visual_mesh_index_report.json').read_text())
    print(f"scene count: {rpt.get('scene_count',0)}")
    print(f"xacro-expanded count: {rpt.get('xacro_expanded_count',0)}")
    print(f"safe_for_preview count: {rpt.get('safe_for_preview_count',0)}")
    print(f"unresolved placeholder count: {rpt.get('unresolved_placeholder_count',0)}")
    print(f"identical-position warning count: {rpt.get('identical_position_warning_count',0)}")
    print(f"emitted visual count: {rpt.get('emitted_visual_count',0)}")
    return 0
if __name__=='__main__': raise SystemExit(main())
