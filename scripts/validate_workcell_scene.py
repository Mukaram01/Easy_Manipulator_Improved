#!/usr/bin/env python3
from __future__ import annotations
import argparse, math
from pathlib import Path

PASS='WORKCELL_SCENE_SCHEMA: PASS'; WARN='WORKCELL_SCENE_SCHEMA: WARN'; FAIL='WORKCELL_SCENE_SCHEMA: FAIL'


def _safe_name(v:str)->bool:
    return bool(v) and all(c.isalnum() or c in '_-' for c in v)

def _pose_ok(vals:str)->bool:
    nums=[x.strip() for x in vals.strip('[]').split(',') if x.strip()]
    if len(nums)!=6:return False
    try:return all(math.isfinite(float(n)) for n in nums)
    except: return False

def validate_text(t:str, strict:bool=False):
    warns=[]; blocks=[]
    for sec in ['scene:','robot:','tool:','compatibility:','placed_objects:','camera:','task:','safety:','metadata:']:
        if sec not in t: blocks.append(f'missing {sec}')
    if 'schema_version: workcell_scene/v1' not in t: blocks.append('schema_version mismatch')
    if 'UNKNOWN_COMPATIBILITY' in t: warns.append('unknown compatibility warns')
    if 'INCOMPATIBLE' in t: blocks.append('incompatible known pair blocks')
    for k,v in [('fake_hardware_first','true'),('motion_command_sent','false'),('runtime_execution_enabled','false'),('real_hardware_enabled','false')]:
        if f'{k}: {v}' not in t: blocks.append(f'safety flag {k} must be {v}')
    if 'pose:' in t and '[x, y, z, roll, pitch, yaw]' in t: warns.append('template pose detected')
    if 'external_stl_warning' in t: (blocks if strict else warns).append('external mesh path warning')
    return warns, blocks

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--scene-dir'); ap.add_argument('--scene-file'); ap.add_argument('--strict', action='store_true')
    a=ap.parse_args()
    scene_file=Path(a.scene_file) if a.scene_file else Path(a.scene_dir)/'environment.yaml'
    if not scene_file.exists(): print(f'{FAIL}\nscene file missing: {scene_file}'); return 1
    t=scene_file.read_text(encoding='utf-8')
    warns, blocks=validate_text(t,a.strict)
    if blocks: print(FAIL)
    elif warns: print(WARN)
    else: print(PASS)
    for b in blocks: print('BLOCKER:',b)
    for w in warns: print('WARNING:',w)
    return 1 if blocks else 0

if __name__=='__main__': raise SystemExit(main())
