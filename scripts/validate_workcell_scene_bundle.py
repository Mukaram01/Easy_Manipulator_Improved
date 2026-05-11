#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, zipfile, stat
from pathlib import PurePosixPath, Path
PASS='WORKCELL_SCENE_BUNDLE: PASS'; WARN='WORKCELL_SCENE_BUNDLE: WARN'; FAIL='WORKCELL_SCENE_BUNDLE: FAIL'


def _unsafe(p:str)->bool:
    pp=PurePosixPath(p)
    return pp.is_absolute() or '..' in pp.parts or p.startswith('~')

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('--bundle', required=True); ap.add_argument('--allow-executables', action='store_true')
    a=ap.parse_args(); warns=[]; blocks=[]
    b=Path(a.bundle)
    if not b.exists(): print(f'{FAIL}\nbundle missing: {b}'); return 1
    try:
        with zipfile.ZipFile(b,'r') as zf:
            names=zf.namelist()
            if 'manifest.json' not in names: blocks.append('manifest.json missing')
            unsafe=[n for n in names if _unsafe(n)]
            if unsafe: blocks.append(f'unsafe paths: {unsafe[:3]}')
            if 'environment.yaml' not in names: blocks.append('environment.yaml missing')
            manifest={}
            if 'manifest.json' in names:
                manifest=json.loads(zf.read('manifest.json').decode('utf-8'))
                if manifest.get('bundle_format')!='workcell_bundle/v1': blocks.append('bundle_format must be workcell_bundle/v1')
                if manifest.get('scene_schema_version')!='workcell_scene/v1': blocks.append('schema_version must be workcell_scene/v1')
                for p in manifest.get('file_manifest',[]):
                    if _unsafe(p): blocks.append(f'unsafe manifest path: {p}')
                    if p not in names: blocks.append(f'missing referenced file: {p}')
                for p in manifest.get('asset_manifest',[]):
                    rel=p.get('bundle_path','') if isinstance(p,dict) else ''
                    if rel and rel not in names: blocks.append(f'missing asset file: {rel}')
                sf=manifest.get('safety_flags',{})
                for k,v in [('fake_hardware_first',True),('real_hardware_enabled',False),('runtime_execution_enabled',False),('motion_command_sent',False)]:
                    if sf.get(k)!=v: blocks.append(f'safety flag {k} must be {v}')
            if any('license' in n.lower() and n.lower().endswith(('.exe','.sh','.bat')) for n in names): warns.append('suspicious license marker executable')
            for zi in zf.infolist():
                if zi.file_size>50*1024*1024: warns.append(f'oversized file: {zi.filename}')
                mode=(zi.external_attr>>16)&0o777
                if (mode & stat.S_IXUSR) and not a.allow_executables:
                    warns.append(f'executable file in bundle: {zi.filename}')
    except zipfile.BadZipFile:
        print(f'{FAIL}\ninvalid zip'); return 1
    if blocks: print(FAIL)
    elif warns: print(WARN)
    else: print(PASS)
    for m in blocks: print('BLOCKER:',m)
    for m in warns: print('WARNING:',m)
    return 1 if blocks else 0

if __name__=='__main__': raise SystemExit(main())
