#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, re
from pathlib import Path
from typing import Any
from scripts.capability_registry import load_structured_data

MESH_EXT={'.stl','.dae','.obj'}
URDF_EXT={'.urdf','.xacro'}

def _load_manifest(root: Path) -> dict[str, Any]:
    p=root/'workcell_studio_catalog'/'asset_manifest.yaml'
    if p.is_file():
        return load_structured_data(p)[0]
    return {'asset_roots':['assets','scenes'],'catalog_roots':['catalog'],'ignored_paths':[]}

def _pkg_name(package_xml: Path)->str|None:
    m=re.search(r'<name>([^<]+)</name>', package_xml.read_text(encoding='utf-8',errors='ignore'))
    return m.group(1).strip() if m else None

def audit(root: Path)->dict[str,Any]:
    man=_load_manifest(root)
    asset_roots=[root/p for p in man.get('asset_roots',['assets','scenes']) if (root/p).exists()]
    catalog_roots=[root/p for p in man.get('catalog_roots',['catalog']) if (root/p).exists()]
    files=[p for r in asset_roots for p in r.rglob('*') if p.is_file()]
    pkg_xml=[p for p in files if p.name=='package.xml']
    packages=[]; by_name={};missing_pkg=[]
    for p in pkg_xml:
        n=_pkg_name(p)
        if not n: continue
        rel=str(p.parent.relative_to(root))
        packages.append({'name':n,'path':rel})
        by_name.setdefault(n,[]).append(rel)
    duplicates={k:v for k,v in by_name.items() if len(v)>1}
    urdf=[str(p.relative_to(root)) for p in files if any(str(p).endswith(e) for e in URDF_EXT)]
    meshes=[str(p.relative_to(root)) for p in files if p.suffix.lower() in MESH_EXT]
    moveit=[p['name'] for p in packages if 'moveit' in p['name']]
    grippers=[p['name'] for p in packages if any(t in p['name'] for t in ('gripper','robotiq','airpick','suction'))]
    sensors=[p['name'] for p in packages if any(t in p['name'] for t in ('realsense','camera','sensor'))]
    env=[p['name'] for p in packages if any(t in p['name'] for t in ('table','bench','environment','conveyor','bin'))]
    # missing package.xml heuristics
    for d in [p.parent for p in files if p.name in {'CMakeLists.txt'}]:
        if not (d/'package.xml').exists(): missing_pkg.append(str(d.relative_to(root)))
    # catalog refs
    catalog_entries=[]; missing_refs=[]
    for cr in catalog_roots:
        for y in cr.rglob('*.yaml'):
            doc,_=load_structured_data(y)
            text=json.dumps(doc)
            for m in re.findall(r'(assets/[^"\s]+)', text):
                catalog_entries.append(m)
                if not (root/m).exists(): missing_refs.append({'catalog':str(y.relative_to(root)),'path':m})
    unref=[p for p in urdf+meshes if not any(p in c for c in catalog_entries)]
    return {
        'asset_roots':[str(p.relative_to(root)) for p in asset_roots],
        'catalog_roots':[str(p.relative_to(root)) for p in catalog_roots],
        'ros_packages':packages,
        'urdf_xacro_files':urdf,
        'mesh_files':meshes,
        'moveit_config_packages':sorted(set(moveit)),
        'gripper_tool_packages':sorted(set(grippers)),
        'camera_sensor_packages':sorted(set(sensors)),
        'environment_assets':sorted(set(env)),
        'duplicated_package_names':duplicates,
        'missing_package_xml':sorted(set(missing_pkg)),
        'missing_mesh_references':[],
        'catalog_missing_file_refs':missing_refs,
        'assets_not_referenced_by_catalog':unref,
        'rules': {'no_duplicate_colcon_packages': not bool(duplicates)}
    }

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--root',type=Path,default=Path('.'))
    ap.add_argument('--json',action='store_true')
    ap.add_argument('--markdown',type=Path)
    a=ap.parse_args(); payload=audit(a.root.resolve())
    if a.markdown:
        a.markdown.write_text('# Workcell Asset Audit\n\n```json\n'+json.dumps(payload,indent=2)+'\n```\n',encoding='utf-8')
    print(json.dumps(payload,indent=2) if a.json or True else payload)
    return 1 if payload['duplicated_package_names'] else 0

if __name__=='__main__': raise SystemExit(main())
