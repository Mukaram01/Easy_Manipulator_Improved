from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import re, shutil, yaml

SCENE_MARKERS=("environment.yaml","scene_manifest.yaml","package.xml","launch/demo.launch.py")

@dataclass
class SceneRecord:
    name:str
    path:Path
    status:str


def sanitize_asset_name(name:str)->str:
    base=re.sub(r'[^a-zA-Z0-9_]+','_',name.strip().lower()).strip('_')
    if not base:
        raise ValueError('invalid asset name')
    if base[0].isdigit():
        base=f"asset_{base}"
    return base


def classify_scene(scene_dir:Path)->str:
    env=(scene_dir/'environment.yaml').exists()
    launch=(scene_dir/'launch'/'demo.launch.py').exists()
    pkg=(scene_dir/'package.xml').exists() or (scene_dir/'scene_manifest.yaml').exists()
    if env and launch:
        return 'READY'
    if env and not launch:
        return 'YAML_ONLY'
    if (not env) and pkg:
        return 'NEEDS_REPAIR'
    if pkg:
        return 'SCAFFOLD_ONLY'
    return 'NEEDS_REPAIR'


def discover_scenes(scenes_root:Path)->list[SceneRecord]:
    out=[]
    if not scenes_root.exists():
        return out
    for d in sorted([p for p in scenes_root.iterdir() if p.is_dir()]):
        if any((d/m).exists() for m in SCENE_MARKERS):
            out.append(SceneRecord(d.name,d,classify_scene(d)))
    return out


def discover_assets(assets_root:Path)->dict:
    result={'Robots':[],'End Effectors':[],'Environment Objects':[],'assets_root':str(assets_root)}
    mapping=[('Robots','robots'),('End Effectors','end_effectors'),('Environment Objects','environment'),('Environment Objects','environment_objects')]
    for cat,folder in mapping:
        root=assets_root/folder
        if not root.exists():
            continue
        for d in sorted([p for p in root.iterdir() if p.is_dir()]):
            result[cat].append({'package':d.name,'path':str(d),'has_package_xml':(d/'package.xml').exists(),'has_urdf_xacro':bool(list((d/'urdf').glob('*.xacro'))) if (d/'urdf').exists() else False,'has_meshes':(d/'meshes').exists()})
    return result


def import_stl_as_environment_asset(stl:Path, assets_root:Path, proposed_name:str|None=None)->Path:
    if not stl.exists() or stl.suffix.lower()!='.stl':
        raise ValueError('STL import failed')
    asset=sanitize_asset_name(proposed_name or stl.stem)
    base=assets_root/'environment'/f'{asset}_description'
    target=base
    idx=2
    while target.exists():
        target=assets_root/'environment'/f'{asset}_{idx}_description'; idx+=1
    (target/'meshes'/'visual').mkdir(parents=True,exist_ok=True)
    (target/'meshes'/'collision').mkdir(parents=True,exist_ok=True)
    (target/'urdf').mkdir(parents=True,exist_ok=True)
    shutil.copy2(stl,target/'meshes'/'visual'/f'{asset}.stl')
    shutil.copy2(stl,target/'meshes'/'collision'/f'{asset}.stl')
    (target/'package.xml').write_text(f'<package format="3"><name>{target.name}</name></package>\n',encoding='utf-8')
    (target/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.8)\nproject(env_asset)\n',encoding='utf-8')
    (target/'urdf'/f'{asset}.urdf.xacro').write_text(f'<robot name="{asset}"></robot>\n',encoding='utf-8')
    return target


def add_environment_asset_to_scene(scene_dir:Path, asset_pkg:str, *, name:str, xyz:list[float], rpy:list[float], scale:list[float], collision_enabled:bool=True)->Path:
    env_file=scene_dir/'environment.yaml'
    data={}
    if env_file.exists():
        data=yaml.safe_load(env_file.read_text(encoding='utf-8')) or {}
    objects=data.setdefault('environment_objects',[])
    objects.append({'name':name,'package':asset_pkg,'xyz':xyz,'rpy':rpy,'scale':scale,'collision_enabled':collision_enabled})
    env_file.write_text(yaml.safe_dump(data,sort_keys=False),encoding='utf-8')
    return env_file
