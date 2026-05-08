#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from typing import Any
import yaml

def _root()->Path: return Path(__file__).resolve().parents[1]

def _map_group(category:str, aid:str)->str:
    c=category.lower(); i=aid.lower()
    if c=='robots': return 'Robots'
    if c in {'end_effectors','grippers','tools'}: return 'Grippers & Tools'
    if c in {'sensors','cameras'} or 'camera' in i: return 'Cameras & Sensors'
    if 'table' in i or 'bench' in i: return 'Tables & Workbenches'
    if 'bin' in i or 'tote' in i: return 'Bins & Totes'
    if 'conveyor' in i: return 'Conveyors'
    if c=='fixtures' or 'fixture' in i or 'jig' in i: return 'Fixtures & Jigs'
    if c=='machines' or 'machine' in i or 'cnc' in i or 'lathe' in i: return 'Machines'
    if c=='safety' or 'safety' in i: return 'Safety'
    if c=='objects': return 'Objects'
    return 'Preview-only Assets' if 'preview' in i else 'Custom STL'

PLACEHOLDERS=[('small_bin','Small Bin','Bins & Totes','assets/environment/bins/meshes/small_bin.stl'),('medium_bin','Medium Bin','Bins & Totes','assets/environment/bins/meshes/medium_bin.stl'),('large_bin','Large Bin','Bins & Totes','assets/environment/bins/meshes/large_bin.stl'),('conveyor_1m','Conveyor 1m','Conveyors','assets/environment/conveyors/meshes/conveyor_1m.stl'),('conveyor_2m','Conveyor 2m','Conveyors','assets/environment/conveyors/meshes/conveyor_2m.stl'),('simple_fixture_plate','Simple Fixture Plate','Fixtures & Jigs','assets/environment/fixtures/meshes/simple_fixture_plate.stl'),('cnc_machine_placeholder','CNC Machine Placeholder','Machines','assets/environment/machines/meshes/cnc_machine_placeholder.stl'),('safety_fence_panel','Safety Fence Panel','Safety','assets/environment/safety/meshes/safety_fence_panel.stl'),('cube_small','Cube Small','Objects','assets/objects/primitives/meshes/cube_small.stl'),('box_large','Box Large','Objects','assets/objects/primitives/meshes/box_large.stl'),('cylinder_small','Cylinder Small','Objects','assets/objects/primitives/meshes/cylinder_small.stl'),('overhead_camera_placeholder','Overhead Camera Placeholder','Cameras & Sensors','assets/environment/cameras/meshes/overhead_camera_placeholder.stl')]

def generate(root:Path|None=None)->dict[str,Any]:
    root=root or _root(); items=[]
    data=yaml.safe_load((root/'workcell_studio_catalog'/'catalog.yaml').read_text()) or {}
    for item in data.get('items',[]):
        aid=item.get('id','')
        g=_map_group(item.get('category',''),aid)
        items.append({'id':aid,'display_name':item.get('display_name',aid),'category':g,'group':g,'folder':item.get('family',''),'mesh_path':item.get('mesh_path') or item.get('preview_mesh_path') or '','support_status':item.get('support_status','supported'),'runtime_status':item.get('runtime_status','fake_hardware_ready'),'preview_only':item.get('runtime_status')=='preview_only','collision_mode_default':'mesh_collision','dimensions':item.get('dimensions',{}),'tags':item.get('tags',[]),'notes':item.get('notes','')})
    ids={x['id'] for x in items}
    for aid,name,cat,mesh in PLACEHOLDERS:
        if aid in ids: continue
        items.append({'id':aid,'display_name':name,'category':cat,'group':cat,'folder':'placeholders','mesh_path':mesh,'support_status':'supported','runtime_status':'preview_only' if 'placeholder' in aid else 'fake_hardware_ready','preview_only':'placeholder' in aid,'collision_mode_default':'bounding_box_collision','dimensions':{'x':0.2,'y':0.2,'z':0.1},'tags':['placeholder',cat.lower()],'notes':'Local generated placeholder mesh for GUI catalog.'})
    required=['Robots','Grippers & Tools','Cameras & Sensors','Tables & Workbenches','Bins & Totes','Conveyors','Fixtures & Jigs','Machines','Safety','Objects','Custom STL','Preview-only Robots','Preview-only Assets']
    categories=sorted(set(required).union({x['category'] for x in items}))
    return {'workflow_sections':['Start','Ingredients','Layout','Task','Perception ROI','Grasp','Validate & Generate','Status'],'asset_categories':categories,'recommended_starter_assets':['UR5','Robotiq 2F','Workbench/Table','Cube Small','Small Bin','RealSense D435i visual asset'],'assets':items}

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('--root',type=Path,default=_root()); ap.add_argument('--output',type=Path)
    a=ap.parse_args(); payload=generate(a.root.resolve()); out=a.output or (a.root/'workcell_studio_catalog'/'generated'/'workcell_builder_gui_catalog.json'); out.parent.mkdir(parents=True,exist_ok=True); out.write_text(json.dumps(payload,indent=2)+'\n'); print(out); return 0
if __name__=='__main__': raise SystemExit(main())
