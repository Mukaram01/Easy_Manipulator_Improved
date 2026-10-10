#!/usr/bin/env python3
"""One opt-in disposable identity experiment. No EPD/ROS/controller launch."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import uuid
import xml.etree.ElementTree as ET
import numpy as np
from stage_a_fragment_identity import identity_scene_fingerprint,validate_live_fragment_capture


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def disposable_world(source,owner,session,trace):
    root=copy.deepcopy(source);world=root.find('world')
    if world is None:raise ValueError('missing world')
    physics=[p for p in world.findall('plugin') if p.get('name')=='ignition::gazebo::systems::Physics']
    if len(physics)!=1:raise ValueError('exactly one original Physics System required')
    models=[]
    # Names select the explicitly requested authored fixture, NEVER runtime identity.
    for name in ('part_00','part_01'):
        matches=[m for m in world.findall('model') if m.get('name')==name]
        if len(matches)!=1:raise ValueError('missing/duplicate authored fixture model')
        model=matches[0];links=model.findall('link')
        if model.findall('model') or len(links)!=1:raise ValueError('unsupported nested/multiple links')
        link=links[0]
        for kind in ('visual','collision'):
            rows=link.findall(kind)
            if len(rows)!=1:raise ValueError('ambiguous visual/collision ownership')
            size=rows[0].findtext('geometry/box/size','').split()
            if len(size)!=3 or not all(math.isfinite(float(x)) and float(x)>0 for x in size):raise ValueError('unsupported BOX geometry')
        visual=link.find('visual');material=visual.find('material')
        if float(visual.findtext('transparency','0'))!=0 or material is None or material.find('pbr') is not None or material.find('script') is not None:raise ValueError('unsupported material')
        for colour in ('ambient','diffuse'):
            values=material.findtext(colour,'0 0 0 1').split()
            if len(values)!=4 or not all(math.isfinite(float(x)) for x in values) or float(values[3])!=1:raise ValueError('unsupported material alpha')
        models.append(copy.deepcopy(model))
    # No support/contact claim: two original cubes, without bin/support/sensors.
    for child in list(world):
        if child.tag in ('model','plugin','include','actor','sensor'):world.remove(child)
    plugin=copy.deepcopy(physics[0]);plugin.set('filename',str(owner));plugin.set('name','ignition::gazebo::systems::WorkcellOwnerPhysics')
    for key,value in dict(owner_output=str(trace),owner_session=session,owner_record_steps='2').items():ET.SubElement(plugin,key).text=value
    world.append(plugin)
    for model in models:world.append(model)
    return root


def prepare(world,binary,owner,output):
    binary=Path(binary).resolve();owner=Path(owner).resolve();world=Path(world).resolve();output=Path(output).resolve()
    # Reuse the already built, tested owner; never silently substitute another ELF.
    root=Path(__file__).resolve().parents[1]
    qualified=json.loads((root/'evidence/stage_a2_geometry/physics_owner/same_fragment/identity_metadata/result.json').read_text())
    if qualified['built_binary_sha256'].get(str(owner))!=sha(owner):raise ValueError('owner ELF differs from tested build')
    for name,digest in qualified['source_sha256'].items():
        if '/physics_owner/' in name and sha(root/name)!=digest:raise ValueError('owner source differs from tested build')
    session=uuid.uuid4().hex
    derived=disposable_world(ET.parse(world).getroot(),owner,session,output/'owner.jsonl')
    output.mkdir() # exclusive: no existing output/capture may be overwritten
    target=output/'world.sdf';ET.ElementTree(derived).write(target,encoding='utf-8',xml_declaration=True)
    record=dict(session=session,output=str(output),binary=str(binary),owner=str(owner),world=str(target),
        input_world=str(world),input_world_sha256=sha(world),execution_goals=0,
        geometry_scope='unchanged authored separated cubes only; support/bin omitted; no contact claim',
        binary_sha256=sha(binary),owner_sha256=sha(owner),world_sha256=sha(target),
        source_sha256={name:sha(root/name) for name in ('scripts/stage_a_rgbd/gazebo_fragment_capture.cpp','scripts/stage_a_rgbd/fragment_mrt.hh')})
    (output/'preflight.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def run(output):
    output=Path(output).resolve();pre=json.loads((output/'preflight.json').read_text())
    for name in ('binary','owner','world'):
        if sha(pre[name])!=pre[name+'_sha256']:raise ValueError('changed preflight '+name)
    for suffix in ('capture.json','capture.json.rgb8','capture.json.ids.u32','owner.jsonl'):
        if (output/suffix).exists():raise ValueError('existing capture/owner output')
    with (output/'execution_claim.json').open('x') as claim:json.dump(dict(session=pre['session'],maximum_runs=1),claim)
    env=dict(os.environ,LIBGL_ALWAYS_SOFTWARE='1',IGN_PARTITION='mrt_'+pre['session'],GZ_PARTITION='mrt_'+pre['session'])
    cmd=['timeout','60s',pre['binary'],pre['world'],str(output/'owner.jsonl'),str(output/'capture.json'),pre['session']]
    with (output/'stdout.log').open('wb') as stdout,(output/'stderr.log').open('wb') as stderr:
        process=subprocess.run(cmd,env=env,stdout=stdout,stderr=stderr,timeout=70,check=False)
    result=dict(exit_code=process.returncode,command=cmd,executed_binary_sha256=pre['binary_sha256'],
        decision='BLOCKED',contact_authority=False,timing_authority='BLOCKED',execution_goals=0)
    try:
        report=json.loads((output/'capture.json').read_text())
        result['native_report']=report
        paths={p for p in report.get('loaded_library_paths',[]) if Path(p).is_file()}
        result['loaded_library_sha256']={p:sha(p) for p in sorted(paths)}
        if process.returncode!=0:raise ValueError(report.get('failure_reason','native experiment failed'))
        owners=[json.loads(line) for line in (output/'owner.jsonl').read_text().splitlines() if line.strip()]
        if len(owners)!=1 or owners[0].get('step')!=2 or owners[0].get('session')!=pre['session']:raise ValueError('incomplete owner schedule/session')
        owner=owners[0];checked=copy.deepcopy(report)
        checked['renderer']['scene_fingerprint']=identity_scene_fingerprint(owner)
        rgb=np.frombuffer((output/'capture.json.rgb8').read_bytes(),dtype=np.uint8).reshape(report['height'],report['width'],3)
        ids=np.frombuffer((output/'capture.json.ids.u32').read_bytes(),dtype='<u4').reshape(report['height'],report['width'])
        checked['rgb_sha256']=sha(output/'capture.json.rgb8');checked['id_sha256']=sha(output/'capture.json.ids.u32')
        result.update(validate_live_fragment_capture(owner,checked,rgb,ids))
        if not np.ptp(rgb):raise ValueError('blank RGB')
        result['decision']='PASS_TESTED_LIVE_GAZEBO_IDENTITY_ONLY'
        result['rgb_sha256']=checked['rgb_sha256'];result['id_sha256']=checked['id_sha256']
        result['pixel_counts']={str(int(k)):int(v) for k,v in zip(*np.unique(ids,return_counts=True))}
        (output/'capture.checked.json').write_text(json.dumps(checked,indent=2)+'\n')
    except (ValueError,KeyError,TypeError,OSError) as error:result['failure_reason']=str(error)
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--world',type=Path);parser.add_argument('--binary',type=Path);parser.add_argument('--owner',type=Path)
    parser.add_argument('--run-prepared',action='store_true');args=parser.parse_args()
    if args.run_prepared:result=run(args.output);print(json.dumps({k:result.get(k) for k in ('decision','exit_code','pixel_counts','failure_reason')}));raise SystemExit(result['exit_code'] or (0 if result['decision'].startswith('PASS_') else 2))
    if not all((args.world,args.binary,args.owner)):parser.error('preflight requires world, binary, owner')
    print(json.dumps(prepare(args.world,args.binary,args.owner,args.output),indent=2))
