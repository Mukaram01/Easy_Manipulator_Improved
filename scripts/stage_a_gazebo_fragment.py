#!/usr/bin/env python3
"""One opt-in disposable identity experiment. No EPD/ROS/controller launch."""
import argparse
import copy
import gzip
import shutil
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


def validate_owner_qualification(pre,report,exit,loaded,digest):
    canonical='ignition::gazebo::v6::systems::WorkcellOwnerPhysics'
    name=report.get('filename')
    valid=(report.get('decision')=='PASS_SYSTEMLOADER_ONLY' and report.get('loaded') is True and
        report.get('instance_class')==canonical and report.get('registered_classes')==[canonical] and
        report.get('requested_class')=='ignition::gazebo::systems::WorkcellOwnerPhysics' and
        all(report.get(k) is True for k in ('system','configure','update')) and
        all(report.get(k) is False for k in ('pre_update','post_update')) and
        exit.get('returncode')==0 and pre.get('pre_run_sha256',{}).get(name)==digest and
        loaded.get(name)==digest)
    if not valid:raise ValueError('owner lacks matching SystemLoader and loaded-library qualification')


def owner_provenance(root,owner):
    folder=root/'evidence/stage_a2_geometry/physics_owner/same_fragment/owner_loader'
    files=[folder/name for name in ('pre_run.json','result.json.gz','exit.json','loaded_sha256.json')]
    pre=json.loads(files[0].read_text());report=json.loads(gzip.decompress(files[1].read_bytes()))
    validate_owner_qualification(pre,report,json.loads(files[2].read_text()),json.loads(files[3].read_text()),sha(owner))
    # Retained qualification source paths came from this repository; relocation is explicit.
    for name,digest in pre['pre_run_sha256'].items():
        if '/scripts/stage_a_rgbd/physics_owner/' in name:
            relative='scripts/stage_a_rgbd/physics_owner/'+name.split('/scripts/stage_a_rgbd/physics_owner/',1)[1]
            if sha(root/relative)!=digest:raise ValueError('qualified owner source changed: '+relative)
    historical=root/'evidence/stage_a2_geometry/physics_owner/same_fragment/identity_metadata/result.json'
    for name,digest in json.loads(historical.read_text())['source_sha256'].items():
        if '/physics_owner/' in name and sha(root/name)!=digest:raise ValueError('owner inventory/readback source changed')
    return {str(p):sha(p) for p in files+[historical]}


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
        if child.tag in ('model','plugin','include','actor','sensor','light'):world.remove(child)
    scenes=world.findall('scene')
    if len(scenes)>1:raise ValueError('multiple scene definitions')
    scene=scenes[0] if scenes else ET.SubElement(world,'scene')
    grids=scene.findall('grid')
    if len(grids)>1:raise ValueError('multiple grid settings')
    grid=grids[0] if grids else ET.SubElement(scene,'grid')
    grid.text='false'
    plugin=copy.deepcopy(physics[0]);plugin.set('filename',str(owner));plugin.set('name','ignition::gazebo::systems::WorkcellOwnerPhysics')
    for key,value in dict(owner_output=str(trace),owner_session=session,owner_record_steps='2').items():ET.SubElement(plugin,key).text=value
    world.append(plugin)
    for model in models:world.append(model)
    return root


def prepare(world,binary,owner,output):
    binary=Path(binary).resolve();owner=Path(owner).resolve();world=Path(world).resolve();output=Path(output).resolve()
    # Reuse the already built, tested owner; never silently substitute another ELF.
    root=Path(__file__).resolve().parents[1]
    qualification=owner_provenance(root,owner)
    if not shutil.which('gdb'):raise ValueError('bounded crash diagnostics require gdb')
    session=uuid.uuid4().hex
    derived=disposable_world(ET.parse(world).getroot(),owner,session,output/'owner.jsonl')
    output.mkdir() # exclusive: no existing output/capture may be overwritten
    target=output/'world.sdf';ET.ElementTree(derived).write(target,encoding='utf-8',xml_declaration=True)
    record=dict(session=session,output=str(output),binary=str(binary),owner=str(owner),world=str(target),
        input_world=str(world),input_world_sha256=sha(world),execution_goals=0,
        debugger=str(Path(shutil.which('gdb')).resolve()),debugger_sha256=sha(shutil.which('gdb')),
        owner_qualification_sha256=qualification,
        geometry_scope='unchanged authored separated cubes only; support/bin omitted; no contact claim',
        binary_sha256=sha(binary),owner_sha256=sha(owner),world_sha256=sha(target),
        source_sha256={name:sha(root/name) for name in ('scripts/stage_a_rgbd/gazebo_fragment_capture.cpp','scripts/stage_a_rgbd/fragment_mrt.hh','scripts/stage_a_rgbd/gl_context_witness.hh','scripts/stage_a_rgbd/gl_context_check.hh','scripts/stage_a_rgbd/material_contract.hh','scripts/stage_a_rgbd/material_witness.hh','scripts/stage_a_rgbd/inventory_compare.hh','scripts/stage_a_rgbd/inventory_ecm_diagnostics.hh','scripts/stage_a_rgbd/renderer_identity.hh','scripts/stage_a_rgbd/renderer_identity_check.hh','scripts/stage_a_gazebo_fragment.py')})
    (output/'preflight.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def validate_prepared_world(pre):
    world=ET.parse(pre['world']).getroot().find('world')
    scenes=world.findall('scene')
    if len(scenes)!=1 or len(scenes[0].findall('grid'))!=1 or scenes[0].findtext('grid')!='false':
        raise ValueError('disposable grid must be disabled')
    plugins=world.findall('plugin')
    if len(plugins)!=1 or len(list(world.iter('plugin')))!=1:raise ValueError('exactly one owner System required')
    p=plugins[0]
    if p.get('name')!='ignition::gazebo::systems::WorkcellOwnerPhysics' or p.get('filename')!=pre['owner'] or \
       p.findtext('owner_session')!=pre['session'] or p.findtext('owner_output')!=str(Path(pre['output'])/'owner.jsonl') or \
       p.findtext('owner_record_steps')!='2':raise ValueError('wrong owner configuration/session/trace')
    if len(world.findall('model'))!=2 or any(e.tag in ('sensor','joint','include','actor','light') for e in world.iter()):
        raise ValueError('unsupported disposable world/controller interface')
    for m in world.findall('model'):
        if len(m.findall('link'))!=1 or len(m.findall('link/visual'))!=1 or len(m.findall('link/collision'))!=1:
            raise ValueError('incomplete two-cube inventory')


def last_lifecycle(path):
    lines=Path(path).read_text(errors='replace').splitlines()
    return next((s for s in reversed(lines) if '[workcell-lifecycle]' in s),None)


def debugger_script(output):
    # Stop on a fatal signal, collect evidence, then quit without continuing it.
    # The inferior starts once. No signal handlers or suppression in the target.
    return """set pagination off
set confirm off
set disable-randomization off
handle SIGILL stop print pass
handle SIGSEGV stop print pass
python
import gdb,json
from pathlib import Path
out=Path(%r)
def exited(event):
    if not (out/'child_exit.json').exists():
        (out/'child_exit.json').write_text(json.dumps({'exit_code':getattr(event,'exit_code',None)}))
def stopped(event):
    if isinstance(event,gdb.SignalEvent):
        pid=gdb.selected_inferior().pid
        (out/'child_exit.json').write_text(json.dumps({'signal':event.stop_signal,'pid':pid,'continued_after_signal':False}))
        try:(out/'crash_maps.txt').write_text(Path('/proc/'+str(pid)+'/maps').read_text())
        except OSError:pass
        gdb.execute('thread apply all bt full')
        gdb.execute('info sharedlibrary')
        gdb.execute('info registers')
gdb.events.exited.connect(exited)
gdb.events.stop.connect(stopped)
end
run
quit
""" % str(output)


def run(output):
    output=Path(output).resolve();pre=json.loads((output/'preflight.json').read_text())
    if (output/'execution_claim.json').exists():raise FileExistsError('experiment already claimed')
    root=Path(__file__).resolve().parents[1]
    if owner_provenance(root,Path(pre['owner']))!=pre['owner_qualification_sha256']:raise ValueError('changed qualification')
    for name,digest in pre['source_sha256'].items():
        if sha(root/name)!=digest:raise ValueError('changed preflight source')
    validate_prepared_world(pre)
    for name in ('binary','owner','world','debugger'):
        if sha(pre[name])!=pre[name+'_sha256']:raise ValueError('changed preflight '+name)
    for suffix in ('capture.json','capture.json.rgb8','capture.json.ids.u32','owner.jsonl',
                   'stdout.log','stderr.log','debugger.gdb','child_exit.json','process_exit.json','result.json'):
        if (output/suffix).exists():raise ValueError('existing capture/owner output')
    with (output/'execution_claim.json').open('x') as claim:json.dump(dict(session=pre['session'],maximum_runs=1),claim)
    env=dict(os.environ,LIBGL_ALWAYS_SOFTWARE='1',IGN_PARTITION='mrt_'+pre['session'],GZ_PARTITION='mrt_'+pre['session'])
    debugger=output/'debugger.gdb'
    debugger.write_text(debugger_script(output))
    cmd=['timeout','60s',pre['debugger'],'--batch','--return-child-result','-x',str(debugger),'--args',
        pre['binary'],pre['world'],str(output/'owner.jsonl'),str(output/'capture.json'),pre['session']]
    with (output/'stdout.log').open('wb') as stdout,(output/'stderr.log').open('wb') as stderr:
        try:
            process=subprocess.run(cmd,env=env,stdout=stdout,stderr=stderr,timeout=65,check=False)
            returncode=process.returncode
        except subprocess.TimeoutExpired:returncode=124
    child=json.loads((output/'child_exit.json').read_text()) if (output/'child_exit.json').exists() else {}
    (output/'process_exit.json').write_text(json.dumps(dict(supervisor_exit_code=returncode,child=child),indent=2)+'\n')
    result=dict(exit_code=returncode,child_exit=child,command=cmd,executed_binary_sha256=pre['binary_sha256'],
        decision='BLOCKED',contact_authority=False,timing_authority='BLOCKED',execution_goals=0)
    try:
        report=json.loads((output/'capture.json').read_text())
        result['native_report']=report
        paths={p for p in report.get('loaded_library_paths',[]) if Path(p).is_file()}
        result['loaded_library_sha256']={p:sha(p) for p in sorted(paths)}
        if returncode!=0 or child.get('exit_code')!=0 or child.get('signal'):
            raise ValueError(report.get('failure_reason','native experiment failed'))
        if result['loaded_library_sha256'].get(pre['owner'])!=pre['owner_sha256']:
            raise ValueError('qualified owner ELF not witnessed loaded')
        if any('libignition-gazebo-physics-system.so' in p for p in paths):raise ValueError('stock Physics DSO also loaded')
        owners=[json.loads(line) for line in (output/'owner.jsonl').read_text().splitlines() if line.strip()]
        if len(owners)!=1 or owners[0].get('step')!=2 or owners[0].get('session')!=pre['session']:raise ValueError('incomplete owner schedule/session')
        owner=owners[0];checked=copy.deepcopy(report)
        checked['renderer']['scene_fingerprint']=identity_scene_fingerprint(owner)
        rgb=np.frombuffer((output/'capture.json.rgb8').read_bytes(),dtype=np.uint8).reshape(report['height'],report['width'],3)
        ids=np.frombuffer((output/'capture.json.ids.u32').read_bytes(),dtype='<u4').reshape(report['height'],report['width'])
        checked['rgb_sha256']=sha(output/'capture.json.rgb8');checked['id_sha256']=sha(output/'capture.json.ids.u32')
        result.update(validate_live_fragment_capture(owner,checked,rgb,ids))
        if not np.ptp(rgb):raise ValueError('blank RGB')
        witness=report.get('occlusion_witness',{})
        if report.get('occlusion_verified') is not True or witness.get('near_id')==witness.get('far_id') or \
           int(ids[witness['y'],witness['x']])!=witness.get('near_id'):
            raise ValueError('native occlusion witness missing/incorrect')
        result['decision']='PASS_TESTED_LIVE_GAZEBO_IDENTITY_ONLY'
        result['rgb_sha256']=checked['rgb_sha256'];result['id_sha256']=checked['id_sha256']
        result['pixel_counts']={str(int(k)):int(v) for k,v in zip(*np.unique(ids,return_counts=True))}
        (output/'capture.checked.json').write_text(json.dumps(checked,indent=2)+'\n')
    except (ValueError,KeyError,TypeError,OSError) as error:result['failure_reason']=str(error)
    mapped=set()
    for name in ('capture.json.maps','crash_maps.txt'):
        path=output/name
        if path.exists():
            for line in path.read_text().splitlines():
                pos=line.find('/')
                if pos>=0 and '.so' in line[pos:] and Path(line[pos:]).is_file():mapped.add(line[pos:])
    result['diagnostic_loaded_library_sha256']={p:sha(p) for p in sorted(mapped)}
    result['qualified_owner_mapped']=result['diagnostic_loaded_library_sha256'].get(pre['owner'])==pre['owner_sha256']
    if (output/'owner.jsonl').exists():result['owner_trace_sha256']=sha(output/'owner.jsonl')
    if child.get('signal'):result['failure_reason']='native fatal signal '+child['signal']
    result['last_lifecycle_marker']=last_lifecycle(output/'stderr.log')
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--world',type=Path);parser.add_argument('--binary',type=Path);parser.add_argument('--owner',type=Path)
    parser.add_argument('--run-prepared',action='store_true');args=parser.parse_args()
    if args.run_prepared:result=run(args.output);print(json.dumps({k:result.get(k) for k in ('decision','exit_code','pixel_counts','failure_reason')}));raise SystemExit(result['exit_code'] or (0 if result['decision'].startswith('PASS_') else 2))
    if not all((args.world,args.binary,args.owner)):parser.error('preflight requires world, binary, owner')
    print(json.dumps(prepare(args.world,args.binary,args.owner,args.output),indent=2))
