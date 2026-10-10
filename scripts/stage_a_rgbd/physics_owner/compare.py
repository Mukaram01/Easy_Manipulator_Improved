#!/usr/bin/env python3
"""One bounded stock/owner comparison. No ROS, bridge, EPD or MoveIt launch."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def loaded_libraries(pid):
    paths=set()
    try:
        for line in Path(f'/proc/{pid}/maps').read_text().splitlines():
            path=line.split()[-1]
            if path.startswith('/') and any(s in path for s in ('physics','dart','libode','gazebo6')) and Path(path).is_file():paths.add(path)
        children=Path(f'/proc/{pid}/task/{pid}/children').read_text().split()
        for child in children:paths.update(loaded_libraries(int(child)))
    except (FileNotFoundError,ProcessLookupError):pass
    return paths


def compare_contact_traces(stock,instrumented,session,schedule):
    """Exact finite-sample comparison; absent fields never acquire a tolerance."""
    result={'decision':'BLOCKED_STEP_SCHEDULE','positions_counts_pairs_equal':False,
            'normal_depth_complete':False,'steps':[],'numeric_tolerance':0}
    if not schedule or schedule!=sorted(set(schedule)) or any(sorted(r.get('step',-1) for r in trace)!=schedule for trace in (stock,instrumented)):
        return result
    missing=False
    def blocked(reason):
        result['decision']=reason;return result
    def positions(record):
        out={};complete=True
        for c in record['diagnostic_contacts']:
            key=(c['collision1'],c['collision2'])
            if key in out or key[0]==key[1] or c['owner_collision_id']!=key[0]:raise ValueError('invalid directed pair')
            points=c['position_m'];normals=c['normal'];depths=c['depth_m']
            if not points:raise ValueError('empty contact pair')
            for v in points+normals:
                if len(v)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in v):raise ValueError('invalid vector')
            if any(type(x) not in (int,float) or not math.isfinite(x) for x in depths):raise ValueError('invalid depth')
            if len(normals) not in (0,len(points)) or len(depths) not in (0,len(points)):raise ValueError('partial contact fields')
            complete=complete and len(normals)==len(points) and len(depths)==len(points)
            out[key]=sorted(tuple(p) for p in points)
        return out,complete
    for a,b in zip(sorted(stock,key=lambda r:r['step']),sorted(instrumented,key=lambda r:r['step'])):
        step=a['step']
        fields=('run_id','world','frame_id','step','stamp_ns','dt_ns','support_collision_id','support_collision_name')
        if (a.get('complete') is not True or b.get('complete') is not True or
            a.get('run_id')!=session or any(a.get(k)!=b.get(k) for k in fields) or
            a.get('dt_ns',0)<=0 or a.get('contact_request_step')!=step or b.get('contact_request_step')!=step):
            return blocked('BLOCKED_STEP_IDENTITY')
        for record in (a,b):
            inventory=record.get('diagnostic_collision_ids',[]);requested=record.get('diagnostic_requested_ids',[])
            if not inventory or len(set(inventory))!=len(inventory) or len(set(requested))!=len(requested) or set(inventory)!=set(requested):
                return blocked('BLOCKED_CONTACT_REQUEST_INVENTORY')
            if not record.get('diagnostic_contacts'):return blocked('BLOCKED_MISSING_CONTACT_EVIDENCE')
        if set(a['diagnostic_collision_ids'])!=set(b['diagnostic_collision_ids']):
            return blocked('BLOCKED_COLLISION_INVENTORY_MISMATCH')
        try:pa,ca=positions(a);pb,cb=positions(b)
        except (KeyError,TypeError,ValueError):return blocked('BLOCKED_INCOMPLETE_CONTACT_FIELDS')
        support=a['support_collision_id'];cubes=[p['collision_id'] for p in a['pairs']]
        required={(x,support) for x in cubes}|{(support,x) for x in cubes}
        if not required<=pa.keys() or not required<=pb.keys():return blocked('BLOCKED_REQUIRED_PAIR_MISSING')
        if any(x not in a['diagnostic_collision_ids'] for pair in pa for x in pair):return blocked('BLOCKED_CONTACT_REQUEST_INVENTORY')
        if pa!=pb:return blocked('BLOCKED_CONTACT_DISCREPANCY')
        if sorted(a['pairs'],key=lambda p:p['collision_id'])!=sorted(b['pairs'],key=lambda p:p['collision_id']):
            return blocked('BLOCKED_CUBE_STATE_DISCREPANCY')
        if ca and cb:
            def full(record):
                return { (c['collision1'],c['collision2']):sorted((tuple(p),tuple(n),d) for p,n,d in zip(c['position_m'],c['normal'],c['depth_m'])) for c in record['diagnostic_contacts'] }
            if full(a)!=full(b):return blocked('BLOCKED_CONTACT_DISCREPANCY')
        missing=missing or not (ca and cb)
        result['steps'].append({'step':step,'stamp_ns':a['stamp_ns'],
            'directed_pair_count':len(pa),'unique_pair_count':len({tuple(sorted(k)) for k in pa}),
            'contact_point_count':sum(len(v) for v in pa.values())//2})
    result['positions_counts_pairs_equal']=True
    result['normal_depth_complete']=not missing
    result['decision']='BLOCKED_MISSING_NORMAL_OR_DEPTH' if missing else 'PASS_FINITE_CONTACT_COMPARISON'
    return result


def read_trace(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]

def run(args,kind,session):
    root=args.output/kind;root.mkdir()
    tree=ET.parse(args.world);world=tree.getroot().find('world')
    physics=[p for p in world.findall('plugin') if p.get('name') in ('ignition::gazebo::systems::Physics','gz::sim::systems::Physics')]
    if len(physics)!=1:raise ValueError('exactly one original Physics System required')
    if kind=='instrumented':
        physics[0].set('filename',str(args.owner.resolve()))
        physics[0].set('name','ignition::gazebo::systems::WorkcellOwnerPhysics')
        for key,value in {'owner_output':str(root.resolve()/'owner.jsonl'),'owner_session':session,'owner_record_steps':' '.join(map(str,args.sample_steps))}.items():ET.SubElement(physics[0],key).text=str(value)
    observer=ET.SubElement(world,'plugin',filename=str(args.observer.resolve()),name='WorkcellPhysicsContactMeasurement')
    for key,value in {'support_collision':'a0::pick_support::support_link::support_collision','workpieces':'part_00 part_01','run_id':session,'diagnostic_output':str(root.resolve()/'ecm.jsonl'),'diagnostic_steps':' '.join(map(str,args.sample_steps))}.items():ET.SubElement(observer,key).text=str(value)
    derived=root/'world.sdf';tree.write(derived,encoding='utf-8',xml_declaration=True)
    env=dict(os.environ,IGN_PARTITION='owner-'+session+'-'+kind)
    paths=set();started=time.monotonic()
    with (root/'gazebo.log').open('w') as log:
        server=subprocess.Popen(['ign','gazebo','-s','-r','--iterations',str(args.steps),str(derived),'-v','4'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while server.poll() is None and time.monotonic()-started<60:
                paths.update(loaded_libraries(server.pid));time.sleep(.1)
            if server.poll() is None:raise TimeoutError('bounded runtime exceeded 60 seconds')
            if server.returncode!=0:raise RuntimeError('Gazebo exit '+str(server.returncode))
        finally:
            if server.poll() is None:
                os.killpg(server.pid,signal.SIGINT)
                try:server.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(server.pid,signal.SIGKILL);server.wait(timeout=5)
    result={'exit':server.returncode,'wall_time_s':time.monotonic()-started,'world_sha256':sha(derived),'physics_xml':ET.tostring(world.find('physics'),encoding='unicode'),'gravity':world.findtext('gravity'),'loaded_libraries':{p:sha(p) for p in sorted(paths)}}
    (root/'run.json').write_text(json.dumps(result,indent=2)+'\n')
    if not (root/'ecm.jsonl').is_file():raise RuntimeError('exact-step ECM diagnostic missing')
    if kind=='instrumented' and not (root/'owner.jsonl').is_file():raise RuntimeError('exact-step owner readback missing')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('world','owner','observer','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--steps',type=int,default=2000)
    parser.add_argument('--sample-steps',type=int,nargs='+',default=[100,250,500,1000,1500,2000])
    args=parser.parse_args()
    if not 1<=args.steps<=10000:parser.error('steps must be in [1,10000]')
    if args.sample_steps!=sorted(set(args.sample_steps)) or not args.sample_steps or not 1<=args.sample_steps[0]<=args.sample_steps[-1]<=args.steps:parser.error('ordered unique sample steps inside runtime required')
    args.output.mkdir(exist_ok=False);session=str(uuid.uuid4())
    result={'session':session,'steps':args.steps,'original_world_sha256':sha(args.world),'execution_goals':0,'moveit_started':False,'instrumented_runtime_only':True}
    for kind in ('stock','instrumented'):result[kind]=run(args,kind,session)
    stock=read_trace(args.output/'stock/ecm.jsonl');instrumented=read_trace(args.output/'instrumented/ecm.jsonl')
    owners=read_trace(args.output/'instrumented/owner.jsonl')
    result['contact_comparison']=compare_contact_traces(stock,instrumented,session,args.sample_steps)
    result['owner_exact_step_complete']=(sorted(o.get('step',-1) for o in owners)==args.sample_steps and
        all(o.get('complete') is True and o.get('session')==session and o.get('dart_frames')==o.get('step') and
            any(o.get('stamp_ns')==r['stamp_ns'] and o.get('step')==r['step'] for r in stock) for o in owners))
    result['physics_configuration_equal']=all(result['stock'][k]==result['instrumented'][k] for k in ('physics_xml','gravity'))
    result['runtime_equivalence']=result['contact_comparison']['decision']
    if not result['owner_exact_step_complete']:result['runtime_equivalence']='BLOCKED_OWNER_READBACK_INCOMPLETE'
    if not result['physics_configuration_equal']:result['runtime_equivalence']='BLOCKED_PHYSICS_CONFIGURATION'
    result['comparison_scope']='one identical-world finite-step run per binary; no stock-backend geometry attestation or universal equivalence claim'
    (args.output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    # A finite comparison never grants physical-contact or planning authority.
    raise SystemExit(0 if result['runtime_equivalence']=='PASS_FINITE_CONTACT_COMPARISON' else 2)

if __name__=='__main__':main()
