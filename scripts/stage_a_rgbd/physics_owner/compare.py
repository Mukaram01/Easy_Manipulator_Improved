#!/usr/bin/env python3
"""One bounded stock/reference/owner comparison. No ROS, bridge, EPD or MoveIt launch."""
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


def compare_reference_owner(reference,owners,session,schedule):
    result={'decision':'BLOCKED_OWNER_STEP','numeric_tolerance':0,'steps':[]}
    if sorted(o.get('step',-1) for o in owners)!=schedule:return result
    def fields(record):
        out={}
        for c in record['diagnostic_contacts']:
            points,normals,depths=c['position_m'],c['normal'],c['depth_m']
            if not points or len(points)!=len(normals) or len(points)!=len(depths):raise ValueError('missing full fields')
            key=(c['collision1'],c['collision2'])
            if key in out:raise ValueError('duplicate pair')
            out[key]=sorted((tuple(p),tuple(n),d) for p,n,d in zip(points,normals,depths))
        return out
    if compare_contact_traces(reference,reference,session,schedule)['decision']!='PASS_FINITE_CONTACT_COMPARISON':
        result['decision']='BLOCKED_REFERENCE_FIELDS';return result
    for r,o in zip(sorted(reference,key=lambda r:r['step']),sorted(owners,key=lambda o:o['step'])):
        if (o.get('complete') is not True or o.get('session')!=session or o.get('world')!=r['world'] or
            o.get('frame_id')!='world' or any(o.get(k)!=r.get(k) for k in ('step','stamp_ns','dt_ns')) or o.get('dart_frames')!=r['step']):return result
        shapes=o.get('shapes',[]);ids=[p.get('collision_id') for p in shapes]
        physics=[p.get('physics_shape_id') for p in shapes];pointers=[p.get('shape_node_identity') for p in shapes]
        if (len(ids)!=len(set(ids)) or set(ids)!=set(r['diagnostic_collision_ids']) or
            len(physics)!=len(set(physics)) or any(type(i) is not int or i<=0 for i in physics) or
            len(pointers)!=len(set(pointers)) or any(not p for p in pointers) or
            {p['collision_id'] for p in shapes if p.get('mobile')}!={p['collision_id'] for p in r['pairs']}):
            result['decision']='BLOCKED_OWNER_INVENTORY';return result
        actual={}
        try:
            for c in o['contacts']:
                a,b=c['collision1'],c['collision2'];p,n,d=c['position_m'],c['normal'],c['depth_m']
                if a==b or a not in ids or b not in ids or len(p)!=3 or len(n)!=3:raise ValueError('invalid contact mapping')
                if any(type(v) not in (int,float) or not math.isfinite(v) for v in p+n+[d]):raise ValueError('nonfinite')
                actual.setdefault((a,b),[]).append((tuple(p),tuple(n),d))
                actual.setdefault((b,a),[]).append((tuple(p),tuple(-v for v in n),d))
            actual={k:sorted(v) for k,v in actual.items()}
            if not actual or actual!=fields(r):raise ValueError('reference owner mismatch')
        except (ValueError,KeyError,TypeError):
            result['decision']='BLOCKED_REFERENCE_OWNER_CONTACT_FIELDS';return result
        result['steps'].append({'step':r['step'],'contact_count':len(o['contacts'])})
    result['decision']='PASS_FINITE_REFERENCE_OWNER';return result

def read_trace(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]

def run(args,kind,session):
    root=args.output/kind;root.mkdir()
    tree=ET.parse(args.world);world=tree.getroot().find('world')
    physics=[p for p in world.findall('plugin') if p.get('name') in ('ignition::gazebo::systems::Physics','gz::sim::systems::Physics')]
    if len(physics)!=1:raise ValueError('exactly one original Physics System required')
    if kind=='reference':
        physics[0].set('filename',str(args.reference.resolve()))
        physics[0].set('name','ignition::gazebo::systems::WorkcellReferencePhysics')
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
    for name in ('world','reference','owner','observer','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--steps',type=int,default=2000)
    parser.add_argument('--sample-steps',type=int,nargs='+',default=[100,250,500,1000,1500,2000])
    args=parser.parse_args()
    if not 1<=args.steps<=10000:parser.error('steps must be in [1,10000]')
    if args.sample_steps!=sorted(set(args.sample_steps)) or not args.sample_steps or not 1<=args.sample_steps[0]<=args.sample_steps[-1]<=args.steps:parser.error('ordered unique sample steps inside runtime required')
    args.output.mkdir(exist_ok=False);session=str(uuid.uuid4())
    result={'session':session,'steps':args.steps,'original_world_sha256':sha(args.world),'execution_goals':0,'moveit_started':False,'instrumented_runtime_only':True}
    for kind in ('stock','reference','instrumented'):result[kind]=run(args,kind,session)
    stock=read_trace(args.output/'stock/ecm.jsonl');instrumented=read_trace(args.output/'instrumented/ecm.jsonl')
    reference=read_trace(args.output/'reference/ecm.jsonl')
    owners=read_trace(args.output/'instrumented/owner.jsonl')
    result['stock_owner_observable_fields']=compare_contact_traces(stock,instrumented,session,args.sample_steps)
    result['stock_reference_observable_fields']=compare_contact_traces(stock,reference,session,args.sample_steps)
    result['reference_owner_full_fields']=compare_reference_owner(reference,owners,session,args.sample_steps)
    result['physics_configuration_equal']=all(result['stock'][k]==result[kind][k] for kind in ('reference','instrumented') for k in ('physics_xml','gravity'))
    backends=[{Path(p).name:h for p,h in result[kind]['loaded_libraries'].items() if '/engine-plugins/' in p} for kind in ('stock','reference','instrumented')]
    result['backend_libraries_equal']=bool(backends[0]) and backends[0]==backends[1]==backends[2]
    result['single_intended_physics_owner']=all(str(path.resolve()) in result[kind]['loaded_libraries'] and
        not any('gazebo6-physics-system' in p for p in result[kind]['loaded_libraries'])
        for kind,path in (('reference',args.reference),('instrumented',args.owner)))
    result['runtime_equivalence']='PASS_FINITE_THREE_WAY'
    if not all(result[k]['positions_counts_pairs_equal'] for k in ('stock_owner_observable_fields','stock_reference_observable_fields')):
        result['runtime_equivalence']='BLOCKED_STOCK_OBSERVABLE_FIELDS'
    if result['reference_owner_full_fields']['decision']!='PASS_FINITE_REFERENCE_OWNER':result['runtime_equivalence']='BLOCKED_REFERENCE_OWNER'
    if not all(result[k] for k in ('physics_configuration_equal','backend_libraries_equal','single_intended_physics_owner')):
        result['runtime_equivalence']='BLOCKED_RUNTIME_PROVENANCE'
    result['comparison_scope']='one identical-world finite-step run per binary; no stock-backend geometry attestation or universal equivalence claim'
    if result['runtime_equivalence']=='PASS_FINITE_THREE_WAY':
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
        from stage_a_physics_contact import enclose_owner_geometry
        try:
            result['geometry']=[enclose_owner_geometry(o,r,session,r['step'])
                for o,r in zip(sorted(owners,key=lambda r:r['step']),sorted(instrumented,key=lambda r:r['step']))]
        except (ValueError,KeyError,TypeError,OverflowError) as exc:
            result['geometry']={'decision':'BLOCKED_INCOMPLETE_GEOMETRY','reason':str(exc),'contact_authority':False}
    (args.output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    # A finite comparison never grants physical-contact or planning authority.
    raise SystemExit(0 if result['runtime_equivalence']=='PASS_FINITE_THREE_WAY' else 2)

if __name__=='__main__':main()
