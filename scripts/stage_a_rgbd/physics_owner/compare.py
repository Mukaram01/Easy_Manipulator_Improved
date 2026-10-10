#!/usr/bin/env python3
"""One bounded stock/owner comparison. No ROS, bridge, EPD or MoveIt launch."""
import argparse
import hashlib
import json
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


def run(args,kind,session):
    root=args.output/kind;root.mkdir()
    tree=ET.parse(args.world);world=tree.getroot().find('world')
    physics=[p for p in world.findall('plugin') if p.get('name') in ('ignition::gazebo::systems::Physics','gz::sim::systems::Physics')]
    if len(physics)!=1:raise ValueError('exactly one original Physics System required')
    if kind=='instrumented':
        physics[0].set('filename',str(args.owner.resolve()))
        physics[0].set('name','ignition::gazebo::systems::WorkcellOwnerPhysics')
        for key,value in {'owner_output':str(root.resolve()/'owner.json'),'owner_session':session,'owner_record_step':args.steps}.items():ET.SubElement(physics[0],key).text=str(value)
    observer=ET.SubElement(world,'plugin',filename=str(args.observer.resolve()),name='WorkcellPhysicsContactMeasurement')
    for key,value in {'support_collision':'a0::pick_support::support_link::support_collision','workpieces':'part_00 part_01','run_id':session,'diagnostic_output':str(root.resolve()/'ecm.json'),'diagnostic_step':args.steps}.items():ET.SubElement(observer,key).text=str(value)
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
    result={'exit':server.returncode,'wall_time_s':time.monotonic()-started,'world_sha256':sha(derived),'loaded_libraries':{p:sha(p) for p in sorted(paths)}}
    (root/'run.json').write_text(json.dumps(result,indent=2)+'\n')
    if not (root/'ecm.json').is_file():raise RuntimeError('exact-step ECM diagnostic missing')
    if kind=='instrumented' and not (root/'owner.json').is_file():raise RuntimeError('exact-step owner readback missing')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('world','owner','observer','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--steps',type=int,default=2000)
    args=parser.parse_args()
    if not 1<=args.steps<=10000:parser.error('steps must be in [1,10000]')
    args.output.mkdir(exist_ok=False);session=str(uuid.uuid4())
    result={'session':session,'steps':args.steps,'original_world_sha256':sha(args.world),'execution_goals':0,'moveit_started':False,'instrumented_runtime_only':True}
    for kind in ('stock','instrumented'):result[kind]=run(args,kind,session)
    stock=json.loads((args.output/'stock/ecm.json').read_text());instrumented=json.loads((args.output/'instrumented/ecm.json').read_text())
    owner=json.loads((args.output/'instrumented/owner.json').read_text())
    owner_valid=(owner.get('complete') is True and owner.get('session')==session
        and owner.get('step')==args.steps and owner.get('stamp_ns')==stock.get('stamp_ns')
        and owner.get('dart_frames')==args.steps and len(owner.get('shapes',[]))>0)
    result['owner_exact_step_complete']=owner_valid
    result['exact_step_ecm_records_equal']=stock==instrumented
    result['stock_contact_evidence_present']=bool(stock.get('diagnostic_contacts'))
    result['instrumented_contact_evidence_present']=bool(instrumented.get('diagnostic_contacts'))
    result['runtime_equivalence']='UNQUALIFIED_FINITE_SAMPLE_ONLY'
    if not owner_valid:result['runtime_equivalence']='BLOCKED_OWNER_READBACK_INCOMPLETE'
    elif stock!=instrumented:result['runtime_equivalence']='BLOCKED_ECM_RECORD_MISMATCH'
    elif not result['stock_contact_evidence_present'] or not result['instrumented_contact_evidence_present']:
        result['runtime_equivalence']='BLOCKED_MISSING_STOCK_OR_INSTRUMENTED_CONTACT_EVIDENCE'
    result['comparison_scope']='one identical-world finite-step run per binary; no stock-backend geometry attestation or universal equivalence claim'
    (args.output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    # A launch or matching ECM sample never grants runtime/contact authority.
    raise SystemExit(2)

if __name__=='__main__':main()
