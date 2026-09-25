#!/usr/bin/env python3
"""Passive, timestamped BOX observations from the live Fortress pose bridge.

World geometry supplies dimensions; only measured poses supply location. This
never moves objects or supplies a grasp, and is not a perception/EPD loop.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
from simulator_backend import digest, verify_receipt_process


def box_models(receipt):
    r=json.loads(Path(receipt).read_text());path=Path(receipt).parent/'world.sdf'
    if digest(path.read_bytes())!=r['world_sha256']:raise ValueError('world changed since launch')
    world=ET.parse(path).getroot().find('world');result={}
    for model in world.findall('model'):
        if model.findtext('static','false').lower()=='true':continue
        collisions=model.findall('link/collision')
        if len(collisions)!=1 or collisions[0].find('geometry/box/size') is None:raise ValueError('observation requires one BOX collision per dynamic model')
        if model.find('link/pose') is not None or collisions[0].find('pose') is not None:raise ValueError('offset collision observation unsupported')
        result[model.get('name')]=[float(x) for x in collisions[0].findtext('geometry/box/size').split()]
    if not result:raise ValueError('no dynamic BOX objects in world')
    return r,result


def pose(t):
    p,q=t.translation,t.rotation
    return [p.x,p.y,p.z,q.x,q.y,q.z,q.w]


def rpy(p):
    x,y,z,w=p[3:]
    return [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1.,min(1.,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]


def settling_error(samples):
    reference=samples[0]['poses']
    errors=[abs(x-y) for sample in samples for name in reference
            for x,y in zip(reference[name],sample['poses'][name])]
    return max(errors) if all(math.isfinite(e) for e in errors) else math.inf


def _quat_from_rpy(values):
    roll,pitch,yaw=values
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def refresh_snapshot_geometry(bound,fresh,position_tolerance=1e-5,orientation_tolerance=1e-5):
    """Renew timestamps only after a fresh Fortress readback proves geometry unchanged."""
    if bound.get('schema_version')!='detected_objects/v1' or fresh.get('schema_version')!='detected_objects/v1':
        raise RuntimeError('snapshot refresh requires detected_objects/v1')
    if bound.get('simulator_receipt_sha256')!=fresh.get('simulator_receipt_sha256'):
        raise RuntimeError('snapshot refresh crossed simulator runs')
    previous={o.get('object_id'):o for o in bound.get('objects',[])}
    current={o.get('object_id'):o for o in fresh.get('objects',[])}
    if set(previous)!=set(current) or None in previous:
        raise RuntimeError('physical object identity changed during snapshot refresh')
    max_position=max_orientation=max_dimension=0.0
    rebound=copy.deepcopy(bound)
    fresh_by_id=current
    for old in rebound['objects']:
        new=fresh_by_id[old['object_id']]
        if old.get('class_id')!=new.get('class_id'):
            raise RuntimeError('physical object class changed during snapshot refresh')
        od=list(old['dimensions']);nd=list(new['dimensions'])
        max_dimension=max(max_dimension,max(abs(a-b) for a,b in zip(od,nd)))
        op=list(old['pose']['xyz']);np=list(new['pose']['xyz'])
        max_position=max(max_position,math.dist(op,np))
        oq,nq=_quat_from_rpy(old['pose']['rpy']),_quat_from_rpy(new['pose']['rpy'])
        max_orientation=max(max_orientation,min(math.dist(oq,nq),math.dist(oq,[-v for v in nq])))
        if max_dimension>1e-12 or max_position>position_tolerance or max_orientation>orientation_tolerance:
            raise RuntimeError(
                f'physical object geometry changed during snapshot refresh: '
                f'position={max_position:.9g}, orientation={max_orientation:.9g}, dimensions={max_dimension:.9g}')
        old['timestamp']=new['timestamp']
        old['confidence']=new.get('confidence')
    rebound['source']='Fortress measured model poses; prior geometry revalidated'
    return rebound,dict(max_position_delta_m=max_position,
                        max_quaternion_delta=max_orientation,
                        max_dimension_delta_m=max_dimension)


def verify_snapshot_binding(snapshot, receipt_sha256):
    if snapshot.get('simulator_receipt_sha256') != receipt_sha256:
        raise RuntimeError('simulator observation belongs to an unknown/different simulator run')


def capture(receipt, output, class_id, timeout=30, refresh_from=None):
    import rclpy
    from tf2_msgs.msg import TFMessage
    import yaml
    verify_receipt_process(receipt)
    r,models=box_models(receipt)
    rclpy.init();node=rclpy.create_node('simulator_box_observations');samples=[]
    def update(msg):
        poses={t.child_frame_id:pose(t.transform) for t in msg.transforms if t.child_frame_id in models}
        if set(poses)==set(models):samples.append(dict(received_at=time.time(),poses=poses))
    sub=node.create_subscription(TFMessage,f"/world/{r['world']}/pose/info",update,20)
    try:
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if len(samples)<2 or samples[-1]['received_at']-samples[0]['received_at']<1:continue
            b=samples[-1]['poses']
            error=settling_error(samples)
            if error<=1e-5:break
            samples=samples[-1:]
        else:raise RuntimeError('physical objects did not settle or pose stream unavailable')
        verify_receipt_process(receipt)
        snapshot=dict(schema_version='detected_objects/v1',source='Fortress measured model poses',
            simulator_receipt_sha256=digest(Path(receipt).read_bytes()),objects=[
            dict(object_id=name,class_id=class_id,confidence=None,timestamp=samples[-1]['received_at'],
                 pose=dict(frame_id='world',xyz=b[name][:3],rpy=rpy(b[name])),dimensions=size)
            for name,size in sorted(models.items())])
        refresh_evidence=None
        if refresh_from is not None:
            bound=yaml.safe_load(Path(refresh_from).read_text())
            snapshot,refresh_evidence=refresh_snapshot_geometry(bound,snapshot)
        Path(output).write_text(yaml.safe_dump(snapshot,sort_keys=False))
        Path(output).with_suffix('.evidence.json').write_text(json.dumps(
            dict(world=r['world'],samples=samples,max_settling_delta=error,
                 refresh_geometry=refresh_evidence),indent=2))
        return snapshot
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--receipt',required=True,type=Path);parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--class-id',required=True)
    parser.add_argument('--refresh-from',type=Path,
        help='Renew timestamps only if fresh Fortress geometry still matches this bound snapshot')
    args=parser.parse_args()
    capture(args.receipt,args.output,args.class_id,refresh_from=args.refresh_from)
