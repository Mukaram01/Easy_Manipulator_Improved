#!/usr/bin/env python3
"""Outward binary64 enclosure of the hash-audited DART/ODE BOX conversion.

The arithmetic model alone is conditional. Authority additionally requires live
ODE detector/object witnesses and matching loaded AND locally audited binaries.
No contact permission or planning pose is produced.
"""
from dataclasses import dataclass
from fractions import Fraction as F
import ctypes
import hashlib
import itertools
import json
import math
from pathlib import Path
import platform
import sys
import xml.etree.ElementTree as ET

from stage_a_physics_contact import corners, enclose_owner_geometry, outward, require

SEMANTICS='audited_x86_64_binary64_sse2'
MIN_NORMAL=F(2)**-1022
CAMPAIGN_CANONICAL_SHA256='8393a8cc7b96f6a53021e73e3e99261239946bcaebdb52f373490c6c72ecd7c6'
LIBRARIES={
 '/tmp/stage_a2_owner_build/libworkcell_owner_physics.so':'58e990834de007fffe3f0f980177e872200e2360340e0d33cf5bf33e8fbfb2d2',
 '/tmp/stage_a2_physics_contact_build/libworkcell_physics_contact_measurement.so':'718754a5a29463bcb6d55a786493c7d9ca44b0b1aa6592f1e7da5c12e87dc82b',
 '/usr/lib/x86_64-linux-gnu/libdart-collision-ode.so.6.12.1':'cc914ae1d651bfd79fa7eb308c6f39a12174e8b59d0fb45318652f0eb61761ee',
 '/usr/lib/x86_64-linux-gnu/libode.so.8.0.2':'0b40593ed29a7b4f72f364a989ac8d8ea0bdab2a7578518366048b5128c0343b',
 '/usr/lib/x86_64-linux-gnu/libdart.so.6.12.1':'5a0048707f55903e0ba62f9bba76456b6f0eb850c1d5016ea499432c1bab089b',
 '/usr/lib/x86_64-linux-gnu/ign-physics-5/engine-plugins/libignition-physics5-dartsim-plugin.so.5.4.0':'44e5d8573b44ef1f555eba6f2fbf01c1f0ab657b0fd32016c441e884ec5e71a1'}


@dataclass(frozen=True)
class Interval:
    lo:F
    hi:F

    @staticmethod
    def point(value):
        require(type(value) in (int,float,F) and (type(value) is not float or math.isfinite(value)), 'finite binary64 input required')
        v=F(value)
        require(v==0 or abs(v)>=MIN_NORMAL,'subnormal arithmetic configuration unsupported')
        require(F(outward(v,False))==v==F(outward(v,True)), 'input is not exactly representable as binary64')
        return Interval(v,v)

    @staticmethod
    def rounded(lo,hi):
        require(lo<=hi,'invalid interval')
        low,high=F(outward(lo,False)),F(outward(hi,True))
        # Reject subnormal endpoints rather than assuming unrecorded FTZ/DAZ.
        require(all(v==0 or abs(v)>=MIN_NORMAL for v in (low,high)), 'subnormal arithmetic configuration unsupported')
        return Interval(low,high)

    def __add__(self,other):
        o=as_interval(other);return self.rounded(self.lo+o.lo,self.hi+o.hi)
    __radd__=__add__
    def __neg__(self):return Interval(-self.hi,-self.lo)
    def __sub__(self,other):return self+-as_interval(other)
    def __rsub__(self,other):return as_interval(other)+-self
    def __mul__(self,other):
        o=as_interval(other);v=[a*b for a in (self.lo,self.hi) for b in (o.lo,o.hi)]
        return self.rounded(min(v),max(v))
    __rmul__=__mul__
    def __truediv__(self,other):
        o=as_interval(other);require(not o.lo<=0<=o.hi,'interval divisor crosses zero')
        v=[a/b for a in (self.lo,self.hi) for b in (o.lo,o.hi)]
        return self.rounded(min(v),max(v))
    def square(self):
        low=0 if self.lo<=0<=self.hi else min(self.lo**2,self.hi**2)
        return self.rounded(low,max(self.lo**2,self.hi**2))
    def sqrt(self):
        require(self.lo>=0,'negative square root interval')
        # 2^-1074 is binary64's smallest representable quantum, not a tolerance.
        # Integer square root provides exact rational brackets without libm.
        scale=1<<1074
        def bracket(v):
            a=math.isqrt((v.numerator*scale*scale)//v.denominator)
            low=F(a,scale);return low,low if low*low==v else F(a+1,scale)
        return self.rounded(bracket(self.lo)[0],bracket(self.hi)[1])


def as_interval(value):return value if isinstance(value,Interval) else Interval.point(value)


def normalize_quaternion(q):
    require(len(q)==4,'four quaternion components required')
    norm=((q[0].square()+q[1].square())+q[2].square())+q[3].square()
    require(norm.lo>0,'unqualified quaternion normalization')
    reciprocal=Interval.point(1)/norm.sqrt()
    return [v*reciprocal for v in q]


def quaternion_rotation(q):
    w,x,y,z=q
    xx=(x+x)*x;yy=(y+y)*y;zz=(z+z)*z
    def twice(v):return v+v
    return [[1-yy-zz,twice(x*y-w*z),twice(x*z+w*y)],
            [twice(x*y+w*z),1-xx-zz,twice(y*z-w*x)],
            [twice(x*z-w*y),twice(y*z+w*x),1-xx-yy]]


def convert_rotation(matrix,semantics=SEMANTICS):
    require(semantics==SEMANTICS,'unsupported precision/compiler semantics')
    require(isinstance(matrix,list) and len(matrix)==3 and all(isinstance(row,list) and len(row)==3 for row in matrix),'3x3 rotation required')
    # Reuse existing rigid-input validity gate; its tolerance is NOT an error
    # allowance. Every accepted binary coefficient is propagated exactly below.
    corners(dict(shape='BOX',support_shape='PLANE',collision_id=1,support_collision_id=2,
        dimensions_m=[1.,1.,1.],centre_world=[0.,0.,0.],axes_world=matrix,
        support_normal_world=[0.,0.,1.],support_point_world=[0.,0.,0.]))
    r=[[Interval.point(v) for v in row] for row in matrix]
    trace=(r[2][2]+r[1][1])+r[0][0]  # actual installed SSE2 order
    candidates=[]
    if trace.hi>0:
        # Restrict the true branch's trace interval before evaluating its body.
        t=Interval(max(F(0),trace.lo),trace.hi)
        root=(t+1).sqrt();scale=Interval.point(.5)/root
        candidates.append([root*.5,(r[2][1]-r[1][2])*scale,
            (r[0][2]-r[2][0])*scale,(r[1][0]-r[0][1])*scale])
    if trace.lo<=0:
        i=0
        if matrix[1][1]>matrix[0][0]:i=1
        if matrix[2][2]>matrix[i][i]:i=2
        j=(i+1)%3;k=(j+1)%3
        root=(((r[i][i]-r[j][j])-r[k][k])+1).sqrt();scale=Interval.point(.5)/root
        q=[Interval.point(0) for _ in range(4)];q[i+1]=root*.5
        q[0]=(r[k][j]-r[j][k])*scale
        q[j+1]=(r[j][i]+r[i][j])*scale;q[k+1]=(r[k][i]+r[i][k])*scale
        candidates.append(q)
    rotations=[quaternion_rotation(normalize_quaternion(q)) for q in candidates]
    return [[Interval(min(r[i][j].lo for r in rotations),max(r[i][j].hi for r in rotations)) for j in range(3)] for i in range(3)]


def box_corners(shape,key):
    require(shape.get('shape_type')=='BoxShape','unsupported detector geometry')
    t=shape[key];r=convert_rotation([row[:3] for row in t[:3]])
    dims=shape['dimensions_m'];require(len(dims)==3 and all(v>0 for v in dims),'invalid BOX dimensions')
    # Audited BOX constructor creates identity/zero geom offset. Composing that
    # offset is exactly equal numerically, including the 2100 m support BOX.
    centre=[Interval.point(t[i][3]) for i in range(3)]
    return [[centre[i]+sum((r[i][j]*(Interval.point(dims[j])/2)*signs[j] for j in range(3)),Interval.point(0)) for i in range(3)]
            for signs in itertools.product((-1,1),repeat=3)]


def penetration_bound(cube,support,key):
    cube_points=box_corners(cube,key);support_points=box_corners(support,key)
    # This campaign has an exactly horizontal support. Do not substitute a
    # bounding volume top for a tilted support surface.
    sr=convert_rotation([row[:3] for row in support[key][:3]])
    require(all(v==Interval.point(int(i==j)) for i,row in enumerate(sr) for j,v in enumerate(row)), 'unsupported detector support orientation')
    top=max(p[2].hi for p in support_points)
    for i in (0,1):
        low=min(p[i].hi for p in support_points);high=max(p[i].lo for p in support_points)
        require(all(low<=p[i].lo and p[i].hi<=high for p in cube_points),'detector BOX exceeds support footprint')
    minimum=min(p[2].lo for p in cube_points)
    errors=[]
    for signs,p in zip(itertools.product((-1,1),repeat=3),cube_points):
        t=cube[key]
        exact=[F(t[i][3])+sum(F(t[i][j])*F(cube['dimensions_m'][j])*signs[j]/2 for j in range(3)) for i in range(3)]
        errors.append(max(max(abs(p[i].lo-exact[i]),abs(p[i].hi-exact[i])) for i in range(3)))
    return dict(penetration_upper_m=outward(max(F(0),top-minimum),True),
        cube_conversion_corner_error_linf_m=outward(max(errors),True),
        support_top_interval_m=[outward(max(p[2].lo for p in support_points),False),outward(top,True)],
        evaluated_corner_count=8)


def verify_binary_profile(loaded_libraries):
    require(platform.machine()=='x86_64' and sys.float_info.mant_dig==53,'unsupported precision/compiler configuration')
    for filename,expected in LIBRARIES.items():
        path=Path(filename)
        require(loaded_libraries.get(filename)==expected and path.is_file() and
            hashlib.sha256(path.read_bytes()).hexdigest()==expected,'unknown ODE conversion binary or loaded-library identity')
    lib=ctypes.CDLL('/usr/lib/x86_64-linux-gnu/libode.so.8.0.2')
    lib.dCheckConfiguration.argtypes=[ctypes.c_char_p];lib.dCheckConfiguration.restype=ctypes.c_int
    require(lib.dCheckConfiguration(b'ODE_double_precision')==1,'unsupported ODE dReal precision')
    return dict(semantics=SEMANTICS,library_sha256=LIBRARIES,
        rounding='all four IEEE-754 rounding modes; subnormal domains rejected',
        compiler_flags='not recovered; relevant operations audited from these exact ELF binaries')


def verify_campaign_world(world_path,run,session):
    require(world_path is not None and run is not None, 'verified fixed-world provenance required')
    data=Path(world_path).read_bytes()
    require(run.get('exit')==0 and hashlib.sha256(data).hexdigest()==run.get('world_sha256'), 'loaded world identity mismatch')
    root=ET.fromstring(data);world=root.find('world')
    require(world is not None,'loaded world required')
    plugins=world.findall('plugin')
    owners=[p for p in plugins if p.get('name')=='ignition::gazebo::systems::WorkcellOwnerPhysics']
    observers=[p for p in plugins if p.get('name')=='WorkcellPhysicsContactMeasurement']
    require(len(owners)==len(observers)==1,'single audited owner and observer required')
    owner,observer=owners[0],observers[0]
    require(owner.get('filename')=='/tmp/stage_a2_owner_build/libworkcell_owner_physics.so' and
        observer.get('filename')=='/tmp/stage_a2_physics_contact_build/libworkcell_physics_contact_measurement.so', 'unknown measurement plugin')
    require({c.tag for c in owner}=={'owner_session','owner_output','owner_record_steps'} and
        owner.findtext('owner_session')==session,'owner session/configuration mismatch')
    require({c.tag for c in observer}=={'support_collision','workpieces','run_id','diagnostic_output','diagnostic_steps'} and
        observer.findtext('run_id')==session and observer.findtext('workpieces')=='part_00 part_01' and
        observer.findtext('support_collision')=='a0::pick_support::support_link::support_collision', 'observer configuration mismatch')
    require(owner.findtext('owner_record_steps')==observer.findtext('diagnostic_steps')=='100 250 500 1000 1500 2000','unsupported physics schedule')
    for child in list(owner):owner.remove(child)
    owner.set('filename','ignition-gazebo-physics-system');owner.set('name','ignition::gazebo::systems::Physics')
    world.remove(observer)
    canonical=ET.canonicalize(ET.tostring(root,encoding='unicode'),strip_text=True)
    require(hashlib.sha256(canonical.encode()).hexdigest()==CAMPAIGN_CANONICAL_SHA256,'unsupported world, construction or physics configuration')
    require(not any('gazebo6-physics-system' in name for name in run['loaded_libraries']), 'duplicate physics owner')
    return CAMPAIGN_CANONICAL_SHA256


def qualify_owner_geometry(owner,observer,loaded_libraries,world_path=None,run=None):
    require(owner.get('collision_detector_type')==owner.get('collision_detector_type_pre')=='ode', 'active detector witness required')
    shapes=owner.get('shapes',[]);nodes=[s['shape_node_identity'] for s in shapes]
    # Actual construction-witness IDs from the hash-pinned two-cube campaign,
    # not a general rule for Gazebo/DART ID arithmetic.
    inventory=dict(zip((7,15,16,17,18,19,23,27),range(12,20)))
    require({s['collision_id'] for s in shapes}==set(inventory) and
        all(s.get('physics_shape_id')==inventory[s['collision_id']] for s in shapes), 'unsupported or incomplete campaign collision inventory')
    require(len(owner.get('detector_shape_nodes',[]))==len(nodes) and set(owner['detector_shape_nodes'])==set(nodes),'incomplete detector inventory')
    for c in owner.get('contacts',[]):
        require(c.get('detector_type1')==c.get('detector_type2')=='ode' and
            c.get('collision_object_type1')==c.get('collision_object_type2')=='N4dart9collision18OdeCollisionObjectE', 'unknown detector collision-object behaviour')
    base=enclose_owner_geometry(owner,observer,owner['session'],owner['step'])
    profile=verify_binary_profile(loaded_libraries)
    profile['campaign_canonical_sha256']=verify_campaign_world(world_path,run,owner['session'])
    shapes={s['collision_id']:s for s in shapes};support=shapes[observer['support_collision_id']]
    for pair in base['pairs']:
        cube=shapes[pair['collision_id']]
        pair['detector_contact_input']=penetration_bound(cube,support,'pre_step_world_transform')
        pair['prospective_post_step_conversion']=penetration_bound(cube,support,'world_transform')
        # Both sampled states must be safe. Only the pre-step state is attested
        # as consumed by collision evaluation; endpoint ODE update is not read.
        require(all(F(pair[k]['penetration_upper_m'])<=F(1,10000) for k in ('detector_contact_input','prospective_post_step_conversion')),'detector whole-shape penetration exceeds 0.1 mm')
    base.update(scope='instrumented_simulation_only_audited_ODE_BOX_contact_input',
        decision='PASS_SIMULATION_ONLY_NUMERICS',binary_profile=profile,
        physical_penetration_upper_m=max(p['detector_contact_input']['penetration_upper_m'] for p in base['pairs']),
        backend_conversion_error_bound_m=max(p['detector_contact_input']['cube_conversion_corner_error_linf_m'] for p in base['pairs']),
        contact_authority=False,support_contact_permission=False,epd_association='BLOCKED',
        covered_state=dict(contact_evaluation_step=owner['step'],dart_input_frame=owner['pre_step_dart_frames'],
            input_nominal_stamp_ns=owner['stamp_ns']-owner['dt_ns'],reported_contact_stamp_ns=owner['stamp_ns']),
        outstanding_proof='independent renderer/physics exposure alignment and unique visible EPD object identity')
    return base


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--owner',type=Path,required=True);p.add_argument('--observer',type=Path,required=True)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--world',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result={'contact_authority':False,'execution_goals':0,'moveit_started':False}
    try:
        owners=[json.loads(l) for l in a.owner.read_text().splitlines() if l.strip()]
        observers=[json.loads(l) for l in a.observer.read_text().splitlines() if l.strip()]
        require(bool(owners) and len(owners)==len(observers),'complete sample schedule required')
        run=json.loads(a.run.read_text());require(run.get('exit')==0,'completed bounded run required')
        require([o['step'] for o in owners]==sorted(set(o['step'] for o in owners)),'unique step schedule required')
        result['states']=[qualify_owner_geometry(o,r,run['loaded_libraries'],a.world,run) for o,r in zip(owners,observers)]
        result['decision']='PASS_SIMULATION_ONLY_NUMERICS'
    except (OSError,ValueError,KeyError,TypeError,OverflowError) as exc:
        result.update(decision='BLOCKED',reason=str(exc))
    result['input_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (a.owner,a.observer,a.run,a.world) if p.is_file()}
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(result['decision']);raise SystemExit(0 if result['decision']=='PASS_SIMULATION_ONLY_NUMERICS' else 2)
