"""Independent Decimal and native ODE oracles; model tests grant no authority."""
import copy
import ctypes
import hashlib
import json
from decimal import Decimal, localcontext
from fractions import Fraction as F
import math
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_ode_numeric as n


def rx(angle):
    c,s=math.cos(angle),math.sin(angle)
    return [[1.,0.,0.],[0.,c,-s],[0.,s,c]]


def decimal_rotation(matrix):
    # Independent high-precision Shoemake path and normalized quaternion formula.
    with localcontext() as ctx:
        ctx.prec=100
        r=[[Decimal.from_float(v) for v in row] for row in matrix]
        t=sum(r[i][i] for i in range(3));q=[Decimal(0)]*4
        if t>0:
            s=(1+t).sqrt();q[0]=s/2
            q[1]=(r[2][1]-r[1][2])/(2*s)
            q[2]=(r[0][2]-r[2][0])/(2*s)
            q[3]=(r[1][0]-r[0][1])/(2*s)
        else:
            i=max(range(3),key=lambda i:r[i][i]);j=(i+1)%3;k=(j+1)%3
            s=(1+r[i][i]-r[j][j]-r[k][k]).sqrt();q[i+1]=s/2
            q[0]=(r[k][j]-r[j][k])/(2*s)
            q[j+1]=(r[j][i]+r[i][j])/(2*s);q[k+1]=(r[k][i]+r[i][k])/(2*s)
        norm=sum(v*v for v in q).sqrt();w,x,y,z=[v/norm for v in q]
        return [[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]]


@pytest.mark.parametrize('angle',[0.,1e-20,1e-9,.15,2*math.pi/3,math.nextafter(2*math.pi/3,math.inf),math.pi,2.9,-2.9])
def test_rotation_encloses_independent_high_precision_oracle(angle):
    r=rx(angle);intervals=n.convert_rotation(r);oracle=decimal_rotation(r)
    for i in range(3):
        for j in range(3):assert intervals[i][j].lo<=F(oracle[i][j])<=intervals[i][j].hi


def test_interval_cancellation_sqrt_and_all_round_modes_enclosure():
    a=n.Interval.point(.1);b=n.Interval.point(.2)
    c=a+b
    assert c.lo<=F(.1)+F(.2)<=c.hi
    root=n.Interval.point(2).sqrt()
    assert root.lo**2<=2<=root.hi**2
    assert (a-a).lo==(a-a).hi==0
    assert n.normalize_quaternion([n.Interval.point(x) for x in [1,0,0,0]])==[n.Interval.point(x) for x in [1,0,0,0]]


@pytest.mark.parametrize('bad', [float('nan'),float('inf')])
def test_nonfinite_rejected(bad):
    r=rx(.1);r[0][0]=bad
    with pytest.raises(ValueError):n.convert_rotation(r)


def test_invalid_and_unsupported_arithmetic_rejected():
    for r in ([[0.]*3]*3, [[-1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]):
        with pytest.raises(ValueError):n.convert_rotation(r)
    with pytest.raises(ValueError):n.convert_rotation(rx(.1),semantics='binary32_fast_math')
    with pytest.raises(ValueError):n.verify_binary_profile({})


def shape(z=.01249999,angle=0.,support=False):
    r=rx(angle);t=[r[i]+[0.,0.,z][i:i+1] for i in range(3)]+[[0.,0.,0.,1.]]
    return dict(shape_type='BoxShape',dimensions_m=[2100.]*3 if support else [.025]*3,
        world_transform=t)


def test_large_support_offset_and_rotated_corner_extremes():
    s=n.box_corners(shape(-1050.,support=True),'world_transform')
    assert max(p[2].hi for p in s)==0
    c=n.box_corners(shape(.01249,.2),'world_transform')
    assert min(p[2].lo for p in c)<-F(1,10000)


@pytest.mark.parametrize('depth,accepted',[(.000099,True),(.0001001,False),(.0002,False)])
def test_unchanged_threshold_uses_outward_bound(depth,accepted):
    s=shape(.0125-depth)
    upper=n.penetration_bound(s,shape(-1050.,support=True),'world_transform')['penetration_upper_m']
    assert (F(upper)<=F(1,10000))==accepted


def test_native_ode_rotation_is_enclosed_when_installed():
    path=Path('/usr/lib/x86_64-linux-gnu/libode.so.8.0.2')
    if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest()!=n.LIBRARIES[str(path)]:
        pytest.skip('hash-qualified double-precision ODE binary not installed on this CI host')
    lib=ctypes.CDLL(str(path));lib.dRfromQ.argtypes=[ctypes.POINTER(ctypes.c_double)]*2
    for values in ([1.,0.,0.,0.],[.5,.5,.5,.5],[.001,.2,-.3,.9]):
        q=(ctypes.c_double*4)(*values);lib.dNormalize4(q)
        actual=(ctypes.c_double*12)();lib.dRfromQ(actual,q)
        bounded=n.quaternion_rotation(n.normalize_quaternion([n.Interval.point(v) for v in values]))
        for i in range(3):
            for j in range(3):assert bounded[i][j].lo<=F(actual[4*i+j])<=bounded[i][j].hi


def test_saved_trace_without_active_detector_witness_cannot_gain_authority():
    from test_stage_a_owner_geometry import fixture
    o,r=fixture()
    with pytest.raises(ValueError,match='detector'):
        n.qualify_owner_geometry(o,r,{})


@pytest.mark.parametrize('axis',[1,2])
@pytest.mark.parametrize('angle',[2.9,math.pi,math.nextafter(math.pi,0.)])
def test_other_dominant_quaternion_branches(axis,angle):
    c,s=math.cos(angle),math.sin(angle)
    r=([[c,0.,s],[0.,1.,0.],[-s,0.,c]] if axis==1 else [[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    bounded=n.convert_rotation(r);oracle=decimal_rotation(r)
    assert all(bounded[i][j].lo<=F(oracle[i][j])<=bounded[i][j].hi for i in range(3) for j in range(3))


def test_trace_branch_boundary_encloses_both_possible_branches():
    # Valid rotation around (1,1,1), just off 120 degrees. Rounded trace
    # additions straddle zero, so both binary branches must be enclosed.
    import numpy as np
    axis=np.sqrt([2/3,4/15,1/15]);c=-.5;sine=math.sqrt(3)/2
    skew=np.array([[0.,-axis[2],axis[1]],[axis[2],0.,-axis[0]],[-axis[1],axis[0],0.]])
    r=(c*np.eye(3)+(1-c)*np.outer(axis,axis)+sine*skew).tolist()
    r[0][0]=.5;r[1][1]=-.1;r[2][2]=-math.nextafter(.4,0.)
    trace=(n.Interval.point(r[2][2])+r[1][1])+r[0][0]
    assert trace.lo==0 and trace.hi>0  # >0 and <=0 branches are both possible
    bounded=n.convert_rotation(r);oracle=decimal_rotation(r)
    assert all(bounded[i][j].lo<=F(oracle[i][j])<=bounded[i][j].hi for i in range(3) for j in range(3))


def test_native_cpp_eigen_ode_under_all_four_rounding_modes(tmp_path):
    import json,shutil,subprocess
    if not Path('/usr/include/ode/ode.h').exists() or not shutil.which('c++'):
        pytest.skip('ODE development headers/compiler unavailable on this CI host')
    source=Path(__file__).resolve().parents[1]/'scripts/stage_a_rgbd/physics_owner/ode_numeric_probe.cpp'
    binary=tmp_path/'ode_numeric_probe'
    subprocess.run(['c++','-std=c++17','-O2','-frounding-math','-fno-fast-math','-ffp-contract=off',
        '-I/usr/include/eigen3',str(source),'-lode','-o',str(binary)],check=True,capture_output=True)
    records=[json.loads(l) for l in subprocess.check_output([str(binary)],text=True).splitlines()]
    assert len(records)==96 and len({r['rounding'] for r in records})==4
    for record in records:
        matrix=[[float.fromhex(x) for x in row] for row in record['input']]
        bounded=n.convert_rotation(matrix)
        for i,row in enumerate(record['output']):
            for j,value in enumerate(row):assert bounded[i][j].lo<=F(float.fromhex(value))<=bounded[i][j].hi


@pytest.fixture
def campaign():
    folder=Path(__file__).resolve().parents[1]/'evidence/stage_a2_geometry/physics_owner/ode_numeric'
    owner=json.loads((folder/'owner_readback.jsonl').read_text().splitlines()[0])
    observer=json.loads((folder/'observer.jsonl').read_text().splitlines()[0])
    run=json.loads((folder/'run.json').read_text())
    return owner,observer,run,folder/'world.sdf'


def test_qualified_saved_capture_exports_no_contact_permission(campaign):
    o,r,run,world=campaign
    if any(not Path(p).is_file() for p in n.LIBRARIES):
        pytest.skip('audited local overlay binaries unavailable on this CI host')
    result=n.qualify_owner_geometry(o,r,run['loaded_libraries'],world,run)
    assert result['decision']=='PASS_SIMULATION_ONLY_NUMERICS'
    assert result['physical_penetration_upper_m']<=.0001
    assert not result['contact_authority'] and not result['support_contact_permission']
    assert result['covered_state']['dart_input_frame']==99


@pytest.mark.parametrize('mutation,reason',[
    ('object','collision-object'),('inventory','inventory'),('mapping','inventory'),
    ('detector','detector'),('contacts','contact'),('step','step')])
def test_capture_rejects_missing_or_wrong_backend_evidence(campaign,mutation,reason):
    o,r,run,world=campaign
    if mutation=='object':o['contacts'][0]['collision_object_type1']='UnknownObject'
    if mutation=='inventory':o['detector_shape_nodes'].pop()
    if mutation=='mapping':o['shapes'][0]['physics_shape_id']=-1
    if mutation=='detector':o['collision_detector_type']='bullet'
    if mutation=='contacts':o['contacts']=[]
    if mutation=='step':o['step']+=1
    with pytest.raises(ValueError,match=reason):
        n.qualify_owner_geometry(o,r,run['loaded_libraries'],world,run)


def test_changed_world_is_not_qualified(campaign,tmp_path):
    o,r,run,world=campaign
    data=world.read_text().replace('2100','2101')
    # Original world uses an SDF plane, so alter gravity if no BOX text exists.
    if data==world.read_text():data=data.replace('9.8','9.7')
    assert data!=world.read_text()
    changed=tmp_path/'world.sdf';changed.write_text(data)
    run['world_sha256']=hashlib.sha256(changed.read_bytes()).hexdigest()
    with pytest.raises(ValueError,match='world, construction'):
        n.verify_campaign_world(changed,run,o['session'])
