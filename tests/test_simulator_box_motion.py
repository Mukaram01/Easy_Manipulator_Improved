"""Captured physical-guard regression; offline tests are not runtime acceptance."""
import copy,json,math,sys
from pathlib import Path
from types import SimpleNamespace as N
import xml.etree.ElementTree as ET
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
import simulator_execution as module
fixture=json.loads((Path(__file__).parent/'fixtures/stage_a1_box_motion.json').read_text())

def test_real_neighbor_combined_motion_stays_within_every_point_limit(tmp_path,monkeypatch):
    f=copy.deepcopy(fixture);origin=f['origin'];last=f['rejected'];now=[origin['wall_ns']/1e9+.001]
    # Recorded before measurement-clock stamps: replay it on one synthetic timeline.
    for record,keys in ((origin,['wall_ns']),(last,['wall_ns']),
                        (f['binding'],['close_goal_accepted_wall_ns','close_goal_terminal_wall_ns'])):
        for key in keys:record[key.replace('wall_ns','monotonic_raw_ns')]=record[key]
    monkeypatch.setattr(module.time,'time',lambda:now[0]);monkeypatch.setattr(module.time,'time_ns',lambda:int(now[0]*1e9))
    monkeypatch.setattr('measurement_clock.now_ns',lambda:int(now[0]*1e9))
    models=''.join(f'<model name="{o["object_id"]}"><link name="link"><collision name="collision"><geometry><box><size>0.025 0.025 0.025</size></box></geometry></collision></link></model>' for o in f['objects'])
    (tmp_path/'world.sdf').write_text('<sdf><world name="a0">'+models+'</world></sdf>')
    m=N(receipt=dict(world='a0',model='workcell_robot',run_id=origin['run_id'],pid=origin['pid']),receipt_path=tmp_path/'receipt.json',robot=ET.fromstring('<robot/>'),fresh=lambda:origin,object_pose=lambda s,n:s['poses']['a0::'+n],frame=lambda s,n:[0,0,0,0,0,0,1],joints=lambda s:{},record=lambda p,s:None)
    guard=module.ContactGuard(m,'runtime::part_06',[.025]*3,['gripper_finger1_finger_tip_link','gripper_finger2_finger_tip_link'],None,N(entry_names=[],entry_values=[]),'tool')
    guard.bind_pile(f['objects'],f['binding']);guard.certify_pile(origin);guard.phase='closing'
    now[0]=last['wall_ns']/1e9+.001
    guard.check_pile(last)
    assert guard.pile_certificate['rejected']==[]
    # Increasing translation enough to move a corner beyond100um still fails.
    bad=copy.deepcopy(last);bad['poses']['a0::part_01'][0]+=.00015
    with pytest.raises(RuntimeError,match='neighbor geometry moved'):guard.check_pile(bad)


def test_corner_bound_remains_conservative_at_threshold():
    p=[0,0,0,0,0,0,1];size=[.025]*3
    assert module.box_motion_bound([.0000999,0,0,0,0,0,1],p,size)<.0001
    assert module.box_motion_bound([.0001,0,0,0,0,0,1],p,size)>.0001
    assert module.box_motion_bound([.000100001,0,0,0,0,0,1],p,size)>.0001
    rotated=[0,0,0,0,0,math.sin(.01/2),math.cos(.01/2)]
    assert module.box_motion_bound(rotated,p,size)>.0001


@pytest.mark.parametrize('quaternion', [[math.ulp(0.)]*3+[0.], [0.,0.,0.,0.], [0.,0.,0.,1.01]])
@pytest.mark.parametrize('which', [0,1])
def test_corner_bound_rejects_quaternions_outside_measured_domain(quaternion,which):
    poses=[[0.,0.,0.,0.,0.,0.,1.],[0.,0.,0.,0.,0.,0.,1.]]
    poses[which][3:]=quaternion
    with pytest.raises(RuntimeError,match='invalid BOX motion orientation'):
        module.box_motion_bound(*poses,[6.5e-5]*3)
