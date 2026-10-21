"""Measured execution guards: synthetic unit fixtures are never physical evidence."""
import sys, copy, json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
import pytest


def test_measured_attachment_supplies_actual_pose_without_duplicate_world_remove():
    from types import SimpleNamespace as N
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from simulator_execution import measured_attachment
    original=CollisionObject()
    original.id='runtime::part_06';original.header.frame_id='world'
    original.pose.orientation.w=1.
    original.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.025]*3)]
    original.primitive_poses=[Pose()]
    original.primitive_poses[0].orientation.w=1.
    measurements=N(frame=lambda s,f:[1.,2.,3.,0.,0.,0.,1.],
                   object_pose=lambda s,n:[1.01,2.02,3.03,0.,0.,0.,1.])
    diff=measured_attachment(original,dict(grasp_frame='tool',allowed_touch_links=['left','right']),measurements,{})
    # MoveIt processes attached ADD before world updates and consumes the
    # same-ID world object itself. A following REMOVE rejects the whole diff.
    assert diff.world.collision_objects==[]
    attached=diff.robot_state.attached_collision_objects[0]
    assert attached.object.id=='runtime::part_06'
    assert attached.link_name==attached.object.header.frame_id=='tool'
    assert attached.touch_links==['left','right']
    assert attached.object.primitives==original.primitives
    pose=attached.object.pose
    assert [pose.position.x,pose.position.y,pose.position.z]==pytest.approx([.01,.02,.03])
    assert pose.orientation.w==1.
    assert original.header.frame_id=='world' and original.pose.position.x==0.


def recorded_pile_guard(tmp_path, monkeypatch):
    """Replay the escaped five-neighbor state through real compiled FCL policy."""
    import xml.etree.ElementTree as ET
    from types import SimpleNamespace as N
    from simulator_execution import ContactGuard
    data=json.loads((Path(__file__).parent/'fixtures/stage_a1_pile_contacts.json').read_text())
    sample=data['sample']
    monkeypatch.setattr('simulator_execution.time.time',lambda:sample['wall_ns']/1e9+.001)
    monkeypatch.setattr('simulator_execution.time.time_ns',lambda:sample['wall_ns']+1000000)
    models=''.join(f'<model name="{o["object_id"]}"><link name="link"><collision name="collision"><geometry><box><size>0.025 0.025 0.025</size></box></geometry></collision></link></model>' for o in data['objects'])
    (tmp_path/'world.sdf').write_text('<sdf><world name="a0">'+models+'</world></sdf>')
    m=N(receipt=dict(world='a0',model='workcell_robot',run_id=sample['run_id'],pid=sample['pid']),
        receipt_path=tmp_path/'receipt.json',robot=ET.fromstring('<robot/>'),
        fresh=lambda:sample,object_pose=lambda s,n:s['poses']['a0::'+n],
        frame=lambda s,n:[0,0,0,0,0,0,1],joints=lambda s:{},record=lambda p,s:None)
    guard=ContactGuard(m,'runtime::part_06',[.025]*3,['left','right'],None,
                       N(entry_names=[],entry_values=[]),'tool')
    guard.bind_pile(data['objects'],data['binding'])
    return guard,sample,data


def provisional_pile_guard(tmp_path,monkeypatch):
    import threading
    import xml.etree.ElementTree as ET
    guard,sample,data=recorded_pile_guard(tmp_path,monkeypatch)
    sample['poses']['a0::part_06']=[0,0,0,0,0,0,1]
    sample['poses']['a0::part_00']=[-.025,0,0,0,0,0,1]
    sample['poses']['a0::part_07']=[0,.025,0,0,0,0,1]
    sample['contacts']=[]
    guard.phase='closing';guard.arm_names=set();guard.open_position=0.
    guard.m.lock=threading.RLock();guard.m.drain=lambda:[]
    guard.m.robot=ET.fromstring('<robot><ros2_control><joint name="leader"><command_interface name="position"/></joint></ros2_control></robot>')
    guard.m.joints=lambda s:{'leader':[.3,0.]}
    for side in ['left','right']:
        name=f'a0::workcell_robot::{side}::collision'
        sample['collisions'].append(name)
        sample['contacts'].append(dict(a='a0::part_06::link::collision',b=name,points=[[0,0,0]]))
    def contact(name,point):
        return dict(a='a0::part_06::link::collision',b=f'a0::part_{name}::link::collision',points=[point])
    def advance():
        sample['iteration']+=1;sample['sim_ns']+=1000000;sample['wall_ns']+=1000000
    sample['contacts'].append(contact('07',[0,.0125,0]))
    return guard,sample,advance,contact


def test_live_postclose_admission_carries_current_geometry_then_freezes(tmp_path,monkeypatch):
    guard,sample,advance,contact=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission()
    assert guard.pile_certificate['certified_set']==['runtime::part_07']
    advance();sample['contacts'].append(contact('00',[-.0125,0,0]));guard.check(sample)
    admitted_iteration=sample['iteration']
    # No repeated physical point is required while every fresh geometry query
    # still proves numerical contact. The neighbor itself does not move.
    sample['contacts']=sample['contacts'][:2]
    sample['poses']['a0::part_06'][1]=-.0002
    sample['poses']['a0::part_06'][2]=.00001
    for _ in range(300):advance();guard.check(sample)
    assert guard.pile_expired=={'runtime::part_07'}
    assert guard.pile_certificate['active_set']==['runtime::part_00']
    assert guard.pile_certificate['admissions'][1]['iteration']==admitted_iteration
    relative=guard.establish()
    assert len(relative)==7
    assert guard.pile_certificate['frozen'] is True
    assert guard.pile_certificate['certified_set']==['runtime::part_00']
    assert guard.held_start_sim_ns==sample['sim_ns']
    assert guard.pile_certificate['freeze']['iteration']==sample['iteration']
    assert guard.pile_certificate['freeze']['freshness_ms']==pytest.approx(1.)
    assert guard.pile_certificate['initial_bottom_m']==pytest.approx(-.01249)
    assert guard.pile_certificate['admissions'][0]['target_pose'][2]==0.
    assert guard.pile_certificate['rejected']==[]
    # Once frozen, even an exact, shallow newly measured pair cannot be added.
    advance();sample['poses']['a0::part_01']=[.025,-.0002,0,0,0,0,1]
    sample['contacts'].append(contact('01',[.0125,-.0002,0]))
    with pytest.raises(RuntimeError,match='new uncertified pile contact.*part_01'):guard.check(sample)
    failure=guard.pile_certificate['rejected_sample']
    assert failure['iteration']==sample['iteration']
    assert failure['contacts'][-1]['b']=='a0::part_01::link::collision'


@pytest.mark.parametrize('freeze',[False,True])
def test_provisional_expiry_is_irreversible_before_and_after_freeze(tmp_path,monkeypatch,freeze):
    guard,sample,advance,contact=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission()
    advance();sample['contacts']=sample['contacts'][:2]
    sample['poses']['a0::part_06'][1]=-.0002;guard.check(sample)
    assert guard.pile_expired=={'runtime::part_07'}
    if freeze:guard.establish()
    advance();sample['poses']['a0::part_06'][1]=0.
    sample['contacts'].append(contact('07',[0,.0125,0]))
    with pytest.raises(RuntimeError,match='recontact.*part_07'):guard.check(sample)
    assert guard.pile_certificate['rejected_sample']['iteration']==sample['iteration']


@pytest.mark.parametrize('bad',['stale','gap','deep','neighbor_moved'])
def test_provisional_admission_preserves_freshness_sequence_and_geometry(tmp_path,monkeypatch,bad):
    guard,sample,advance,contact=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();advance()
    if bad=='stale':monkeypatch.setattr('simulator_execution.time.time',lambda:sample['wall_ns']/1e9+.251)
    if bad=='gap':sample['iteration']+=1
    if bad=='deep':sample['poses']['a0::part_06'][1]+=.001
    if bad=='neighbor_moved':sample['poses']['a0::part_07'][0]+=.001
    with pytest.raises(RuntimeError):guard.check(sample)
    assert guard.pile_certificate['rejected_sample']['iteration']==sample['iteration']


def test_frozen_live_certificate_rejects_sequence_gap_and_preserves_sample(tmp_path,monkeypatch):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();advance();guard.check(sample);guard.establish()
    advance();sample['iteration']+=1
    with pytest.raises(RuntimeError,match='skipped measurement'):guard.check(sample)
    assert guard.pile_certificate['rejected_sample']['iteration']==sample['iteration']


def test_live_certificate_preserves_rejected_opposing_contact_sample(tmp_path,monkeypatch):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish();advance()
    sample['contacts']=sample['contacts'][1:]
    with pytest.raises(RuntimeError,match='retention lost'):guard.check(sample)
    assert guard.pile_certificate['rejected_sample']['contacts']==sample['contacts']


def test_checked_current_drains_pending_before_latest_retention_sample(tmp_path,monkeypatch):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish()
    pending=[]
    for _ in range(4):advance();pending.append(copy.deepcopy(sample))
    def drain():
        result=pending[:];pending.clear();return result
    guard.m.drain=drain
    current=guard.checked_current()
    assert current['iteration']==sample['iteration']
    assert pending==[]
    assert guard.pile_last_iteration==sample['iteration']
    # Retention uses that same serialized helper, never latest-before-queue.
    for _ in range(4):advance();pending.append(copy.deepcopy(sample))
    evidence=guard.retention_evidence(min_duration_ns=0)
    assert evidence['final_iteration']==sample['iteration']
    assert pending==[]
    assert guard.pile_certificate['rejected']==[]


def test_recorded_five_neighbor_certificate_uses_measured_geometry(tmp_path,monkeypatch):
    guard,sample,data=recorded_pile_guard(tmp_path,monkeypatch)
    evidence=guard.certify_pile(sample)
    assert evidence['certified_set']==['runtime::part_00','runtime::part_01','runtime::part_03','runtime::part_04','runtime::part_07']
    assert evidence['binding']==data['binding']
    assert evidence['iteration']==41679
    assert all(0<=c['geometry']['depth_m']<=.0001 for c in evidence['contacts'])
    assert guard.identity('a0::part_00::link::collision')=='runtime::part_00'
    with pytest.raises(RuntimeError,match='identity'):
        guard.identity('a0::part_00::other_link::collision')


SUPPORT_BINDING=dict(object_id='workcell::support_surface_table',model='pick_support',link='support_link',collision='support_collision')


def test_identity_maps_only_the_bound_support_collision_to_its_planning_object(tmp_path,monkeypatch):
    guard,_,_=recorded_pile_guard(tmp_path,monkeypatch)
    assert guard.identity('a0::pick_support::support_link::support_collision')=='workcell::pick_support'
    guard.m.receipt['support_geometry_binding']=dict(SUPPORT_BINDING)
    assert guard.identity('a0::pick_support::support_link::support_collision')=='workcell::support_surface_table'
    # Other geometry of the bound model, and other models, keep the conventional identity.
    assert guard.identity('a0::pick_support::support_link::other_collision')=='workcell::pick_support'
    assert guard.identity('a0::pick_support::other_link::support_collision')=='workcell::pick_support'
    assert guard.identity('a0::target_bin_physics::bin_link::floor')=='workcell::target_bin_physics'


def test_pile_contacts_exempt_the_resting_contact_on_the_bound_support(tmp_path,monkeypatch):
    # run-full-cycle-130349/04-stationary: part_04 pinched by both fingertips while still resting on
    # pick_support was rejected as 'uncertified target contact identity workcell::pick_support'.
    guard,sample,_=recorded_pile_guard(tmp_path,monkeypatch)
    guard.support=dict(object_id='runtime::part_06',support_id='workcell::support_surface_table',floor_z=0.,tool_link='tool0')
    support='a0::pick_support::support_link::support_collision'
    sample['collisions']+= [support,'a0::workcell_robot::left::collision','a0::workcell_robot::right::collision']
    sample['contacts']=[dict(a=support,b='a0::part_06::link::collision',points=[[0.41,-0.19,-6e-7],[0.41,-0.21,-3e-5]]),
        dict(a='a0::part_06::link::collision',b='a0::workcell_robot::left::collision',points=[[0,0,0]]),
        dict(a='a0::part_06::link::collision',b='a0::workcell_robot::right::collision',points=[[0,0,0]])]
    with pytest.raises(RuntimeError,match='uncertified target contact identity workcell::pick_support'):
        guard.pile_contacts(sample)
    guard.m.receipt['support_geometry_binding']=dict(SUPPORT_BINDING)
    assert guard.pile_contacts(sample)=={}
    # An unbound neighbor is still rejected with the binding present.
    sample['contacts'].append(dict(a='a0::part_06::link::collision',b='a0::target_bin_physics::bin_link::floor',points=[[0,0,0]]))
    sample['collisions'].append('a0::target_bin_physics::bin_link::floor')
    with pytest.raises(RuntimeError,match='uncertified target contact identity workcell::target_bin_physics'):
        guard.pile_contacts(sample)


def test_identity_binding_requires_object_id_and_exact_depth(tmp_path,monkeypatch):
    guard,_,_=recorded_pile_guard(tmp_path,monkeypatch)
    guard.m.receipt['support_geometry_binding']={k:v for k,v in SUPPORT_BINDING.items() if k!='object_id'}
    assert guard.identity('a0::pick_support::support_link::support_collision')=='workcell::pick_support'
    guard.m.receipt['support_geometry_binding']=dict(SUPPORT_BINDING)
    assert guard.identity('a0::pick_support::support_link::support_collision::extra')=='workcell::pick_support'


def test_carried_part_resting_on_bound_support_is_admitted_only_on_the_certified_floor(tmp_path,monkeypatch):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.m.receipt['support_geometry_binding']=dict(SUPPORT_BINDING)
    guard.support=dict(object_id='runtime::part_06',support_id='workcell::support_surface_table',floor_z=-.0125,tool_link='tool')
    guard.begin_pile_admission();guard.establish();advance()
    support='a0::pick_support::support_link::support_collision'
    sample['collisions'].append(support)
    rest=dict(a=support,b='a0::part_06::link::collision',points=[[0,0,-.0125],[.01,0,-.01251]])
    sample['contacts'].append(rest)
    guard.check(sample)  # held, both fingers, resting on the bound support at floor_z
    advance();rest['points'][1][2]=-.0123  # 0.2 mm above the certified floor
    with pytest.raises(RuntimeError,match='not on certified floor'):guard.check(sample)
    assert guard.pile_certificate['rejected']  # a rejection poisons the certificate: use a fresh guard below
    # Binding present but the planner bound a different support: the resting contact is not exempt.
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.m.receipt['support_geometry_binding']=dict(SUPPORT_BINDING)
    guard.support=dict(object_id='runtime::part_06',support_id='workcell::other_support',floor_z=-.0125,tool_link='tool')
    guard.begin_pile_admission();guard.establish();advance()
    sample['collisions'].append(support);sample['contacts'].append(dict(a=support,b='a0::part_06::link::collision',points=[[0,0,-.0125]]))
    with pytest.raises(RuntimeError,match='uncertified target contact identity workcell::support_surface_table'):guard.check(sample)


def test_contact_guard_refuses_acm_allowance_between_touch_links_and_bound_support(tmp_path,monkeypatch):
    from types import SimpleNamespace as N
    from simulator_execution import ContactGuard
    guard,_,_=recorded_pile_guard(tmp_path,monkeypatch)
    names=['left','workcell::support_surface_table']
    acm=N(entry_names=names,entry_values=[N(enabled=[False,True]),N(enabled=[True,False])])
    support=dict(object_id='runtime::part_06',support_id='workcell::support_surface_table',floor_z=0.,tool_link='tool')
    with pytest.raises(RuntimeError,match='SUPPORT_CONTACT_POLICY.*left'):
        ContactGuard(guard.m,'runtime::part_06',[.025]*3,['left','right'],support,acm,'tool')
    ContactGuard(guard.m,'runtime::part_06',[.025]*3,['left','right'],None,acm,'tool')  # no bound support: unchanged


DESTINATION_BINDING=dict(object_id='workcell::target_bin_default',model='target_bin_physics',link='bin_link',collision='floor',floor_z=.1)
BIN_FLOOR='a0::target_bin_physics::bin_link::floor'


def destination_guard(tmp_path,monkeypatch):
    guard,sample,advance,contact=provisional_pile_guard(tmp_path,monkeypatch)
    guard.m.receipt['destination_geometry_binding']=dict(DESTINATION_BINDING)
    guard.destination=dict(object_id=guard.object,support_id='workcell::target_bin_default',floor_z=.1)
    sample['collisions'].append(BIN_FLOOR)
    return guard,sample,advance


def test_identity_maps_the_bound_destination_floor_only(tmp_path,monkeypatch):
    guard,_,_=recorded_pile_guard(tmp_path,monkeypatch)
    assert guard.identity(BIN_FLOOR)=='workcell::target_bin_physics'
    guard.m.receipt['destination_geometry_binding']=dict(DESTINATION_BINDING)
    assert guard.identity(BIN_FLOOR)=='workcell::target_bin_default'
    assert guard.identity('a0::target_bin_physics::bin_link::wall_x_minus')=='workcell::target_bin_physics'


def test_destination_floor_contact_is_admitted_only_in_place_opening_released(tmp_path,monkeypatch):
    # run-full-cycle-160944: the placed part settled on the bin floor while the fingers opened and was
    # rejected as 'uncertified target contact identity workcell::target_bin_physics'.
    guard,sample,advance=destination_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish();advance()
    rest=dict(a=BIN_FLOOR,b='a0::part_06::link::collision',points=[[.44,.21,.1],[.46,.23,.09999]])
    def carried_away(guard,sample):  # the initial pile neighbour has separated, as it must before transfer
        sample['contacts']=[c for c in sample['contacts'] if 'part_07' not in c['b']];guard.pile_expired.add('runtime::part_07')
    carried_away(guard,sample);sample['contacts'].append(rest)
    guard.phase='transfer'
    with pytest.raises(RuntimeError,match='uncertified target contact identity workcell::target_bin_default'):guard.check(sample)
    guard,sample,advance=destination_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish();advance()
    carried_away(guard,sample);sample['contacts'].append(rest);guard.phase='opening'
    guard.check(sample)  # resting on the bound destination floor while the fingers still touch
    advance();rest['points'][1][2]=.1002  # 0.2 mm above the bound floor
    with pytest.raises(RuntimeError,match='not on the bound floor'):guard.check(sample)


def test_finger_contact_with_the_destination_floor_stays_unpermitted_while_opening(tmp_path,monkeypatch):
    guard,sample,advance=destination_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish();advance()
    sample['contacts']=[c for c in sample['contacts'] if 'part_07' not in c['b']];guard.pile_expired.add('runtime::part_07')
    sample['contacts'].append(dict(a=BIN_FLOOR,b='a0::workcell_robot::left::collision',points=[[.44,.21,.1]]))
    guard.phase='opening'
    with pytest.raises(RuntimeError,match='unpermitted physical contact'):guard.check(sample)


def test_pile_contacts_skip_the_destination_floor_in_place_phase_only(tmp_path,monkeypatch):
    guard,sample,advance=destination_guard(tmp_path,monkeypatch)
    sample['contacts'].append(dict(a=BIN_FLOOR,b='a0::part_06::link::collision',points=[[.44,.21,.1]]))
    guard.phase='place'
    assert 'workcell::target_bin_default' not in guard.pile_contacts(sample)
    guard.phase='lift'
    with pytest.raises(RuntimeError,match='uncertified target contact identity workcell::target_bin_default'):guard.pile_contacts(sample)


def test_compiled_predicate_rejects_destination_wall_normal_contact(tmp_path,monkeypatch):
    from simulator_execution import validate_measured_contacts
    from types import SimpleNamespace as N
    guard,_,_=recorded_pile_guard(tmp_path,monkeypatch)
    dest=dict(object_id=guard.object,support_id='workcell::target_bin_default',floor_z=.1)
    def contact(normal,z=.1):  # MoveIt reports the support first; the planner predicate's normal points support -> part
        return N(contact_body_1='workcell::target_bin_default',contact_body_2=guard.object,body_type_1=1,body_type_2=2,
                 position=N(x=.45,y=.22,z=z),normal=N(x=normal[0],y=normal[1],z=normal[2]),depth=.00001)
    validate_measured_contacts(N(valid=False,contacts=[contact((0.,0.,1.))]),None,True,guard.predicate,destination=dest)
    with pytest.raises(RuntimeError,match='destination contact rejected'):
        validate_measured_contacts(N(valid=False,contacts=[contact((1.,0.,0.))]),None,True,guard.predicate,destination=dest)
    with pytest.raises(RuntimeError,match='destination contact rejected'):
        validate_measured_contacts(N(valid=False,contacts=[contact((0.,0.,1.),z=.1003)]),None,True,guard.predicate,destination=dest)


def test_contact_guard_refuses_acm_allowance_between_touch_links_and_destination(tmp_path,monkeypatch):
    from types import SimpleNamespace as N
    from simulator_execution import ContactGuard
    guard,_,_=recorded_pile_guard(tmp_path,monkeypatch)
    guard.m.receipt['destination_geometry_binding']=dict(DESTINATION_BINDING)
    names=['right','workcell::target_bin_default']
    acm=N(entry_names=names,entry_values=[N(enabled=[False,True]),N(enabled=[True,False])])
    with pytest.raises(RuntimeError,match='SUPPORT_CONTACT_POLICY.*right'):
        ContactGuard(guard.m,'runtime::part_06',[.025]*3,['left','right'],None,acm,'tool')
    clean=N(entry_names=names,entry_values=[N(enabled=[False,False]),N(enabled=[False,False])])
    g=ContactGuard(guard.m,'runtime::part_06',[.025]*3,['left','right'],None,clean,'tool')
    assert g.destination==dict(object_id='runtime::part_06',support_id='workcell::target_bin_default',floor_z=.1)


def test_measured_fcl_destination_contact_uses_the_compiled_floor_predicate(tmp_path,monkeypatch):
    from simulator_execution import validate_measured_contacts
    from types import SimpleNamespace as N
    calls=[]
    def predicate(obj,sup,b1,b2,floor,point):
        calls.append((obj,sup,b1,b2,floor,list(point)));return floor==.1 and abs(point[2]-.1)<1e-4
    dest=dict(object_id='runtime::part_06',support_id='workcell::target_bin_default',floor_z=.1)
    def contact(z=.1,kind=2):
        return N(contact_body_1='runtime::part_06',contact_body_2='workcell::target_bin_default',body_type_1=kind,body_type_2=1,
                 position=N(x=.45,y=.22,z=z),normal=N(x=0.,y=0.,z=1.),depth=.00001)
    validate_measured_contacts(N(valid=False,contacts=[contact()]),None,True,predicate,destination=dest)
    assert calls[0][:2]==(b'runtime::part_06',b'workcell::target_bin_default') and calls[0][4]==.1
    with pytest.raises(RuntimeError,match='destination contact rejected'):
        validate_measured_contacts(N(valid=False,contacts=[contact(z=.105)]),None,True,predicate,destination=dest)
    with pytest.raises(RuntimeError,match='measured carried/robot collision'):
        validate_measured_contacts(N(valid=False,contacts=[contact()]),None,True,predicate,destination=None)
    with pytest.raises(RuntimeError,match='measured carried/robot collision'):
        validate_measured_contacts(N(valid=False,contacts=[contact(kind=1)]),None,True,predicate,destination=dest)


def test_pile_certificate_rejects_unknown_identity_stale_or_deep_contact(tmp_path,monkeypatch):
    guard,sample,_=recorded_pile_guard(tmp_path,monkeypatch)
    for change in ('unknown','stale','deep','point','preclose'):
        bad=copy.deepcopy(sample)
        if change=='unknown':bad['contacts'][0]['a']='a0::intruder::link::collision'
        if change=='stale':bad['wall_ns']-=300000000
        if change=='deep':bad['poses']['a0::part_06'][2]-=.001
        if change=='point':bad['contacts'][0]['points'][0][0]+=.01
        if change=='preclose':guard.pile_binding['close_goal_terminal_wall_ns']=sample['wall_ns']+1
        with pytest.raises(RuntimeError):guard.certify_pile(bad)
        assert guard.pile_certificate['rejected']


@pytest.mark.parametrize('phase',['closing','lift'])
def test_pile_allowance_expires_and_cannot_recapture_contact(tmp_path,monkeypatch,phase):
    guard,sample,_=recorded_pile_guard(tmp_path,monkeypatch)
    guard.certify_pile(sample)
    guard.phase=phase
    if phase=='lift':guard.begin_pile_separation(sample)
    separated=copy.deepcopy(sample);separated['iteration']+=1;separated['sim_ns']+=1000000
    separated['poses']['a0::part_06'][2]+=.001;separated['contacts']=[]
    guard.check_pile(separated)
    # The four lower supports clear after 1 mm. The same-height side neighbor
    # remains touching in measured FCL geometry and MUST NOT expire yet.
    assert guard.pile_expired=={'runtime::part_00','runtime::part_01','runtime::part_03','runtime::part_04'}
    with pytest.raises(RuntimeError,match='recontact'):guard.check_pile(sample)


@pytest.mark.parametrize('delta',[(.003,0,0),(0,0,-.00001),(0,0,.011)])
def test_pile_departure_preserves_bounded_monotonic_corridor(tmp_path,monkeypatch,delta):
    guard,sample,_=recorded_pile_guard(tmp_path,monkeypatch)
    guard.certify_pile(sample);guard.phase='lift';guard.begin_pile_separation(sample)
    moved=copy.deepcopy(sample)
    moved['poses']['a0::part_06'][:3]=[a+b for a,b in zip(moved['poses']['a0::part_06'][:3],delta)]
    with pytest.raises(RuntimeError):guard.check_pile(moved)


def test_pile_certificate_rechecks_current_depth_and_neighbor_motion(tmp_path,monkeypatch):
    guard,sample,_=recorded_pile_guard(tmp_path,monkeypatch)
    guard.certify_pile(sample)
    bad=copy.deepcopy(sample);bad['poses']['a0::part_00'][2]+=.001
    with pytest.raises(RuntimeError):guard.check_pile(bad)


def test_measured_fcl_pile_exception_requires_exact_active_pair_and_depth(tmp_path,monkeypatch):
    from simulator_execution import validate_measured_contacts
    from types import SimpleNamespace as N
    guard,sample,_=recorded_pile_guard(tmp_path,monkeypatch)
    guard.certify_pile(sample)
    def contact(neighbor='runtime::part_00',depth=.00001,kind=2):
        return N(contact_body_1=guard.object,contact_body_2=neighbor,body_type_1=kind,body_type_2=1,
                 position=N(x=0.,y=0.,z=.025),normal=N(x=0.,y=0.,z=-1.),depth=depth)
    validate_measured_contacts(N(valid=False,contacts=[contact()]),None,False,pile_guard=guard)
    for c in (contact('runtime::intruder'),contact(depth=.000101),contact(kind=0)):
        with pytest.raises(RuntimeError):
            validate_measured_contacts(N(valid=False,contacts=[c]),None,False,pile_guard=guard)
    guard.pile_expired.add('runtime::part_00')
    with pytest.raises(RuntimeError):
        validate_measured_contacts(N(valid=False,contacts=[contact()]),None,False,pile_guard=guard)


def test_measurement_rejects_stale_wrong_run_and_missing_samples():
    from simulator_execution import validate_sample
    sample=dict(run_id='r',pid=12,iteration=10,sim_ns=10000000,wall_ns=1000000000,
                poses={'cell::part':[0,0,0,0,0,0,1]},joints={'cell::leader':[.3,0]},contacts=[],collisions=['cell::part::body::shape'])
    receipt=dict(run_id='r',pid=12)
    validate_sample(sample,receipt,1.01,9)
    for patch in [dict(run_id='wrong'),dict(pid=13),dict(iteration=8),dict(wall_ns=0),dict(poses={}),dict(error='broken')]:
        with pytest.raises(RuntimeError):validate_sample(dict(sample,**patch),receipt,1.01,9)


def test_support_expiry_and_non_monotonic_separation():
    from simulator_execution import Separation
    p=Separation([0,0,.01,0,0,0,1],0)
    p.check([0,0,.01001,0,0,0,1],0,True)
    p.check([0,0,.011,0,0,0,1],.001,False)
    assert p.expired
    with pytest.raises(RuntimeError):p.check([0,0,.011,0,0,0,1],.001,True)
    for pose in ([.003,0,.01,0,0,0,1],[0,0,.009,0,0,0,1]):
        with pytest.raises(RuntimeError):Separation([0,0,.01,0,0,0,1],0).check(pose,0,True)


def test_separation_descent_is_bounded_by_support_tolerance_from_running_maximum():
    from simulator_execution import Separation,SUPPORT_CONTACT_TOLERANCE_M
    assert SUPPORT_CONTACT_TOLERANCE_M==1e-4
    # run-stationary-152227: the close lifted part_04 12.9 um, then it settled 1e-8 m.
    p=Separation([.4,-.2,.0125,0,0,0,1],0)
    p.check([.4,-.2,.0125+1.211e-5,0,0,0,1],1.211e-5,False)
    p.check([.4,-.2,.0125+1.210e-5,0,0,0,1],1.210e-5,False)
    assert not p.expired and p.height==pytest.approx(1.211e-5)
    # The running maximum never moves down: a sample 0.99e-4 below it passes
    # (it would also pass previous-sample semantics) but leaves the maximum
    # unchanged, and 1.01e-4 below the maximum fails although it is only
    # 0.02e-4 below the previous sample.
    p.check([.4,-.2,.0125+1.211e-5-.99e-4,0,0,0,1],0,True)
    assert p.height==pytest.approx(1.211e-5)
    with pytest.raises(RuntimeError,match='corridor'):p.check([.4,-.2,.0125+1.211e-5-1.01e-4,0,0,0,1],0,True)
    # First sample: at most the tolerance below the origin is admitted.
    Separation([0,0,.01,0,0,0,1],0).check([0,0,.01-.99e-4,0,0,0,1],0,True)
    with pytest.raises(RuntimeError,match='corridor'):Separation([0,0,.01,0,0,0,1],0).check([0,0,.01-1.01e-4,0,0,0,1],0,True)
    # Slow sagging cannot accumulate: each step is inside the band, the total is not.
    q=Separation([0,0,.01,0,0,0,1],0);q.check([0,0,.01005,0,0,0,1],.00005,True)
    q.check([0,0,.01005-.6e-4,0,0,0,1],0,True)
    with pytest.raises(RuntimeError,match='corridor'):q.check([0,0,.01005-1.2e-4,0,0,0,1],0,True)
    # The other corridor limits are unchanged.
    for pose in ([.0026,0,.01,0,0,0,1],[0,0,.0201,0,0,0,1],[0,0,.01,0,0,.0051,.99998699]):
        with pytest.raises(RuntimeError,match='corridor'):Separation([0,0,.01,0,0,0,1],0).check(pose,0,True)


def test_grasp_requires_two_contacting_fingers_and_rejects_slip():
    from simulator_execution import HeldObject
    pose=[0,0,0,0,0,0,1]
    for fingers in ([],['left']):
        with pytest.raises(RuntimeError):HeldObject(pose,pose,fingers,['left','right'],.3,0)
    held=HeldObject(pose,pose,['left','right'],['left','right'],.3,0)
    held.check([0,0,.1,0,0,0,1],[0,0,.1,0,0,0,1],['left','right'])
    with pytest.raises(RuntimeError):held.check([0,0,.1,0,0,0,1],[.004,0,.1,0,0,0,1],['left','right'])
    with pytest.raises(RuntimeError):held.check(pose,pose,['left'])


def test_cancel_waits_for_terminal_and_measured_stop_then_reconciles():
    from simulator_execution import cancel_owned
    events=[]
    cancel_owned(lambda:events.append('cancel') or True,lambda:events.append('terminal') or True,
                 lambda:events.append('stop') or True,lambda:events.append('revoke'),lambda:events.append('reconcile'))
    assert events==['cancel','terminal','stop','revoke','reconcile']
    events=[]
    with pytest.raises(RuntimeError):
        cancel_owned(lambda:True,lambda:True,lambda:False,lambda:events.append('revoke'),lambda:events.append('reconcile'))
    assert events==['revoke']


def test_runtime_uses_same_compiled_floor_predicate():
    import ctypes
    from ament_index_python.packages import get_package_prefix
    lib=ctypes.CDLL(str(Path(get_package_prefix('workcell_builder'))/'lib/libworkcell_support_contact.so'))
    f=lib.workcell_support_contact_valid
    f.argtypes=[ctypes.c_char_p]*4+[ctypes.c_double,ctypes.POINTER(ctypes.c_double)];f.restype=ctypes.c_bool
    def valid(a='floor',b='part',p=(0,0,0,0,0,1,.00001)):
        return f(b'part',b'floor',a.encode(),b.encode(),0,(ctypes.c_double*7)(*p))
    assert valid()
    assert not valid(a='wrong_support')
    assert not valid(b='unrelated')
    assert not valid(p=(0,0,0,1,0,0,.00001))
    assert not valid(p=(0,0,.1,0,0,1,.00001))
    assert not valid(p=(0,0,0,0,0,1,.001))


def test_failed_closure_and_cancel_do_not_authorize_full_cycle(tmp_path):
    from simulator_execution import require_trial_evidence
    p=tmp_path/'trials.json';p.write_text('[{"result":"FAIL"}]')
    with pytest.raises(RuntimeError):require_trial_evidence(p,{})


def test_live_collision_query_keeps_carried_collisions_and_expires_support():
    from simulator_execution import validate_measured_contacts
    from types import SimpleNamespace as N
    def contact(a='support',b='part',z=0,nz=1,depth=.00001):
        return N(contact_body_1=a,contact_body_2=b,body_type_1=1,body_type_2=2,position=N(x=0,y=0,z=z),normal=N(x=0,y=0,z=nz),depth=depth)
    response=N(valid=False,contacts=[contact()])
    policy=dict(object_id='part',support_id='support',floor_z=0)
    validate_measured_contacts(response,policy,False)
    for contacts in ([contact(a='wall')],[contact(z=.1)],[contact(nz=0)],[contact(b='other')],[]):
        with pytest.raises(RuntimeError):validate_measured_contacts(N(valid=False,contacts=contacts),policy,False)
    with pytest.raises(RuntimeError):validate_measured_contacts(response,policy,True)


def test_bridge_publisher_identity_is_node_not_executable_name():
    from simulator_execution import validate_publisher
    from types import SimpleNamespace as N
    validate_publisher([N(node_name='ros_gz_bridge',node_namespace='/')])
    for pubs in ([],[N(node_name='parameter_bridge',node_namespace='/')],
                 [N(node_name='ros_gz_bridge',node_namespace='/')]*2):
        with pytest.raises(RuntimeError):validate_publisher(pubs)


def test_controlled_cancel_requires_measured_arm_movement_after_goal_acceptance():
    from simulator_execution import CancellationMotion
    start=dict(iteration=10,sim_ns=100000000,wall_ns=1000000000)
    trial=CancellationMotion('owned',start,{'arm':[0.,0.]},{'arm'},accepted_wall_ns=1100000000)
    assert not trial.observe(dict(start,iteration=11,sim_ns=200000000),{'arm':[.003,.01]})
    assert not trial.observe(dict(start,iteration=12,sim_ns=210000000,wall_ns=1110000000),{'arm':[.002,0.]})
    assert trial.observe(dict(start,iteration=13,sim_ns=220000000,wall_ns=1120000000),{'arm':[.003,.01]})
    assert trial.evidence['movement']['joints']['arm']==[.003,.01]
    assert trial.evidence['movement']['max_displacement_rad']==pytest.approx(.003)
    with pytest.raises(RuntimeError,match='missing'):
        CancellationMotion('owned',start,{'gripper':[0.,0.]},{'arm'})


def test_cancel_acknowledgement_must_name_the_owned_goal():
    from simulator_execution import cancel_response_matches
    from types import SimpleNamespace as N
    response=lambda ids:N(return_code=0,goals_canceling=[N(goal_id=N(uuid=i)) for i in ids])
    owned=[1]*16
    assert cancel_response_matches(owned,response([owned]))
    assert not cancel_response_matches(owned,response([[2]*16]))
    assert not cancel_response_matches(owned,response([]))
    assert not cancel_response_matches(owned,N(return_code=1,goals_canceling=response([owned]).goals_canceling))


def test_stop_window_requires_complete_contiguous_sustained_measured_stop():
    from simulator_execution import StopWindow
    window=StopWindow({'arm','gripper'})
    def sample(i,ns):return dict(iteration=i,sim_ns=ns,wall_ns=1000000000+ns)
    stopped={'arm':[.3,0.],'gripper':[0.,0.]}
    assert not window.observe(sample(1,0),stopped)
    assert not window.observe(sample(2,200000000),stopped)
    assert not window.observe(sample(3,300000000),dict(stopped,arm=[.3,.01]))
    assert not window.observe(sample(4,400000000),stopped)
    assert window.observe(sample(5,700000000),stopped)
    assert window.evidence['duration_sim_ns']==300000000
    assert len(window.evidence['samples'])==2
    with pytest.raises(RuntimeError,match='missing'):StopWindow({'arm'}).observe(sample(1,0),{})
    with pytest.raises(RuntimeError,match='gap'):window.observe(sample(7,800000000),stopped)


def test_cancellation_acceptance_requires_movement_canceled_stop_and_alive_cleanup():
    from simulator_execution import require_cancellation_acceptance
    accepted=dict(cancellation_confirmed=True,cancellation_accepted=True,cancellation_movement_verified=True,
        interrupted_action_terminal_status=5,motion_stop_verified=True,
        controller_cancellation={'arm':dict(uuid='abc',executing_before_cancel=True,status=5,error_code=0,result_wall_ns=12)},
        owned_execution_goal=dict(uuid='01',accepted=True),
        cancellation_motion=dict(movement={'iteration':12}),
        stopped_window=dict(duration_sim_ns=300000000,samples=[{},{}]),
        measured_reconciliation=dict(acm_restored=True,measured_geometry_matches=True,held=False,attached_ids=[]),
        recovery_scene=dict(contact_acm_restored=True,attached_ids=[]))
    require_cancellation_acceptance(accepted)
    for patch in (dict(cancellation_movement_verified=False),dict(interrupted_action_terminal_status=4),
                  dict(cancellation_accepted=False),dict(motion_stop_verified=False),
                  dict(recovery_inspection_failure='service unavailable'),
                  dict(measured_reconciliation=dict(acm_restored=True,measured_geometry_matches=False,held=False,attached_ids=[])),
                  dict(recovery_scene=dict(contact_acm_restored=False,attached_ids=[]))):
        with pytest.raises(RuntimeError):require_cancellation_acceptance(dict(accepted,**patch))


def test_early_successful_approach_cannot_fall_through_cancel_trial():
    """Exercise the actual nested action owner with an immediately ended action."""
    import ast,time
    from types import SimpleNamespace as N
    tree=ast.parse((Path(__file__).parents[1]/'scripts/perceived_object_grasp_execute.py').read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    from contextlib import nullcontext
    from perceived_object_grasp_execute import PlanningCrossCheckGate
    owner=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='action')
    def future(result):return N(done=lambda:True,result=lambda:result)
    terminal=future(N(status=4,result=N(error_code=N(val=1))))
    handle=N(accepted=True,goal_id=N(uuid=[1]*16),get_result_async=lambda:terminal,
             cancel_goal_async=lambda:future(N(return_code=3,goals_canceling=[])))
    client=N(wait_for_server=lambda **kw:True,send_goal_async=lambda goal:future(handle))
    sample=dict(iteration=1,sim_ns=100,wall_ns=time.time_ns())
    summary={};events=[]
    context=dict(time=time,node=None,summary=summary,measurements=N(fresh=lambda:sample,joints=lambda s:{'arm':[0.,0.]}),
        execute_client=client,plan_client=object(),controlled_cancel=True,controller_audit=None,contract={'home_joint_names':['arm']},
        apply_client=(scene:=object()),exchanged_clients={scene},
        cross_check_gate=PlanningCrossCheckGate({}),nullcontext=nullcontext,
        rclpy=N(spin_until_future_complete=lambda *args,**kw:None),execution_monitor=lambda:None,
        wait_stopped=lambda:True,apply=lambda diff:events.append('revoke'),PlanningScene=N,baseline=N(),
        measured_reconcile=lambda:events.append('reconcile'))
    exec(compile(ast.Module(body=[owner],type_ignores=[]),'<actual-action-owner>','exec'),context)
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory
    goal=ExecuteTrajectory.Goal(trajectory=RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=['arm'])))
    with pytest.raises(RuntimeError,match='ENDED_BEFORE_CANCEL'):context['action'](client,goal,1)
    assert summary['owned_execution_goal']['accepted']
    assert summary['interrupted_action_terminal_status']==4
    assert summary['cancellation_confirmed'] is False
    assert events==['revoke']


def test_cancellation_requires_independent_controller_terminal():
    from simulator_execution import validate_controller_cancellation
    controller=dict(uuid='abc',executing_before_cancel=True,status=5,error_code=0,result_wall_ns=12)
    validate_controller_cancellation({'arm':controller})
    for bad in ({},{'arm':dict(controller,status=4)},{'arm':dict(controller,executing_before_cancel=False)}):
        with pytest.raises(RuntimeError):validate_controller_cancellation(bad)


def test_acquisition_gap_is_latched_even_if_following_samples_are_valid():
    import threading,time,json
    from types import SimpleNamespace as N
    from simulator_execution import Measurements
    m=Measurements.__new__(Measurements);m.lock=threading.RLock();m.receipt=dict(run_id='r',pid=12)
    m.latest=None;m.pending=[];m.previous=1;m.armed=True;m.error=None;m.timing=[]
    def send(i):
        m.update(N(data=json.dumps(dict(run_id='r',pid=12,iteration=i,sim_ns=i,wall_ns=time.time_ns(),
            poses={'p':[0,0,0,0,0,0,1]},joints={'j':[0,0]},contacts=[],collisions=['c']))))
    send(3);send(2)
    with pytest.raises(RuntimeError,match='skipped'):m.fresh()


def test_cancel_metrics_include_motion_during_controller_result_wait():
    from simulator_execution import cancellation_metrics
    from types import SimpleNamespace as N
    anchor=dict(iteration=1,sim_ns=0,wall_ns=1000000000,joints={'arm':[0.,.1]})
    samples=[dict(iteration=i,sim_ns=(i-1)*100000000,wall_ns=1000000000+(i-1)*100000000,
                  joints={'arm':[.02,0.]}) for i in range(2,7)]
    summary=dict(cancellation_request={'wall_ns':1010000000},cancel_measurement=anchor,
        cancellation_motion={'accepted':{'joints':{'arm':[0.,.1]}}},
        stopped_window={'samples':[samples[2]]},
        controller_cancellation={'arm':{'result_wall_ns':1300000000,'terminal_status_receive_wall_ns':1250000000}})
    result=cancellation_metrics(summary,N(joints=lambda s:s['joints']),samples)
    assert result['cancel_to_measured_stop_ms']==90
    assert result['cancel_to_controller_result_ms']['arm']==240
    assert result['additional_joint_travel_rad']['arm']==pytest.approx(.02)


def test_motion_telemetry_metrics_keep_250ms_guard_and_contiguous_iterations():
    from simulator_execution import telemetry_metrics
    timing=[]
    for i in range(1,6):
        source=1_000_000_000+i*10_000_000
        timing.append(dict(event='receive',iteration=i,sim_ns=i*10_000_000,
            source_wall_ns=source,publish_wall_ns=source+1_000_000,
            source_serialization_ns=500_000,receive_wall_ns=source+2_000_000,
            callback_ns=200_000,error=None))
        timing.append(dict(event='fresh',iteration=i,read_wall_ns=source+3_000_000,
            age_ns=3_000_000,lock_wait_ns=10_000,error=None))
    result=telemetry_metrics(timing)
    assert result['samples']==5
    assert result['max_fresh_age_ms']==pytest.approx(3.)
    assert result['max_delivery_ms']==pytest.approx(2.)
    assert result['real_time_factor']==pytest.approx(1.)
    stale=copy.deepcopy(timing)
    stale[-1]['age_ns']=250_000_000
    with pytest.raises(RuntimeError,match='250 ms'):telemetry_metrics(stale)
    gap=copy.deepcopy(timing)
    gap[4]['iteration']=4
    with pytest.raises(RuntimeError,match='iteration gap'):telemetry_metrics(gap)


def test_full_cycle_prerequisites_require_all_four_qualified_trials(tmp_path):
    from simulator_execution import require_trial_evidence
    capability_sha='freshly-qualified-build'
    overlay_sha='freshly-qualified-tem-overlay'
    move_group_sha='freshly-qualified-move-group'
    current=dict(sha256=capability_sha,moveit_overlay={'library':{'sha256':overlay_sha},
        'move_group':{'executable':{'sha256':move_group_sha}}})
    common=dict(commissioning_capability=current,
        backend_identity={'backend':'simulator'},
        motion_backend_identity={'backend':'simulator'},
        measured_reconciliation={'acm_restored':True,'attached_ids':[],'measured_geometry_matches':True,'held':False},
        recovery_scene={'contact_acm_restored':True,'attached_ids':[]})
    cancellation=dict(common,result='CANCELLATION_TRIAL_PASS',
        cancellation_confirmed=True,cancellation_accepted=True,cancellation_movement_verified=True,
        interrupted_action_terminal_status=5,motion_stop_verified=True,
        controller_cancellation={'arm':dict(uuid='abc',executing_before_cancel=True,status=5,error_code=0,result_wall_ns=12)},
        owned_execution_goal=dict(uuid='01',accepted=True),
        cancellation_motion=dict(movement={'iteration':12}),
        stopped_window=dict(duration_sim_ns=300000000,samples=[{},{}]))
    telemetry=dict(common,result='MOTION_TELEMETRY_PASS',
        motion_telemetry={'max_fresh_age_ms':12.,'max_delivery_ms':8.})
    retention=dict(common,result='STATIONARY_RETENTION_PASS',
        stationary_retention={'duration_sim_ns':1_100_000_000,'samples':50})
    contact=dict(common,result='CONTACT_RELEASE_PASS',verified_lift_clearance_m=.02,
        release_evidence={'settled':True})
    records=[copy.deepcopy(r) for r in (cancellation,telemetry,retention,contact)]
    path=tmp_path/'evidence.json';path.write_text(json.dumps(records))
    accepted=require_trial_evidence(path,current)
    assert accepted['capability_sha256']==capability_sha
    assert accepted['moveit_overlay_sha256']==overlay_sha
    assert accepted['move_group_sha256']==move_group_sha
    for index in range(4):
        for invalid_overlay in ({'library':{'sha256':'different-tem-overlay'}},
                {'library':{'sha256':''}}, {'library':{}}, {}):
            invalid=copy.deepcopy(records)
            invalid[index]['commissioning_capability']['moveit_overlay']=invalid_overlay
            path.write_text(json.dumps(invalid))
            with pytest.raises(RuntimeError,match='MoveIt overlay'):
                require_trial_evidence(path,current)
        invalid=copy.deepcopy(records)
        del invalid[index]['commissioning_capability']['moveit_overlay']
        path.write_text(json.dumps(invalid))
        with pytest.raises(RuntimeError,match='MoveIt overlay'):
            require_trial_evidence(path,current)
    path.write_text(json.dumps(records))
    for invalid_current in ({'sha256':capability_sha},
            dict(current,moveit_overlay={'library':{'sha256':'stale-tem-overlay'}}),
            dict(current,moveit_overlay={'library':{'sha256':''}})):
        with pytest.raises(RuntimeError,match='MoveIt overlay'):
            require_trial_evidence(path,invalid_current)
    for index in range(4):
        for invalid_executable in ({'executable':{'sha256':'different-move-group'}},
                {'executable':{'sha256':''}}, {'executable':{}}, {}):
            invalid=copy.deepcopy(records)
            invalid[index]['commissioning_capability']['moveit_overlay']['move_group']=invalid_executable
            path.write_text(json.dumps(invalid))
            with pytest.raises(RuntimeError,match='MoveGroup executable'):
                require_trial_evidence(path,current)
        invalid=copy.deepcopy(records)
        del invalid[index]['commissioning_capability']['moveit_overlay']['move_group']
        path.write_text(json.dumps(invalid))
        with pytest.raises(RuntimeError,match='MoveGroup executable'):
            require_trial_evidence(path,current)
    path.write_text(json.dumps(records))
    for invalid_executable in ({'executable':{'sha256':'stale-move-group'}},
            {'executable':{'sha256':''}}, {'executable':{}}, {}):
        invalid_current=copy.deepcopy(current)
        invalid_current['moveit_overlay']['move_group']=invalid_executable
        with pytest.raises(RuntimeError,match='MoveGroup executable'):
            require_trial_evidence(path,invalid_current)
    invalid_current=copy.deepcopy(current)
    del invalid_current['moveit_overlay']['move_group']
    with pytest.raises(RuntimeError,match='MoveGroup executable'):
        require_trial_evidence(path,invalid_current)
    path.write_text(json.dumps([cancellation,telemetry,retention]))
    with pytest.raises(RuntimeError):require_trial_evidence(path,current)


def test_same_attempt_retention_proof_is_bound_to_actual_frozen_interval(tmp_path,monkeypatch):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish()
    start=copy.deepcopy(sample);frozen=copy.deepcopy(guard.pile_certificate['freeze'])
    for _ in range(1000):advance();guard.check(sample)
    proof=guard.retention_evidence()
    assert proof['run_id']==sample['run_id']
    assert proof['target']==guard.object
    assert proof['binding']==guard.pile_binding
    assert proof['binding']['close_goal_uuid']
    assert proof['binding']['resolution_sha256']
    assert proof['start_iteration']==start['iteration']
    assert proof['start_sim_ns']==start['sim_ns']
    assert proof['start_wall_ns']==start['wall_ns']
    assert proof['end_iteration']==proof['final_iteration']==sample['iteration']
    assert proof['end_sim_ns']==proof['final_sim_ns']==sample['sim_ns']
    assert proof['end_wall_ns']==sample['wall_ns']
    assert proof['duration_sim_ns']==1_000_000_000
    assert proof['required_contact_links']==proof['contact_links']==['left','right']
    assert guard.pile_certificate['freeze']==frozen
    assert guard.pile_certificate['certified_set']==['runtime::part_07']
    proof['binding']['close_goal_uuid']='foreign-close'
    assert guard.retention_evidence()['binding']['close_goal_uuid']==guard.pile_binding['close_goal_uuid']


@pytest.mark.parametrize('bad',['another_run','stale','changed_close','changed_resolution'])
def test_live_retention_rejects_foreign_stale_or_changed_attempt(tmp_path,monkeypatch,bad):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish()
    for _ in range(1000):advance();guard.check(sample)
    if bad=='another_run':sample['run_id']='foreign-run'
    elif bad=='stale':monkeypatch.setattr('simulator_execution.time.time',lambda:sample['wall_ns']/1e9+.251)
    elif bad=='changed_close':guard.pile_binding['close_goal_uuid']='foreign-close'
    else:guard.pile_binding['resolution_sha256']='foreign-resolution'
    with pytest.raises(RuntimeError):guard.retention_evidence()


@pytest.mark.parametrize('bad',['contact_loss','translation_slip','rotation_slip'])
def test_failed_current_hold_cannot_later_produce_retention_proof(tmp_path,monkeypatch,bad):
    import math
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    # Isolate the held-object limits from pile geometry; exact fingertips remain required.
    sample['contacts']=sample['contacts'][:2]
    guard.begin_pile_admission();guard.establish()
    for _ in range(500):advance();guard.check(sample)
    good=copy.deepcopy(sample)
    advance()
    if bad=='contact_loss':sample['contacts']=sample['contacts'][:1]
    elif bad=='translation_slip':sample['poses']['a0::part_06'][0]=.0021
    else:sample['poses']['a0::part_06'][3:]=[0,0,math.sin(.011/2),math.cos(.011/2)]
    with pytest.raises(RuntimeError,match='retention lost'):guard.check(sample)
    assert guard.pile_certificate['rejected_sample']['iteration']==sample['iteration']
    sample['contacts']=good['contacts'];sample['poses']=good['poses']
    for _ in range(500):advance()
    with pytest.raises(RuntimeError,match='missing valid pile certificate'):guard.retention_evidence()


def test_current_grasp_retention_cannot_reuse_a_shorter_hold(tmp_path,monkeypatch):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish()
    for _ in range(999):advance();guard.check(sample)
    with pytest.raises(RuntimeError,match='duration is too short'):guard.retention_evidence()


def physics_pose_sample():
    """Synthetic declared-source contract, not relabelled historical physics data."""
    method='physics_link_frame_data_at_offset'
    receipt=dict(run_id='physics-run',pid=12,world='a0',measurement_pose_source=method)
    sample=dict(run_id='physics-run',pid=12,iteration=10,sim_ns=10_000_000,wall_ns=1_000_000_000,
        poses={'a0::part':[0,0,0,0,0,0,1],'a0::part::link':[0,0,0,0,0,0,1]},
        joints={'a0::robot::j':[0,0]},collisions=['collision'],contacts=[],
        pose_source=dict(method=method,frame='world',query_iteration=10,query_sim_ns=10_000_000,
                         gazebo_version='6.17.0',read_only=True),
        pose_entities={'a0::part':dict(entity=2,link_entity=3,query_entity=5,kind='model'),
                       'a0::part::link':dict(entity=3,link_entity=3,query_entity=5,kind='link')})
    return sample,receipt


def test_authoritative_physics_pose_source_accepts_complete_current_identity():
    from simulator_execution import validate_sample
    sample,receipt=physics_pose_sample()
    validate_sample(sample,receipt,1.01,9)


@pytest.mark.parametrize('bad',['missing_source','wrong_method','wrong_receipt_method','wrong_frame',
    'writable','missing_version','stale_iteration','stale_sim_time','missing_entities','missing_link',
    'extra_entity','zero_entity','wrong_link_identity','wrong_kind','foreign_world','query_error','stale_wall',
    'wrong_model_link','wrong_model_query','duplicate_entity','shared_link_query','query_is_original'])
def test_authoritative_physics_pose_source_fails_closed(tmp_path,bad):
    from simulator_execution import validate_sample
    sample,receipt=physics_pose_sample()
    if bad=='missing_source':sample.pop('pose_source')
    elif bad=='wrong_method':sample['pose_source']['method']='cached_ecs'
    elif bad=='wrong_receipt_method':receipt['measurement_pose_source']='cached_ecs'
    elif bad=='wrong_frame':sample['pose_source']['frame']='model'
    elif bad=='writable':sample['pose_source']['read_only']=False
    elif bad=='missing_version':sample['pose_source']['gazebo_version']=''
    elif bad=='stale_iteration':sample['pose_source']['query_iteration']-=1
    elif bad=='stale_sim_time':sample['pose_source']['query_sim_ns']-=1
    elif bad=='missing_entities':sample.pop('pose_entities')
    elif bad=='missing_link':sample['pose_entities'].pop('a0::part::link')
    elif bad=='extra_entity':sample['pose_entities']['a0::extra']=sample['pose_entities']['a0::part']
    elif bad=='zero_entity':sample['pose_entities']['a0::part']['query_entity']=0
    elif bad=='wrong_link_identity':sample['pose_entities']['a0::part::link']['link_entity']=20
    elif bad=='wrong_kind':sample['pose_entities']['a0::part']['kind']='visual'
    elif bad=='wrong_model_link':sample['pose_entities']['a0::part']['link_entity']=20
    elif bad=='wrong_model_query':sample['pose_entities']['a0::part']['query_entity']=20
    elif bad=='duplicate_entity':sample['pose_entities']['a0::part']['entity']=3
    elif bad=='shared_link_query':
        sample['poses']['a0::part::other']=[0,0,0,0,0,0,1]
        sample['pose_entities']['a0::part::other']=dict(entity=6,link_entity=6,query_entity=5,kind='link')
    elif bad=='query_is_original':
        for entity in sample['pose_entities'].values():entity['query_entity']=2
    elif bad=='foreign_world':
        sample['poses']['other::part']=sample['poses'].pop('a0::part')
        sample['pose_entities']['other::part']=sample['pose_entities'].pop('a0::part')
    elif bad=='query_error':sample['error']='physics pose query unavailable'
    elif bad=='stale_wall':sample['wall_ns']=700_000_000
    with pytest.raises(RuntimeError):validate_sample(sample,receipt,1.01,9)


@pytest.mark.parametrize('method',[None,'cached_ecs'])
def test_measurements_rejects_old_pose_receipt_before_acquisition(tmp_path,monkeypatch,method):
    from simulator_execution import Measurements
    receipt={'world':'a0'}
    if method is not None:receipt['measurement_pose_source']=method
    monkeypatch.setattr('simulator_backend.verify_receipt_process',lambda _:receipt)
    with pytest.raises(RuntimeError,match='physics pose source'):
        Measurements(None,tmp_path/'receipt.json',tmp_path/'measurements.jsonl')
    assert not (tmp_path/'measurements.jsonl').exists()


def release_boundary_guard(tmp_path, monkeypatch):
    """A lifted, still held version of the existing physical pile fixture."""
    guard, sample, advance, contact = provisional_pile_guard(tmp_path, monkeypatch)
    guard.begin_pile_admission();guard.establish()
    guard.phase = 'opening'
    guard.pile_expired = set(guard.pile_certificate['certified_set'])
    guard.pile_certificate['active_set'] = []
    sample['contacts'] = sample['contacts'][:2]
    position = [.3]
    guard.m.joints = lambda s: {'leader': [position[0], 0.]}
    goal = dict(uuid='a'*32,accepted=True,stage='COMMISSION_RELEASE',
        wall_ns=sample['wall_ns']+1,run_id=sample['run_id'],target=guard.object,
        resolution_sha256=guard.pile_binding['resolution_sha256'],
        execution_attempt=guard.pile_binding['execution_attempt'],
        selected_grasp_index=guard.pile_binding.get('selected_grasp_index',0))
    guard.begin_release(goal, copy.deepcopy(sample))
    return guard, sample, advance, position, goal, contact


def test_release_requires_owned_success_physical_loss_and_detachment(tmp_path, monkeypatch):
    guard, sample, advance, position, goal, _ = release_boundary_guard(tmp_path, monkeypatch)
    assert guard.ownership == 'RELEASING'
    with pytest.raises(RuntimeError, match='physical release'):
        guard.finish_release(dict(held=False,attached_ids=[],acm_restored=True,measured_geometry_matches=True))
    advance();position[0]=.28;sample['poses']['a0::part_06'][0]+=.0001
    guard.check(sample)
    assert guard.release_candidate is None  # Exact fingertips still touch.
    sample['contacts']=[];advance();guard.check(sample)
    assert guard.release_candidate['iteration']==sample['iteration']
    assert guard.ownership=='RELEASING'
    with pytest.raises(RuntimeError, match='terminal'):
        guard.finish_release(dict(held=False,attached_ids=[],acm_restored=True,measured_geometry_matches=True))
    with pytest.raises(RuntimeError, match='owned'):
        guard.terminal_release('b'*32,4,1,sample['wall_ns'])
    guard.terminal_release(goal['uuid'],4,1,sample['wall_ns']+1)
    with pytest.raises(RuntimeError, match='attachment'):
        guard.finish_release(dict(held=False,attached_ids=[guard.object],acm_restored=True,measured_geometry_matches=True))
    assert guard.ownership=='RELEASING'
    position[0]=0.
    with pytest.raises(RuntimeError,match='post-terminal'):
        guard.release_ready()
    advance()
    evidence=guard.finish_release(dict(held=False,attached_ids=[],acm_restored=True,measured_geometry_matches=True))
    assert evidence['run_id']==sample['run_id'] and evidence['target']==guard.object
    assert evidence['goal']['uuid']==goal['uuid']
    assert evidence['separation']['wall_ns']<evidence['goal']['terminal_wall_ns']
    assert guard.ownership=='FREE_SETTLING' and guard.held is None
    with pytest.raises(RuntimeError, match='cannot regress'):
        guard.begin_release(goal,sample)
    sample['contacts']=[dict(a='a0::workcell_robot::arm_link::collision',
                             b='a0::part_06::link::collision',points=[[0,0,0]])]
    advance()
    with pytest.raises(RuntimeError,match='released target recontacted robot/tool'):guard.check(sample)


def test_release_landing_is_accepted_only_after_measured_loss_and_robot_contact_still_fails(tmp_path,monkeypatch):
    guard,sample,advance,position,goal,contact=release_boundary_guard(tmp_path,monkeypatch)
    advance();position[0]=.28;sample['poses']['a0::part_06'][0]+=.0001
    guard.check(sample)
    sample['contacts']=[];advance();guard.check(sample)
    assert guard.release_candidate is not None
    advance();sample['contacts']=[contact('07',[0,.0125,0])]
    guard.check(sample)  # Expired pair landing is no longer carried-object recontact.
    assert guard.ownership=='RELEASING'
    sample['contacts'].append(dict(a='a0::workcell_robot::arm_link::collision',
                                  b='a0::part_07::link::collision',points=[[0,0,0]]))
    advance()
    with pytest.raises(RuntimeError,match='unpermitted physical contact'):guard.check(sample)


@pytest.mark.parametrize('fault',['other_run','stale','foreign_goal','failed_open','no_motion'])
def test_release_rejects_foreign_stale_failed_or_unmoved_evidence(tmp_path,monkeypatch,fault):
    guard,sample,advance,position,goal,_=release_boundary_guard(tmp_path,monkeypatch)
    advance()
    if fault=='other_run':sample['run_id']='foreign'
    elif fault=='stale':monkeypatch.setattr('simulator_execution.time.time',lambda:sample['wall_ns']/1e9+.3)
    elif fault!='no_motion':position[0]=.28
    sample['contacts']=[];sample['poses']['a0::part_06'][0]+=.0001
    if fault in ('other_run','stale'):
        with pytest.raises(RuntimeError):guard.check(sample)
        return
    guard.check(sample)
    if fault=='no_motion':
        assert guard.release_candidate is None
        return
    if fault=='foreign_goal':
        with pytest.raises(RuntimeError,match='owned'):guard.terminal_release('b'*32,4,1,sample['wall_ns'])
    else:
        with pytest.raises(RuntimeError,match='successful'):guard.terminal_release(goal['uuid'],5,1,sample['wall_ns'])
    assert guard.ownership=='RELEASING'


def test_release_ready_consumes_pending_physical_separation_before_rejecting(tmp_path,monkeypatch):
    guard,sample,advance,position,goal,_=release_boundary_guard(tmp_path,monkeypatch)
    guard.terminal_release(goal['uuid'],4,1,sample['wall_ns']+1)
    advance();position[0]=0.;sample['contacts']=[]
    sample['poses']['a0::part_06'][0]+=.0001
    guard.m.drain=lambda:[copy.deepcopy(sample)]
    assert guard.release_candidate is None
    ready=guard.release_ready()
    assert ready['iteration']==sample['iteration']
    assert guard.release_candidate['iteration']==sample['iteration']


@pytest.mark.parametrize('armed',[False,True])
def test_age_only_receive_reacquires_only_before_motion_arm(armed,monkeypatch):
    import json,threading
    from types import SimpleNamespace as N
    import simulator_execution as module
    m=module.Measurements.__new__(module.Measurements)
    m.lock=threading.RLock();m.receipt=dict(run_id='r',pid=12)
    m.latest=None;m.pending=[];m.previous=None;m.armed=armed;m.error=None;m.timing=[]
    monkeypatch.setattr(module.time,'time',lambda:2.)
    monkeypatch.setattr(module.time,'time_ns',lambda:2_000_000_000)
    def sample(i,wall):
        return dict(run_id='r',pid=12,iteration=i,sim_ns=i,wall_ns=wall,
            poses={'p':[0,0,0,0,0,0,1]},joints={'j':[0,0]},contacts=[],collisions=['c'])
    m.update(N(data=json.dumps(sample(1,1_616_000_000))))
    assert m.latest is None and m.pending==[] and m.previous is None
    assert m.timing[0]['error'] and m.timing[0]['source_wall_ns']==1_616_000_000
    with pytest.raises(RuntimeError):m.fresh()
    m.update(N(data=json.dumps(sample(2,1_998_000_000))))
    if armed:
        with pytest.raises(RuntimeError):m.fresh()
    else:
        assert m.error is None
        assert m.fresh()['iteration']==2
        assert m.timing[0]['error']


@pytest.mark.parametrize('patch',[
    {'run_id':'wrong'},{'pid':13},{'error':'physics failed'},
    {'poses':{'p':[0,0,0,0,0,0,0]}},{'sim_ns':0},
    {'iteration':None},{'iteration':True},{'iteration':2.0},
])
def test_stale_prearm_sample_does_not_hide_other_invalidity(patch,monkeypatch):
    import json,threading
    from types import SimpleNamespace as N
    import simulator_execution as module
    m=module.Measurements.__new__(module.Measurements)
    m.lock=threading.RLock();m.receipt=dict(run_id='r',pid=12)
    m.latest=dict(sim_ns=1);m.pending=[];m.previous=1;m.armed=False;m.error=None;m.timing=[]
    monkeypatch.setattr(module.time,'time',lambda:2.)
    monkeypatch.setattr(module.time,'time_ns',lambda:2_000_000_000)
    sample=dict(run_id='r',pid=12,iteration=2,sim_ns=2,wall_ns=1_616_000_000,
        poses={'p':[0,0,0,0,0,0,1]},joints={'j':[0,0]},contacts=[],collisions=['c'])
    m.update(N(data=json.dumps(dict(sample,**patch))))
    assert m.error is not None
    m.update(N(data=json.dumps(dict(sample,iteration=3,sim_ns=3,wall_ns=1_998_000_000))))
    with pytest.raises(RuntimeError):m.fresh()


def live_measurement_sample(i,wall_ns,run_id='r',pid=12):
    from simulator_backend import PHYSICS_POSE_METHOD
    return dict(run_id=run_id,pid=pid,iteration=i,sim_ns=i*1000000,wall_ns=wall_ns,publish_wall_ns=wall_ns+200000,
        serialization_ns=100000,poses={'a0::part':[0,0,0,0,0,0,1],'a0::part::link':[0,0,0,0,0,0,1]},
        joints={'a0::robot::leader':[.3,0.]},contacts=[],collisions=['a0::part::link::collision'],
        pose_source=dict(method=PHYSICS_POSE_METHOD,frame='world',read_only=True,gazebo_version='6.16',
            query_iteration=i,query_sim_ns=i*1000000),
        pose_entities={'a0::part':dict(kind='model',entity=1,link_entity=2,query_entity=3),
                       'a0::part::link':dict(kind='link',entity=2,link_entity=2,query_entity=3)})


def test_live_acquisition_takes_every_sample_writes_source_text_and_latches_stale(tmp_path,monkeypatch):
    """The wait-set acquisition loop must behave exactly like the executor it replaces:
    every published sample reaches update() in order, armed samples queue for the
    consumer, the evidence line is the simulator's own text, a stale sample still
    latches the unchanged 250 ms guard, and shutdown leaves no thread behind."""
    import os,threading,time
    import rclpy
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
    import simulator_execution
    from simulator_backend import PHYSICS_POSE_METHOD
    receipt=dict(world='a0',model='robot',run_id='r',pid=12,measurement_pose_source=PHYSICS_POSE_METHOD)
    monkeypatch.setattr('simulator_backend.verify_receipt_process',lambda _:receipt)
    monkeypatch.setattr(simulator_execution,'validate_publisher',lambda publishers:None)
    (tmp_path/'robot.urdf').write_text('<robot name="r"><link name="base"/></robot>')
    (tmp_path/'receipt.json').write_text(json.dumps(receipt))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1');monkeypatch.setenv('ROS_DOMAIN_ID',os.environ.get('ROS_DOMAIN_ID','201'))
    rclpy.init();m=None
    try:
        node=rclpy.create_node('live_acquisition_test')
        pub=node.create_publisher(String,'/world/a0/workcell_measurements',
            QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST))
        m=simulator_execution.Measurements(node,tmp_path/'receipt.json',tmp_path/'measurements.jsonl')
        assert m.acquisition.is_alive() and m.writer.is_alive()
        published={}
        def publish(i,wall_ns=None):
            s=live_measurement_sample(i,time.time_ns() if wall_ns is None else wall_ns)
            # One multi-line text proves the writer falls back to a single-line dump.
            text=json.dumps(s,sort_keys=True,indent=1 if i==60 else None);published[i]=(s,text);pub.publish(String(data=text))
        until=time.monotonic()+5
        while pub.get_subscription_count()==0 and time.monotonic()<until:time.sleep(.01)
        for i in range(1,51):publish(i);time.sleep(.002)
        until=time.monotonic()+5
        while (m.latest is None or m.latest['iteration']<50) and time.monotonic()<until:time.sleep(.005)
        assert m.latest['iteration']==50 and m.error is None,(m.error,[e for e in m.timing if e['error']][:2])
        m.fresh();m.arm()
        # Main thread competes the way the executor does: spinning its own node.
        for i in range(51,151):
            publish(i);rclpy.spin_once(node,timeout_sec=.001)
        until=time.monotonic()+5
        while (m.latest['iteration']<150) and time.monotonic()<until:time.sleep(.005)
        with m.lock:
            pending=m.drain()
        assert [s['iteration'] for s in pending]==list(range(51,151))
        for s in pending:m.record('test',s)
        # An unpaced 400-message burst (~0.8 s of stream) must arrive complete and
        # ordered. It stays well inside the 250 ms guard once processed in order.
        for i in range(151,551):publish(i)
        until=time.monotonic()+10
        while (m.latest['iteration']<550) and time.monotonic()<until:time.sleep(.005)
        with m.lock:
            received=[e['iteration'] for e in m.timing if e['event']=='receive']
            assert m.latest['iteration']==550,(m.latest['iteration'],len(received),received[-3:],m.error)
            burst=m.drain()
        assert [s['iteration'] for s in burst]==list(range(151,551))
        for s in burst:m.record('test',s)
        receive=[e for e in m.timing if e['event']=='receive']
        assert [e['iteration'] for e in receive]==list(range(1,551))
        assert all('lock_wait_ns' in e and e['error'] is None for e in receive)
        # A structurally valid but old sample latches the unchanged guard once armed.
        publish(551,time.time_ns()-300000000)
        until=time.monotonic()+5
        while m.error is None and time.monotonic()<until:time.sleep(.005)
        assert isinstance(m.error,simulator_execution.StaleMeasurement)
        with pytest.raises(simulator_execution.StaleMeasurement):m.fresh()
    finally:
        try:
            if m is not None:m.close()
        finally:rclpy.shutdown()
    assert not m.acquisition.is_alive() and not m.writer.is_alive() and not m.watch.thread.is_alive()
    lines=[json.loads(l) for l in (tmp_path/'measurements.jsonl').read_text().splitlines()]
    assert [l['measurement']['iteration'] for l in lines]==list(range(51,551))
    assert all(l['phase']=='test' and l['measurement']==published[l['measurement']['iteration']][0] for l in lines)
    # The evidence line carries the simulator's own serialization, not a re-dump.
    raw=(tmp_path/'measurements.jsonl').read_text().splitlines()[0]
    assert raw=='{"phase": "test", "measurement": '+published[51][1]+'}'
    assert len((tmp_path/'measurements.jsonl').read_text().splitlines())==500
    timing=json.loads((tmp_path/'measurements.jsonl.timing.json'.replace('.jsonl.timing','.timing')).read_text())
    assert {e['event'] for e in timing}>={'receive','fresh','consume_log','process_watch'}
    assert all(w['threads'] for w in timing if w['event']=='process_watch')


def _hold_gil(seconds):
    """A single C call that keeps the interpreter lock (builtin sum over a range)."""
    import time as _time
    started=_time.perf_counter();n=10**6
    while _time.perf_counter()-started<seconds/4:sum(range(n));n*=2
    started=_time.perf_counter();sum(range(n*4))
    return _time.perf_counter()-started


def test_gil_hold_watch_names_the_thread_holding_the_lock_and_records_the_overrun(tmp_path):
    import time
    from simulator_execution import GilHoldWatch
    watch=GilHoldWatch(tmp_path/'summary.measurements.gil-holds.txt',100)
    watch.start()
    try:
        time.sleep(.3)                                   # idle: the heartbeat keeps re-arming, nothing is dumped
        quiet=[e for e in watch.events if e['event']=='gil_hold']
        held=_hold_gil(.4)
        time.sleep(.2)
    finally:
        watch.stop()
    assert quiet==[] and held>=.15
    text=(tmp_path/'summary.measurements.gil-holds.txt').read_text()
    assert 'Timeout (0:00:00.100000)!' in text and 'in _hold_gil' in text   # the holder's Python stack is in the dump
    overruns=[e for e in watch.events if e['event']=='gil_hold']
    assert overruns and all(e['gap_ms']>100 for e in overruns) and overruns[0]['overrun']==1
    summary,=[e for e in watch.events if e['event']=='gil_watch_summary']
    assert summary['dumps']>=1 and summary['overruns']==len(overruns) and summary['threshold_ms']==100
    assert summary['dump_file']=='summary.measurements.gil-holds.txt'


@pytest.mark.parametrize('value',['10','2000','nan','inf'])
def test_gil_hold_watch_rejects_thresholds_outside_its_bounds(tmp_path,value):
    from simulator_execution import GilHoldWatch
    with pytest.raises(RuntimeError,match=r'within \[50, 1000\] ms'):GilHoldWatch(tmp_path/'x.txt',value)


def test_gil_hold_watch_is_opt_in_and_wired_beside_the_stall_watch(tmp_path,monkeypatch):
    from simulator_execution import GIL_WATCHDOG_ENV,GilHoldWatch
    monkeypatch.delenv(GIL_WATCHDOG_ENV,raising=False)
    assert GilHoldWatch.from_environment(tmp_path/'x.txt') is None             # default: no watchdog, no file
    assert not (tmp_path/'x.txt').exists()
    monkeypatch.setenv(GIL_WATCHDOG_ENV,'150')
    assert GilHoldWatch.from_environment(tmp_path/'x.txt').threshold==.15
    source=(Path(__file__).parents[1]/'scripts/simulator_execution.py').read_text()
    assert "self.gil_watch=GilHoldWatch.from_environment(self.log_path.with_suffix('.gil-holds.txt'))" in source
    assert "if self.gil_watch is not None:self.gil_watch.stop();self.timing.extend(self.gil_watch.events)" in source
    assert source.index("if self.gil_watch is not None:self.gil_watch.stop()")<source.index("with_suffix('.timing.json').write_text")


def test_gil_hold_watch_stop_cancels_the_process_timer_and_never_closes_under_a_live_heartbeat(tmp_path,monkeypatch):
    import faulthandler
    from simulator_execution import GilHoldWatch
    watch=GilHoldWatch(tmp_path/'d.txt',100);watch.start()
    cancelled=[]
    real_cancel=faulthandler.cancel_dump_traceback_later
    monkeypatch.setattr(faulthandler,'cancel_dump_traceback_later',lambda:(cancelled.append(1),real_cancel()))
    class Stuck:                                         # a heartbeat that did not stop within the join bound
        def join(self,timeout):pass
        def is_alive(self):return True
    live=watch.thread;watch.thread=Stuck()
    watch.stop()
    assert cancelled and not watch.file.closed           # timer cancelled; file kept open for the live thread
    incomplete,=[e for e in watch.events if e['event']=='gil_watch_summary']
    assert incomplete['thread_alive'] is True           # the evidence shows the teardown was incomplete
    watch.thread=live;watch.stop()
    final=[e for e in watch.events if e['event']=='gil_watch_summary'][-1]
    assert watch.file.closed and 'thread_alive' not in final and final['dumps']==0


def test_bad_watchdog_value_fails_before_any_measurement_thread_starts():
    source=(Path(__file__).parents[1]/'scripts/simulator_execution.py').read_text()
    init=source.index("self.gil_watch=GilHoldWatch.from_environment(")
    assert init<source.index("self.writer=threading.Thread(target=self._write")
    assert init<source.index("self.acquisition=threading.Thread(target=self._acquire")
    assert init<source.index("self.watch=ProcessStallWatch();self.watch.start()")


def _timed_measurements(module):
    m=module.Measurements.__new__(module.Measurements)
    m.lock=module.TimedRLock();m.receipt=dict(run_id='r',pid=12)
    m.latest=None;m.pending=[];m.previous=None;m.armed=True;m.error=None;m.timing=[];m.raw={}
    return m


def _plain_sample(i,wall_ns):
    return dict(run_id='r',pid=12,iteration=i,sim_ns=i,wall_ns=wall_ns,
        poses={'p':[0,0,0,0,0,0,1]},joints={'j':[0,0]},contacts=[],collisions=['c'])


def test_update_parses_outside_the_lock_and_still_validates_inside_it(monkeypatch):
    import json,time
    from types import SimpleNamespace as N
    import simulator_execution as module
    m=_timed_measurements(module);seen=[]
    real_loads=json.loads;real_validate=module.validate_sample
    monkeypatch.setattr(module,'json',N(loads=lambda text:(seen.append(('parse',m.lock.held_by_current_thread())),real_loads(text))[1],
                                        dumps=json.dumps))
    monkeypatch.setattr(module,'validate_sample',lambda *a,**k:(seen.append(('validate',m.lock.held_by_current_thread())),real_validate(*a,**k))[1])
    m.update(N(data=json.dumps(_plain_sample(1,time.time_ns()))))
    assert seen==[('parse',False),('validate',True)]        # order and lock scope; checks unchanged
    assert m.latest['iteration']==1 and len(m.pending)==1 and m.timing[-1]['error'] is None
    assert m.timing[-1]['lock_wait_ns']>=0 and m.timing[-1]['parse_ns']>=0
    assert m.timing[-1]['callback_ns']>=m.timing[-1]['parse_ns']+m.timing[-1]['lock_wait_ns']   # callback still spans parse + wait


def test_update_parse_failure_still_latches_and_records_the_receive():
    from types import SimpleNamespace as N
    import simulator_execution as module
    m=_timed_measurements(module)
    m.update(N(data='{not json'))
    assert m.error is not None and m.latest is None and m.pending==[]
    assert m.timing[-1]['event']=='receive' and m.timing[-1]['error']
    with pytest.raises(type(m.error)):m.fresh()


def test_fresh_validates_its_locked_snapshot_outside_the_lock_unless_the_caller_holds_it(monkeypatch):
    import json,time
    from types import SimpleNamespace as N
    import simulator_execution as module
    m=_timed_measurements(module);m.update(N(data=json.dumps(_plain_sample(1,time.time_ns()))))
    held=[];real_validate=module.validate_sample
    def validate(sample,*args):
        held.append(m.lock.held_by_current_thread())
        if sample['iteration']==1 and len(held)==1:              # a newer sample arrives during validation
            m.update(N(data=json.dumps(_plain_sample(2,time.time_ns()))))
        return real_validate(sample,*args)
    monkeypatch.setattr(module,'validate_sample',validate)
    assert m.fresh()['iteration']==1                            # the snapshot read under the lock is returned
    assert held[0] is False and m.fresh()['iteration']==2
    with m.lock:                                                # e.g. ContactGuard.checked_current
        m.fresh()
    assert held[-1] is True
    events=[e for e in m.timing if e['event']=='fresh']
    assert [e['iteration'] for e in events]==[1,2,2] and all(e['error'] is None for e in events)


def test_fresh_rejection_outside_the_lock_is_recorded_and_raised(monkeypatch):
    import json,time
    from types import SimpleNamespace as N
    import simulator_execution as module
    m=_timed_measurements(module);wall=time.time_ns();m.update(N(data=json.dumps(_plain_sample(1,wall))))
    monkeypatch.setattr(module.time,'time_ns',lambda:wall+300_000_000)
    with pytest.raises(module.StaleMeasurement):m.fresh()
    assert m.timing[-1]['event']=='fresh' and 'stale' in m.timing[-1]['error'] and m.timing[-1]['age_ns']==300_000_000


def test_drain_refuses_to_consume_when_an_error_latches_after_its_fresh_read(monkeypatch):
    import json,time
    from types import SimpleNamespace as N
    import simulator_execution as module
    m=_timed_measurements(module);m.update(N(data=json.dumps(_plain_sample(1,time.time_ns()))))
    real_validate=module.validate_sample
    def validate(*args):
        real_validate(*args);m.error=RuntimeError('SIMULATOR_TELEMETRY_INVALID: skipped measurement')   # acquisition latches now
    monkeypatch.setattr(module,'validate_sample',validate)
    with pytest.raises(RuntimeError,match='skipped'):m.drain()
    assert len(m.pending)==1                                    # nothing consumed
    assert m.timing[-1]['event']=='fresh' and 'skipped' in m.timing[-1]['error']   # the refusal is in the evidence
    monkeypatch.setattr(module,'validate_sample',real_validate);m.error=None
    assert [s['iteration'] for s in m.drain()]==[1] and m.pending==[]


def test_timed_rlock_is_reentrant_and_accounts_each_thread_once_per_outer_hold():
    import threading,time
    from simulator_execution import TimedRLock
    lock=TimedRLock()
    with lock:
        with lock:assert lock.held_by_current_thread()
        time.sleep(.01)
    assert not lock.held_by_current_thread()
    entered=threading.Event()
    def contender():
        entered.set()
        with lock:pass
    with lock:
        t=threading.Thread(target=contender,name='measurement_acquisition');t.start();entered.wait();time.sleep(.02)
    t.join()
    rows=lock.profile()['rows']
    main=[r for r in rows if r['thread']==threading.current_thread().name]
    acq=[r for r in rows if r['thread']=='measurement_acquisition']
    assert sum(r['count'] for r in main)==2 and max(r['hold_max_ms'] for r in main)>=10    # nested hold counted once
    assert sum(r['count'] for r in acq)==1 and acq[0]['wait_ms']>=15                       # waited for the main hold
    assert all(isinstance(r['second'],int) for r in rows)


def test_measurements_use_the_timed_lock_and_write_its_profile_into_the_timing_evidence():
    source=(Path(__file__).parents[1]/'scripts/simulator_execution.py').read_text()
    assert "self.lock=TimedRLock();self.latest=None" in source
    assert "if isinstance(self.lock,TimedRLock):self.timing.append(self.lock.profile())" in source
    assert source.index("self.timing.append(self.lock.profile())")<source.index("with_suffix('.timing.json').write_text")


@pytest.fixture(scope='session')
def built_gil_probe(tmp_path_factory):
    # One build per test process: a process loads the probe (and its PC signal handler) once, as the executor does.
    import shutil,subprocess,sysconfig
    if shutil.which('cc') is None:pytest.skip('no C compiler')
    library=tmp_path_factory.mktemp('gil_probe')/'gil_probe.so'
    subprocess.run(['cc','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC','-I'+sysconfig.get_paths()['include'],
                    '-o',str(library),str(Path(__file__).parents[1]/'scripts/gil_probe.c')],check=True)
    return library


def _probe_holds(probe):
    return [e for e in probe.events if e['event']=='gil_probe_hold']


def _sampler_running(probe):
    # A hold that starts before the sampler's first loop has no waiting sampler to snapshot it
    # (documented limit); the executor starts the probe long before planning or motion.
    import time
    deadline=time.monotonic()+5
    while probe.sampler_wakes<3:
        assert time.monotonic()<deadline,'GIL probe sampler did not start'
        time.sleep(.01)


def test_gil_probe_attributes_a_native_hold_with_cpu_syscall_and_exact_release_frames(tmp_path,built_gil_probe):
    import threading,time
    from simulator_execution import GilProbe
    probe=GilProbe(built_gil_probe,tmp_path/'summary.measurements.gil-probe.txt',80);probe.start()
    holder={};held=threading.Event();done=threading.Event()
    def worker():
        holder['seconds']=_hold_gil(.4);held.set();done.wait(5)   # alive afterwards: its /proc end sample exists
    t=threading.Thread(target=worker,name='measurement_acquisition')
    try:
        _sampler_running(probe)
        t.start();held.wait(5)              # blocked, not polling: the sampler is the only waiter, as in the executor
        time.sleep(.1);busy_start=time.time_ns()
        def busy():
            stop=time.perf_counter()+.3;x=0
            while time.perf_counter()<stop:x+=1
        u=threading.Thread(target=busy,name='busy');u.start();busy();u.join()   # pure-Python contention switches every 5 ms
        time.sleep(.05)
    finally:
        done.set();t.join();probe.stop()
    holds=_probe_holds(probe)
    # _hold_gil's calibration calls are C calls too; any may exceed 80 ms. All holds are the worker's.
    assert holds and all(h['thread']=='measurement_acquisition' and h['attribution']=='verified' for h in holds)
    assert all(h['thread_id']==t.ident and h['native_id']==t.native_id and h['holder_proc_data']=='holder' for h in holds)
    longest=max(holds,key=lambda h:h['hold_ms'])
    assert longest['hold_ms']>=holder['seconds']*1e3*.8
    assert all(h['end_wall_ns']<=busy_start+20_000_000 for h in holds)       # nothing during pure-Python contention
    cpu=longest['holder_cpu']
    assert cpu and cpu['state_begin']=='R' and cpu['on_cpu_ms']>=(longest['hold_ms']-80)*.5 and cpu['majflt']==0
    assert longest['holder_syscall']['samples']>=5 and longest['holder_syscall']['running']>=longest['holder_syscall']['samples']-1
    # exact unless another waiter let the holder run before the sampler (see the exactness test)
    assert longest['release_frames_status'] in ('exact','holder may have run again')
    if longest['release_frames_status']=='exact':assert longest['release_frames'][0]['function']=='_hold_gil'
    assert longest['native_pc']==[]                                    # native PC sampling is a separate opt-in
    summary,=[e for e in probe.events if e['event']=='gil_probe_summary']
    assert summary['holds']==len(holds) and summary['threshold_ms']==80 and summary['file']=='summary.measurements.gil-probe.txt'
    assert summary['unmapped_holds']==0 and summary['snapshot_failures']==0 and summary['snapshots']>=1
    assert summary['sampler_alive_before_stop'] and summary['sampler_wakes']>10 and summary['thread_hook']
    assert {'MainThread','measurement_acquisition','gil_frame_sampler','busy'}<=set(summary['registered_threads'])
    assert summary['native_pc'] is False and summary['pc_sent']==0 and len(summary['library_sha256'])==64
    assert (tmp_path/'summary.measurements.gil-probe.maps').read_text().count('\n')>10


def test_gil_probe_native_pc_samples_land_in_the_native_function_that_keeps_the_gil(tmp_path,built_gil_probe):
    import ctypes,threading,time
    from simulator_execution import GilProbe
    from gil_probe_report import report
    probe=GilProbe(built_gil_probe,tmp_path/'summary.measurements.gil-probe.txt',80,native_pc=True);probe.start()
    spin=ctypes.PyDLL(str(built_gil_probe)).gil_probe_selftest_spin
    spin.argtypes=[ctypes.c_longlong];spin.restype=ctypes.c_ulonglong
    done=threading.Event()
    def worker():spin(300_000_000);done.wait(5)
    t=threading.Thread(target=worker,name='spinner')
    try:
        _sampler_running(probe)
        t.start();time.sleep(.45)
    finally:
        done.set();t.join();probe.stop()
    hold,=_probe_holds(probe)
    assert hold['thread']=='spinner' and hold['attribution']=='verified' and hold['hold_ms']>=250
    assert hold['release_frames_status'] in ('exact','holder may have run again')
    if hold['release_frames_status']=='exact':assert hold['release_frames'][0]['function']=='worker'
    pcs=hold['native_pc']
    assert len(pcs)>=40 and all(p['tid']==t.native_id and p['stack_bytes']>0 for p in pcs)
    assert max(p['delivery_us'] for p in pcs)<50_000
    summary,=[e for e in probe.events if e['event']=='gil_probe_summary']
    assert summary['native_pc'] and summary['pc_signal'] and summary['pc_received']==len(pcs)
    (tmp_path/'summary.measurements.timing.json').write_text(json.dumps(probe.events))
    row,=report(tmp_path)['holds']
    # the spin loop itself or the clock it reads; the stack scan finds the spinning function as a caller
    leaf=[name for name,_ in row['native_pc']['leaf']]
    assert leaf and all(n.startswith(('gil_probe.so','[vdso]')) or (n.startswith('libc.so.6:') and 'clock_gettime' in n) for n in leaf)
    assert any(name.startswith('gil_probe.so:gil_probe_selftest_spin') for name,_ in row['native_pc']['stack_scan_heuristic']+row['native_pc']['leaf'])


def test_gil_probe_release_frames_are_exact_when_the_sampler_is_the_only_waiter(tmp_path,built_gil_probe):
    import ctypes,threading,time
    from simulator_execution import GilProbe
    spin=ctypes.PyDLL(str(built_gil_probe)).gil_probe_selftest_spin
    spin.argtypes=[ctypes.c_longlong];spin.restype=ctypes.c_ulonglong
    statuses=[]
    for attempt in range(5):
        probe=GilProbe(built_gil_probe,tmp_path/f'p{attempt}.txt',80);probe.start()
        done=threading.Event()
        def worker():spin(120_000_000);done.wait(5)
        t=threading.Thread(target=worker,name='spinner')
        try:
            _sampler_running(probe)
            t.start();time.sleep(.2)
        finally:
            done.set();t.join();probe.stop()
        hold,=_probe_holds(probe)
        statuses.append(hold['release_frames_status'])
        assert hold['attribution']=='verified' and hold['thread']=='spinner'
        if hold['release_frames_status']=='exact':
            assert hold['release_frames'][0]['function']=='worker' and hold['release_snapshot_switches_after']<=2
    # offline: 116 of 120 exact, the rest labelled 'holder may have run again', none missing
    assert set(statuses)<={'exact','holder may have run again'} and statuses.count('exact')>=3


def test_gil_probe_native_pc_never_interrupts_a_holder_that_makes_syscalls(tmp_path,built_gil_probe):
    import ctypes,threading
    from simulator_execution import GilProbe
    mixed=ctypes.PyDLL(str(built_gil_probe)).gil_probe_selftest_mixed
    mixed.argtypes=[ctypes.c_longlong,ctypes.c_longlong,ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_int)];mixed.restype=ctypes.c_int
    results=[]
    for use_poll in (0,1):
        probe=GilProbe(built_gil_probe,tmp_path/f'm{use_poll}.txt',80,native_pc=True);probe.start()
        try:
            _sampler_running(probe)
            calls=ctypes.c_int()
            def worker():results.append((mixed(400_000_000,3_000_000,2,use_poll,ctypes.byref(calls)),calls.value))
            t=threading.Thread(target=worker,name='mixed');t.start();t.join()
        finally:
            probe.stop()
        summary,=[e for e in probe.events if e['event']=='gil_probe_summary']
        hold=max(_probe_holds(probe),key=lambda h:h['hold_ms'])
        assert hold['thread']=='mixed' and hold['holder_syscall']['in_syscall']        # its sys records show the calls
        assert summary['pc_skipped_in_syscall']>=1 and summary['pc_sent']<=3           # sampling stopped at the first syscall
    # before the fix about 30 % of these calls failed with EINTR; one remains possible (microsecond race)
    assert sum(e for e,_ in results)<=1 and all(n>=50 for _,n in results)


def test_gil_probe_snapshot_runs_no_gc_and_reports_a_switch_during_it(built_gil_probe):
    import ctypes,gc,threading,time
    snap=ctypes.PyDLL(str(built_gil_probe)).gil_probe_snapshot
    snap.argtypes=[ctypes.py_object,ctypes.c_int];snap.restype=ctypes.c_int
    calls=[];stop=threading.Event()
    def callback(phase,info):calls.append(time.monotonic_ns())
    def spinner():
        while not stop.is_set():sum(range(200))
    workers=[threading.Thread(target=spinner,daemon=True) for _ in range(2)];[w.start() for w in workers]
    gc.callbacks.append(callback);before=gc.get_threshold()
    try:
        for _ in range(3):
            gc.disable();junk=[[i] for i in range(300_000)]                           # a large young generation
            gc.set_threshold(gc.get_count()[0]+2);gc.enable();calls.clear()
            out=[];started=time.monotonic_ns();snap(out,24);ended=time.monotonic_ns()
            switch_start,switch_end,_,_,threads,tstates=out[0]
            assert not [t for t in calls if started<=t<=ended]                          # no collection ran inside
            assert switch_start==switch_end and gc.isenabled()                         # nothing ran; GC restored
            assert len(threads)>=3 and len(tstates)>=3
            del junk
    finally:
        gc.callbacks.remove(callback);gc.set_threshold(*before);stop.set();[w.join() for w in workers]


def test_gil_probe_never_names_a_later_thread_that_reused_the_holders_address(tmp_path,built_gil_probe):
    import ctypes,threading,time
    from simulator_execution import GilProbe
    spin=ctypes.PyDLL(str(built_gil_probe)).gil_probe_selftest_spin
    spin.argtypes=[ctypes.c_longlong];spin.restype=ctypes.c_ulonglong
    labels=[]
    for attempt in range(8):
        probe=GilProbe(built_gil_probe,tmp_path/f'r{attempt}.txt',80);probe.start()
        try:
            _sampler_running(probe)
            t=threading.Thread(target=spin,args=(120_000_000,),name='short_lived');t.start();t.join()   # holds, then exits
            n=threading.Thread(target=time.sleep,args=(.05,),name='next_worker');n.start()            # may reuse its address and ident
            time.sleep(.1);n.join()
        finally:
            probe.stop()
        for h in _probe_holds(probe):
            labels.append((h['thread'],h['attribution']))
            assert h['thread']!='next_worker'
            if h['attribution']=='verified':assert h['thread']=='short_lived'
    assert any(label==('short_lived','verified') for label in labels)


def test_gil_probe_reports_a_holder_that_never_registered_without_guessing(tmp_path,built_gil_probe):
    import threading,time
    from simulator_execution import GilProbe
    go=threading.Event();held=threading.Event();done=threading.Event()
    def worker():go.wait(5);_hold_gil(.3);held.set();done.wait(5)   # alive after its hold, as executor threads are
    t=threading.Thread(target=worker,name='started_before_probe');t.start()   # no registration hook for it
    probe=GilProbe(built_gil_probe,tmp_path/'p.txt',80);probe.start()
    try:
        _sampler_running(probe)
        go.set();held.wait(5);time.sleep(.05)
    finally:
        done.set();t.join();probe.stop()
    holds=_probe_holds(probe)
    assert holds and all(h['native_id']==-1 and h['holder_cpu'] is None and h['holder_proc_data']=='none' for h in holds)
    # the snapshot taken under the GIL right after the release still names it, marked as such
    assert all(h['attribution'] in ('snapshot only','unattributed') for h in holds)
    assert any(h['attribution']=='snapshot only' and h['thread']=='started_before_probe' for h in holds)
    summary,=[e for e in probe.events if e['event']=='gil_probe_summary']
    assert summary['unmapped_holds']==len(holds) and 'started_before_probe' in summary['unregistered_at_start']


def test_gil_probe_releasing_native_calls_and_python_loops_are_not_holds(tmp_path,built_gil_probe):
    import ctypes,threading,time
    from simulator_execution import GilProbe
    probe=GilProbe(built_gil_probe,tmp_path/'p.txt',80);probe.start()
    try:
        libc=ctypes.CDLL(None)                                    # CDLL releases the GIL around the call
        sleeper=threading.Thread(target=libc.usleep,args=(300000,),name='releaser');sleeper.start()
        def loop():
            stop=time.perf_counter()+.3;x=0
            while time.perf_counter()<stop:x+=1
        workers=[threading.Thread(target=loop,name=f'loop{i}') for i in range(2)];[w.start() for w in workers]
        loop();[w.join() for w in workers];sleeper.join();time.sleep(.05)
    finally:
        probe.stop()
    assert _probe_holds(probe)==[]


def test_gil_probe_parse_labels_identity_and_frames_only_as_far_as_the_evidence_goes():
    from simulator_execution import GilProbe
    reg=lambda ident,tid,name,tstate_id,switch:dict(ident=ident,native_id=tid,name=name,tstate_id=tstate_id,switch_number=switch,registered_wall_ns=0)
    # address 0x10 is reused: 'early' (id 1) registered at switch 1, 'later' (id 2) at switch 9 after it exited
    registry={0x10:[reg(1,11,'early',1,1),reg(1,12,'later',2,9)],0x20:[reg(3,13,'gone',3,2)],0x40:[reg(5,15,'reused',5,2)]}
    def snap(wall,switch,tstates,end=None,waited=50_000_000,names=None):
        return dict(event='gil_frame_snapshot',wall_ns=wall,switch_number=switch,switch_number_end=switch if end is None else end,
            monotonic_ns=10**12,asked_monotonic_ns=10**12-waited,tstates={str(k):list(v) for k,v in tstates.items()},
            names=names or {'1':'early','4':'native','6':'creator'},
            threads={'1':[dict(file='a.py',function='f',firstlineno=1,lasti=4,line=2)],'4':[dict(file='n.py',function='n',firstlineno=1,lasti=2,line=1)],
                     '6':[dict(file='c.py',function='c',firstlineno=1,lasti=2,line=1)]})
    text=('hold 1000 90000000 89.000 0x10 5 11 1.100\n'           # early's own hold, confirmed by its id
          'cpu 1000 11 begin R 10 1 5 0 100 0 3\n'
          'cpu 1000 11 end R 18 1 6 0 9000000 1000000 9\n'
          'sys 1000 2000 running\n'
          'sys 1000 7000 202 0x1 0x2 0x3 0x4 0x5 0x6 0x7ffd 0x7f00\n'
          'pc 1000 3000 25 11 0x401000 0x7ffd00 4096 2 0x401100 0x402000\n'
          'hold 95000000 190000000 95.000 0x10 6 11 1.000\n'      # 'later' registered after it: early again, but frames moved
          'hold 200000000 300000000 100.000 0x20 20 13 1.0\n'     # its thread is gone from the snapshot
          'hold 400000000 500000000 100.000 0x30 30 -1 1.0\n'     # never registered, the live map names it
          'hold 600000000 700000000 100.000 0x20 40 13 1.0\n'     # the next sampler was not waiting during it
          'hold 710000000 800000000 90.000 0x40 50 15 1.0\n'      # address now holds another thread state (id 9)
          'hold 810000000 900000000 90.000 0x50 60 -1 1.0\n'      # a starting thread carries its creator's ident
          'hold 910000000 999000000 89.000 0x10 70 12 1.0\n'      # a thread ran during the snapshot itself
          'late 50000000 60000000 10.000\n'
          'garbage line\n')
    snaps=[snap(90_500_000,6,{0x10:(1,1)}),snap(190_500_000,9,{0x10:(1,1)}),snap(300_400_000,21,{}),
           snap(500_200_000,31,{0x30:(4,4)}),snap(705_000_000,41,{0x20:(3,3)},waited=1_000_000),
           snap(800_300_000,51,{0x40:(5,9)}),snap(900_300_000,61,{0x50:(6,10),0x60:(6,6)}),
           snap(999_300_000,71,{0x10:(1,2)},end=72)]
    holds={h['start_wall_ns']:h for h in GilProbe.parse(text,registry,{},snaps) if h['event']=='gil_probe_hold'}
    first=holds[1000]
    assert (first['thread'],first['attribution'],first['release_frames_status'],first['overlaps_probe_late'])==('early','verified','exact',True)
    assert first['holder_proc_data']=='holder' and first['max_poll_gap_ms']==1.1
    assert first['holder_cpu']['utime_ticks']==8 and abs(first['holder_cpu']['on_cpu_ms']-8.9999)<1e-9
    assert first['holder_syscall']==dict(samples=2,running=1,unavailable=0,in_syscall=['202'])
    assert first['native_pc'][0]['rip']==0x401000 and first['native_pc'][0]['candidates']==[0x401100,0x402000]
    second=holds[95000000]
    assert (second['thread'],second['attribution'],second['release_frames_status'])==('early','verified','holder may have run again')
    assert (holds[200000000]['attribution'],holds[200000000]['thread'],holds[200000000]['release_frames_status'])==('registered, unconfirmed','gone','holder not attributable')
    assert (holds[400000000]['attribution'],holds[400000000]['thread'],holds[400000000]['holder_proc_data'])==('snapshot only','native','none')
    assert holds[600000000]['release_frames_status']=='no snapshot'
    reused=holds[710000000]
    assert (reused['attribution'],reused['thread'],reused['release_frames'],reused['holder_proc_data'])==('address reused, ambiguous',None,None,'not attributable to the holder')
    assert (holds[810000000]['attribution'],holds[810000000]['thread'])==('ambiguous (thread starting)',None)
    moving=holds[910000000]
    assert (moving['thread'],moving['attribution'],moving['release_frames_status'])==('later','verified','holder may have run again')


def test_gil_probe_refuses_a_pc_signal_that_is_already_in_use(tmp_path,built_gil_probe):
    import subprocess,sys
    code=('import signal,sys\n'
          f'sys.path.insert(0,{str(Path(__file__).parents[1]/"scripts")!r})\n'
          'from simulator_execution import GilProbe\n'
          'signal.signal(signal.SIGRTMAX-3,lambda *a:None)\n'
          f'p=GilProbe({str(built_gil_probe)!r},{str(tmp_path/"p.txt")!r},80,native_pc=True)\n'
          'try:p.start()\n'
          'except RuntimeError as e:print(e)\n')
    out=subprocess.run([sys.executable,'-c',code],capture_output=True,text=True,timeout=60)
    assert 'GIL probe failed to start (-5)' in out.stdout,out.stderr


def test_loop_timer_records_slow_calls_only_while_the_probe_runs():
    from simulator_execution import Measurements,TimedRLock
    m=Measurements.__new__(Measurements);m.lock=TimedRLock();m.timing=[]
    m.gil_probe=None
    assert m.loop_timer() is None
    class Probe:
        loop_slow_ns=20_000_000
        def note_slow(self,timing,call,a,b):timing.append(dict(event='loop_slow',call=call,duration_ns=b-a))
    m.gil_probe=Probe();timer=m.loop_timer()
    timer('spin_once',0,19_999_999);timer('take_message',0,25_000_000)
    assert m.timing==[dict(event='loop_slow',call='take_message',duration_ns=25_000_000)]
    m.gil_probe.note_slow=lambda *a:1/0;timer=m.loop_timer();timer('spin_once',0,30_000_000)   # never fails the caller


def test_executor_and_acquisition_loops_time_their_native_calls_only_through_the_probe_timer():
    execute=(Path(__file__).parents[1]/'scripts/perceived_object_grasp_execute.py').read_text()
    assert "timer=getattr(measurements,'loop_timer',lambda:None)() if measurements else None" in execute
    assert "if timer:timer('spin_once',started,spun)" in execute and "if timer:timer('execution_monitor',spun,time.monotonic_ns())" in execute
    source=(Path(__file__).parents[1]/'scripts/simulator_execution.py').read_text()
    for call in ('wait_set_rebuild','wait_set_wait','take_message'):assert f"timer('{call}'" in source


def test_gil_probe_is_opt_in_bounded_and_needs_both_settings(tmp_path,monkeypatch,built_gil_probe):
    from simulator_execution import GIL_PROBE_LIB_ENV,GIL_PROBE_MS_ENV,GilProbe
    monkeypatch.delenv(GIL_PROBE_LIB_ENV,raising=False);monkeypatch.delenv(GIL_PROBE_MS_ENV,raising=False)
    assert GilProbe.from_environment(tmp_path/'p.txt') is None and not (tmp_path/'p.txt').exists()
    monkeypatch.setenv(GIL_PROBE_MS_ENV,'80')
    with pytest.raises(RuntimeError,match='both its library and its threshold'):GilProbe.from_environment(tmp_path/'p.txt')
    for bad in ('10','2000','nan'):
        with pytest.raises(RuntimeError,match=r'within \[20, 1000\] ms'):GilProbe(built_gil_probe,tmp_path/'p.txt',bad)
    monkeypatch.setenv(GIL_PROBE_LIB_ENV,str(built_gil_probe))
    probe=GilProbe.from_environment(tmp_path/'p.txt');assert probe.threshold_ms==80
    probe.stop()                                                   # never started: nothing written
    assert probe.events==[]


def test_gil_probe_is_validated_before_and_stopped_beside_the_other_diagnostics():
    source=(Path(__file__).parents[1]/'scripts/simulator_execution.py').read_text()
    init=source.index("self.gil_probe=GilProbe.from_environment(self.log_path.with_suffix('.gil-probe.txt'))")
    assert init<source.index("self.writer=threading.Thread(target=self._write")
    assert "if self.gil_probe is not None:self.gil_probe.stop();self.timing.extend(self.gil_probe.events)" in source
    assert source.index("self.gil_probe.stop()")<source.index("with_suffix('.timing.json').write_text")


def test_gil_probe_refuses_a_library_built_for_another_python_and_records_what_it_loaded(tmp_path,monkeypatch,built_gil_probe):
    import hashlib,platform
    from simulator_execution import GilProbe
    probe=GilProbe(built_gil_probe,tmp_path/'p.txt',80)
    assert probe.python_version==platform.python_version()
    assert probe.library_sha256==hashlib.sha256(built_gil_probe.read_bytes()).hexdigest()
    monkeypatch.setattr(platform,'python_version',lambda:'3.10.99')
    with pytest.raises(RuntimeError,match='compiled for Python'):GilProbe(built_gil_probe,tmp_path/'q.txt',80)


def test_gil_probe_failed_start_raises_and_writes_nothing(tmp_path,built_gil_probe):
    from simulator_execution import GilProbe
    probe=GilProbe(built_gil_probe,tmp_path/'missing-dir'/'p.txt',80)
    with pytest.raises(RuntimeError,match='failed to start'):probe.start()
    assert probe.started is False and probe.events==[]
    probe.stop()                                                     # still a no-op


def test_gil_probe_starts_before_the_measurement_threads_and_names_them_after():
    source=(Path(__file__).parents[1]/'scripts/simulator_execution.py').read_text()
    start=source.index("if self.gil_probe is not None:self.gil_probe.start()")
    assert start<source.index("self.writer=threading.Thread(target=self._write")
    assert source.index("self.acquisition=threading.Thread(target=self._acquire")<source.index("if self.gil_probe is not None:self.gil_probe.capture_names()")


