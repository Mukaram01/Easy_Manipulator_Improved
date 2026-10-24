"""End-effector hold profiles: family methods, actuation binding and receipt binding."""
import copy
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

REPO=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(REPO/'scripts'))
import end_effector_hold as hold  # noqa: E402

CATALOG=REPO/'catalog/capabilities/end_effectors'


def record(name):
    return yaml.safe_load((CATALOG/name).read_text())['end_effector']


def test_every_catalog_hold_block_validates_and_matches_its_family():
    seen=set()
    for path in sorted(CATALOG.glob('*.yaml')):
        ee=yaml.safe_load(path.read_text())['end_effector']
        if 'hold' not in ee:continue
        assert hold.hold_block_errors(ee)==[],path.name
        assert ee['hold']['method']==hold.FAMILY_METHODS[ee['family']]
        seen.add(ee['hold']['method'])
    assert seen=={'opposing_contact','multi_contact','vacuum_seal'}     # two-finger, three-finger, vacuum


@pytest.mark.parametrize('change,message',[
    (lambda h:h.update(method='multi_contact'),"must be 'opposing_contact'"),
    (lambda h:h['contact_verification'].update(min_contact_links=1),'min_contact_links: all'),
    (lambda h:h.update(hold_reserve_rad=0.02),'hold_reserve_rad'),
    (lambda h:h.update(hold_reserve_rad=0.),'hold_reserve_rad'),
    (lambda h:h['actuation'].update(commanded_grasp_force_n=300),'within grasp_force_range_n'),
    (lambda h:h['actuation'].update(commanded_grasp_force_n=5),'within grasp_force_range_n'),
    (lambda h:h['actuation'].update(grasp_force_range_n=[235,20]),'0 < min < max'),
    (lambda h:h['actuation']['pad_force_per_actuator_torque'].update(n_per_nm=0),'n_per_nm'),
    (lambda h:h['actuation']['pad_force_per_actuator_torque'].pop('evidence'),'evidence'),
    (lambda h:h.update(schema='end_effector_hold/v0'),'schema'),
    (lambda h:h.pop('release'),'release'),
])
def test_two_finger_hold_block_is_validated_fail_closed(change,message):
    ee=copy.deepcopy(record('ee_robotiq_2f.yaml'));change(ee['hold'])
    errors=hold.hold_block_errors(ee)
    assert errors and any(message in e for e in errors),errors
    with pytest.raises(hold.HoldProfileError):hold.profile_from_record('x',ee,'p',b'')


def test_vacuum_hold_block_requires_signal_seal_and_explicit_simulator_gap():
    ee=copy.deepcopy(record('ee_airpick_suction.yaml'))
    assert hold.hold_block_errors(ee)==[]
    for change,message in [(lambda v:v.update(vacuum_signal='nope'),'required_io_signals'),
                           (lambda v:v.update(seal_contact_links='any'),'seal_contact_links: all'),
                           (lambda v:v.update(simulator_support='assumed'),'simulator_support: none'),
                           (lambda v:v.update(effective_area_mm2=-1),'positive numbers')]:
        bad=copy.deepcopy(ee);change(bad['hold']['vacuum'])
        assert any(message in e for e in hold.hold_block_errors(bad))


def test_two_finger_profile_requires_both_cell_contact_links_and_derives_the_actuator_cap():
    profile=hold.load_hold_profile('robotiq_2f_85')
    assert profile.method=='opposing_contact' and profile.hold_reserve_rad==0.002
    assert profile.required_contact_links(['f1','f2'],{'f1'})==['f1','f2']      # link names come from the cell
    assert profile.contact_verified(['f1','f2'],{'f1','f2'}) and not profile.contact_verified(['f1','f2'],{'f1'})
    commanded=profile.actuation['commanded_grasp_force_n']            # the frozen functional value lives in the catalog
    assert 20<=commanded<=235 and profile.expected_pad_force_n()==commanded
    assert profile.simulated_actuator_effort_nm()==pytest.approx(commanded/8.94)
    assert profile.sha256==__import__('hashlib').sha256((CATALOG/'ee_robotiq_2f.yaml').read_bytes()).hexdigest()
    evidence=profile.evidence()
    assert evidence['capability_id']=='robotiq_2f_85' and evidence['commanded_grasp_force_n']==commanded
    assert evidence['grasp_force_range_n']==[20,235] and evidence['simulated_actuator_effort_nm']==pytest.approx(commanded/8.94)
    with pytest.raises(hold.HoldProfileError):profile.required_contact_links(['only'],{'only'})


def test_three_finger_profile_accepts_any_two_measured_fingers():
    profile=hold.load_hold_profile('robotiq_3f')
    assert profile.method=='multi_contact' and profile.min_contact_links==2
    assert profile.required_contact_links(['a','b','c'],{'b','c'})==['b','c']
    assert profile.contact_verified(['a','b','c'],{'a','c'})
    assert not profile.contact_verified(['a','b','c'],{'c'})
    assert profile.required_contact_links(['a','b','c'],{'c'})==['a','b']      # not satisfiable: callers fail


def test_vacuum_profiles_fail_closed_without_a_measured_seal_source():
    for capability in ('onrobot_airpick_style','generic_vacuum_array'):
        profile=hold.load_hold_profile(capability)
        assert profile.method=='vacuum_seal' and profile.simulated_actuator_effort_nm() is None
        with pytest.raises(hold.HoldVerificationUnsupported,match='HOLD_VERIFICATION_UNSUPPORTED'):
            profile.contact_verified(['cup'],{'cup'})
        assert profile.evidence()['vacuum']['simulator_support']=='none'


@pytest.mark.parametrize('capability',[None,'','missing_tool','suction_basic'])
def test_unbound_or_profileless_capability_is_refused(capability):
    with pytest.raises(hold.HoldProfileError):hold.load_hold_profile(capability)


def test_the_stage_a_cell_binds_the_two_finger_profile_and_unbound_cells_bind_none(tmp_path):
    bound=hold.load_cell_hold_profile(REPO/'scenes/ur5_2f_test/cell_definition.yaml')
    assert bound.capability_id=='robotiq_2f_85'
    (tmp_path/'cell_definition.yaml').write_text('end_effector: {capability: null}\n')
    assert hold.load_cell_hold_profile(tmp_path/'cell_definition.yaml') is None


ROBOT='''<robot><joint name="arm"><limit effort="150"/></joint><joint name="leader"><limit effort="10.114"/></joint>
<joint name="f1"><mimic joint="leader" multiplier="1"/><limit effort="0.1"/></joint>
<joint name="f2"><mimic joint="leader" multiplier="-1"/><limit effort="0.1"/></joint></robot>'''


def test_actuation_binding_lowers_only_the_commanded_linkage_driver_and_records_it():
    profile=hold.load_hold_profile('robotiq_2f_85');root=ET.fromstring(ROBOT)
    binding=hold.bind_actuation(root,['arm','leader'],profile)
    expected=profile.actuation['commanded_grasp_force_n']/8.94
    assert float(root.find("joint[@name='leader']/limit").get('effort'))==pytest.approx(expected)
    assert root.find("joint[@name='arm']/limit").get('effort')=='150'
    assert [j.find('limit').get('effort') for j in root.findall('joint') if j.find('mimic') is not None]==['0.1','0.1']
    assert binding['actuated_joint']=='leader' and binding['source_effort_nm']==10.114
    assert binding['derived_effort_nm']==pytest.approx(expected) and binding['commanded_grasp_force_n']==profile.actuation['commanded_grasp_force_n']
    assert binding['sha256']==profile.sha256 and binding['hold_reserve_rad']==0.002


def test_actuation_binding_never_raises_the_authored_limit_and_needs_one_driver():
    profile=hold.load_hold_profile('robotiq_2f_85')
    weak=ROBOT.replace('effort="10.114"','effort="1.0"')          # below any in-range commanded force
    with pytest.raises(hold.HoldProfileError,match='EXCEEDS_AUTHORED_LIMIT'):
        hold.bind_actuation(ET.fromstring(weak),['arm','leader'],profile)
    with pytest.raises(hold.HoldProfileError,match='HOLD_ACTUATION_UNBOUND'):
        hold.bind_actuation(ET.fromstring(ROBOT),['arm'],profile)             # driver is not commanded
    with pytest.raises(hold.HoldProfileError,match='HOLD_ACTUATION_UNBOUND'):
        hold.bind_actuation(ET.fromstring(ROBOT),['arm','leader'],hold.load_hold_profile('onrobot_airpick_style'))


def test_receipt_binding_must_match_the_loaded_profile_exactly():
    profile=hold.load_hold_profile('robotiq_2f_85')
    binding=hold.bind_actuation(ET.fromstring(ROBOT),['arm','leader'],profile)
    evidence=hold.verify_hold_binding(profile,{'end_effector_hold':binding})
    assert evidence['profile']['sha256']==profile.sha256 and evidence['receipt_binding']==binding
    for change in [lambda b:b.update(sha256='0'*64),lambda b:b.update(derived_effort_nm=5.),
                   lambda b:b.update(hold_reserve_rad=.003),lambda b:b.update(capability_id='other'),
                   lambda b:b.update(method='multi_contact')]:
        bad=dict(binding);change(bad)
        with pytest.raises(RuntimeError,match='HOLD_PROFILE_BINDING_MISMATCH'):
            hold.verify_hold_binding(profile,{'end_effector_hold':bad})
    with pytest.raises(RuntimeError,match='HOLD_PROFILE_BINDING_MISSING'):hold.verify_hold_binding(profile,{})


SOURCE=('<robot><joint name="arm"><limit effort="150"/></joint><joint name="leader"><limit effort="10.114"/></joint>'
        '<joint name="f1"><mimic joint="leader" multiplier="1"/><limit effort="0.1"/></joint>'
        '<ros2_control name="arm"><joint name="arm"><command_interface name="position"/></joint></ros2_control>'
        '<ros2_control name="hand"><joint name="leader"><command_interface name="position"/></joint>'
        '<joint name="f1"><param name="mimic">leader</param></joint></ros2_control></robot>')


def test_source_description_binding_and_spec_augmentation_stay_outside_the_evidence_bound_backend(tmp_path):
    import json
    profile=hold.load_hold_profile('robotiq_2f_85')
    bound,binding=hold.bind_source_description(SOURCE,profile)
    root=ET.fromstring(bound)
    assert float(root.find("joint[@name='leader']/limit").get('effort'))==pytest.approx(profile.actuation['commanded_grasp_force_n']/8.94)
    assert root.find("joint[@name='arm']/limit").get('effort')=='150'
    assert binding['actuated_joint']=='leader' and binding['source_effort_nm']==10.114
    (tmp_path/'spec.json').write_text(json.dumps({'world':'a0'}))
    assert hold.augment_spec(tmp_path,binding)==binding
    assert json.loads((tmp_path/'spec.json').read_text())=={'world':'a0','end_effector_hold':binding}
    launch=(REPO/'scenes/ur5_2f_test/launch/demo.launch.py').read_text()
    simulator=launch[launch.index('if backend == "simulator":'):launch.index('ExecuteProcess(cmd=[sys.executable, simulator_backend.__file__')]
    assert simulator.index('end_effector_hold.load_cell_hold_profile(')<simulator.index('end_effector_hold.bind_source_description(')\
        <simulator.index('simulator_backend.prepare(')<simulator.index('end_effector_hold.augment_spec(output, hold_binding)')
    assert 'requires the cell to bind an end-effector capability with a hold profile' in simulator
    backend=(REPO/'scripts/simulator_backend.py').read_text()
    assert 'end_effector_hold' not in backend and 'hold_profile' not in backend      # evidence-bound module untouched
