"""Capability-driven tool interfaces: schema, lifecycle values, legacy parity and revalidation."""
import copy
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'scripts'))
import tool_interface as ti  # noqa: E402

CATALOG = REPO/'catalog/capabilities/end_effectors'


def record(name):
    return yaml.safe_load((CATALOG/name).read_text())['end_effector']


def test_every_catalog_interface_validates_and_covers_joint_and_io_tools():
    kinds = {}
    for path in sorted(CATALOG.glob('*.yaml')):
        ee = yaml.safe_load(path.read_text())['end_effector']
        if 'interface' not in ee:
            continue
        assert ti.interface_block_errors(ee) == [], path.name
        kinds[ee['id']] = ee['interface']['kind']
    assert kinds == {'robotiq_2f_85': 'joint_position', 'robotiq_3f': 'joint_position',
                     'onrobot_airpick_style': 'io_signal', 'generic_vacuum_array': 'io_signal'}


def test_the_2f_interface_reproduces_the_legacy_pipeline_values_exactly():
    tool, legacy = ti.load_tool_interface('robotiq_2f_85'), ti.LEGACY_TWO_FINGER
    # Main's preplanner sampled open+((closed-open)/80)*i and reserved 0.804/80; bit-identical, not approximately.
    assert tool.closure_positions() == [0.0+((0.804-0.0)/80)*i for i in range(1, 81)] == legacy.closure_positions()
    assert tool.closure_step() == 0.804/80 == legacy.closure_step()
    assert tool.engage_goal(.5) == {'gripper_finger1_joint': .5} == legacy.engage_goal(.5)
    assert tool.release_goal() == {'gripper_finger1_joint': 0.0} == legacy.release_goal()
    assert (tool.planning_group, tool.max_aperture_m, tool.closed_position_rad) == ('gripper', .085, .804)
    assert tool.touch_links == legacy.touch_links and tool.tcp_link == legacy.tcp_link == 'ee_palm'
    assert ti.interface_from_contract({}) is legacy                       # an unbound cell keeps the legacy values
    assert ti.interface_from_contract({'tool_interface': tool.as_contract()}) == tool


def test_multi_joint_engagement_closes_every_command_joint_by_the_same_travel():
    block = copy.deepcopy(record('ee_robotiq_3f.yaml'))
    block['interface']['closure']['closed_position_rad'] = 1.2
    block['interface']['open_positions_rad']['finger_middle_joint_1'] = 0.1
    tool = ti.interface_from_record('three', block)
    assert tool.engage_goal(.5) == {'finger_1_joint_1': .5, 'finger_2_joint_1': .5, 'finger_middle_joint_1': .6}
    assert tool.closure_positions()[-1] == pytest.approx(1.2) and len(tool.closure_positions()) == 80
    assert tool.feedback == {'method': 'measured_contacts', 'contact_links': ['finger_1_link_3', 'finger_2_link_3', 'finger_middle_link_3']}


def test_uncommissioned_and_io_tools_fail_closed_before_any_engagement():
    three = ti.load_tool_interface('robotiq_3f')
    with pytest.raises(ti.ToolEngagementUnsupported, match='TOOL_CLOSURE_UNCOMMISSIONED'):
        three.closure_positions()
    suction = ti.load_tool_interface('onrobot_airpick_style')
    for call in (suction.closure_positions, lambda: suction.engage_goal(.1), lambda: suction.closure_joint):
        with pytest.raises(ti.ToolEngagementUnsupported, match='TOOL_ENGAGEMENT_UNSUPPORTED'):
            call()
    assert suction.release_goal() == {}
    assert suction.feedback == {'method': 'vacuum_signal', 'signal': 'vacuum_on'}
    assert suction.signals['release_mode'] == 'pulse'


@pytest.mark.parametrize('name,change,message', [
    ('ee_robotiq_2f.yaml', lambda b: b.update(kind='magnetic'), 'kind'),
    ('ee_robotiq_2f.yaml', lambda b: b['open_positions_rad'].pop('gripper_finger1_joint'), 'open_positions_rad'),
    ('ee_robotiq_2f.yaml', lambda b: b['closure'].update(closed_position_rad=-1.0), 'closed_position_rad'),
    ('ee_robotiq_2f.yaml', lambda b: b['closure'].update(search_steps=1), 'search_steps'),
    ('ee_robotiq_2f.yaml', lambda b: b.update(max_aperture_m=0), 'max_aperture_m'),
    ('ee_robotiq_2f.yaml', lambda b: b.update(touch_links=[]), 'touch_links'),
    ('ee_robotiq_2f.yaml', lambda b: b['recovery'].update(release_outside_destination=True), 'recovery'),
    ('ee_robotiq_3f.yaml', lambda b: b['closure'].pop('commissioning_note'), 'commissioning_note'),
    ('ee_airpick_suction.yaml', lambda b: b['signals'].update(engage='not_wired'), 'required_io_signals'),
    ('ee_airpick_suction.yaml', lambda b: b['signals'].update(release_mode='maybe'), 'release_mode'),
])
def test_invalid_interface_blocks_are_rejected(name, change, message):
    block = copy.deepcopy(record(name)); change(block['interface'])
    errors = ti.interface_block_errors(block)
    assert errors and any(message in e for e in errors), errors
    with pytest.raises(ti.ToolInterfaceError):
        ti.interface_from_record('x', block)


def test_a_capability_without_an_interface_is_refused():
    with pytest.raises(ti.ToolInterfaceError, match='TOOL_INTERFACE_REQUIRED'):
        ti.load_tool_interface('suction_basic')
    with pytest.raises(ti.ToolInterfaceError, match='TOOL_INTERFACE_REQUIRED'):
        ti.load_tool_interface(None)


URDF = '''<robot name="r">
  <link name="tool0"/><link name="ee_palm"/><link name="gripper_base_link"><collision/></link>
  <link name="gripper_finger1_finger_tip_link"><collision/></link>
  <link name="gripper_finger2_finger_tip_link"><collision/></link>
  <link name="gripper_finger1_knuckle_link"/>
  <joint name="gripper_finger1_joint" type="revolute"><limit lower="0.0" upper="0.804" effort="10" velocity="2"/></joint>
  <joint name="gripper_finger2_joint" type="revolute"><mimic joint="gripper_finger1_joint"/><limit lower="-3.14" upper="3.14"/></joint>
</robot>'''
SRDF = '<robot name="r"><group name="gripper"><joint name="gripper_finger1_joint"/></group></robot>'


def test_revalidation_passes_only_against_a_matching_description():
    tool = ti.load_tool_interface('robotiq_2f_85')
    report = ti.revalidate_against_description(tool, URDF, srdf_xml=SRDF)
    assert report['status'] == 'VALID' and {c['code'] for c in report['checks']} >= {
        'TOUCH_LINK_PRESENT', 'TOUCH_LINK_HAS_COLLISION', 'TCP_LINK_PRESENT', 'COMMAND_JOINT_ACTUATED',
        'OPEN_POSITION_WITHIN_LIMITS', 'CLOSED_POSITION_WITHIN_LIMITS', 'PLANNING_GROUP_HAS_COMMAND_JOINTS'}
    failing = lambda xml, **kw: {c['code'] for c in ti.revalidate_against_description(tool, xml, **kw)['checks']
                                 if c['status'] == 'FAIL'}
    assert failing(URDF.replace('upper="0.804"', 'upper="0.7"')) == {'CLOSED_POSITION_WITHIN_LIMITS'}
    assert failing(URDF.replace('<link name="gripper_finger2_finger_tip_link"><collision/></link>',
                                '<link name="gripper_finger2_finger_tip_link"/>')) == {'TOUCH_LINK_HAS_COLLISION'}
    assert failing(URDF, tcp_link='airpick_tcp') == {'TCP_LINK_PRESENT'}
    assert failing(URDF, srdf_xml=SRDF.replace('gripper_finger1_joint', 'other')) == {'PLANNING_GROUP_HAS_COMMAND_JOINTS'}
    assert ti.revalidate_against_description(tool, '<robot', srdf_xml=None)['status'] == 'INVALID'
    # A mimic follower is not an actuated command joint.
    mimic = tool.as_contract() | {'command_joints': ['gripper_finger2_joint'],
                                  'open_positions_rad': {'gripper_finger2_joint': 0.0}}
    assert 'COMMAND_JOINT_ACTUATED' in {c['code'] for c in ti.revalidate_against_description(
        ti.interface_from_contract({'tool_interface': mimic}), URDF)['checks'] if c['status'] == 'FAIL'}


def test_a_three_finger_tool_does_not_revalidate_against_a_two_finger_description():
    report = ti.revalidate_against_description(ti.load_tool_interface('robotiq_3f'), URDF, srdf_xml=SRDF)
    failed = {c['code'] for c in report['checks'] if c['status'] == 'FAIL'}
    assert report['status'] == 'INVALID' and {'COMMAND_JOINT_PRESENT', 'TOUCH_LINK_PRESENT', 'TCP_LINK_PRESENT'} <= failed


def test_unbound_interface_uses_the_launched_cell_tool_instead_of_legacy_joint_names():
    contract = dict(tool_command_joint='cell_finger', tool_group='cell_tool', tool_open_rad=.1,
                    tool_closed_limit_rad=.9, tool_aperture_max_m=.12,
                    allowed_touch_links=['cell_left', 'cell_right'], grasp_frame='cell_tcp')
    tool = ti.interface_from_contract(contract)
    assert tool.release_goal() == {'cell_finger': .1}
    assert tool.engage_goal(.5) == {'cell_finger': .5}
    assert tool.closure_positions()[0] == pytest.approx(.11)
    assert tool.closure_positions()[-1] == pytest.approx(.9)
    assert tool.planning_group == 'cell_tool' and tool.max_aperture_m == .12
    assert tool.tcp_link == 'cell_tcp' and tool.touch_links == ('cell_left', 'cell_right')


@pytest.mark.parametrize('change', [
    {'tool_command_joint': 'different_joint'}, {'tool_group': 'different_group'},
    {'tool_open_rad': .1}, {'tool_closed_limit_rad': .7}, {'tool_aperture_max_m': .07},
])
def test_catalog_interface_cannot_override_a_conflicting_launched_tool(change):
    contract = dict(tool_interface=ti.load_tool_interface('robotiq_2f_85').as_contract(),
                    tool_command_joint='gripper_finger1_joint', tool_group='gripper',
                    tool_open_rad=0., tool_closed_limit_rad=.804, tool_aperture_max_m=.085)
    contract.update(change)
    with pytest.raises(ti.ToolInterfaceError, match='TOOL_BINDING_MISMATCH'):
        ti.interface_from_contract(contract)


def test_catalog_interface_can_use_a_commissioned_subrange_of_the_launched_joint():
    contract = dict(tool_interface=ti.load_tool_interface('robotiq_2f_85').as_contract(),
                    tool_command_joint='gripper_finger1_joint', tool_group='gripper',
                    tool_open_rad=-.1, tool_closed_limit_rad=1., tool_aperture_max_m=.1)
    tool = ti.interface_from_contract(contract)
    assert tool.release_goal() == {'gripper_finger1_joint': 0.}
    assert tool.closure_positions()[-1] == pytest.approx(.804)
