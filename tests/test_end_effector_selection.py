"""Configuration-driven end-effector selection: compatibility, readiness, revalidation, apply."""
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'scripts'))
import end_effector_selection as sel  # noqa: E402
from test_tool_interface import SRDF, URDF  # noqa: E402


def cell(**end_effector):
    return {'robot': {'model': 'ur5', 'capability': None},
            'end_effector': dict({'id': 'robotiq_85_gripper', 'capability': 'robotiq_2f_85', 'grasp_frame': 'ee_palm',
                                  'allowed_touch_links': ['gripper_finger1_finger_tip_link', 'gripper_finger2_finger_tip_link']},
                                 **end_effector),
            'builder_task_intent': {'pick': {'grasp': {'policy': 'PREFERRED', 'strategy_ref': 'finger_pinch_basic',
                                                       'required_capability': 'two_finger_parallel'}}}}


ENVIRONMENT = {'end_effector': {'description': {'package': 'robotiq_85_description'}}}


def codes(report, kind='blockers'):
    return {item['code'] for item in report[kind]}


def test_the_installed_2f_with_either_experimental_force_is_compatible_ready_and_revalidated():
    for variant in (None, 'stage_a_20n', 'stage_a_90n'):
        report = sel.evaluate_selection(cell(), environment=ENVIRONMENT, hold_variant=variant, urdf_xml=URDF, srdf_xml=SRDF)
        assert (report['compatibility'], report['execution_readiness'], report['description_revalidation']) == \
            ('COMPATIBLE', 'READY', 'VALID'), report['blockers']
        assert report['robot'] == {'id': 'ur5', 'family': 'serial_6_axis'}
        assert report['hold_profile']['force_variant_status'] == 'experimental'      # never declared reliable
    assert sel.evaluate_selection(cell(), environment=ENVIRONMENT, hold_variant='stage_a_90n')['hold_profile'][
        'commanded_grasp_force_n'] == 90.4


@pytest.mark.parametrize('capability,expected', [
    ('robotiq_3f', {'REQUIRED_CAPABILITY_UNAVAILABLE', 'TOOL_MODEL_CHANGE_REQUIRES_SCENE_GENERATION',
                    'TOOL_CLOSURE_UNCOMMISSIONED', 'DESCRIPTION_REVALIDATION_FAILED'}),
    ('onrobot_airpick_style', {'REQUIRED_CAPABILITY_UNAVAILABLE', 'TOOL_MODEL_CHANGE_REQUIRES_SCENE_GENERATION',
                               'TOOL_ENGAGEMENT_UNSUPPORTED'}),
    ('generic_vacuum_array', {'REQUIRED_CAPABILITY_UNAVAILABLE', 'TOOL_MODEL_UNVERIFIED', 'TOOL_ENGAGEMENT_UNSUPPORTED'}),
    ('no_such_tool', {'TOOL_CAPABILITY_UNKNOWN'}),
])
def test_replacing_the_tool_reports_every_reason_it_cannot_run_here(capability, expected):
    report = sel.evaluate_selection(cell(), environment=ENVIRONMENT, capability_id=capability, urdf_xml=URDF, srdf_xml=SRDF)
    assert report['compatibility'] == 'INCOMPATIBLE' and report['execution_readiness'] == 'BLOCKED'
    assert expected <= codes(report), report['blockers']


def test_shared_link_names_cannot_hide_a_tool_model_change():
    # 2F-85 and AirPick both name a gripper_base_link; only the authored model identity catches the swap.
    report = sel.evaluate_selection(cell(), environment=ENVIRONMENT, capability_id='onrobot_airpick_style', urdf_xml=URDF)
    assert report['description_revalidation'] == 'VALID'
    assert 'TOOL_MODEL_CHANGE_REQUIRES_SCENE_GENERATION' in codes(report)


def test_robot_family_strategy_and_touch_link_checks():
    delta = cell(); delta['robot'] = {'model': 'generic_delta_900'}
    assert 'ROBOT_FAMILY_INCOMPATIBLE' in codes(sel.evaluate_selection(delta, environment=ENVIRONMENT))
    exact = cell(); exact['builder_task_intent']['pick']['grasp'].update(policy='EXACT', strategy_ref='suction_top_basic')
    assert 'STRATEGY_TOOL_INCOMPATIBLE' in codes(sel.evaluate_selection(exact, environment=ENVIRONMENT))
    drifted = cell(allowed_touch_links=['gripper_finger1_finger_tip_link'])
    assert 'TOUCH_LINKS_DIFFER_FROM_TOOL' in codes(sel.evaluate_selection(drifted, environment=ENVIRONMENT), 'warnings')
    unknown_robot = cell(); unknown_robot['robot'] = {'model': 'mystery_arm'}
    assert 'ROBOT_CAPABILITY_UNKNOWN' in codes(sel.evaluate_selection(unknown_robot, environment=ENVIRONMENT), 'warnings')


def scene(tmp_path, end_effector=None):
    environment = {'end_effector': dict({'id': 'robotiq_85_gripper', 'description': {'package': 'robotiq_85_description'},
                                         'grasp_frame': 'old_frame', 'allowed_touch_links': ['old_link']},
                                        **(end_effector or {})), 'robot': {'model': 'ur5'}}
    (tmp_path/'environment.yaml').write_text(yaml.safe_dump(environment))
    (tmp_path/'cell_definition.yaml').write_text(yaml.safe_dump(cell()))
    return tmp_path


def test_apply_writes_only_the_end_effector_binding_into_the_authored_environment(tmp_path):
    root = scene(tmp_path)
    report = sel.evaluate_selection(cell(), environment=yaml.safe_load((root/'environment.yaml').read_text()),
                                    hold_variant='stage_a_90n', urdf_xml=URDF, srdf_xml=SRDF)
    applied = sel.apply_selection(root, report)
    environment = yaml.safe_load((root/'environment.yaml').read_text())
    assert environment['end_effector'] == {'id': 'robotiq_85_gripper', 'description': {'package': 'robotiq_85_description'},
        'grasp_frame': 'ee_palm', 'allowed_touch_links': ['gripper_finger1_finger_tip_link', 'gripper_finger2_finger_tip_link'],
        'capability': 'robotiq_2f_85', 'hold_variant': 'stage_a_90n'}
    assert environment['robot'] == {'model': 'ur5'} and applied['end_effector']['hold_variant'] == 'stage_a_90n'


def test_apply_refuses_an_incompatible_or_unrevalidated_selection(tmp_path):
    root = scene(tmp_path)
    before = (root/'environment.yaml').read_text()
    for report in (sel.evaluate_selection(cell(), environment=ENVIRONMENT, capability_id='robotiq_3f'),
                   sel.evaluate_selection(cell(), environment=ENVIRONMENT, urdf_xml=URDF.replace('upper="0.804"', 'upper="0.1"'))):
        with pytest.raises(RuntimeError, match='SELECTION_REJECTED'):
            sel.apply_selection(root, report)
    assert (root/'environment.yaml').read_text() == before


def test_cli_exit_status_follows_the_verdict(tmp_path, capsys):
    root = scene(tmp_path)
    (tmp_path/'robot.urdf').write_text(URDF); (tmp_path/'robot.srdf').write_text(SRDF)
    assert sel.main(['--scene', str(root), '--urdf', str(tmp_path/'robot.urdf'), '--srdf', str(tmp_path/'robot.srdf')]) == 0
    assert 'robotiq_2f_85: compatibility COMPATIBLE, execution READY, description VALID' in capsys.readouterr().out
    assert sel.main(['--scene', str(root), '--capability', 'robotiq_3f', '--json']) == 1
    assert sel.main(['--scene', str(root), '--capability', 'robotiq_3f', '--apply']) == 1
    assert yaml.safe_load((root/'environment.yaml').read_text())['end_effector']['grasp_frame'] == 'old_frame'


@pytest.mark.parametrize('variant,force', [('stage_a_20n', 20.), ('stage_a_90n', 90.4)])
def test_selection_survives_studio_save_reopen_generation_and_hold_receipt(tmp_path, variant, force):
    from r4_1b_cells import author_cell_a, studio_save
    from test_r4_1_authored_cell_flow import generate_and_validate
    from task_intent_authoring import load_authoring
    from end_effector_hold import load_cell_hold_profile, bind_source_description, verify_hold_binding
    from physics_cycle_preflight import check_generated
    scene, steps = author_cell_a(tmp_path)
    assert steps['validation_rc'] == 0
    definition = yaml.safe_load((scene / 'cell_definition.yaml').read_text())
    environment = yaml.safe_load((scene / 'environment.yaml').read_text())
    report = sel.evaluate_selection(definition, environment=environment, hold_variant=variant,
                                    urdf_xml=URDF, srdf_xml=SRDF)
    assert report['execution_readiness'] == 'READY', report['blockers']
    sel.apply_selection(scene, report)
    assert check_generated(scene) is not None           # changed binding must invalidate the old handoff
    studio_save(scene, {})                            # same authored projection used by Builder Save
    reopened = load_authoring(scene)
    assert reopened['task_intent']['safety']['real_hardware_enabled'] is False
    assert generate_and_validate(scene)[0] == 0
    generated = yaml.safe_load((scene / 'cell_definition.yaml').read_text())
    assert generated['end_effector']['hold_variant'] == variant
    assert yaml.safe_load((scene / 'environment.yaml').read_text())['end_effector']['hold_variant'] == variant
    assert check_generated(scene) is None
    profile = load_cell_hold_profile(scene / 'cell_definition.yaml')
    xml = URDF.replace('effort="10"', 'effort="100"').replace('</robot>',
        '<ros2_control name="tool"><joint name="gripper_finger1_joint"><command_interface name="position"/></joint></ros2_control></robot>')
    _, binding = bind_source_description(xml, profile)
    receipt = verify_hold_binding(profile, {'end_effector_hold': binding})
    assert receipt['receipt_binding']['commanded_grasp_force_n'] == force
    assert receipt['receipt_binding']['derived_effort_nm'] == pytest.approx(force / 8.94)
    assert receipt['receipt_binding']['force_variant'] == variant
