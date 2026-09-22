import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "perceived_object_grasp_execute.py"
SPEC = importlib.util.spec_from_file_location("perceived_object_grasp_execute", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def parameter(value):
    return SimpleNamespace(bool_value=value)


def component(class_type):
    return SimpleNamespace(class_type=class_type)


def box(object_id, dimensions):
    return {"id": object_id, "shape": "BOX", "frame_id": "world",
            "dimensions": dimensions, "pose": [0, 0, 0, 0, 0, 0, 1]}


def test_graspable_selection_rejects_oversize_lower_id_and_preserves_live_id():
    selected = MODULE.select_graspable_box([
        box("3", [0.40, 0.27, 0.05]), box("4", [0.11, 0.06, 0.05])])
    assert selected["id"] == "4"


def test_supported_object_proxy_penetration_is_corrected_minimally():
    mouse = box("6", [0.11, 0.06, 0.047])
    mouse["pose"][2] = 0.312
    suitcase = box("3", [0.40, 0.27, 0.046])
    suitcase["pose"][2] = 0.286
    correction, support_id = MODULE.support_penetration_correction(mouse, [suitcase, mouse])
    assert support_id == "3"
    assert correction == pytest.approx(0.0215)


def test_canonical_place_target_rejects_layout_without_physical_handoff(tmp_path):
    layout = tmp_path / "layout"
    layout.mkdir()
    (layout / "workcell_studio_layout.yaml").write_text(
        "items:\n- id: target_bin_default\n  pose:\n    xyz: [0.25, 0.45, 0.20]\n",
        encoding="utf-8")
    with pytest.raises(RuntimeError, match="generated cell handoff is missing"):
        MODULE.load_canonical_place_target(tmp_path)


def test_translated_pose_preserves_orientation_and_input():
    from geometry_msgs.msg import PoseStamped

    original = PoseStamped()
    original.pose.position.x = 1.0
    original.pose.orientation.w = 1.0
    translated = MODULE.translated_pose(original, dx=0.2, dy=-0.3, dz=0.4)
    assert [translated.pose.position.x, translated.pose.position.y,
            translated.pose.position.z] == pytest.approx([1.2, -0.3, 0.4])
    assert translated.pose.orientation.w == 1.0
    assert original.pose.position.x == 1.0


def test_cartesian_corridor_accepts_horizontal_x_path_and_rejects_departure():
    start = [0.50, -0.20, 0.30, 0.0, -0.7071067811865475, 0.0, 0.7071067811865476]
    goal = [0.42, -0.20, 0.30, 0.0, -0.7071067811865475, 0.0, 0.7071067811865476]
    assert MODULE.pose_within_cartesian_corridor(
        [0.46, -0.199, 0.30] + goal[3:], start, goal)
    assert not MODULE.pose_within_cartesian_corridor(
        [0.46, -0.19, 0.30] + goal[3:], start, goal)
    assert not MODULE.pose_within_cartesian_corridor(
        [0.41, -0.20, 0.30] + goal[3:], start, goal)


def test_candidate_three_is_prioritized_without_dropping_existing_candidates():
    indices = MODULE.candidate_indices(16)
    assert indices[0] == 3
    assert sorted(indices) == list(range(16))


def test_plan_only_retry_policy_accepts_only_measured_stochastic_failures():
    # The Stage-A workstation observed a fresh pregrasp TIMED_OUT (-6) after
    # the same bound candidate had previously passed. Retrying keeps the exact
    # request/scene/candidate; other planning failures remain terminal.
    assert MODULE.retryable_plan_failure(-2)  # INVALID_MOTION_PLAN
    assert MODULE.retryable_plan_failure(-6)  # TIMED_OUT
    for code in (-1, -3, -4, -5, -7, 0, 1):
        assert not MODULE.retryable_plan_failure(code)


def test_plan_segment_timeout_retry_reuses_identical_private_request():
    """Exercise the real nested planner boundary, not the helper in isolation."""
    import ast
    import copy
    import time
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (
        Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
        PlanningScene, RobotState, RobotTrajectory)
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    tree = ast.parse(SCRIPT.read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    segment = next(n for n in main.body if isinstance(n, ast.FunctionDef) and n.name == 'plan_segment')

    initial = PlanningScene(robot_state=RobotState(
        joint_state=JointState(name=['arm'], position=[0.0])))
    original = copy.deepcopy(initial)
    trajectory = RobotTrajectory(joint_trajectory=JointTrajectory(
        joint_names=['arm'], points=[
            JointTrajectoryPoint(positions=[0.0]),
            JointTrajectoryPoint(positions=[0.5]),
        ]))
    goals = []
    def action(client, goal, timeout):
        goals.append(copy.deepcopy(goal))
        if len(goals) < 3:
            raise MODULE.MoveItActionFailure(6, -6)
        return MoveGroup.Result(
            error_code=MoveItErrorCodes(val=1),
            trajectory_start=copy.deepcopy(original.robot_state),
            planned_trajectory=trajectory,
            planning_time=.1)

    summary = {}
    context = dict(
        vars(MODULE),
        copy=copy, time=time, MotionPlanRequest=MotionPlanRequest, MoveGroup=MoveGroup,
        stage=lambda name: None, deadline=time.monotonic()+10,
        contract={'planning_group':'arm_group','home_joint_names':['arm'],'tool_link':'tcp'},
        args=SimpleNamespace(segment_planning_time=3.), mimics=[], initial=initial,
        plan_client=object(), action=action, trace=lambda *args: None, summary=summary,
        joint_constraints=lambda values: Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=v, tolerance_above=.0001,
                            tolerance_below=.0001, weight=1.) for n, v in values.items()]))
    exec(compile(ast.Module(body=[segment], type_ignores=[]),
                 '<actual-plan-segment-timeout-retry>', 'exec'), context)

    result = context['plan_segment'](
        initial, 'PREPLAN_APPROACH', {'arm': 0.5}, group='arm_group')

    assert len(goals) == 3
    assert goals[0] == goals[1] == goals[2]
    assert initial == original
    assert summary['planning_retries'] == [
        {'stage':'PREPLAN_APPROACH','moveit_code':-6,'attempt':1,'retry_kind':'timed_out',
         'search_pass':'normal'},
        {'stage':'PREPLAN_APPROACH','moveit_code':-6,'attempt':2,'retry_kind':'timed_out',
         'search_pass':'normal'},
    ]
    assert result['metadata']['success'] is True
    assert result['metadata']['planning_attempts'] == 3
    assert result['metadata']['allowed_planning_time'] == pytest.approx(3.0)
    assert result['after'].robot_state.joint_state.position == pytest.approx([0.5])


def test_plan_segment_discovery_policy_does_not_retry_before_other_candidates():
    """The unresolved discovery pass gets exactly one bounded planning window."""
    import ast
    import copy
    import time
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (
        Constraints, JointConstraint, MotionPlanRequest, PlanningScene, RobotState)
    from sensor_msgs.msg import JointState

    tree = ast.parse(SCRIPT.read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    segment = next(n for n in main.body if isinstance(n, ast.FunctionDef) and n.name == 'plan_segment')

    initial = PlanningScene(robot_state=RobotState(
        joint_state=JointState(name=['arm'], position=[0.0])))
    goals = []
    def action(client, goal, timeout):
        goals.append(copy.deepcopy(goal))
        raise MODULE.MoveItActionFailure(6, -6)

    contract = {
        'planning_group':'arm_group', 'home_joint_names':['arm'], 'tool_link':'tcp',
        '_candidate_planning_attempts':1, '_candidate_planning_time':1.0,
        '_candidate_search_pass':'discovery',
    }
    summary = {}
    context = dict(
        vars(MODULE),
        copy=copy, time=time, MotionPlanRequest=MotionPlanRequest, MoveGroup=MoveGroup,
        stage=lambda name: None, deadline=time.monotonic()+10, contract=contract,
        args=SimpleNamespace(segment_planning_time=3.), mimics=[], initial=initial,
        plan_client=object(), action=action, trace=lambda *args: None, summary=summary,
        joint_constraints=lambda values: Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=v, tolerance_above=.0001,
                            tolerance_below=.0001, weight=1.) for n, v in values.items()]))
    exec(compile(ast.Module(body=[segment], type_ignores=[]),
                 '<actual-plan-segment-discovery>', 'exec'), context)

    with pytest.raises(RuntimeError, match='MoveIt action failed'):
        context['plan_segment'](
            initial, 'PREPLAN_APPROACH', {'arm': 0.5}, group='arm_group')

    assert len(goals) == 1
    assert summary.get('planning_retries', []) == []
    assert goals[0].request.allowed_planning_time == pytest.approx(1.0)


def test_straight_segments_bind_ompl_to_cartesian_corridor_before_postcheck():
    source = SCRIPT.read_text()
    assert 'cartesian_corridor=(a, b)' in source
    assert 'request.path_constraints = cartesian_corridor_constraints(' in source
    assert 'PositionConstraint()' in source
    assert 'OrientationConstraint()' in source
    # Keep the independent trajectory-point verification as a second guard.
    assert 'pose_within_cartesian_corridor(actual_pose, a, b)' in source


def test_fake_hardware_guard_requires_moveit_flag_and_mock_component():
    evidence = MODULE.fake_hardware_evidence(
        [parameter(True)], [component("mock_components/GenericSystem")])
    assert evidence["real_hardware"] is False


@pytest.mark.parametrize("params,components", [
    ([parameter(False)], [component("mock_components/GenericSystem")]),
    ([parameter(True)], [component("ur_robot_driver/URPositionHardwareInterface")]),
    ([parameter(True)], []),
])
def test_fake_hardware_guard_fails_closed(params, components):
    with pytest.raises(RuntimeError, match="execution rejected"):
        MODULE.fake_hardware_evidence(params, components)


def test_attachment_preserves_id_and_atomically_removes_world_object():
    from moveit_msgs.msg import CollisionObject

    original = CollisionObject()
    original.id = "17"
    original.header.frame_id = "world"
    diff = MODULE.attachment_diff(original, "ee_palm", ["ee_palm", "tool0"])
    assert diff.world.collision_objects[0].id == "17"
    assert diff.world.collision_objects[0].operation == CollisionObject.REMOVE
    attached = diff.robot_state.attached_collision_objects[0]
    assert attached.object.id == "17"
    assert attached.object.operation == CollisionObject.ADD
    assert attached.link_name == "ee_palm"


def test_attachment_tolerates_live_lifecycle_removing_world_object_first():
    from moveit_msgs.msg import CollisionObject

    original = CollisionObject()
    original.id = "17"
    original.header.frame_id = "world"
    diff = MODULE.attachment_diff(original, "ee_palm", ["ee_palm"], remove_world=False)
    assert diff.world.collision_objects == []
    assert diff.robot_state.attached_collision_objects[0].object.id == "17"


def test_attachment_status_requires_removed_world_and_matching_link():
    attached = SimpleNamespace(object=SimpleNamespace(id="1"), link_name="ee_palm")
    scene = SimpleNamespace(world=SimpleNamespace(collision_objects=[]),
                            robot_state=SimpleNamespace(attached_collision_objects=[attached]))
    assert MODULE.attachment_status(scene, "1", "ee_palm")["valid"] is True
    scene.world.collision_objects.append(SimpleNamespace(id="1"))
    assert MODULE.attachment_status(scene, "1", "ee_palm")["valid"] is False


def test_failure_cleanup_removes_exact_attachment():
    from moveit_msgs.msg import CollisionObject

    cleanup = MODULE.detachment_cleanup_diff("23", "ee_palm")
    item = cleanup.robot_state.attached_collision_objects[0]
    assert item.object.id == "23"
    assert item.object.operation == CollisionObject.REMOVE
    assert item.link_name == "ee_palm"


def test_place_detachment_preserves_id_and_restores_one_world_object():
    from moveit_msgs.msg import CollisionObject

    original = CollisionObject()
    original.id = "6"
    original.pose.orientation.w = 1.0
    diff = MODULE.place_detachment_diff(original, "ee_palm", [0.25, 0.45, 0.20])
    detached = diff.robot_state.attached_collision_objects[0]
    placed = diff.world.collision_objects[0]
    assert detached.object.id == placed.id == "6"
    assert detached.object.operation == CollisionObject.REMOVE
    assert placed.operation == CollisionObject.ADD
    assert [placed.pose.position.x, placed.pose.position.y, placed.pose.position.z] == [0.25, 0.45, 0.20]


def matrix_fixture():
    from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
    # Existing adjacent-link allowance must survive the contact transition.
    return AllowedCollisionMatrix(
        entry_names=["tip1", "tip2", "palm", "camera", "epd::other"],
        entry_values=[AllowedCollisionEntry(enabled=row) for row in [
            [False, False, True, False, False],
            [False, False, True, False, False],
            [True, True, False, False, False],
            [False] * 5, [False] * 5]])


def test_target_contact_preserves_existing_acm_and_other_objects():
    base = matrix_fixture()
    changed = MODULE.target_contact_matrix(base, "epd::selected", ["tip1", "tip2"])
    assert len(base.entry_names) == 5
    for i in range(5):
        assert changed.entry_values[i].enabled[:5] == base.entry_values[i].enabled
    assert changed.entry_values[-1].enabled == [True, True, False, False, False, False]


@pytest.mark.parametrize("fail_at", ["planning", "execution", "attachment", None])
def test_contact_acm_is_restored_on_every_exit(fail_at):
    baseline = matrix_fixture()
    applied = []
    def work():
        with MODULE.temporary_target_contact(baseline, "epd::selected", ["tip1", "tip2"], applied.append):
            if fail_at:
                raise RuntimeError(fail_at)
    if fail_at:
        with pytest.raises(RuntimeError, match=fail_at):
            work()
    else:
        work()
    assert applied[-1] == baseline


def test_acm_restore_attempted_after_uncertain_apply_failure():
    baseline = matrix_fixture()
    applied = []
    def apply(matrix):
        applied.append(matrix)
        if len(applied) == 1:
            raise RuntimeError("service response lost")
    with pytest.raises(RuntimeError, match="response lost"):
        with MODULE.temporary_target_contact(baseline, "epd::selected", ["tip1"], apply):
            pytest.fail("must not enter contact stage")
    assert applied[-1] == baseline


def test_contact_rejects_unknown_links_and_broad_target_defaults():
    baseline = matrix_fixture()
    with pytest.raises(ValueError, match="absent"):
        MODULE.target_contact_matrix(baseline, "epd::selected", ["missing"])
    baseline.default_entry_names = ["epd::selected"]
    baseline.default_entry_values = [True]
    with pytest.raises(ValueError, match="broad"):
        MODULE.target_contact_matrix(baseline, "epd::selected", ["tip1"])


@pytest.mark.parametrize("pairs", [[], [("epd::selected", "palm")],
                                    [("epd::other", "tip1")], [("camera", "wrist")]])
def test_grasp_rejects_missing_or_unintended_contacts(pairs):
    contacts = [SimpleNamespace(contact_body_1=a, contact_body_2=b) for a, b in pairs]
    with pytest.raises(RuntimeError, match="contact"):
        MODULE.verify_selected_contacts(contacts, "epd::selected", ["tip1", "tip2"])


def test_grasp_accepts_only_measured_selected_fingertip_contacts():
    contacts = [SimpleNamespace(contact_body_1="epd::selected", contact_body_2="tip1")]
    MODULE.verify_selected_contacts(contacts, "epd::selected", ["tip1", "tip2"])


def test_long_narrow_bottle_is_not_rejected_by_an_unrelated_length_limit():
    assert MODULE.select_graspable_box([box("epd::bottle::0", [.229, .065, .042])])["id"] == "epd::bottle::0"


def test_fake_guard_rejects_mixed_mock_and_unknown_hardware():
    with pytest.raises(RuntimeError, match="exclusively"):
        MODULE.fake_hardware_evidence([parameter(True)], [
            component("mock_components/GenericSystem"), component("custom/Hardware")])


def test_detachment_propagates_actual_tool_rotation_and_preserves_local_geometry():
    import math
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    original = CollisionObject(id="epd::selected")
    original.pose.orientation.w = 1.0
    original.pose.position.x = 0.1
    start, end = Pose(), Pose()
    start.orientation.w = 1.0
    end.position.x = 0.3
    end.orientation.z = math.sin(math.pi / 4)
    end.orientation.w = math.cos(math.pi / 4)
    achieved = MODULE.object_pose_after_motion(original, start, end)
    assert achieved[:3] == pytest.approx([0.3, 0.1, 0.0])
    diff = MODULE.place_detachment_diff(original, "ee_palm", achieved[:3], achieved[3:])
    assert MODULE.pose_values(diff.world.collision_objects[0].pose) == pytest.approx(achieved)


def test_motion_failure_diagnostics_exclude_private_touch_and_changed_world():
    from copy import deepcopy
    from moveit_msgs.msg import ContactInformation, CollisionObject, PlanningScene
    live = PlanningScene()
    live.world.collision_objects = [CollisionObject(id='target'), CollisionObject(id='container')]
    planned = deepcopy(live)
    planned.allowed_collision_matrix = MODULE.target_contact_matrix(matrix_fixture(), 'target', ['tip1', 'tip2'])
    contacts = [ContactInformation(contact_body_1=world, body_type_1=1,
                                  contact_body_2=link, body_type_2=0, depth=.002)
                for world, link in [('target', 'tip1'), ('container', 'wrist')]]
    kept = MODULE.contacts_in_planned_scene(contacts, planned, live)
    assert [c.contact_body_2 for c in kept] == ['wrist']
    planned.world.collision_objects[1].pose.position.x = .1
    assert MODULE.contacts_in_planned_scene(contacts, planned, live) == []


def test_collision_aware_ik_only_uses_matching_live_scene():
    from copy import deepcopy
    from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
    live = PlanningScene()
    live.world.collision_objects = [CollisionObject(id='target')]
    planned = deepcopy(live)
    assert MODULE.ik_scene_matches_live(planned, live)
    planned.robot_state.joint_state.name = ['arbitrary_joint']
    planned.robot_state.joint_state.position = [.2]
    assert MODULE.ik_scene_matches_live(planned, live)  # IK carries its own seed.
    planned.allowed_collision_matrix = MODULE.target_contact_matrix(matrix_fixture(), 'target', ['tip1'])
    assert not MODULE.ik_scene_matches_live(planned, live)
    planned = deepcopy(live)
    planned.world.collision_objects = []
    assert not MODULE.ik_scene_matches_live(planned, live)
    planned = deepcopy(live)
    planned.robot_state.attached_collision_objects = [AttachedCollisionObject(link_name='generic_tool')]
    assert not MODULE.ik_scene_matches_live(planned, live)


def test_collision_diagnostic_never_default_wins_over_always_default():
    from moveit_msgs.msg import PlanningScene, CollisionObject, ContactInformation
    scene = PlanningScene()
    scene.world.collision_objects = [CollisionObject(id='container')]
    scene.allowed_collision_matrix.default_entry_names = ['container', 'tool']
    scene.allowed_collision_matrix.default_entry_values = [False, True]
    contact = ContactInformation(contact_body_1='container', body_type_1=1,
                                 contact_body_2='tool', body_type_2=0)
    assert MODULE.contacts_in_planned_scene([contact], scene, scene) == [contact]


def test_support_binding_uses_reviewed_asset_footprint_not_nearest_obstacle():
    obj = dict(id='part', frame_id='world', pose=[.1, .2, .0155, 0., 0., 0., 1.], dimensions=[.025]*3)
    environment = {'assets': [dict(id='fixture', frame='world', pose_xyz=[.1,.2,.1],
        pose_rpy=[0.,0.,0.], collision={'enabled': True}, usable_placement=dict(
        pose_xyz=[0.,0.,.01], pose_rpy=[0.,0.,0.], dimensions=[.24,.12,.14]))]}
    manifest = {'objects': [dict(id='scene::fixture', source_item_id='fixture')]}
    assert MODULE.legitimate_support_ids(obj, environment, manifest) == {'scene::fixture'}
    obj['pose'][0] = .23  # Outside reviewed footprint: lip/nearby obstacle is not a floor binding.
    assert MODULE.legitimate_support_ids(obj, environment, manifest) == set()
    obj['pose'][:3] = [.1, .2, .2125]  # Above usable volume, e.g. resting on a lip.
    assert MODULE.legitimate_support_ids(obj, environment, manifest) == set()
    obj['pose'][:3] = [.1, .2, .0155]
    environment['assets'][0].pop('usable_placement')
    assert MODULE.legitimate_support_ids(obj, environment, manifest) == set()


def test_observation_geometry_accepts_quaternion_sign_but_rejects_changed_geometry():
    observed={'pose':[1.,2.,3.,.5,.5,.5,.5],'dimensions':[.025,.025,.025]}
    actual={'pose':[1.,2.,3.,-.5,-.5,-.5,-.5],'dimensions':[.025,.025,.025]}
    assert MODULE.observation_geometry_matches(observed,actual)
    actual['pose'][0]+=.00001
    assert not MODULE.observation_geometry_matches(observed,actual)
    actual['pose'][0]=1.
    actual['pose'][3]=-.4
    assert not MODULE.observation_geometry_matches(observed,actual)


def test_simulator_handoff_reuses_only_exact_observations():
    observations_to_insert = MODULE.observations_to_insert
    obj={'id':'runtime::x','pose':[0.,0.,0.,0.,0.,0.,1.],'dimensions':[.02]*3}
    assert observations_to_insert([obj],{'runtime::x':obj},'simulator')==[]
    assert observations_to_insert([obj],{},'simulator')==[obj]
    import pytest
    with pytest.raises(RuntimeError):observations_to_insert([obj],{'runtime::x':obj},'fake')
    moved=dict(obj,pose=[.001,0.,0.,0.,0.,0.,1.])
    with pytest.raises(RuntimeError):observations_to_insert([obj],{'runtime::x':moved},'simulator')
    with pytest.raises(RuntimeError):observations_to_insert([obj],{'runtime::old':obj},'simulator')


def test_bound_approach_seed_preserves_current_state_and_rejects_changed_inputs():
    import copy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import RobotState
    from sensor_msgs.msg import JointState
    target = PoseStamped(); target.header.frame_id = 'world'; target.pose.orientation.w = 1.
    contract = dict(home_joint_names=['arm'], planning_group='arm_group', tool_link='tcp', robot_model_sha256='model')
    branch = MODULE.approach_ik_binding(target, contract, {'arm': -2.436685763798283})
    current = RobotState(joint_state=JointState(name=['arm','leader','follower'], position=[1.57,.2,-.2]))
    seeded = MODULE.bound_approach_seed(branch, target, contract, current, [('follower','leader',-1.,0.)])
    assert list(seeded.joint_state.position) == [-2.436685763798283,.2,-.2]
    assert list(current.joint_state.position) == [1.57,.2,-.2]
    for key, value in [('tool_link','other'),('robot_model_sha256','changed'),('planning_group','other')]:
        with pytest.raises(RuntimeError, match='APPROACH_IK_BINDING'):
            MODULE.bound_approach_seed(branch,target,dict(contract,**{key:value}),current,[])
    moved = copy.deepcopy(target); moved.pose.position.x += .001
    with pytest.raises(RuntimeError, match='APPROACH_IK_BINDING'):
        MODULE.bound_approach_seed(branch,moved,contract,current,[])
    broken = copy.deepcopy(branch); broken['joint_positions']['arm'] = float('nan')
    with pytest.raises(RuntimeError, match='APPROACH_IK_BINDING'):
        MODULE.bound_approach_seed(broken,target,contract,current,[])


@pytest.mark.parametrize('changed_branch',[False,True])
def test_bound_approach_rechecks_ik_without_replacing_fresh_planning_start(changed_branch):
    """Exercise the actual shared segment boundary; only ROS replies are doubles."""
    import ast,copy,time
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints,JointConstraint,MotionPlanRequest,PlanningScene,RobotState,RobotTrajectory
    from moveit_msgs.srv import GetPositionIK
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
    target=PoseStamped();target.header.frame_id='world';target.pose.orientation.w=1.
    contract=dict(home_joint_names=['arm'],planning_group='arm_group',tool_link='tcp',robot_model_sha256='model')
    binding=MODULE.approach_ik_binding(target,contract,{'arm':-2.436685763798283})
    initial=PlanningScene(robot_state=RobotState(joint_state=JointState(
        name=['arm','leader','follower'],position=[1.57,.2,-.2],velocity=[0.,.01,-.01])))
    original=copy.deepcopy(initial)
    requests=[];goals=[]
    def solve(client,request):
        requests.append(copy.deepcopy(request))
        # IK's non-arm output must not replace current gripper/mimic state.
        solution=RobotState(joint_state=JointState(name=['arm','leader','follower'],
            position=[binding['joint_positions']['arm']+(1. if changed_branch else 0.),.9,-.9]))
        return GetPositionIK.Response(solution=solution,error_code=MODULE_error(val=1))
    from moveit_msgs.msg import MoveItErrorCodes as MODULE_error
    trajectory=RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=['arm'],points=[
        JointTrajectoryPoint(positions=[1.57]),
        JointTrajectoryPoint(positions=[binding['joint_positions']['arm']])]))
    def plan(client,goal,timeout):
        goals.append(copy.deepcopy(goal))
        return MoveGroup.Result(error_code=MODULE_error(val=1),trajectory_start=copy.deepcopy(original.robot_state),
            planned_trajectory=trajectory,planning_time=.1)
    tree=ast.parse(SCRIPT.read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    segment=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='plan_segment')
    context=dict(vars(MODULE),copy=copy,time=time,MotionPlanRequest=MotionPlanRequest,GetPositionIK=GetPositionIK,
        MoveGroup=MoveGroup,stage=lambda name:None,deadline=time.monotonic()+10,contract=contract,
        args=SimpleNamespace(segment_planning_time=3.),mimics=[('follower','leader',-1.,0.)],initial=initial,
        ik_client=object(),plan_client=object(),call=solve,action=plan,trace=lambda *args:None,
        joint_constraints=lambda values:Constraints(joint_constraints=[JointConstraint(joint_name=n,position=v,
            tolerance_above=.0001,tolerance_below=.0001,weight=1.) for n,v in values.items()]))
    exec(compile(ast.Module(body=[segment],type_ignores=[]),'<actual-plan-segment>','exec'),context)
    if changed_branch:
        with pytest.raises(RuntimeError,match='APPROACH_IK_BRANCH_CHANGED'):
            context['plan_segment'](initial,'PREPLAN_APPROACH',target,ik_binding=binding)
        assert goals==[]
    else:
        result=context['plan_segment'](initial,'PREPLAN_APPROACH',target,ik_binding=binding)
        assert goals[0].request.start_state==original.robot_state
        assert goals[0].planning_options.planning_scene_diff==original
        assert goals[0].planning_options.plan_only
        assert result['metadata']['approach_ik']==binding
        assert list(result['after'].robot_state.joint_state.position[1:])==[.2,-.2]
    assert initial==original
    assert len(requests)==1 and requests[0].ik_request.avoid_collisions
    assert requests[0].ik_request.pose_stamped==target
    assert list(requests[0].ik_request.robot_state.joint_state.position)==[-2.436685763798283,.2,-.2]
    assert list(requests[0].ik_request.robot_state.joint_state.velocity)==[0.,.01,-.01]


def test_simulator_commissioning_modes_include_telemetry_retention_and_full_cycle():
    source=SCRIPT.read_text()
    assert "choices=('cancel', 'telemetry', 'stationary', 'contact-release', 'full-cycle')" in source
    assert "MOTION_TELEMETRY_PASS" in source
    assert "STATIONARY_RETENTION_PASS" in source
    assert "CONTACT_RELEASE_PASS" in source
    assert "full_cycle_physical_acceptance" in source
    # The physical path must keep the original freshness guard; no timeout inflation.
    assert "max_fresh_age_ms']>=250.0" not in source  # policy lives in simulator_execution
