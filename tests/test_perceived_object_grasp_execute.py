import importlib.util
import math
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


def test_plan_segment_candidate_wall_deadline_is_not_misreported_as_global_budget():
    import ast
    import time
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, MotionPlanRequest, PlanningScene

    tree = ast.parse(SCRIPT.read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    segment = next(n for n in main.body if isinstance(n, ast.FunctionDef) and n.name == 'plan_segment')
    global_deadline = time.monotonic() + 30.0
    contract = {
        'planning_group':'arm_group', 'home_joint_names':['arm'], 'tool_link':'tcp',
        '_candidate_wall_deadline':time.monotonic() - 0.001,
    }
    context = dict(
        vars(MODULE), time=time, MotionPlanRequest=MotionPlanRequest, MoveGroup=MoveGroup,
        stage=lambda name: None, deadline=global_deadline, contract=contract,
        args=SimpleNamespace(segment_planning_time=3.), mimics=[], initial=PlanningScene(),
        summary={},
        joint_constraints=lambda values: Constraints())
    exec(compile(ast.Module(body=[segment], type_ignores=[]),
                 '<actual-plan-segment-wall-budget>', 'exec'), context)

    with pytest.raises(RuntimeError, match='candidate wall-clock slice exhausted'):
        context['plan_segment'](
            PlanningScene(), 'PREPLAN_APPROACH', {'arm': 0.5}, group='arm_group')


def test_straight_segments_bind_ompl_to_cartesian_corridor_before_postcheck():
    source = SCRIPT.read_text()
    assert 'cartesian_corridor=(a, b)' in source
    assert 'request.path_constraints = cartesian_corridor_constraints(' in source
    assert "'workcell_cartesian_path:' + json.dumps" in source
    assert "'workcell/StraightCartesianPath' not in support_adapters.split()" in source
    assert "'initial_separation_object_ids': list(initial_separation_object_ids or [])" in source
    assert "bool(initial_separation_object_ids) and name == 'PREPLAN_LIFT'" in source
    assert "c.depth > 0.0001" in source
    assert 'request.trajectory_constraints.constraints = [marker]' in source
    assert 'PositionConstraint()' in source
    assert 'OrientationConstraint()' in source
    # One private-scene Cartesian interpolation replaces the old chain of
    # independent 5 mm MoveGroup requests.
    assert 'for i in range(1, count+1)' not in source
    assert 'cartesian_waypoints=1' in source
    assert 'cartesian_validation_segments=count' in source
    # Keep an independently densified FK verification as a second guard.
    assert 'subdivisions = max(1, math.ceil(max_delta / 0.02))' in source
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
        ik_joints={},ik_client=object(),plan_client=object(),call=solve,action=plan,trace=lambda *args:None,
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


def transfer_segment_fixture():
    import ast,copy,time
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (Constraints,JointConstraint,MotionPlanRequest,PlanningScene,
        RobotState,RobotTrajectory,MoveItErrorCodes,AttachedCollisionObject)
    from moveit_msgs.srv import GetPositionIK
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
    target=PoseStamped();target.header.frame_id='world';target.pose.orientation.w=1.;target.pose.position.x=.61
    contract=dict(home_joint_names=['arm'],planning_group='arm_group',tool_link='tcp',robot_model_sha256='model')
    seed=dict(schema='workcell_transfer_ik_seed/v1',robot_model_sha256='model',planning_group='arm_group',
              tool_link='tcp',frame_id='world',stage='PREPLAN_TRANSFER',joint_positions={'arm':-2.4})
    initial=PlanningScene(robot_state=RobotState(joint_state=JointState(
        name=['arm','leader','follower'],position=[1.57,.2,-.2],velocity=[0.,.01,-.01])))
    view=copy.deepcopy(initial)
    attached=AttachedCollisionObject();attached.link_name='tcp';attached.object.id='target'
    view.robot_state.attached_collision_objects=[attached]
    requests=[];goals=[]
    def solve(client,request):
        if not isinstance(request,GetPositionIK.Request):return SimpleNamespace(contacts=[])
        requests.append(copy.deepcopy(request))
        # Fresh numerical IK may differ slightly within the existing branch tolerance.
        return GetPositionIK.Response(solution=RobotState(joint_state=JointState(
            name=['arm','leader','follower'],position=[-2.39995,.9,-.9])),error_code=MoveItErrorCodes(val=1))
    trajectory=RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=['arm'],points=[
        JointTrajectoryPoint(positions=[1.57]),JointTrajectoryPoint(positions=[-2.39995])]))
    def plan(client,goal,timeout):
        goals.append(copy.deepcopy(goal))
        return MoveGroup.Result(error_code=MoveItErrorCodes(val=1),trajectory_start=copy.deepcopy(view.robot_state),
            planned_trajectory=trajectory,planning_time=.1)
    tree=ast.parse(SCRIPT.read_text());main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    segment=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='plan_segment')
    context=dict(vars(MODULE),copy=copy,time=time,MotionPlanRequest=MotionPlanRequest,GetPositionIK=GetPositionIK,
        MoveGroup=MoveGroup,stage=lambda name:None,deadline=time.monotonic()+10,contract=contract,
        args=SimpleNamespace(segment_planning_time=3.),mimics=[('follower','leader',-1.,0.)],initial=initial,
        ik_joints={},ik_client=object(),plan_client=object(),call=solve,action=plan,trace=lambda *args:None,summary={},
        joint_constraints=lambda values:Constraints(joint_constraints=[JointConstraint(joint_name=n,position=v,
            tolerance_above=.0001,tolerance_below=.0001,weight=1.) for n,v in values.items()]))
    exec(compile(ast.Module(body=[segment],type_ignores=[]),'<actual-transfer-segment>','exec'),context)
    return context,view,target,seed,requests,goals


def test_transfer_seed_rechecks_fresh_target_without_replacing_current_start_or_gripper():
    import copy
    context,view,target,seed,requests,goals=transfer_segment_fixture();original=copy.deepcopy(view)
    result=context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
    assert list(requests[0].ik_request.robot_state.joint_state.position)==[-2.4,.2,-.2]
    assert list(requests[0].ik_request.robot_state.joint_state.velocity)==[0.,.01,-.01]
    assert requests[0].ik_request.pose_stamped==target
    assert requests[0].ik_request.avoid_collisions is False
    assert goals[0].request.start_state==original.robot_state
    assert goals[0].planning_options.planning_scene_diff==original
    assert goals[0].planning_options.plan_only
    assert goals[0].request.goal_constraints[0].joint_constraints[0].position==-2.39995
    assert list(result['after'].robot_state.joint_state.position)==[-2.39995,.2,-.2]
    assert result['metadata']['transfer_ik_seed']==seed
    result['metadata']['transfer_ik_seed']['joint_positions']['arm']=0.
    assert view==original and seed['joint_positions']=={'arm':-2.4}


@pytest.mark.parametrize('backend,wait', [('simulator',120.),('fake',12.)])
def test_plan_only_wait_includes_simulator_continuous_certificate(backend,wait):
    context,view,target,seed,requests,goals=transfer_segment_fixture()
    context['args'].backend=backend
    original=context['action'];seen=[]
    def plan(client,goal,timeout):
        seen.append(timeout)
        assert goal.planning_options.plan_only
        return original(client,goal,timeout)
    context['action']=plan
    context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
    assert seen==[wait]


@pytest.mark.parametrize('completion_seconds', [77.4096,130.,250.])
def test_simulator_plan_only_observed_slow_certificate_and_bounded_drain(monkeypatch,completion_seconds):
    """Virtual-clock unit test of real segment/action code, not runtime proof.

    The actual 2026-10-01 revalidation transfer completed successfully after
    77.4096 seconds. A longer wait must still reject timeout/nonterminal goals.
    """
    import ast,copy
    monkeypatch.syspath_prepend(str(SCRIPT.parent))
    from full_cycle_preplanner import PlanningLifecycleFailure
    context,view,target,seed,requests,goals=transfer_segment_fixture()
    initial=copy.deepcopy(view)
    clock=[1000.]
    context['time']=SimpleNamespace(monotonic=lambda:clock[0])
    context['deadline']=1900.
    context['contract']['_candidate_wall_deadline']=1120.
    context['args'].backend='simulator'
    result=[None]
    ready=lambda:clock[0]>=1000.+completion_seconds
    response=lambda:SimpleNamespace(status=4,result=result[0]) if ready() else None
    future=SimpleNamespace(done=ready,result=response)
    cancel=SimpleNamespace(done=lambda:True,
        result=lambda:SimpleNamespace(return_code=1,goals_canceling=[]))
    cancellations=[]
    def request_cancel():
        cancellations.append(clock[0])
        return cancel
    handle=SimpleNamespace(accepted=True,goal_id=SimpleNamespace(uuid=[4]*16),
        get_result_async=lambda:future,cancel_goal_async=request_cancel)
    sent=SimpleNamespace(done=lambda:True,result=lambda:handle)
    fixture_plan=context['action']
    def send(goal):
        result[0]=fixture_plan(None,goal,None)
        return sent
    client=SimpleNamespace(wait_for_server=lambda **kwargs:True,send_goal_async=send)
    waits=[]
    def spin(node,pending,timeout_sec):
        assert 0.<timeout_sec<=120.
        if pending is future:
            waits.append(timeout_sec)
            clock[0]+=min(timeout_sec,max(0.,1000.+completion_seconds-clock[0]))
    context.update(plan_client=client,execute_client=object(),controlled_cancel=False,
        controller_audit=None,node=object(),measurements=None,execution_monitor=None,
        rclpy=SimpleNamespace(spin_until_future_complete=spin))
    main=next(n for n in ast.parse(SCRIPT.read_text()).body
              if isinstance(n,ast.FunctionDef) and n.name=='main')
    action=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='action')
    exec(compile(ast.Module(body=[action],type_ignores=[]),'<actual-slow-plan-action>','exec'),context)
    if completion_seconds<120.:
        planned=context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
        assert planned['metadata']['success'] and planned['metadata']['moveit_code']==1
        assert waits==[120.] and cancellations==[]
        assert 'interrupted_planning_goals' not in context['summary']
        assert clock[0]==pytest.approx(1000.+completion_seconds)
    else:
        error=RuntimeError if completion_seconds<240. else PlanningLifecycleFailure
        message='action result timed out' if completion_seconds<240. else 'PLAN_ONLY_GOAL_NOT_TERMINAL'
        with pytest.raises(error,match=message):
            context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
        assert waits==[120.,120.] and cancellations==[1120.]
        record,=context['summary']['interrupted_planning_goals']
        assert record['terminal_wait_s']==120. and record['cancel_accepted'] is False
        assert record['terminal_status']==(4 if completion_seconds<240. else None)
        assert clock[0]<=1240.
    assert len(goals)==1 and len(requests)==1
    assert goals[0].planning_options.plan_only
    assert goals[0].planning_options.planning_scene_diff==initial
    assert goals[0].request.start_state==initial.robot_state
    assert goals[0].request.allowed_planning_time==3.
    assert context['deadline']==1900. and context['contract']['_candidate_wall_deadline']==1120.
    assert view==initial


@pytest.mark.parametrize('attempts,first_status,first_code', [(3,6,-6),(1,6,-6),(3,5,-6),(3,6,1)])
def test_drained_moveit_timeout_after_client_wait_uses_identical_request_policy(monkeypatch,attempts,
                                                                             first_status,first_code):
    """Virtual-clock unit test of real segment/action code, not runtime proof.

    run-full-cycle-220343 revalidation PREPLAN_HOME: the 120 s result wait
    expired, then the bounded drain received MoveGroup's own ABORTED/TIMED_OUT
    (PLANNING_DEADLINE_EXCEEDED in the controller certificate). That owned
    terminal result now reaches the existing retry policy unchanged.
    """
    import ast,copy
    monkeypatch.syspath_prepend(str(SCRIPT.parent))
    from full_cycle_preplanner import MotionFeasibilityFailure
    context,view,target,seed,requests,goals=transfer_segment_fixture()
    initial=copy.deepcopy(view)
    clock=[1000.]
    context['time']=SimpleNamespace(monotonic=lambda:clock[0])
    context['deadline']=1900.
    context['contract'].update(_candidate_wall_deadline=1800.,_candidate_planning_attempts=attempts)
    context['args'].backend='simulator'
    fixture_plan=context['action']
    owned=[];cancellations=[];waits=[]
    def send(goal):
        index=len(owned)
        # First owned goal: adapter allowance ends 120.1 s after acceptance.
        # Identical retry: an ordinary certified plan after 25 s.
        ends=clock[0]+(120.1 if index==0 else 25.)
        planned=fixture_plan(None,goal,None)
        if index==0:
            planned=copy.deepcopy(planned);planned.error_code.val=first_code
            planned.planned_trajectory.joint_trajectory.points=[]
        ready=lambda:clock[0]>=ends
        future=SimpleNamespace(done=ready,ends=ends,
            result=lambda:SimpleNamespace(status=first_status if index==0 else 4,result=planned) if ready() else None)
        def request_cancel():
            cancellations.append((index,clock[0]))
            return SimpleNamespace(done=lambda:True,
                result=lambda:SimpleNamespace(return_code=1,goals_canceling=[]))
        handle=SimpleNamespace(accepted=True,goal_id=SimpleNamespace(uuid=[index+5]*16),
            get_result_async=lambda:future,cancel_goal_async=request_cancel)
        owned.append(future)
        return SimpleNamespace(done=lambda:True,result=lambda:handle)
    client=SimpleNamespace(wait_for_server=lambda **kwargs:True,send_goal_async=send)
    def spin(node,pending,timeout_sec):
        assert 0.<timeout_sec<=120.
        if pending in owned:
            waits.append(timeout_sec)
            clock[0]+=min(timeout_sec,max(0.,pending.ends-clock[0]))
    context.update(plan_client=client,execute_client=object(),controlled_cancel=False,
        controller_audit=None,node=object(),measurements=None,execution_monitor=None,
        rclpy=SimpleNamespace(spin_until_future_complete=spin))
    main=next(n for n in ast.parse(SCRIPT.read_text()).body
              if isinstance(n,ast.FunctionDef) and n.name=='main')
    action=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='action')
    exec(compile(ast.Module(body=[action],type_ignores=[]),'<actual-home-timeout-action>','exec'),context)
    if (first_status,first_code)!=(6,-6):
        # A late CANCELED or a terminal without MoveIt's failure is no owned
        # planning verdict: it remains the unretried client timeout.
        with pytest.raises(RuntimeError,match='action result timed out') as failure:
            context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
        assert not isinstance(failure.value,MODULE.MoveItActionFailure)
        assert len(goals)==1 and 'planning_retries' not in context['summary']
        record,=context['summary']['interrupted_planning_goals']
        assert record['terminal_status']==first_status and record['moveit_code']==first_code
        return
    if attempts==3:
        planned=context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
        assert planned['metadata']['success'] and planned['metadata']['moveit_code']==1
        assert len(goals)==2 and goals[0]==goals[1]  # identical request, scene and start
        assert waits==[120.,120.,120.] and cancellations==[(0,1120.)]
        assert context['summary']['planning_retries']==[dict(stage='PREPLAN_TRANSFER',moveit_code=-6,
            attempt=1,retry_kind='timed_out',search_pass='normal')]
        assert clock[0]==pytest.approx(1145.1)
    else:
        with pytest.raises(MotionFeasibilityFailure) as failure:
            context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
        assert failure.value.details['failure_kind']=='planning' and failure.value.details['moveit_code']==-6
        assert len(goals)==1 and 'planning_retries' not in context['summary']
    # The original owned failure evidence is preserved, not replaced.
    record=context['summary']['interrupted_planning_goals'][0]
    assert record['terminal_status']==6 and record['moveit_code']==-6
    assert record['cancel_accepted'] is False and record['terminal_wait_s']==120.
    assert len(context['summary']['interrupted_planning_goals'])==1
    assert goals[0].planning_options.plan_only
    assert goals[0].planning_options.planning_scene_diff==initial
    assert goals[0].request.start_state==initial.robot_state
    assert view==initial


@pytest.mark.parametrize('terminal', [None,4,5,6])
def test_interrupted_plan_requires_owned_terminal_before_search_continues(terminal):
    from full_cycle_preplanner import PlanningLifecycleFailure
    uuid=[1]*16
    cancel=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(return_code=0,
        goals_canceling=[SimpleNamespace(goal_id=SimpleNamespace(uuid=uuid))]))
    result=SimpleNamespace(status=terminal,result=SimpleNamespace(error_code=SimpleNamespace(val=1)))
    future=SimpleNamespace(done=lambda:terminal is not None,result=lambda:result)
    handle=SimpleNamespace(cancel_goal_async=lambda:cancel)
    waits=[];summary={}
    def spin(item,timeout):waits.append((item,timeout))
    if terminal is None:
        with pytest.raises(PlanningLifecycleFailure,match='PLAN_ONLY_GOAL_NOT_TERMINAL'):
            MODULE.finish_interrupted_plan(handle,future,uuid,spin,summary)
    else:
        MODULE.finish_interrupted_plan(handle,future,uuid,spin,summary)
    assert waits==[(cancel,5.),(future,5.)]
    evidence=summary['interrupted_planning_goals'][0]
    assert evidence['goal_uuid']==bytes(uuid).hex()
    assert evidence['terminal_status']==terminal


def test_unknown_plan_acceptance_stops_search_and_cancels_late_acceptance():
    import ast
    from full_cycle_preplanner import PlanningLifecycleFailure
    tree=ast.parse(SCRIPT.read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    boundary=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='action')
    callbacks=[];cancelled=[]
    sent=SimpleNamespace(done=lambda:False,add_done_callback=callbacks.append)
    client=SimpleNamespace(wait_for_server=lambda **kw:True,send_goal_async=lambda goal:sent)
    context=dict(vars(MODULE),execute_client=object(),plan_client=client,controlled_cancel=False,
        controller_audit=None,node=object(),summary={},
        rclpy=SimpleNamespace(spin_until_future_complete=lambda *a,**kw:None))
    exec(compile(ast.Module(body=[boundary],type_ignores=[]),'<actual-plan-action>','exec'),context)
    with pytest.raises(PlanningLifecycleFailure,match='PLAN_ONLY_ACCEPTANCE_UNKNOWN'):
        context['action'](client,object(),60.)
    assert context['summary']['planning_acceptance_unknown'] is True
    handle=SimpleNamespace(accepted=True,cancel_goal_async=lambda:cancelled.append(True))
    callbacks[0](SimpleNamespace(result=lambda:handle))
    assert cancelled==[True]


def test_accepted_plan_without_result_future_stops_search():
    import ast
    from full_cycle_preplanner import PlanningLifecycleFailure
    tree=ast.parse(SCRIPT.read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    boundary=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='action')
    cancelled=[]
    def result():raise RuntimeError('GetResult transport failed')
    handle=SimpleNamespace(accepted=True,goal_id=SimpleNamespace(uuid=[2]*16),get_result_async=result,
        cancel_goal_async=lambda:cancelled.append(True))
    sent=SimpleNamespace(done=lambda:True,result=lambda:handle)
    client=SimpleNamespace(wait_for_server=lambda **kw:True,send_goal_async=lambda goal:sent)
    context=dict(vars(MODULE),execute_client=object(),plan_client=client,controlled_cancel=False,
        controller_audit=None,node=object(),summary={},
        rclpy=SimpleNamespace(spin_until_future_complete=lambda *a,**kw:None))
    exec(compile(ast.Module(body=[boundary],type_ignores=[]),'<actual-plan-action>','exec'),context)
    with pytest.raises(PlanningLifecycleFailure,match='PLAN_ONLY_GOAL_NOT_TERMINAL'):
        context['action'](client,object(),60.)
    assert cancelled==[True]
    assert context['summary']['planning_result_request_error']=='GetResult transport failed'


@pytest.mark.parametrize('position',[-2.3,-2.4+2*MODULE.math.pi,float('nan'),float('inf'),None])
def test_transfer_seed_rejects_changed_or_missing_fresh_branch_before_planning(position):
    from moveit_msgs.msg import RobotState,MoveItErrorCodes
    from moveit_msgs.srv import GetPositionIK
    from sensor_msgs.msg import JointState
    context,view,target,seed,requests,goals=transfer_segment_fixture()
    def solve(client,request):
        requests.append(request)
        joint_state=JointState(name=[] if position is None else ['arm'],
                               position=[] if position is None else [position])
        return GetPositionIK.Response(solution=RobotState(joint_state=joint_state),
                                      error_code=MoveItErrorCodes(val=1))
    context['call']=solve
    with pytest.raises(RuntimeError,match='TRANSFER_IK_BRANCH_CHANGED'):
        context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
    assert len(requests)==1 and goals==[]


def test_resolved_cycle_requires_proven_transfer_binding_before_planning():
    import ast,time
    tree=ast.parse(SCRIPT.read_text())
    authored=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='plan_authored_cycle')
    evaluate=next(n for n in authored.body if isinstance(n,ast.FunctionDef) and n.name=='evaluate_once')
    context=dict(time=time,contract={},deadline=time.monotonic()+10,resolved={'saved':True})
    exec(compile(ast.Module(body=[evaluate],type_ignores=[]),'<actual-resolved-evaluator>','exec'),context)
    result=context['evaluate_once']({'approach_ik':{'bound':True}},
        search_pass='revalidate',planning_attempts=3,segment_time=3.)
    assert result['success'] is False
    assert result['reason_code']=='TASK_TRANSFER_IK_UNBOUND'
    assert result['retryable'] is False


@pytest.mark.parametrize('field,value', [
    ('schema','other'),('robot_model_sha256','changed'),('planning_group','other'),
    ('tool_link','other'),('frame_id','other'),('stage','PREPLAN_PLACE'),
    ('joint_positions',{}),('joint_positions',{'arm':float('nan')}),
    ('joint_positions',{'arm':-2.4,'leader':.9}),('joint_positions',{'arm':True}),
])
def test_transfer_seed_rejects_changed_context_or_malformed_arm_positions(field,value):
    context,view,target,seed,requests,goals=transfer_segment_fixture();seed[field]=value
    with pytest.raises(RuntimeError,match='TRANSFER_IK_SEED'):
        context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
    assert requests==goals==[]


def test_transfer_seed_is_not_accepted_for_another_stage():
    context,view,target,seed,requests,goals=transfer_segment_fixture()
    with pytest.raises(RuntimeError,match='TRANSFER_IK_SEED'):
        context['plan_segment'](view,'PREPLAN_APPROACH',target,ik_seed=seed)
    assert requests==goals==[]


def test_transfer_seed_is_not_accepted_for_another_requested_group():
    context,view,target,seed,requests,goals=transfer_segment_fixture()
    with pytest.raises(RuntimeError,match='TRANSFER_IK_SEED'):
        context['plan_segment'](view,'PREPLAN_TRANSFER',target,group='other',ik_seed=seed)
    assert requests==goals==[]


def test_transfer_seed_does_not_accept_newly_colliding_private_plan():
    import copy
    context,view,target,seed,requests,goals=transfer_segment_fixture()
    def reject(client,goal,timeout):
        goals.append(copy.deepcopy(goal))
        raise MODULE.MoveItActionFailure(6,-27)
    context['action']=reject
    with pytest.raises(RuntimeError,match='MoveIt action failed'):
        context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
    assert len(requests)==len(goals)==1
    assert goals[0].planning_options.planning_scene_diff==view
    assert context['summary'].get('planning_retries',[])==[]


@pytest.mark.parametrize('with_contact', [False, True])
def test_measured_fcl_preserves_first_rejected_query_before_cancellation(with_contact):
    """A rejected validity response must survive cancellation/reconciliation."""
    import ast
    import copy
    from moveit_msgs.msg import ContactInformation, PlanningScene, RobotState
    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState

    tree = ast.parse(SCRIPT.read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    boundary = next(n for n in main.body if isinstance(n, ast.FunctionDef) and n.name == 'measured_fcl')
    sample = {'sim_ns': 69389000000, 'iteration': 69389, 'wall_ns': 1000,
              'joints': {'arm': [0.2, 0.0]}}
    contact = ContactInformation(contact_body_1='wrist', contact_body_2='bin', depth=.0002)
    response = GetStateValidity.Response(valid=False, contacts=[contact] if with_contact else [])
    requests = []
    def query(request):
        requests.append(copy.deepcopy(request))
        return SimpleNamespace(done=lambda: True, result=lambda: response)
    summary = {'last_measured_collision_check': {'sim_ns': 69387000000, 'valid': True, 'contacts': 0}}
    context = dict(vars(MODULE), copy=copy,
        measurements=SimpleNamespace(fresh=lambda: sample, joints=lambda s: s['joints']),
        initial=PlanningScene(robot_state=RobotState(joint_state=JointState(name=['arm'], position=[0.0]))),
        contact_guard=SimpleNamespace(planning_attached=False, held=False, phase='approach',
                                     separation=None, predicate=None),
        PlanningScene=PlanningScene, GetStateValidity=GetStateValidity,
        validity_client=SimpleNamespace(call_async=query), node=object(), summary=summary,
        rclpy=SimpleNamespace(spin_until_future_complete=lambda *a, **kw: None))
    exec(compile(ast.Module(body=[boundary], type_ignores=[]), '<actual-measured-fcl>', 'exec'), context)
    with pytest.raises(RuntimeError, match='measured carried/robot collision or invalid state'):
        context['measured_fcl']()
    evidence = copy.deepcopy(summary['rejected_measured_collision_check'])
    assert evidence['measurement'] == sample
    assert evidence['robot_state']['joint_state']['position'] == [0.2]
    assert evidence['response']['valid'] is False
    assert len(evidence['response']['contacts']) == int(with_contact)
    if with_contact:
        assert evidence['response']['contacts'][0]['contact_body_1'] == 'wrist'
        assert evidence['response']['contacts'][0]['depth'] == .0002
    assert summary['last_measured_collision_check']['sim_ns'] == 69387000000
    sample['iteration'] += 1
    sample['joints']['arm'][0] = .3
    with pytest.raises(RuntimeError):
        context['measured_fcl']()
    assert summary['rejected_measured_collision_check'] == evidence


def test_owned_execution_evidence_preserves_exact_command_trajectory():
    import ast
    import copy
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from rosidl_runtime_py.convert import message_to_ordereddict

    tree = ast.parse(SCRIPT.read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    boundary = next(n for n in main.body if isinstance(n, ast.FunctionDef) and n.name == 'action')
    goal = ExecuteTrajectory.Goal(trajectory=RobotTrajectory(joint_trajectory=JointTrajectory(
        joint_names=['arm'], points=[JointTrajectoryPoint(positions=[.1]), JointTrajectoryPoint(positions=[.2])])))
    expected = message_to_ordereddict(copy.deepcopy(goal.trajectory))
    response = SimpleNamespace(status=4, result=ExecuteTrajectory.Result(error_code=MoveItErrorCodes(val=1)))
    finished = SimpleNamespace(done=lambda: True, result=lambda: response)
    handle = SimpleNamespace(accepted=True, goal_id=SimpleNamespace(uuid=list(range(16))), get_result_async=lambda: finished)
    sent = SimpleNamespace(done=lambda: True, result=lambda: handle)
    client = SimpleNamespace(wait_for_server=lambda **kw: True, send_goal_async=lambda g: sent)
    summary = {'current_stage': 'EXECUTE_APPROACH'}
    context = dict(vars(MODULE), summary=summary, controlled_cancel=False, controller_audit=None,
        execute_client=client, measurements=None, execution_monitor=None, node=object(),
        rclpy=SimpleNamespace(spin_until_future_complete=lambda *a, **kw: None))
    exec(compile(ast.Module(body=[boundary], type_ignores=[]), '<actual-owned-action>', 'exec'), context)
    context['action'](client, goal, 5)
    goal.trajectory.joint_trajectory.points[1].positions[0] = .9
    assert summary['owned_execution_goal']['trajectory'] == expected
    assert summary['owned_execution_goal']['stage'] == 'EXECUTE_APPROACH'


def test_trajectory_serialization_failure_occurs_before_goal_submission(monkeypatch):
    import ast
    import rosidl_runtime_py.convert as convert
    tree=ast.parse(SCRIPT.read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    boundary=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='action')
    submissions=[]
    finished=SimpleNamespace(done=lambda:True,result=lambda:None)
    handle=SimpleNamespace(accepted=True,goal_id=SimpleNamespace(uuid=[1]*16),get_result_async=lambda:finished)
    def send(goal):
        submissions.append(goal)
        return SimpleNamespace(done=lambda:True,result=lambda:handle)
    client=SimpleNamespace(wait_for_server=lambda **kw:True,send_goal_async=send)
    def broken(message):raise RuntimeError('serialization failed')
    monkeypatch.setattr(convert,'message_to_ordereddict',broken)
    context=dict(vars(MODULE),execute_client=client,controlled_cancel=False,controller_audit=None,
                 node=object(),summary={},rclpy=SimpleNamespace(spin_until_future_complete=lambda *a,**kw:None))
    exec(compile(ast.Module(body=[boundary],type_ignores=[]),'<actual-owned-action>','exec'),context)
    with pytest.raises(RuntimeError,match='serialization failed'):
        context['action'](client,SimpleNamespace(trajectory=object()),5)
    assert submissions==[]


@pytest.mark.parametrize('backend,stage_name,expected',[
    ('simulator','PREPLAN_LIFT',.02),('fake','PREPLAN_LIFT',.2),
    # Carried transfer/place at 10 %: the 20 % transfer pivoted the held part 0.69 deg
    # in the pinch at ~1 g (run-full-cycle-135626); 2 % would exceed the 120 s wait.
    ('simulator','PREPLAN_TRANSFER',.1),('simulator','PREPLAN_PLACE',.1),
    ('fake','PREPLAN_TRANSFER',.2),('fake','PREPLAN_PLACE',.2),
    ('simulator','PREPLAN_APPROACH',.2),('simulator','PREPLAN_RETREAT',.2),('simulator','PREPLAN_HOME',.2),
    ('simulator','PREPLAN_CLOSE_GRIPPER',.2),('simulator','PREPLAN_OPEN_GRIPPER',.2)])
def test_physical_carried_phases_use_conservative_scaling_without_changing_request_geometry(backend,stage_name,expected):
    import ast
    import copy
    import time
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, PlanningScene, RobotState, RobotTrajectory
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    tree=ast.parse(SCRIPT.read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    boundary=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='plan_segment')
    initial=PlanningScene(robot_state=RobotState(joint_state=JointState(name=['arm'],position=[.1])))
    trajectory=RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=['arm'],points=[
        JointTrajectoryPoint(positions=[.1]),JointTrajectoryPoint(positions=[.2])]))
    goals=[]
    def action(client,goal,timeout):
        goals.append(copy.deepcopy(goal))
        return MoveGroup.Result(error_code=MoveItErrorCodes(val=1),trajectory_start=copy.deepcopy(initial.robot_state),
                                planned_trajectory=trajectory,planning_time=.1)
    context=dict(vars(MODULE),copy=copy,time=time,MotionPlanRequest=MotionPlanRequest,MoveGroup=MoveGroup,
        stage=lambda name:None,deadline=time.monotonic()+10,contract={'planning_group':'arm_group','home_joint_names':['arm'],'tool_link':'tcp'},
        args=SimpleNamespace(backend=backend,segment_planning_time=3.),mimics=[],initial=initial,
        plan_client=object(),action=action,trace=lambda *a:None,summary={},
        joint_constraints=lambda values:Constraints(joint_constraints=[JointConstraint(joint_name=n,position=v,
            tolerance_above=.0001,tolerance_below=.0001,weight=1.) for n,v in values.items()]))
    exec(compile(ast.Module(body=[boundary],type_ignores=[]),'<actual-lift-scaling>','exec'),context)
    context['plan_segment'](initial,stage_name,{'arm':.2},group='arm_group')
    request=goals[0].request
    assert request.max_velocity_scaling_factor==expected
    assert request.max_acceleration_scaling_factor==expected
    assert request.start_state==initial.robot_state
    assert request.goal_constraints[0].joint_constraints[0].position==.2
    assert request.goal_constraints[0].joint_constraints[0].tolerance_above==.0001
    assert request.allowed_planning_time==3.
    assert goals[0].planning_options.planning_scene_diff==initial


@pytest.mark.parametrize('mode',['stationary','contact-release','full-cycle'])
@pytest.mark.parametrize('retention_failure',[None,'duration','contact','slip'])
def test_live_attach_requires_current_grasp_retention_before_handoff(mode,retention_failure):
    """Execute the real attach branch with observable boundary calls."""
    import ast
    calls=[];current={'iteration':1200};summary={'owned_execution_goal':{'uuid':'close','wall_ns':1},'close_terminal_wall_ns':2}
    def record(name,result=None):
        def call(*args,**kwargs):
            calls.append(name)
            return result
        return call
    def hold(seconds):
        assert seconds==1.1
        calls.append('hold')
        if retention_failure in ('contact','slip'):
            raise RuntimeError('physical retention '+retention_failure+' rejected during hold')
    def retention():
        calls.append('retention')
        if retention_failure=='duration':raise RuntimeError('physical retention duration rejected')
        return {'duration_sim_ns':1_100_000_000,'samples':1101}
    guard=SimpleNamespace(pile_binding={},begin_pile_admission=record('admission'),
        establish=record('establish',[0,0,0,0,0,0,1]),checked_current=record('checked',current),
        retention_evidence=retention,held=object(),separation=object(),planning_attached=False)
    def attachment(original,contract,measurements,sample):
        assert sample is current
        assert calls.index('retention')<len(calls)
        calls.append('attachment')
        return 'attached-diff'
    scope={'summary':summary,'contact_guard':guard,'args':SimpleNamespace(simulator_commission=mode),
        'wait_stopped':record('stop',True),'measurements':SimpleNamespace(fresh=record('unchecked',{'iteration':1})),
        'monitored_hold':hold,'measured_reconcile':record('reconcile'),'measured_attachment':attachment,
        'step':{'original':'original'},'contract':{},'apply':record('apply'),'baseline':'baseline',
        'PlanningScene':lambda **kwargs:kwargs,'RuntimeError':RuntimeError}
    tree=ast.parse(SCRIPT.read_text())
    branch=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and
        ast.unparse(n.test)=="step['kind'] == 'attach'" and 'begin_pile_admission' in ast.unparse(n))
    function=ast.parse('def exercise():\n    for _ in [None]:\n        pass\n').body[0]
    function.body[0].body=branch.body
    module=ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[]))
    exec(compile(module,str(SCRIPT),'exec'),scope)
    if retention_failure:
        with pytest.raises(RuntimeError,match='physical retention'):
            scope['exercise']()
        assert 'attachment' not in calls and 'apply' not in calls
        assert 'stationary_retention' not in summary
    else:
        scope['exercise']()
        assert summary['stationary_retention']['duration_sim_ns']>=1_000_000_000
        assert calls.index('admission')<calls.index('stop')<calls.index('establish')<calls.index('hold')<calls.index('retention')
        assert calls.count('establish')==1 and calls.count('admission')==1
        assert 'unchecked' not in calls
        if mode=='stationary':
            assert 'attachment' not in calls
            assert summary['result']=='STATIONARY_RETENTION_PASS'
        else:
            assert calls.index('retention')<calls.index('attachment')<calls.index('apply')
            assert summary['closure_measurement'] is current
            assert guard.planning_attached


def test_measured_collision_query_stops_virtual_carry_after_physical_separation():
    from types import SimpleNamespace as N
    guard=N(planning_attached=True,ownership='CARRIED',release_candidate=None)
    assert MODULE.measured_payload_attached_for_collision(guard)
    guard.ownership='RELEASING'
    assert MODULE.measured_payload_attached_for_collision(guard)
    guard.release_candidate={'iteration':10}
    assert not MODULE.measured_payload_attached_for_collision(guard)
    guard.ownership='FREE_SETTLING';guard.planning_attached=False
    assert not MODULE.measured_payload_attached_for_collision(guard)


def test_interrupted_plan_drains_through_its_bounded_certificate_wait():
    uuid=[3]*16
    cancel=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(return_code=1,goals_canceling=[]))
    ready=[False]
    terminal=SimpleNamespace(status=4,result=SimpleNamespace(error_code=SimpleNamespace(val=1)))
    future=SimpleNamespace(done=lambda:ready[0],result=lambda:terminal if ready[0] else None)
    handle=SimpleNamespace(cancel_goal_async=lambda:cancel)
    waits=[];summary={}
    def spin(item,timeout):
        waits.append(timeout)
        if item is future and timeout>=17.625:ready[0]=True
    MODULE.finish_interrupted_plan(handle,future,uuid,spin,summary,terminal_timeout=60.)
    assert waits==[5.,60.]
    assert summary['interrupted_planning_goals'][0]['terminal_status']==4
    assert summary['interrupted_planning_goals'][0]['cancel_accepted'] is False


def test_plan_only_action_does_not_read_cancellation_motion_measurement():
    import ast
    tree=ast.parse(SCRIPT.read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    boundary=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='action')
    result=SimpleNamespace(error_code=SimpleNamespace(val=1))
    future=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(status=4,result=result))
    handle=SimpleNamespace(accepted=True,goal_id=SimpleNamespace(uuid=[3]*16),get_result_async=lambda:future)
    sent=SimpleNamespace(done=lambda:True,result=lambda:handle)
    client=SimpleNamespace(wait_for_server=lambda **kw:True,send_goal_async=lambda goal:sent)
    def unexpected():raise AssertionError('PLAN_ONLY has no cancellation motion start')
    context=dict(vars(MODULE),execute_client=object(),plan_client=client,controlled_cancel=False,
        controller_audit=None,node=object(),summary={},measurements=SimpleNamespace(fresh=unexpected),
        execution_monitor=None,rclpy=SimpleNamespace(spin_until_future_complete=lambda *a,**kw:None))
    exec(compile(ast.Module(body=[boundary],type_ignores=[]),'<actual-plan-action>','exec'),context)
    assert context['action'](client,object(),60.) is result


def test_ik_winding_uses_nearest_equivalent_inside_model_safety_limits():
    import math, xml.etree.ElementTree as ET
    # Values captured from the real Stage-A transfer request on 2026-09-30.
    names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint']
    current=dict(zip(names,[5.505437711794195,-1.6436449324806555,2.0106194656440257,-1.9377704026269704]))
    values=dict(zip(names,[-2.4671832326751395,4.243731006703432,-1.1621885199580988,1.6308453269345686]))
    joints={name:ET.fromstring(f'<joint name="{name}" type="revolute"><limit lower="{-bound-.15}" upper="{bound+.15}"/><safety_controller soft_lower_limit="{-bound}" soft_upper_limit="{bound}"/></joint>')
            for name,bound in zip(names,[2*math.pi-.15,2*math.pi-.15,math.pi-.15,2*math.pi-.15])}
    adjusted=MODULE.nearest_ik_winding(values,current,joints)
    assert adjusted==pytest.approx(dict(zip(names,[3.8160020745044467,-2.0394543004761543,-1.1621885199580988,-4.652339980245017])))
    for name in names:
        assert math.remainder(adjusted[name]-values[name],2*math.pi)==pytest.approx(0.,abs=1e-15)
    assert values['shoulder_pan_joint']==-2.4671832326751395


@pytest.mark.parametrize('kind,lo,hi,value,reference,expected',[
    ('revolute',-6.3,6.3,-.2,6.2,-.2),  # hard limit permits a turn; soft limit does not
    ('revolute',-3.,3.,-2.4,1.57,-2.4), # no equivalent turn fits the model
    ('prismatic',-10.,10.,-.2,6.2,-.2),
    ('continuous',None,None,-.2,6.2,6.083185307179586),
])
def test_ik_winding_preserves_bounds_and_nonrotating_joints(kind,lo,hi,value,reference,expected):
    import xml.etree.ElementTree as ET
    bounds='' if lo is None else f'<limit lower="{lo}" upper="{hi}"/><safety_controller soft_lower_limit="{-6.}" soft_upper_limit="{6.}"/>'
    joint=ET.fromstring(f'<joint name="axis" type="{kind}">{bounds}</joint>')
    assert MODULE.nearest_ik_winding({'axis':value},{'axis':reference},{'axis':joint})['axis']==pytest.approx(expected)


def test_unbound_transfer_maps_ik_turns_without_changing_scene_or_start():
    import copy, math, xml.etree.ElementTree as ET
    context,view,target,seed,requests,goals=transfer_segment_fixture()
    context['ik_joints']={'arm':ET.fromstring('<joint name="arm" type="revolute"><limit lower="-6.28" upper="6.28"/></joint>')}
    before=copy.deepcopy(view)
    context['plan_segment'](view,'PREPLAN_TRANSFER',target)
    assert goals[0].request.goal_constraints[0].joint_constraints[0].position==pytest.approx(-2.39995+2*math.pi)
    assert goals[0].request.start_state==before.robot_state
    assert goals[0].planning_options.planning_scene_diff==before
    assert goals[0].planning_options.plan_only


def test_bound_transfer_preserves_proven_winding_even_when_another_turn_is_nearer():
    import xml.etree.ElementTree as ET
    context,view,target,seed,requests,goals=transfer_segment_fixture()
    context['ik_joints']={'arm':ET.fromstring('<joint name="arm" type="revolute"><limit lower="-6.28" upper="6.28"/></joint>')}
    context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
    assert goals[0].request.goal_constraints[0].joint_constraints[0].position==pytest.approx(-2.39995)


def test_bound_transfer_accepts_equivalent_ik_turn_without_changing_saved_branch():
    import math, xml.etree.ElementTree as ET
    context,view,target,seed,requests,goals=transfer_segment_fixture()
    context['ik_joints']={'arm':ET.fromstring('<joint name="arm" type="revolute"><limit lower="-6.28" upper="6.28"/></joint>')}
    solve=context['call']
    def wrapped(client,request):
        response=solve(client,request)
        response.solution.joint_state.position[0]+=math.tau
        return response
    context['call']=wrapped
    result=context['plan_segment'](view,'PREPLAN_TRANSFER',target,ik_seed=seed)
    assert goals[0].request.goal_constraints[0].joint_constraints[0].position==pytest.approx(-2.39995)
    assert result['metadata']['transfer_ik_seed']==seed


def test_ik_winding_does_not_change_a_mimic_leader_or_follower():
    import xml.etree.ElementTree as ET
    model=ET.fromstring('<robot><joint name="leader" type="continuous"/>'
        '<joint name="follower" type="continuous"><mimic joint="leader" multiplier="0.5"/></joint></robot>')
    values={'leader':-.2,'follower':-.1}
    assert MODULE.nearest_ik_winding(values,{'leader':6.2,'follower':3.1},
        {j.get('name'):j for j in model.findall('joint')})==values


def _support_box(z, height, quaternion=(0.0, 0.0, 0.0, 1.0), shape=1):
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    obj = CollisionObject(); obj.id = 'workcell::support_surface_table'; obj.header.frame_id = 'world'
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = 0.51, 0.0023, z
    obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w = quaternion
    primitive = SolidPrimitive(); primitive.type = shape
    primitive.dimensions = [1.2084, 0.8134, height] if shape == 1 else [0.1]
    obj.primitives = [primitive]; obj.primitive_poses = [Pose()]; obj.primitive_poses[0].orientation.w = 1.0
    return obj


def test_support_plane_height_is_the_box_top_not_a_contact_point():
    execute = MODULE
    # Geometry from the 2026-10-19 sparse commissioning table: top face at z=0.
    table = _support_box(-0.4531188535094261, 0.9062377070188522)
    assert abs(execute.support_plane_height(table)) < 1e-15
    yawed = _support_box(-0.45, 0.9, (0.0, 0.0, math.sin(0.2), math.cos(0.2)))
    assert abs(execute.support_plane_height(yawed)) < 1e-15


def test_support_plane_height_rejects_tilted_or_non_box_supports():
    execute = MODULE
    with pytest.raises(RuntimeError, match='SUPPORT_PLANE_UNSUPPORTED'):
        execute.support_plane_height(_support_box(-0.45, 0.9, (math.sin(0.01), 0.0, 0.0, math.cos(0.01))))
    with pytest.raises(RuntimeError, match='SUPPORT_PLANE_UNSUPPORTED'):
        execute.support_plane_height(_support_box(-0.45, 0.9, shape=2))


def test_execution_boundary_refresh_precedes_unchanged_expiry_check():
    """Live runs renew the observation only through the measured refresh, and the
    existing age check still runs afterwards on the renewed timestamps."""
    text=(Path(__file__).resolve().parents[1]/'scripts/perceived_object_grasp_execute.py').read_text()
    start=text.index('refresh_at_execution_boundary(');expiry=text.index("'selected observation expired before execution'")
    block=text[text.rindex('if measurements:',0,start):expiry]
    assert start<expiry and 'if not motion or not motion[0]' in block
    assert 'time.time()' not in block.split('fresh = select_observations')[0]
    assert "select_observations(intent, physical, objects, time.time())" in text[start:expiry]


# ---------------------------------------------------------------- carried-step owned execution wait
def _traj(duration_s):
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    pts=[JointTrajectoryPoint(positions=[0.]),JointTrajectoryPoint(positions=[.1],time_from_start=Duration(sec=int(duration_s),nanosec=int((duration_s%1)*1e9)))]
    return RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=['arm'],points=pts))


def test_measured_real_time_factor_requires_a_second_of_wall_time_and_forward_sim():
    f=MODULE.measured_real_time_factor
    assert f(None,{'wall_ns':0,'sim_ns':0}) is None
    assert f({'wall_ns':0,'sim_ns':0},{'wall_ns':int(.5e9),'sim_ns':int(.1e9)}) is None
    assert f({'wall_ns':0,'sim_ns':int(5e9)},{'wall_ns':int(10e9),'sim_ns':int(5e9)}) is None
    assert f({'wall_ns':0,'sim_ns':0},{'wall_ns':int(67.5e9),'sim_ns':int(5.8e9)})==pytest.approx(5.8/67.5)
    assert f({'wall_ns':0,'sim_ns':0},{'wall_ns':int(100e9),'sim_ns':int(.1e9)})==.02  # floor


def test_carried_execution_wait_follows_plan_and_rtf_within_bounds():
    w=MODULE.carried_execution_wait
    assert w(_traj(10.8),None)==120.                       # no measured RTF: unchanged base wait
    assert w(_traj(0.),.08)==120.
    assert w(_traj(4.1),.5)==120.                          # fast simulation never shortens the base
    assert w(_traj(10.8),.086)==pytest.approx(10.8/.086*1.5+30)   # run-135626 lift-window RTF: ~218 s
    assert w(_traj(54.),.08)==600.                         # cap
    assert w(_traj(10.8),.086,cap=150.)==150.


def test_execution_loop_derives_the_wait_only_for_carried_steps():
    source=SCRIPT.read_text()
    assert "wait=carried_execution_wait(step['trajectory'],rtf)" in source
    assert "if step['stage'] in CARRIED_STAGES:" in source
    assert "ExecuteTrajectory.Goal(trajectory=step['trajectory']),wait)" in source
    assert "ExecuteTrajectory.Goal(trajectory=step['trajectory']),120)" not in source
    assert MODULE.CARRIED_STAGES==('PREPLAN_LIFT','PREPLAN_TRANSFER','PREPLAN_PLACE')
    # the release trajectory and all non-carried steps keep their fixed waits
    assert "ExecuteTrajectory.Goal(trajectory=release['trajectory']),30)" in source
