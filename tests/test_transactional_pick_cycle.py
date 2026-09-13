import copy
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location('transactional_executor',ROOT/'scripts/perceived_object_grasp_execute.py')
executor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(executor)
spec = importlib.util.spec_from_file_location('transactional_inputs',ROOT/'scripts/runtime_pick_inputs.py')
inputs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(inputs)


def eligible():
    cell = yaml.safe_load((ROOT/'scenes/ur5_2f_test/cell_definition.yaml').read_text())
    task = inputs.task_request(yaml.safe_load((ROOT/'config/runtime/r1_4b_task.yaml').read_text()),cell)
    snapshot = inputs.replay_snapshot(yaml.safe_load((ROOT/'config/runtime/r1_4_replay.yaml').read_text()),100)
    second = copy.deepcopy(snapshot['objects'][0])
    second.update(object_id='second-cup', confidence=0.9)
    snapshot['objects'].append(second)
    objects = inputs.normalize(snapshot,100,executor._PLANNER)
    return inputs.filter_targets(objects,task,cell,100,executor._PLANNER)[0]


def test_full_candidate_retry_only_accepts_complete_success():
    targets = eligible()
    attempts = []
    calls = []
    def preplan(target,index,record):
        calls.append((target['id'],index))
        if target['id'] == targets[0]['id']:
            raise executor.CandidateFailure('PREPLAN_TRANSFER','attached object blocked')
        return dict(object_id=target['id'],grasp_index=index,full_cycle_prevalidated=True)
    result = executor.choose_cycle(list(reversed(targets)),executor.candidate_indices(8),preplan,attempts)
    assert result['object_id'] == targets[1]['id']
    assert result['full_cycle_prevalidated']
    assert calls[:8] == [(targets[0]['id'],i) for i in executor.candidate_indices(8)]
    assert calls[8] == (targets[1]['id'],3)
    assert all(a['failed_stage']=='PREPLAN_TRANSFER' for a in attempts[:-1])
    assert attempts[-1]['full_cycle_prevalidated']
    assert len(targets)==2 and all(t['class_id']=='cup' for t in targets)


@pytest.mark.parametrize('failed_stage',['PREPLAN_TRANSFER','PREPLAN_PLACE','PREPLAN_RETREAT'])
def test_late_planning_failure_rejects_all_candidates_without_acceptance(failed_stage):
    attempts = []
    def fail(target,index,record):
        raise executor.CandidateFailure(failed_stage,'obstacle')
    with pytest.raises(executor.CandidateFailure) as error:
        executor.choose_cycle(eligible(),[3,0],fail,attempts)
    assert error.value.stage==failed_stage
    assert len(attempts)==4
    assert not any(a['full_cycle_prevalidated'] for a in attempts)


def test_grasp_retry_before_next_target_and_tie_break_by_identity():
    targets = eligible()
    targets[0]['confidence']=targets[1]['confidence']
    expected = sorted(t['id'] for t in targets)
    calls=[]
    def plan(t,i,r):
        calls.append((t['id'],i))
        if i==3: raise executor.CandidateFailure('PREPLAN_LIFT','blocked')
        return dict(full_cycle_prevalidated=True)
    executor.choose_cycle(targets,[3,0],plan,[])
    assert calls==[(expected[0],3),(expected[0],0)]


@pytest.mark.parametrize('start,validated',[(False,True),(False,False),(True,False)])
def test_no_execution_without_both_gates(start,validated):
    with pytest.raises(RuntimeError):
        executor.require_prevalidated_execution(start,dict(full_cycle_prevalidated=validated))


def test_execution_gate_accepts_full_cycle():
    executor.require_prevalidated_execution(True,dict(full_cycle_prevalidated=True))


def test_private_attachment_is_relative_and_does_not_mutate_live_world():
    pytest.importorskip('moveit_msgs')
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import PlanningScene,CollisionObject
    from shape_msgs.msg import SolidPrimitive
    scene = PlanningScene()
    target = CollisionObject(id='arbitrary-track')
    target.header.frame_id='world'
    target.pose.position.x=0.5
    target.pose.orientation.w=1.0
    target.primitives=[SolidPrimitive(type=1,dimensions=[0.05,0.05,0.10])]
    local=Pose();local.orientation.w=1.0
    target.primitive_poses=[local]
    obstacle=copy.deepcopy(target);obstacle.id='distractor'
    scene.world.collision_objects=[target,obstacle]
    saved=copy.deepcopy(scene)
    frame=Pose();frame.position.x=0.4;frame.orientation.w=1.0
    attached = executor.private_attachment(scene,target,'ee_palm',frame,['tip'])
    assert scene==saved
    assert [o.id for o in attached.world.collision_objects]==['distractor']
    body=attached.robot_state.attached_collision_objects[0]
    assert body.object.header.frame_id=='ee_palm'
    assert body.object.pose.position.x==pytest.approx(0.1)
    assert body.touch_links==['tip']
    # The same attached state is advanced for all held-object segments.
    from sensor_msgs.msg import JointState
    attached.robot_state.joint_state=JointState(name=['joint'],position=[0.0])
    for value in (0.1,0.2,0.3):
        attached.robot_state=executor.updated_state(attached.robot_state,{'joint':value})
        assert attached.robot_state.attached_collision_objects[0]==body
    diff=executor.place_detachment_diff(target,'ee_palm',[0.6,0.2,0.1])
    assert diff.robot_state.attached_collision_objects[0].object.operation==CollisionObject.REMOVE
    assert diff.world.collision_objects[0].id==target.id


def test_joint_divergence_fails_before_next_trajectory():
    pytest.importorskip('moveit_msgs')
    from moveit_msgs.msg import RobotState
    from sensor_msgs.msg import JointState
    state=RobotState(joint_state=JointState(name=['joint'],position=[0.0]))
    executor.assert_joint_match(state,copy.deepcopy(state))
    with pytest.raises(RuntimeError,match='joint state diverged'):
        executor.assert_joint_match(state,executor.updated_state(state,{'joint':0.1}))


def test_obstacle_change_invalidates_execution_snapshot():
    pytest.importorskip('moveit_msgs')
    from moveit_msgs.msg import PlanningScene
    from sensor_msgs.msg import JointState
    scene=inputs.scene_diff([dict(id='obstacle',pose=[0.0,0.0,0.1,0.0,0.0,0.0,1.0],dimensions=[0.1,0.1,0.1])])
    scene.robot_state.joint_state=JointState(name=['joint'],position=[0.0])
    executor.assert_scene_match(scene,copy.deepcopy(scene))
    changed=copy.deepcopy(scene)
    changed.world.collision_objects[0].pose.position.x += 0.01
    with pytest.raises(RuntimeError,match='world collision geometry diverged'):
        executor.assert_scene_match(changed,scene)


def test_mesh_equality_ignores_transport_stamps_but_detects_vertex_changes():
    pytest.importorskip('moveit_msgs')
    from moveit_msgs.msg import PlanningScene,CollisionObject
    from shape_msgs.msg import Mesh,MeshTriangle
    from geometry_msgs.msg import Point,Pose
    obj=CollisionObject(id='mesh')
    obj.header.frame_id='world'
    obj.pose.orientation.w=1.0
    obj.meshes=[Mesh(vertices=[Point(x=0.0,y=0.0,z=0.0),Point(x=1.0,y=0.0,z=0.0),Point(x=0.0,y=1.0,z=0.0)],
                     triangles=[MeshTriangle(vertex_indices=[0,1,2])])]
    p=Pose();p.orientation.w=1.0;obj.mesh_poses=[p]
    scene=PlanningScene();scene.world.collision_objects=[obj]
    actual=copy.deepcopy(scene);actual.world.collision_objects[0].header.stamp.sec=123
    executor.assert_scene_match(actual,scene)
    actual.world.collision_objects[0].meshes[0].vertices[0].x=0.01
    with pytest.raises(RuntimeError,match='geometry diverged'):
        executor.assert_scene_match(actual,scene)


def test_initial_joint_publication_can_arrive_after_collision_geometry(monkeypatch):
    from types import SimpleNamespace as NS
    def scene(names, positions):
        return NS(robot_state=NS(joint_state=NS(name=names, position=positions), is_diff=False))
    pending = scene(['shoulder_pan_joint', 'gripper_finger1_joint'], [0.0, 0.0])
    ready = scene(['shoulder_pan_joint', 'gripper_finger1_joint'], [1.57, 0.0])
    snapshots = iter([pending, ready])
    monkeypatch.setattr(executor.time, 'sleep', lambda _: None)
    result = executor.wait_for_robot_baseline(lambda: next(snapshots), {'shoulder_pan_joint': 1.57})
    assert result is ready
    assert pending.robot_state.joint_state.position == [0.0, 0.0]


@pytest.mark.parametrize('positions', [[0.0, 0.0], [1.57, 0.1]])
def test_baseline_timeout_does_not_accept_wrong_home_or_closed_gripper(positions):
    from types import SimpleNamespace as NS
    state = NS(joint_state=NS(name=['shoulder_pan_joint', 'gripper_finger1_joint'], position=positions), is_diff=False)
    with pytest.raises(RuntimeError, match='canonical home/open-gripper state unavailable'):
        executor.wait_for_robot_baseline(lambda: NS(robot_state=state), {'shoulder_pan_joint': 1.57}, timeout=0)
