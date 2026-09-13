import copy
import importlib.util
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import runtime_pick_inputs as inputs
import perceived_object_grasp_plan as geometry


def fixtures():
    cell = yaml.safe_load((ROOT / 'scenes/ur5_2f_test/cell_definition.yaml').read_text())
    task = inputs.task_request(yaml.safe_load((ROOT / 'config/runtime/r1_4b_task.yaml').read_text()), cell)
    snapshot = inputs.replay_snapshot(yaml.safe_load((ROOT / 'config/runtime/r1_4_replay.yaml').read_text()), 100)
    return cell, task, snapshot


def test_normalized_contract_and_class_filter_preserve_obstacle():
    cell, task, snapshot = fixtures()
    objects = inputs.normalize(snapshot, 100, geometry)
    selected, rejected = inputs.filter_targets(objects, task, cell, 100, geometry)
    assert len(objects) == 2
    assert [o['class_id'] for o in selected] == ['cup']
    assert list(rejected.values()) == ['class_mismatch']
    task['target_class'] = 'bottle'
    assert inputs.filter_targets(objects, task, cell, 100, geometry)[0][0]['class_id'] == 'bottle'


@pytest.mark.parametrize('change', ['stale', 'outside', 'low_confidence'])
def test_target_rejections(change):
    cell, task, snapshot = fixtures()
    obj = snapshot['objects'][0]
    if change == 'stale': obj['timestamp'] = -100
    if change == 'outside': obj['pose']['xyz'][0] = 2
    if change == 'low_confidence': obj['confidence'] = 0.1
    objects = inputs.normalize(snapshot, 100, geometry)
    assert not inputs.filter_targets(objects, task, cell, 100, geometry)[0]


@pytest.mark.parametrize('change', ['frame', 'dimensions', 'nan', 'duplicate', 'future', 'missing_stamp'])
def test_malformed_observations_fail_closed(change):
    _, _, snapshot = fixtures()
    obj = snapshot['objects'][0]
    if change == 'frame': obj['pose']['frame_id'] = 'unknown_camera'
    if change == 'dimensions': obj['dimensions'][0] = 0
    if change == 'nan': obj['pose']['rpy'][0] = float('nan')
    if change == 'duplicate': snapshot['objects'][1]['object_id'] = obj['object_id']
    if change == 'future': obj['timestamp'] = 101
    if change == 'missing_stamp': del obj['timestamp']
    with pytest.raises((ValueError, KeyError)):
        inputs.normalize(snapshot, 100, geometry)


def test_rotated_zone_checks_whole_bounds():
    obj = dict(pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dimensions=[0.4, 0.1, 0.1])
    region = dict(pose_xyz=[0., 0., 0.], pose_rpy=[0., 0., 1.5707963267948966], dimensions=[0.2, 0.5, 0.2])
    assert inputs.contained(obj, region, geometry)
    obj['pose'][0] = 0.1
    assert not inputs.contained(obj, region, geometry)


def test_scene_inserts_target_and_distractor_with_stable_identity():
    pytest.importorskip('moveit_msgs')
    _, _, snapshot = fixtures()
    objects = inputs.normalize(snapshot, 100, geometry)
    diff = inputs.scene_diff(objects)
    assert diff.is_diff
    assert [o.id for o in diff.world.collision_objects] == [o['id'] for o in objects]
    assert all(o.operation == o.ADD for o in diff.world.collision_objects)
    assert len(diff.world.collision_objects) == 2


def test_replay_is_new_observation_and_does_not_mutate_source():
    _, _, snapshot = fixtures()
    original = copy.deepcopy(snapshot)
    inputs.replay_snapshot(snapshot, 200)
    assert snapshot == original
    snapshot['source']['mode'] = 'live_epd'
    with pytest.raises(ValueError): inputs.replay_snapshot(snapshot, 200)


def test_import_starts_no_ros_or_motion(monkeypatch):
    class Forbidden:
        def __getattr__(self, name):
            raise AssertionError('ROS accessed during import')
    monkeypatch.setitem(sys.modules, 'rclpy', Forbidden())
    spec = importlib.util.spec_from_file_location('idle_executor', ROOT / 'scripts/perceived_object_grasp_execute.py')
    spec.loader.exec_module(importlib.util.module_from_spec(spec))


def test_executor_reports_exact_acquisition_failure_without_execution(tmp_path):
    pytest.importorskip('rclpy')
    import json
    import subprocess
    _, _, snapshot = fixtures()
    snapshot['objects'][0]['pose']['frame_id'] = 'unresolved_camera'
    detections = tmp_path / 'objects.yaml'
    detections.write_text(yaml.safe_dump(snapshot))
    report = tmp_path / 'result.json'
    result = subprocess.run([sys.executable, str(ROOT / 'scripts/perceived_object_grasp_execute.py'),
        '--scene-package', str(ROOT / 'scenes/ur5_2f_test'),
        '--task-request', str(ROOT / 'config/runtime/r1_4b_task.yaml'),
        '--detections', str(detections), '--summary-output', str(report)],
        capture_output=True, text=True, timeout=15)
    assert result.returncode == 1, result.stderr
    status = json.loads(report.read_text())
    assert status['failed_stage'] == 'ACQUIRE_OBJECTS'
    assert 'unresolved_camera' in status['failure']
    assert status['execution_attempted'] is False
    assert status['current_stage'] == 'FAILED'


def test_replay_and_timestamped_live_input_share_downstream_contract():
    cell, task, replay = fixtures()
    live = copy.deepcopy(replay)
    live['source']['mode'] = 'live_perception'
    replay_objects = inputs.normalize(replay, 100, geometry)
    live_objects = inputs.normalize(live, 100, geometry)
    assert replay_objects == live_objects
    replay_target = inputs.filter_targets(replay_objects, task, cell, 100, geometry)[0][0]
    live_target = inputs.filter_targets(live_objects, task, cell, 100, geometry)[0][0]
    assert geometry.generate_box_grasp_candidates(geometry.build_grasp_target(replay_target), 0.12) == \
        geometry.generate_box_grasp_candidates(geometry.build_grasp_target(live_target), 0.12)
