"""Fixtures derived from epd_msgs/{EPDObjectLocalization,EPDObjectTracking,LocalizedObject}.msg.
Publisher: easy_perception_deployment.hpp, epd_localize_output/epd_tracking_output.
No camera, MoveIt or execution required.
"""
import copy
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import capture_epd_detected_objects as bridge
import runtime_pick_inputs as runtime
import perceived_object_grasp_plan as planner


def message(labels=('cup',), frame='camera_color_optical_frame'):
    # The ROS publisher sets pose.position=centroid and exports metric dimensions.
    objects = []
    for i, label in enumerate(labels):
        xyz = dict(x=0.26 + i * 0.14, y=-0.08, z=0.051)
        objects.append(dict(name=label, centroid=xyz,
            pose=dict(position=xyz, orientation=dict(x=0., y=0., z=0., w=1.)),
            length=0.06, breadth=0.06, height=0.10,
            axis=dict(x=1., y=0., z=0.), roi={}, segmented_binary_mask={}, segmented_pcl={}))
    return dict(header=dict(frame_id=frame, stamp=dict(sec=100, nanosec=500000000)),
                objects=objects, ppx=320., ppy=240., fx=600., fy=600.,
                frame_width=640, frame_height=480, depth_image={}, process_time=12)


def convert(msg):
    return bridge.convert_epd_runtime_message(msg, '/easy_perception_deployment/epd_localize_output',
                                             'ur5_2f_test', missing_confidence=0.8)


def namespace(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{k: namespace(v) for k, v in value.items()})
    if isinstance(value, list):
        return [namespace(v) for v in value]
    return value


@pytest.mark.parametrize('labels', [('cup',), ('cup', 'bottle')])
def test_perception_selection_grasp_probe(labels):
    msg = message(labels)
    snapshot = convert(namespace(msg))  # Same attribute access as generated ROS messages.
    assert snapshot == convert(msg)
    assert snapshot['schema_version'] == 'detected_objects/v1'
    assert snapshot['objects'][0]['timestamp'] == 100.5
    assert snapshot['objects'][0]['attributes']['confidence_available'] is False
    # Explicit calibrated fixture transform, through the capture script's existing TF hook.
    def transform(source, target, xyz, rpy, timeout):
        assert source == 'camera_color_optical_frame' and target == 'world'
        return [xyz[0], xyz[1], xyz[2] + 0.01], rpy, 'fixture calibration'
    for obj in snapshot['objects']:
        bridge._normalize_pose_with_tf(obj, 'world', 1., transform)
        assert obj['raw_pose']['frame_id'] == 'camera_color_optical_frame'
    objects = runtime.normalize(snapshot, 100.5, planner)
    cell = dict(environment=dict(task_zones=[dict(id='source', frame='world',
        pose_xyz=[0.3, 0., 0.15], pose_rpy=[0., 0., 0.], dimensions=[1., 1., 1.]),
        dict(id='destination', frame='world', pose_xyz=[1., 0., 0.], dimensions=[1., 1., 1.])]))
    task = runtime.task_request(dict(action='pick_and_place', target_class='cup',
        source_zone='source', destination_zone='destination'), cell)
    selected, rejected = runtime.filter_targets(objects, task, cell, 100.5, planner)
    assert [o['class_id'] for o in selected] == ['cup']
    assert list(rejected.values()) == (['class_mismatch'] if len(labels) == 2 else [])
    target = planner.build_grasp_target(selected[0])
    assert target['planning_frame'] == 'world'
    assert target['target_dimensions'] == [0.06, 0.06, 0.10]
    assert target['target_pose'][:3] == pytest.approx([0.26, -0.08, 0.061])
    candidates = planner.generate_box_grasp_candidates(target)
    assert len(candidates) == 8
    assert candidates[0][:3] == pytest.approx([0.26, -0.08, 0.231])


@pytest.mark.parametrize('frame', ['camera_color_optical_frame', 'wrong_frame'])
def test_unresolved_frame_rejected_by_existing_runtime(frame):
    snapshot = convert(message(frame=frame))
    assert snapshot['objects'][0]['pose']['frame_id'] == frame
    with pytest.raises(ValueError, match='no valid world transform'):
        runtime.normalize(snapshot, 100.5, planner)


@pytest.mark.parametrize('field,value,error', [
    ('pose', None, 'pose.position'), ('pose', {}, 'pose.position'),
    ('length', 0., 'positive'), ('height', float('nan'), 'finite'),
    ('name', '', 'name required'),
])
def test_incomplete_object(field, value, error):
    msg = message()
    msg['objects'][0][field] = value
    with pytest.raises(ValueError, match=error):
        convert(msg)


def test_invalid_header_quaternion_and_tracking():
    msg = message()
    msg['header']['frame_id'] = ''
    with pytest.raises(ValueError, match='frame_id required'): convert(msg)
    msg = message()
    msg['header']['stamp'] = {}
    with pytest.raises(ValueError, match='observation time'): convert(msg)
    msg = message()
    msg['objects'][0]['pose']['orientation']['w'] = 0.
    with pytest.raises(ValueError, match='quaternion'): convert(msg)
    msg = message(('cup', 'bottle'))
    msg['object_ids'] = ['track-2', 'track-7']
    assert [o['object_id'] for o in convert(msg)['objects']] == msg['object_ids']
    msg['object_ids'] = ['same', 'same']
    with pytest.raises(ValueError, match='unique'): convert(msg)
    msg['object_ids'] = ['one']
    with pytest.raises(ValueError, match='align'): convert(msg)


def test_identity_and_explicit_confidence_policy():
    msg = message()
    original = copy.deepcopy(msg)
    assert convert(msg)['objects'][0]['object_id'] == convert(msg)['objects'][0]['object_id']
    assert msg == original
    for confidence in (None, -1, 2, float('nan')):
        with pytest.raises(ValueError):
            bridge.convert_epd_runtime_message(msg, 'topic', 'scene', missing_confidence=confidence)


def test_generated_ros_messages():
    epd = pytest.importorskip('epd_msgs.msg')
    from rosidl_runtime_py.set_message import set_message_fields
    for message_type in (epd.EPDObjectLocalization, epd.EPDObjectTracking):
        raw = message(('cup', 'bottle'), frame='world')
        if message_type is epd.EPDObjectTracking:
            raw['object_ids'] = ['cup-track', 'bottle-track']
        msg = message_type()
        set_message_fields(msg, raw)
        snapshot = convert(msg)
        objects = runtime.normalize(snapshot, 100.5, planner)
        assert [o['class_id'] for o in objects] == ['cup', 'bottle']
        assert len(planner.generate_box_grasp_candidates(planner.build_grasp_target(objects[0]))) == 8
