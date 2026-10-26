"""Calibrated live EPD observations -> world objects: transform, refusals and explicit virtual counterparts."""
import copy
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import live_world_objects as lwo  # noqa: E402
from test_camera_world_calibration import FRAME, TRUE, camera_of, make_cell  # noqa: E402

NOW = 1_793_049_674_597_366_211
MOUSE_WORLD = [0.45, -0.20, 0.02]
METER_WORLD = [0.55, -0.10, 0.025]
ENVIRONMENT = dict(workspace=dict(bounds=dict(x_min=-1.0, x_max=1.0, y_min=-1.0, y_max=1.0, z_min=0.0, z_max=1.8)),
                   perception=dict(live_counterparts=[
                       dict(id='mouse_1', label='mouse', shape='box', size_m=[0.115, 0.063, 0.044]),
                       dict(id='phone_1', label='cell phone', shape='box', size_m=[0.147, 0.072, 0.009])]))


def epd_object(label, world, extent, confidence=0.99, ratio=0.98, index=0, frame=FRAME):
    return dict(object_id=f'localization:{NOW}:{index}', label=label, confidence=confidence, shape='box',
                dimensions_xyz=extent, attributes=dict(valid_depth_ratio=ratio, roi_xywh=[420, 123, 126, 141]),
                pose=dict(frame_id=frame, position=camera_of([world])[0], orientation_xyzw=[0.0, 0.0, 0.0, 1.0]))


def snapshot(*objects, frame=FRAME, camera='realsense_overhead', stamp=NOW, mode='live'):
    return dict(schema_version='workcell_perception_snapshot/v1', runtime_mode=mode, camera_id=camera, frame_id=frame,
                timestamp_ns=stamp, objects=list(objects), provenance=dict(source_topic='/easy_perception_deployment/epd_localize_output'))


@pytest.fixture
def calibrated(tmp_path):
    import camera_world_calibration as cwc
    cell, binding = make_cell(tmp_path)
    return cwc.load_qualified(cell, binding), binding


def world(calibrated, snap, environment=ENVIRONMENT, age_s=0.4, calibration=True):
    record, binding = calibrated
    return lwo.world_snapshot(snap, record if calibration else None, binding, environment, now_ns=NOW, age_s=age_s)


def test_a_detection_is_placed_in_world_and_keeps_its_camera_measurement(calibrated):
    mouse = epd_object('mouse', MOUSE_WORLD, [0.114, 0.062, 0.044])
    result = world(calibrated, snapshot(mouse))
    (obj,) = result['objects']
    assert obj['pose']['frame_id'] == 'world' and np.allclose(obj['pose']['xyz'], MOUSE_WORLD, atol=0.003)
    assert obj['perception']['camera']['position'] == mouse['pose']['position']         # original data preserved
    assert obj['perception']['source_stamp_ns'] == NOW and obj['epd_object_id'] == mouse['object_id']
    assert obj['class_id'] == 'mouse' and obj['confidence'] == 0.99 and obj['class_verified'] is False
    rotation = np.array(TRUE)[:3, :3]                     # identity camera orientation -> camera axes in world
    from live_world_objects import _quaternion_matrix
    assert np.allclose(_quaternion_matrix(obj['pose']['orientation_xyzw']), rotation, atol=0.01)
    assert obj['planning']['status'] == 'READY' and obj['planning']['counterpart_id'] == 'mouse_1'
    assert obj['planning']['size_m'] == [0.115, 0.063, 0.044]           # geometry from the authored counterpart
    assert result['source']['mode'] == 'live_epd' and result['source']['calibration']['camera_serial'] == '841612070219'


def test_a_mislabelled_object_does_not_become_the_counterpart_of_its_label(calibrated):
    # The laser meter EPD calls "cell phone": its 5 cm height cannot be a 9 mm phone lying flat.
    meter = epd_object('cell phone', METER_WORLD, [0.117, 0.056, 0.051], confidence=0.97)
    (obj,) = world(calibrated, snapshot(meter))['objects']
    assert obj['planning']['status'] == 'BLOCKED' and 'does not fit' in obj['planning']['reason']
    chair = epd_object('chair', METER_WORLD, [0.3, 0.1, 0.2])
    (obj,) = world(calibrated, snapshot(chair))['objects']
    assert 'no virtual counterpart is authored' in obj['planning']['reason']


def test_two_detections_for_one_counterpart_bind_neither(calibrated):
    a = epd_object('mouse', MOUSE_WORLD, [0.11, 0.06, 0.04], index=0)
    b = epd_object('mouse', [0.40, -0.25, 0.02], [0.11, 0.06, 0.04], index=1)
    result = world(calibrated, snapshot(a, b))
    assert [o['planning']['status'] for o in result['objects']] == ['BLOCKED', 'BLOCKED']
    assert all('ambiguous' in o['planning']['reason'] for o in result['objects'])


@pytest.mark.parametrize('change,code', [
    (dict(age_s=2.5), 'STALE'),
    (dict(age_s=-1.0), 'CLOCK_DOMAIN'),
    (dict(calibration=False), 'CALIBRATION_MISSING'),
    (dict(snap=dict(frame='camera_depth_optical_frame')), 'FRAME_MISMATCH'),
    (dict(snap=dict(camera='other_camera')), 'CAMERA_MISMATCH'),
    (dict(snap=dict(mode='replay')), 'SNAPSHOT_INVALID'),
    (dict(environment={}), 'WORKSPACE_UNDEFINED'),
])
def test_unsafe_snapshots_refuse_every_object(calibrated, change, code):
    mouse = epd_object('mouse', MOUSE_WORLD, [0.11, 0.06, 0.04])
    kwargs = {k: v for k, v in change.items() if k != 'snap'}
    result = world(calibrated, snapshot(mouse, **change.get('snap', {})), **kwargs)
    assert result['objects'] == [] and any(r.startswith(code) for r in result['refused_objects'][0]['reasons'])


@pytest.mark.parametrize('mutate,code', [
    (lambda o: o['attributes'].update(valid_depth_ratio=0.3), 'INVALID_DEPTH'),
    (lambda o: o['attributes'].pop('valid_depth_ratio'), 'INVALID_DEPTH'),
    (lambda o: o['pose'].update(frame_id='camera_link'), 'FRAME_MISMATCH'),
    (lambda o: o['pose'].update(position=[float('nan'), 0, 1]), 'NO_3D_MEASUREMENT'),
    (lambda o: o.pop('dimensions_xyz'), 'NO_EXTENT'),
])
def test_unmeasured_objects_are_refused_individually(calibrated, mutate, code):
    good, bad = epd_object('mouse', MOUSE_WORLD, [0.11, 0.06, 0.04]), epd_object('mouse', MOUSE_WORLD, [0.11, 0.06, 0.04], index=1)
    mutate(bad)
    result = world(calibrated, snapshot(good, bad))
    assert len(result['objects']) == 1 and any(r.startswith(code) for r in result['refused_objects'][0]['reasons'])


def test_objects_outside_the_cell_workspace_are_refused(calibrated):
    environment = copy.deepcopy(ENVIRONMENT)
    environment['workspace']['bounds'].update(x_max=0.40)
    result = world(calibrated, snapshot(epd_object('mouse', MOUSE_WORLD, [0.11, 0.06, 0.04])), environment=environment)
    (refused,) = result['refused_objects']
    assert refused['reasons'][0].startswith('OUT_OF_WORKSPACE') and np.allclose(refused['world_xyz'], MOUSE_WORLD, atol=0.003)


def test_counterparts_must_be_explicit_boxes():
    with pytest.raises(ValueError, match='COUNTERPART_INVALID'):
        lwo.counterparts(dict(perception=dict(live_counterparts=[dict(id='x', label='mouse', size_m=[0.1, 0, 0.1])])))
    assert lwo.counterparts({}) == []
