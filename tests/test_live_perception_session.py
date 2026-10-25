"""Live perception: camera-frame observations against the authored task, with calibration kept explicit."""
import json
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import live_perception_session as lps  # noqa: E402

NOW = 1_000_000_000_000
UNCALIBRATED = dict(status='UNCALIBRATED', frame='camera_color_optical_frame', reason='no qualified camera-to-world transform')


def obs(label, confidence=0.9, **extra):
    return dict(object_id=f'localization:1:{label}', label=label, confidence=confidence,
                pose={'frame_id': 'camera_color_optical_frame', 'position': [0.06, 0.05, 0.5]},
                dimensions_xyz=[0.11, 0.06, 0.03], attributes={'roi_xywh': [1, 2, 3, 4], 'valid_depth_ratio': 0.99}, **extra)


FILTERS = dict(class_id='mouse', exclude_class_ids=['cell phone'], min_confidence=0.5, max_age_seconds=2.0)


def evaluate(item, age_s=0.4, filters=FILTERS, calibration=UNCALIBRATED):
    return lps.evaluate_observation(item, now_ns=NOW, snapshot_ns=NOW - int(age_s * 1e9), filters=filters,
                                    live_ceiling_s=2.0, calibration=calibration)


def test_task_policy_selects_the_pick_class_and_never_an_excluded_one():
    mouse, phone, chair = evaluate(obs('mouse')), evaluate(obs('cell phone')), evaluate(obs('chair'))
    assert mouse['task_eligible'] and mouse['reasons'] == []
    assert not phone['task_eligible'] and phone['excluded'] and "excluded by the task" in phone['reasons'][0]
    assert phone['obstacle'].startswith('candidate obstacle once a qualified camera-to-world')   # never one now
    assert not chair['task_eligible'] and not chair['excluded'] and "not the task's pick class" in chair['reasons'][0]
    assert mouse['position_camera'] == [0.06, 0.05, 0.5]                                        # camera frame kept


def test_stale_low_confidence_or_unmeasured_observations_are_not_eligible():
    assert 'stale: 2.50 s old (live limit 2.0 s)' in evaluate(obs('mouse'), age_s=2.5)['reasons']
    assert 'stamped' in evaluate(obs('mouse'), age_s=-1.0)['reasons'][0]                         # clock domain
    assert 'confidence 0.30 below task minimum 0.50' in evaluate(obs('mouse', confidence=0.3))['reasons']
    assert 'confidence unavailable for the task minimum' in evaluate(obs('mouse', confidence=None))['reasons']
    unmeasured = obs('mouse'); unmeasured.pop('pose'); unmeasured.pop('dimensions_xyz')
    assert {'no measured 3D position', 'no measured dimensions'} <= set(evaluate(unmeasured)['reasons'])
    assert evaluate(obs('mouse'), filters=None)['reasons'] == ['the cell has no saved v2 task']


def test_robot_planning_is_blocked_until_a_qualified_calibration_exists():
    assert evaluate(obs('mouse'))['robot_planning'].startswith('BLOCKED: no qualified camera-to-world transform')
    qualified = dict(status='QUALIFIED', frame='camera_color_optical_frame')
    assert evaluate(obs('mouse'), calibration=qualified)['robot_planning'] == 'READY'


def test_calibration_and_binding_come_only_from_the_saved_cell(tmp_path):
    cell = tmp_path / 'cell'; (cell / 'config').mkdir(parents=True)
    (cell / 'environment.yaml').write_text(yaml.safe_dump({'camera': {'camera_id': 'realsense_overhead'}}))
    with pytest.raises(lps.PerceptionUnavailable, match='no live camera binding'):
        lps.camera_binding(cell)
    assert lps.main(['bind', '--cell', str(cell), '--rgb-topic', '/cam/color/image_raw',
                     '--aligned-depth-topic', '/cam/aligned_depth_to_color/image_raw',
                     '--camera-info-topic', '/cam/color/camera_info', '--optical-frame', 'cam_optical']) == 0
    binding = lps.camera_binding(cell)
    assert binding['camera_id'] == 'realsense_overhead' and binding['calibration']['status'] == 'uncalibrated'
    assert lps.calibration_status(cell, binding)['status'] == 'UNCALIBRATED'                     # flag alone is not enough
    binding['calibration'] = dict(status='qualified', camera_to_world=None)
    assert lps.calibration_status(cell, binding)['status'] == 'UNCALIBRATED'                     # no transform recorded


def test_the_perception_runtime_must_be_registered(tmp_path, capsys):
    assert lps.main(['env', '--workspace', str(tmp_path)]) == 2
    assert 'PERCEPTION_RUNTIME_UNAVAILABLE' in capsys.readouterr().out
    source = (ROOT / 'scripts/live_perception_session.py').read_text()
    assert '/camera/camera' not in source and 'realsense_overhead' not in source                  # no hardcoded camera
