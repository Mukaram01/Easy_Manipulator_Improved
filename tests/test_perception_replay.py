"""Stage-B camera-free perception: calibration, replay-clock semantics and physical binding (no ROS, no camera).

The recordings here are tiny synthetic test fixtures written by the test itself, never presented as camera data.
"""
import copy
import hashlib
import json
import math
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import perception_replay as pr  # noqa: E402

LINK_POSE = [1.05, -0.2, 0.55, 0.0, math.atan2(0.55, 0.65), math.pi]
FRAMES = [1_400_000_000, 1_600_000_000]


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def cell(tmp_path, pose=LINK_POSE):
    scene = tmp_path / 'cell'
    scene.mkdir()
    (scene / 'environment.yaml').write_text(yaml.safe_dump({'camera': {
        'enabled': True, 'camera_id': 'cam', 'parent_frame': 'world', 'frame_id': 'camera_color_optical_frame',
        'pose': list(pose), 'acceptance_stream': {'color_width': 640, 'color_height': 480}}}))
    return scene


def recording(tmp_path, scene, calibration=None):
    rec = tmp_path / 'rec'
    rec.mkdir()
    observations, rows = [], []
    for i, stamp in enumerate(FRAMES):
        for kind in ('rgb', 'depth'):
            (rec / f'{kind}_{i}.png').write_bytes(f'synthetic test {kind} {i}'.encode())
        observations.append(dict(stamp_ns=stamp, rgb=f'rgb_{i}.png', depth=f'depth_{i}.png'))
        rows.append(dict(stamp_ns=stamp, rgb_sha256=_sha(rec / f'rgb_{i}.png'), depth_sha256=_sha(rec / f'depth_{i}.png')))
    (rec / 'fixture.json').write_text(json.dumps(dict(schema_version=1, observations=observations)))
    camera = pr.cell_camera(yaml.safe_load((scene / 'environment.yaml').read_text()))
    manifest = dict(schema=pr.RECORDING_SCHEMA, provenance=pr.PROVENANCE, clock_domain=pr.CLOCK_DOMAIN, cell=str(scene),
                    physics_world_sha256='world-a', camera=camera, fixture='fixture.json',
                    fixture_sha256=_sha(rec / 'fixture.json'), frames=rows,
                    calibration=calibration if calibration is not None else dict(
                        status=pr.SIMULATION_EXACT, camera_id='cam', camera_link_pose_world=list(LINK_POSE),
                        optical_in_link=[list(r) for r in pr.OPTICAL_IN_LINK]))
    (rec / 'recording.json').write_text(json.dumps(manifest))
    return rec, manifest


def epd_snapshot(objects, stamp=FRAMES[-1], runtime_mode='replay'):
    return dict(schema_version='workcell_perception_snapshot/v1', runtime_mode=runtime_mode, camera_id='cam',
                frame_id='camera_color_optical_frame', timestamp_ns=stamp, objects=objects, rejected_objects=[
                    dict(label='cup', confidence=0.99, failure_reasons=['insufficient_depth'])])


def epd_object(label, optical, confidence=0.9, oid='1'):
    return dict(object_id=oid, label=label, confidence=confidence, dimensions_xyz=[0.08, 0.06, 0.1],
                pose=dict(frame_id='camera_color_optical_frame', position=list(optical)), attributes={'quality': 'valid'})


def replay(tmp_path, objects=None, **kw):
    scene = cell(tmp_path)
    rec, manifest = recording(tmp_path, scene)
    objects = objects if objects is not None else [epd_object('cup', [-0.0528, -0.0419, 0.8217])]
    snapshot = pr.replay_snapshot_from_epd([epd_snapshot(objects, **kw)], manifest, rec / 'recording.json',
                                           pr.cell_camera(yaml.safe_load((scene / 'environment.yaml').read_text())))
    return scene, snapshot


def test_optical_detection_reaches_its_physical_world_position():
    # Measured on LENLEG: EPD's cup centroid in the rendered frame -> the cup's box centre (0.40, -0.25, 0.04).
    world = pr.optical_to_world(LINK_POSE, [-0.05285574495792389, -0.04189075529575348, 0.8216514587402344])
    assert math.dist(world[:2], [0.40, -0.25]) < 0.01 and abs(world[2] - 0.05) < 0.02


def test_replay_snapshot_is_a_replay_on_the_recording_clock(tmp_path):
    scene, snapshot = replay(tmp_path)
    obj = snapshot['objects'][0]
    assert snapshot['source']['mode'] == 'replayed_snapshot' and snapshot['source']['provenance'] == pr.PROVENANCE
    assert obj['age_seconds'] == obj['perception']['replay_age_s'] == 0.0           # last frame of the recording
    assert snapshot['source']['rejected_detections'][0]['reasons'] == ['insufficient_depth']   # missing depth: never a target
    pr.validate_replay(snapshot, scene, physics_world_sha256='world-a')


def test_an_earlier_frame_is_aged_by_the_recording_clock_and_stale_beyond_the_ceiling(tmp_path):
    scene, snapshot = replay(tmp_path, stamp=FRAMES[0])
    assert snapshot['objects'][0]['age_seconds'] == pytest.approx(0.2)
    pr.validate_replay(snapshot, scene)
    stale = copy.deepcopy(snapshot)
    stale['objects'][0]['age_seconds'] = stale['objects'][0]['perception']['replay_age_s'] = 2.5
    with pytest.raises(pr.PerceptionBlocked, match='PERCEPTION_STALE'):
        pr.validate_replay(stale, scene)


@pytest.mark.parametrize('mutate, code', [
    (lambda s: s['source'].update(mode='live'), 'LIVE_INPUT_NOT_REPLAYABLE'),                 # stale live input
    (lambda s: s['source'].update(clock_domain='host_wall_clock'), 'CLOCK_DOMAIN_MISMATCH'),
    (lambda s: s['source']['epd'].update(snapshot_timestamp_ns=1792965101193819423), 'CLOCK_DOMAIN_MISMATCH'),
    (lambda s: s['source'].pop('calibration'), 'CALIBRATION_MISSING'),
    (lambda s: s['source']['calibration'].update(status='uncalibrated'), 'CALIBRATION_INVALID'),
    (lambda s: s['source'].update(producer='hand_written'), 'REPLAY_INPUT_INVALID'),
    (lambda s: s.update(objects=[]), 'OBSERVATION_UNAVAILABLE'),
])
def test_invalid_replays_are_blocked_with_an_actionable_code(tmp_path, mutate, code):
    scene, snapshot = replay(tmp_path)
    mutate(snapshot)
    with pytest.raises(pr.PerceptionBlocked, match=code):
        pr.validate_replay(snapshot, scene)


def test_a_moved_camera_or_another_world_invalidates_the_recording(tmp_path):
    scene, snapshot = replay(tmp_path)
    with pytest.raises(pr.PerceptionBlocked, match='RECORDING_WORLD_MISMATCH'):
        pr.validate_replay(snapshot, scene, physics_world_sha256='world-b')
    (scene / 'environment.yaml').write_text((scene / 'environment.yaml').read_text().replace('0.55', '0.56'))
    with pytest.raises(pr.PerceptionBlocked, match='CALIBRATION_INVALID: the cell camera moved'):
        pr.validate_replay(snapshot, scene)


def test_live_epd_output_is_never_turned_into_a_replay(tmp_path):
    with pytest.raises(pr.PerceptionBlocked, match='LIVE_INPUT_NOT_REPLAYABLE'):
        replay(tmp_path, runtime_mode='live')


def test_a_changed_recording_frame_is_rejected(tmp_path):
    scene, snapshot = replay(tmp_path)
    (pr.resolve_recording(snapshot['source']['recording'], scene).parent / 'rgb_1.png').write_bytes(b'tampered')
    with pytest.raises(pr.PerceptionBlocked, match='RECORDING_INVALID'):
        pr.validate_replay(snapshot, scene)


def perceived(label, xyz, oid):
    return dict(object_id=oid, class_id=label, confidence=0.8, pose=dict(frame_id='world', xyz=list(xyz)),
                perception=dict(replay_age_s=0.0))


MEASURED = {'object_cup': ([0.40, -0.25, 0.0405, 0, 0, 0], [0.05, 0.05, 0.081]),
            'object_bottle': ([0.47, -0.09, 0.15, 0, 0, 0], [0.074, 0.074, 0.30])}


def test_each_measured_object_takes_only_its_own_detection():
    bindings, unbound = pr.associate(MEASURED, [perceived('cup', [0.404, -0.254, 0.051], '2'),
                                                perceived('bottle', [0.495, -0.096, 0.157], '1')], 'cup')
    assert bindings['object_cup']['class_id'] == 'cup' and bindings['object_bottle']['class_id'] == 'bottle'
    assert unbound == []


def test_an_object_with_no_or_conflicting_detections_stays_unperceived():
    bindings, _ = pr.associate(MEASURED, [perceived('bottle', [0.48, -0.09, 0.15], '1')], 'cup')
    assert bindings['object_cup'] == dict(class_id=pr.UNPERCEIVED, confidence=None, reason='no EPD detection')
    bindings, _ = pr.associate(MEASURED, [perceived('cup', [0.40, -0.25, 0.05], 'a'),
                                          perceived('bowl', [0.41, -0.25, 0.05], 'b')], 'cup')
    assert bindings['object_cup']['class_id'] == pr.UNPERCEIVED and 'conflicting' in bindings['object_cup']['reason']


def test_a_target_detection_with_no_physical_object_blocks():
    with pytest.raises(pr.PerceptionBlocked, match='PERCEPTION_WORLD_MISMATCH'):
        pr.associate(MEASURED, [perceived('cup', [0.20, -0.30, 0.05], '9')], 'cup')
    _, unbound = pr.associate(MEASURED, [perceived('chair', [0.20, -0.30, 0.05], '9')], 'cup')
    assert unbound[0]['class_id'] == 'chair'                      # reported, not a target: does not block


def test_a_recording_inside_the_cell_is_referenced_relative_to_it(tmp_path):
    scene = cell(tmp_path)
    inside = scene / 'perception/recording/recording.json'
    assert pr.recording_reference(inside, scene) == 'perception/recording/recording.json'
    assert pr.resolve_recording('perception/recording/recording.json', scene) == scene / 'perception/recording/recording.json'
    assert pr.recording_reference(tmp_path / 'elsewhere/recording.json', scene) == str((tmp_path / 'elsewhere/recording.json').resolve())


def test_a_detection_inside_one_box_binds_to_it_even_with_a_neighbour_within_the_margin():
    # Collision scene: bottle 4 mm from the cup. The cup's centroid is inside the cup box only.
    measured = {'object_cup': ([0.50, -0.25, 0.0225, 0, 0, 0], [0.03, 0.03, 0.045]),
                'object_bottle': ([0.50, -0.30, 0.115, 0, 0, 0], [0.062, 0.062, 0.23])}
    bindings, unbound = pr.associate(measured, [perceived('cup', [0.5036, -0.2532, 0.0283], '2'),
                                                perceived('bottle', [0.521, -0.300, 0.112], '1')], 'cup')
    assert bindings['object_cup']['class_id'] == 'cup' and bindings['object_bottle']['class_id'] == 'bottle' and not unbound
    # Outside both, within the margin of both: ambiguous, and a target detection blocks.
    with pytest.raises(pr.PerceptionBlocked, match='ambiguous'):
        pr.associate(measured, [perceived('cup', [0.50, -0.2675, 0.02], '3')], 'cup')


def test_physics_readiness_needs_the_registered_epd_runtime_that_every_session_detects_with(tmp_path, monkeypatch):
    import live_perception_session as lps
    import physics_cycle_preflight as preflight
    scene = tmp_path / 'cell'
    (scene / 'config').mkdir(parents=True)
    (scene / 'perception').mkdir()
    (scene / 'perception/perception_replay.yaml').write_text('{}')
    (scene / 'config/workcell_builder_task_intent.yaml').write_text(yaml.safe_dump(dict(
        schema='workcell_builder_task_intent/v2', pick=dict(selection=dict(source_type='perception')),
        perception=dict(mode='replayed_snapshot', replay_path='perception/perception_replay.yaml'),
        safety=dict(execution_backend='simulator'))))
    monkeypatch.setattr(pr, 'validate_replay', lambda *args, **kwargs: None)   # a valid replay of this cell
    blocker = preflight.check_perception(scene, None, tmp_path)                 # no perception runtime registered
    assert blocker.startswith('PERCEPTION_RUNTIME_UNAVAILABLE') and 'live_perception_session.py register' in blocker
    monkeypatch.setattr(lps, 'load_runtime', lambda workspace: {})
    assert preflight.check_perception(scene, None, tmp_path) is None
    runner = (ROOT / 'scripts/run_stage_a1_finish.py').read_text()               # refused before the build, too
    assert runner.index('load_runtime(workspace)') < runner.index('build=build_commissioning(')
