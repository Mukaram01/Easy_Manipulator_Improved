"""Camera-to-world calibration: robust rigid fit, qualification limits, measurement and the saved record."""
import copy
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import camera_world_calibration as cwc  # noqa: E402

FRAME = 'camera_color_optical_frame'


def rotation(rx, ry, rz):
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    return (np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]) @
            np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]))


# A camera 0.8 m above the table looking down and slightly tilted: world = R @ camera + t.
TRUE = cwc.matrix(rotation(math.pi - 0.2, 0.05, 1.3), np.array([0.45, -0.2, 0.8]))
WORLD = [[0.30, -0.35, 0.0], [0.60, -0.35, 0.0], [0.60, -0.05, 0.0], [0.30, -0.05, 0.0], [0.45, -0.20, 0.10],
         [0.38, -0.12, 0.05]]


def camera_of(world, transform=TRUE):
    m = np.linalg.inv(transform)
    return [(m[:3, :3] @ np.array(w) + m[:3, 3]).tolist() for w in world]


def points(noise=0.0, seed=0, world=WORLD, std=0.001):
    rng = np.random.default_rng(seed)
    return [dict(id=f'p{i}', world_xyz=w, camera_xyz=(np.array(c) + rng.normal(0, noise, 3)).tolist(), std_m=std)
            for i, (w, c) in enumerate(zip(world, camera_of(world)))]


def test_exact_points_recover_the_transform_and_qualify():
    result = cwc.solve(points())
    assert result['status'] == cwc.QUALIFIED and result['reasons'] == []
    assert np.allclose(result['camera_to_world'], TRUE, atol=1e-9)
    assert result['residuals']['rms_m'] < 1e-9 and result['residuals']['max_leave_one_out_m'] < 1e-9
    assert np.linalg.det(np.array(result['camera_to_world'])[:3, :3]) == pytest.approx(1.0)


def test_millimetre_noise_qualifies_and_reports_honest_errors():
    result = cwc.solve(points(noise=0.0015))
    r = result['residuals']
    assert result['status'] == cwc.QUALIFIED
    assert 0 < r['rms_m'] < 0.005 and r['max_leave_one_out_m'] >= r['max_residual_m'] * 0.5
    assert len(r['per_point']) == 6 and all('leave_one_out_m' in p for p in r['per_point'])
    centre = cwc.apply(result['camera_to_world'], camera_of([[0.45, -0.2, 0.0]])[0])
    assert np.linalg.norm(np.array(centre) - [0.45, -0.2, 0.0]) < 0.005


def test_one_mismeasured_point_is_rejected_as_an_outlier_without_corrupting_the_fit():
    bad = points()
    bad[2]['world_xyz'] = [0.70, -0.05, 0.0]                      # tape read 10 cm wrong
    result = cwc.solve(bad)
    assert result['residuals']['outliers'] == ['p2'] and result['status'] == cwc.QUALIFIED
    assert np.allclose(result['camera_to_world'], TRUE, atol=1e-9)
    worse = copy.deepcopy(bad)
    worse[4]['world_xyz'] = [0.45, -0.20, 0.30]
    result = cwc.solve(worse)                                      # 2 of 6 outliers exceed the 25 % allowance
    assert result['status'] == cwc.REJECTED and any('outliers' in r for r in result['reasons'])


@pytest.mark.parametrize('world,noise,message', [
    ([[0.3 + 0.1 * i, -0.2, 0.0] for i in range(5)], 0.0, 'no three reference points'),       # collinear
    ([[0.40, -0.2, 0.0], [0.42, -0.2, 0.0], [0.40, -0.18, 0.0], [0.42, -0.18, 0.01]], 0.0, 'span'),   # clustered
    (WORLD, 0.012, 'residual'),                                                               # too noisy
])
def test_degenerate_or_inaccurate_reference_sets_are_rejected(world, noise, message):
    try:
        result = cwc.solve(points(noise=noise, world=world, seed=3))
    except cwc.CalibrationError as exc:
        assert message in str(exc)
        return
    assert result['status'] == cwc.REJECTED and any(message in r for r in result['reasons']), result['reasons']


def test_too_few_or_unrepeatable_points_are_refused():
    with pytest.raises(cwc.CalibrationError, match='at least 4'):
        cwc.solve(points()[:3])
    unmeasured = points(); unmeasured[0]['camera_xyz'] = None; unmeasured[1]['camera_xyz'] = None; unmeasured[2]['camera_xyz'] = None
    with pytest.raises(cwc.CalibrationError, match='at least 4'):
        cwc.solve(unmeasured)
    shaky = points(); shaky[1]['std_m'] = 0.009
    result = cwc.solve(shaky)
    assert result['status'] == cwc.REJECTED and any('varies 9.0 mm' in r for r in result['reasons'])


K = [600.0, 0, 320.0, 0, 600.0, 240.0, 0, 0, 1]


def test_deprojection_uses_the_registered_depth_median_and_refuses_holes():
    depth = np.full((480, 640), 0.8)
    depth[240, 420] = 5.0                                          # a speckle inside the window does not move the median
    assert cwc.deproject(depth, 420, 240, K) == pytest.approx([100 / 600 * 0.8, 0.0, 0.8])
    depth[230:251, 410:431] = 0.0
    assert cwc.deproject(depth, 420, 240, K) is None
    assert cwc.deproject(depth, 1, 1, K) is None                   # window leaves the image


def test_aruco_targets_are_located_and_averaged_over_frames():
    import cv2
    aruco = cv2.aruco
    dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
    rgb = np.full((480, 640, 3), 255, np.uint8)
    rgb[100:180, 200:280] = cv2.cvtColor(aruco.drawMarker(dictionary, 7, 80), cv2.COLOR_GRAY2RGB)
    assert cwc.aruco_centres(rgb, 'DICT_4X4_50')[7] == pytest.approx((239.5, 139.5), abs=0.6)
    doc = dict(points=[dict(id='m7', aruco_id=7, world_xyz=[0.1, 0.2, 0.0]),
                       dict(id='px', pixel=[400, 300], world_xyz=[0.3, 0.2, 0.0])])
    frames = [(rgb, np.full((480, 640), 0.6 + 0.0005 * i), K) for i in range(6)]
    measured = {p['id']: p for p in cwc.measure(frames, doc)}
    assert measured['m7']['frames_measured'] == 6 and measured['m7']['camera_xyz'][2] == pytest.approx(0.60125)
    assert measured['px']['camera_xyz'] == pytest.approx([80 / 600 * 0.60125, 60 / 600 * 0.60125, 0.60125])
    assert 0 < measured['px']['std_m'] < 0.002
    few = cwc.measure(frames[:3], doc)
    assert all(p['camera_xyz'] is None for p in few)               # below the minimum frame count: unmeasured


def test_reference_point_files_must_be_complete_and_attributed(tmp_path):
    path = tmp_path / 'points.yaml'
    base = dict(schema=cwc.POINTS_SCHEMA, world_frame='world', measured_by='operator', measurement_method='tape from base',
                points=[dict(id='a', aruco_id=1, world_xyz=[0, 0, 0])])
    for change, message in [(dict(world_frame='camera'), 'world_frame'), (dict(measured_by=None), 'measured_by'),
                            (dict(points=[dict(id='a', world_xyz=[0, 0])] ), 'world_xyz'),
                            (dict(points=[dict(id='a', world_xyz=[0, 0, 0])]), 'exactly one'),
                            (dict(points=[dict(id='a', aruco_id=1, world_xyz=[0, 0, 0])] * 2), 'unique')]:
        path.write_text(yaml.safe_dump(dict(base, **change)))
        with pytest.raises(cwc.CalibrationError, match=message):
            cwc.load_points(path)
    path.write_text(yaml.safe_dump(base))
    assert cwc.load_points(path)['points'][0]['id'] == 'a'


def make_cell(tmp_path, solution=None):
    cell = tmp_path / 'cell'; (cell / 'config').mkdir(parents=True)
    binding = dict(camera_id='realsense_overhead', rgb_topic='/camera/camera/color/image_raw', optical_frame=FRAME,
                   aligned_depth_topic='/camera/camera/aligned_depth_to_color/image_raw',
                   camera_info_topic='/camera/camera/color/camera_info',
                   calibration=dict(status='uncalibrated', camera_to_world=None))
    (cell / 'environment.yaml').write_text(yaml.safe_dump(dict(camera=dict(camera_id='realsense_overhead'),
                                                               perception=dict(live_camera=binding))))
    capture = dict(points=points(noise=0.001), points_file='points.yaml', points_sha256='0' * 64, measured_by='operator',
                   measurement_method='tape', frames=dict(count=15),
                   camera=dict(serial='841612070219', optical_frame=FRAME, rgb_topic=binding['rgb_topic'],
                               depth_source='registered', color_intrinsics=dict(width=640, height=480, k=K)))
    record = cwc.build_record(solution or cwc.solve(capture['points']), capture, binding)
    cwc.save(cell, record, binding)
    return cell, yaml.safe_load((cell / 'environment.yaml').read_text())['perception']['live_camera']


def test_a_qualified_record_is_saved_with_the_cell_and_reloads(tmp_path):
    cell, binding = make_cell(tmp_path)
    assert binding['calibration']['status'] == 'qualified' and binding['calibration']['camera_serial'] == '841612070219'
    record = cwc.load_qualified(cell, binding)
    assert record['source_frame'] == FRAME and record['target_frame'] == 'world'
    assert record['provenance']['emd_commit'] and record['reference_points'][0]['world_xyz'] == WORLD[0]
    q = record['camera_to_world']['quaternion_xyzw']
    assert sum(v * v for v in q) == pytest.approx(1.0)


def test_tampered_mismatched_or_rejected_calibrations_are_refused(tmp_path):
    cell, binding = make_cell(tmp_path)
    path = cell / binding['calibration']['record']
    for change, message in [(lambda b: b.update(camera_id='other'), 'CALIBRATION_MISMATCH'),
                            (lambda b: b.update(optical_frame='camera_depth_optical_frame'), 'CALIBRATION_MISMATCH'),
                            (lambda b: b['calibration'].update(camera_to_world=np.eye(4).tolist()), 'binding transform')]:
        changed = copy.deepcopy(binding); change(changed)
        with pytest.raises(cwc.CalibrationError, match=message):
            cwc.load_qualified(cell, changed)
    record = yaml.safe_load(path.read_text())
    record['camera_to_world']['matrix'][0][3] += 0.05
    path.write_text(yaml.safe_dump(record, sort_keys=False))
    with pytest.raises(cwc.CalibrationError, match='changed after it was accepted'):
        cwc.load_qualified(cell, binding)
    rejected_cell = tmp_path / 'rejected'
    rejected_cell.mkdir()
    noisy = cwc.solve(points(noise=0.012, seed=3))
    assert noisy['status'] == cwc.REJECTED
    cell2, binding2 = make_cell(rejected_cell, solution=noisy)
    assert binding2['calibration']['status'] == 'uncalibrated'          # a rejected fit never becomes the binding


def test_stricter_limits_on_reload_refuse_an_old_calibration(tmp_path):
    cell, binding = make_cell(tmp_path)
    with pytest.raises(cwc.CalibrationError, match='CALIBRATION_ERROR_EXCESSIVE'):
        cwc.load_qualified(cell, binding, limits=dict(cwc.LIMITS, rms_m=1e-6))
