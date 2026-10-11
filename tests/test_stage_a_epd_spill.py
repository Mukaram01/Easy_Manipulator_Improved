"""Offline retained regression; every accepting mask here is TEST FIXTURE ONLY."""
import copy
import gzip
import importlib.util
import json
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import stage_a_epd_retained as adapter
from stage_a_fragment_identity import mask_identity


def diagnostic():
    spec = importlib.util.find_spec('stage_a_epd_spill')
    assert spec is not None, 'missing offline spill regression/diagnostic'
    import stage_a_epd_spill
    return stage_a_epd_spill


@pytest.fixture(scope='module')
def retained():
    return diagnostic().load_retained()


def test_exact_retained_baseline_and_native_spill():
    report = diagnostic().diagnose()
    assert report['accepted'] == 0
    rows = report['detections']
    assert [r['confidence'] for r in rows] == [
        0.9756200909614563, 0.8621252775192261, 0.05048725754022598]
    assert [r['mapped']['id_counts'] for r in rows] == [
        {'0': 123, '65517': 2079}, {'0': 160, '65523': 2166}, {'0': 19, '65517': 10}]
    assert [r['native']['id_counts'] for r in rows] == [
        {'0': 296, '65517': 8313}, {'0': 438, '65523': 8663}, {'0': 58, '65517': 35}]
    assert [r['inverse_new_background_cells'] for r in rows] == [0, 0, 0]
    assert [r['inverse_background_fill_samples'] for r in rows] == [196, 202, 18]
    assert all(r['association'] == 'REJECTED' for r in rows)
    assert [r['native']['boundary_background_pixels'] for r in rows] == [227, 266, 22]
    assert [r['mapped']['boundary_background_pixels'] for r in rows] == [114, 135, 11]
    assert all(r[s]['cross_object_pixels'] == r[s]['detached_background_component_pixels'] == 0
               for r in rows for s in ('native', 'mapped'))
    # Reconstruct every diagnostic coordinate from the compact visualizable runs.
    _, _, _, ids, _, rois, masks = diagnostic().load_retained()
    for row, roi, mapped in zip(rows, rois, masks):
        whole = np.zeros((512, 512), bool)
        x0, y0, x1, y1 = row['bbox']; whole[y0:y1, x0:x1] = roi > .5
        for scale, mask, identity in [('native', whole, ids.repeat(2, 0).repeat(2, 1)), ('mapped', mapped, ids)]:
            restored = np.zeros(mask.shape, bool)
            for y, x0, x1 in row[scale]['background_runs_y_x0_x1_exclusive']:
                restored[y, x0:x1] = True
            assert np.array_equal(restored, mask & (identity == 0))


def test_checked_in_diagnostic_matches_verified_replay():
    path = ROOT / 'evidence/stage_a2_geometry/physics_owner/same_fragment/spill_regression/diagnostic.json'
    assert diagnostic().diagnose() == json.loads(path.read_text())


def test_clean_single_id_fixtures_accept_but_one_background_or_mixed_ids_reject(retained):
    o, r, rgb, ids, dets, rois, masks = retained
    # Test-only silhouettes, never returned as genuine masks by the replay.
    fixture_masks = [ids == 65517, ids == 65523]
    assert adapter.associate(o, r, rgb, ids, dets[:2], fixture_masks)['accepted'] == 2
    for mask in fixture_masks:
        assert mask_identity(r, rgb, ids, mask, r['session'], r['frame'], r['rgb_sha256']) in (65517, 65523)
        bad = mask.copy(); bad[0, 0] = True
        assert adapter.associate(o, r, rgb, ids, dets[:1], [bad])['accepted'] == 0
    assert adapter.associate(o, r, rgb, ids, dets[:1], [fixture_masks[0] | fixture_masks[1]])['accepted'] == 0
    assert adapter.associate(o, r, rgb, ids, [dets[0]] * 2, [fixture_masks[0]] * 2)['accepted'] == 0


@pytest.mark.parametrize('kind', ['rgb', 'id', 'rgb_hash', 'id_hash', 'session', 'mapping'])
def test_altered_capture_rejects(retained, kind):
    o, r, rgb, ids, dets, _, masks = copy.deepcopy(retained)
    if kind == 'rgb': rgb[0, 0, 0] ^= 1
    if kind == 'id': ids[0, 0] = 65517
    if kind == 'rgb_hash': r['rgb_sha256'] = 'wrong'
    if kind == 'id_hash': r['id_sha256'] = 'wrong'
    if kind == 'session': r['acquisition']['session'] = 'wrong'
    if kind == 'mapping': r['renderer']['visuals'][1]['renderer_id'] = r['renderer']['visuals'][0]['renderer_id']
    with pytest.raises(ValueError): adapter.associate(o, r, rgb, ids, dets, masks)


@pytest.mark.parametrize('kind', ['artifact', 'manifest', 'session'])
def test_retained_tampering_rejects_before_analysis(tmp_path, kind):
    api = diagnostic()
    folder = tmp_path / 'retained'
    shutil.copytree(api.RETAINED, folder)
    if kind == 'manifest':
        (folder / 'epd/sha256.json').write_text('{}')
    else:
        path = folder / ('epd/input.rgb8.gz' if kind == 'artifact' else 'epd/preflight.json.gz')
        raw = gzip.decompress(path.read_bytes())
        if kind == 'artifact': raw = bytes([raw[0] ^ 1]) + raw[1:]
        else:
            pre = json.loads(raw); pre['session'] = 'wrong'
            raw = json.dumps(pre).encode()
        path.write_bytes(gzip.compress(raw))
    with pytest.raises(ValueError): api.load_retained(folder)


def test_changed_original_fail_closed_source_rejects(tmp_path, monkeypatch):
    api = diagnostic()
    paths = set(json.loads((api.RETAINED / 'source_sha256.json').read_text())) | set(api.CONTRACT_HASHES)
    paths.add('scripts/stage_a_rgbd/epd_retained.cpp')
    for name in paths:
        dest = tmp_path / name; dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes((ROOT / name).read_bytes())
    (tmp_path / 'scripts/stage_a_fragment_identity.py').write_text('changed')
    monkeypatch.setattr(api, 'ROOT', tmp_path)
    with pytest.raises(ValueError, match='changed baseline source'):
        api.load_retained()


def test_threshold_rejections_preserve_original_contract(retained):
    o, r, rgb, ids, dets, _, _ = retained
    fixture = ids == 65517
    det = copy.deepcopy(dets[0]); det['confidence'] = np.nextafter(.8, 0)
    assert adapter.associate(o, r, rgb, ids, [det], [fixture])['accepted'] == 0
    # The existing confidence rule is inclusive at .8; do not change it.
    det['confidence'] = .8
    assert adapter.associate(o, r, rgb, ids, [det], [fixture])['accepted'] == 1
    mask = adapter.map_mask(np.full((512, 512), .5, np.float32), [0, 0, 512, 512])
    assert adapter.associate(o, r, rgb, ids, [det], [mask])['accepted'] == 0


@pytest.mark.parametrize('roi,box', [
    (np.ones((1, 1), np.float64), [0, 0, 1, 1]),
    (np.ones((1, 1), np.float32), [-1, 0, 0, 1]),
    (np.ones((1, 1), np.float32), [511, 511, 513, 512]),
    (np.ones((1, 1), np.float32), [0, 0, 2, 1]),
    (np.ones((1, 1), np.float32), [0., 0, 1, 1]),
    (np.array([[np.nan]], np.float32), [0, 0, 1, 1]),
    (np.array([[np.inf]], np.float32), [0, 0, 1, 1]),
    (np.array([[-.01]], np.float32), [0, 0, 1, 1]),
    (np.array([[1.01]], np.float32), [0, 0, 1, 1]),
])
def test_malformed_roi_rejects(roi, box):
    with pytest.raises(ValueError): adapter.map_mask(roi, box)


def test_threshold_is_strict_and_union_uses_every_sample_centre():
    low = np.nextafter(np.float32(.5), np.float32(0))
    high = np.nextafter(np.float32(.5), np.float32(1))
    for value, expected in [(low, 0), (.5, 0), (high, 1)]:
        for y, x in [(0, 0), (1, 1), (40, 61), (41, 60), (511, 511)]:
            mask = adapter.map_mask(np.array([[value]], np.float32), [x, y, x+1, y+1])
            assert mask.sum() == expected
            assert mask[y//2, x//2] == bool(expected)
    # Threshold-before-union preserves a solitary >.5 sample, unlike averaging.
    roi = np.array([[high, 0], [0, 0]], np.float32)
    assert adapter.map_mask(roi, [10, 10, 12, 12])[5, 5]
    assert not (cv2.resize(roi, (1, 1), interpolation=cv2.INTER_LINEAR) > .5).any()


def test_pixel_centre_linear_interpolation_and_bbox_placement():
    # OpenCV's inverse coordinate is (u512+.5)/2-.5, with edge replication.
    rgb = np.zeros((256, 256, 3), np.uint8)
    rgb[:, 1:, :] = [80, 40, 20]
    bgr = adapter.prepared_bgr(rgb)
    assert bgr[100, :4].tolist() == [[0, 0, 0], [5, 10, 20], [15, 30, 60], [20, 40, 80]]
    roi = np.ones((2, 2), np.float32)
    mapped = adapter.map_mask(roi, [1, 1, 3, 3])
    assert np.argwhere(mapped).tolist() == [[0, 0], [0, 1], [1, 0], [1, 1]]


def test_component_distance_and_coordinate_diagnostics_are_exact():
    ids = np.zeros((8, 8), np.uint32); ids[2:4, 2:4] = 7
    mask = ids == 7; mask[2, 4] = True; mask[7, 7] = True
    row = diagnostic().support(mask, ids, 7)
    assert row['boundary_background_pixels'] == 1
    assert row['off_boundary_background_pixels'] == 1
    assert row['detached_background_component_pixels'] == 1
    assert row['background_distance_squared_histogram'] == {'1': 1, '32': 1}
    assert row['background_runs_y_x0_x1_exclusive'] == [[2, 4, 5], [7, 7, 8]]
    assert row['mask_component_sizes'] == [5, 1]
