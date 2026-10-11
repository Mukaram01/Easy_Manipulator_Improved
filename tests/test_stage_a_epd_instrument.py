"""Instrumented experiment tests; never execute model inference."""
import importlib.util
import gzip
import json
import shutil
import sys
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))


def api():
    assert importlib.util.find_spec('stage_a_epd_instrument'), 'missing isolated observer preparation'
    import stage_a_epd_instrument
    return stage_a_epd_instrument


def test_observer_insertion_removes_to_exact_original_sources():
    module = api()
    for name in ('ort_base.cpp', 'p3_ort_base.cpp'):
        path = ROOT / 'evidence/stage_a2_geometry/physics_owner/same_fragment/instrumented_inference/external/src' / (name + '.gz')
        original = gzip.decompress(path.read_bytes()).decode() if path.exists() else (module.EPD / 'src' / name).read_text()
        modified = module.instrument(name, original)
        assert module.strip_observers(modified) == original
        assert modified != original


def test_changed_source_is_blocked_before_build_or_inference():
    module = api()
    with pytest.raises(ValueError): module.instrument('ort_base.cpp', '// changed')


def test_claim_prevents_second_execution(tmp_path):
    module = api()
    module.claim_once(tmp_path)
    with pytest.raises(FileExistsError): module.claim_once(tmp_path)


def test_stage_comparison_detects_added_removed_and_shifted_support():
    import numpy as np
    assert importlib.util.find_spec('stage_a_epd_origin'), 'missing stage attribution analysis'
    import stage_a_epd_origin as analysis
    before = np.zeros((8, 8), np.float32); before[2, 2] = .8; before[7, 7] = .9
    copied = before.copy(); roi = copied[1:4, 1:4].copy()
    row = analysis.compare_stages(before, copied, roi, [1, 1, 4, 4])
    assert row['raw_to_full_copy'] == {'values_changed': 0, 'foreground_added': 0, 'foreground_removed': 0}
    assert row['raw_to_roi_embedded'] == {'values_changed': 1, 'foreground_added': 0, 'foreground_removed': 1}
    assert row['roi_values_changed_at_identical_coordinates'] == 0
    shifted = roi.copy(); shifted[1, 1] = 0; shifted[1, 2] = .8
    row = analysis.compare_stages(before, copied, shifted, [1, 1, 4, 4])
    assert row['raw_to_roi_embedded']['foreground_added'] == 1
    assert row['raw_to_roi_embedded']['foreground_removed'] == 2
    assert row['roi_values_changed_at_identical_coordinates'] == 2


@pytest.mark.parametrize('raw_bg,added_bg,classification', [(3, 0, 'MODEL OUTPUT'), (0, 2, 'POSTPROCESSING'), (3, 2, 'MIXED'), (0, 0, 'UNKNOWN')])
def test_classification_requires_observed_stage_support(raw_bg, added_bg, classification):
    assert importlib.util.find_spec('stage_a_epd_origin'), 'missing stage attribution analysis'
    import stage_a_epd_origin as analysis
    assert analysis.classify(raw_bg, added_bg) == classification


def test_genuine_single_inference_reproduces_historical_masks_and_attributes_spill():
    import stage_a_epd_origin as analysis
    report = analysis.analyse(analysis.FROZEN)
    assert report['classification'] == 'MODEL OUTPUT'
    assert report['inference_attempts'] == 1 and report['accepted'] == 0
    rows = report['detections']
    assert [r['raw_foreground_id_counts'] for r in rows] == [
        {'0': 449, '65517': 8313}, {'0': 509, '65523': 8663}, {'0': 58, '65517': 35}]
    assert [r['removed_background_samples'] for r in rows] == [153, 71, 0]
    assert [r['stage_differences']['raw_to_roi_embedded']['foreground_removed'] for r in rows] == [153, 71, 0]
    for row in rows:
        assert row['stage_differences']['raw_to_full_copy'] == {'values_changed': 0, 'foreground_added': 0, 'foreground_removed': 0}
        assert row['stage_differences']['raw_to_roi_embedded']['foreground_added'] == 0
        assert row['stage_differences']['roi_values_changed_at_identical_coordinates'] == 0
        assert row['historical_confidence_equal'] and row['historical_bbox_equal']
        assert row['historical_roi_values_changed'] == row['historical_mapped_pixels_changed'] == 0
        assert row['post_added_background_samples'] == 0
    assert [r['quantization_vs_float_pixel_centre_crop']['foreground_removed'] for r in rows] == [84, 0, 0]


def test_retained_preflight_preserves_typed_values_and_original_p3_path_bytes():
    import numpy as np
    import stage_a_epd_origin as analysis
    folder = analysis.FROZEN
    def raw(name): return gzip.decompress((folder / (name + '.gz')).read_bytes())
    manifest = json.loads((folder / 'sha256.json').read_text())
    prefix = 'synthetic_original_output/'
    for name in manifest:
        if name.startswith(prefix):
            assert raw(name) == raw(name.replace(prefix, 'synthetic_observed_output/', 1))
    assert np.frombuffer(raw(prefix + 'synthetic_ort_boundary_0.bin'), '<u4').tolist() == [0x80000000, 0x3f000000, 0x3f000001, 0x7f800000]
    assert np.frombuffer(raw(prefix + 'synthetic_ort_boundary_1.bin'), '<i8').tolist() == [1, 4294967297]


def test_checked_in_origin_report_matches_frozen_experiment():
    import stage_a_epd_origin as analysis
    assert analysis.analyse(analysis.FROZEN) == json.loads((analysis.FROZEN / 'analysis.json').read_text())


@pytest.mark.parametrize('kind', ['manifest', 'raw_tensor', 'roi', 'exit', 'source'])
def test_frozen_inference_tampering_fails_before_attribution(tmp_path, kind):
    import stage_a_epd_origin as analysis
    folder = tmp_path / 'bundle'; shutil.copytree(analysis.FROZEN, folder)
    if kind == 'manifest': (folder / 'sha256.json').write_text('{}')
    else:
        name = {'raw_tensor': 'output/session_outputs_after_Run_3.bin', 'roi': 'output/detection_0.roi.f32',
                'exit': 'process_exit.json', 'source': 'external/src/ort_base.cpp'}[kind]
        path = folder / (name + '.gz'); data = gzip.decompress(path.read_bytes())
        path.write_bytes(gzip.compress(bytes([data[0] ^ 1]) + data[1:]))
    with pytest.raises(ValueError): analysis.analyse(folder)
