#!/usr/bin/env python3
"""Offline comparison at the instrumented exported-ONNX/P3/ROI boundaries."""
import argparse
import gzip
import json
from pathlib import Path

import numpy as np

import stage_a_epd_retained as adapter
import stage_a_epd_spill as baseline

FROZEN = baseline.RETAINED.parent / 'instrumented_inference'
MANIFEST_SHA256 = '11f9fe0899926a07f666b060df50c29eff6b89219a454bdcada03f707dfb19c6'


def difference(before, after):
    adapter.require(before.shape == after.shape and before.dtype == after.dtype == np.float32,
                    'incomparable mask buffers')
    adapter.require(np.all(np.isfinite(before)) and np.all(np.isfinite(after)), 'nonfinite masks')
    a, b = before > .5, after > .5
    return {'values_changed': int(np.count_nonzero(before.view(np.uint32) != after.view(np.uint32))),
            'foreground_added': int(np.count_nonzero(b & ~a)), 'foreground_removed': int(np.count_nonzero(a & ~b))}


def compare_stages(raw, copied, roi, box):
    x0, y0, x1, y1 = box
    embedded = np.zeros_like(raw); embedded[y0:y1, x0:x1] = roi
    return {'raw_to_full_copy': difference(raw, copied),
            'raw_to_roi_embedded': difference(raw, embedded),
            'roi_values_changed_at_identical_coordinates': difference(raw[y0:y1, x0:x1].copy(), roi)['values_changed']}


def classify(raw_background, post_added_background):
    if raw_background and post_added_background: return 'MIXED'
    if post_added_background: return 'POSTPROCESSING'
    if raw_background: return 'MODEL OUTPUT'
    return 'UNKNOWN'


def analyse(folder):
    """Accept an original experiment directory or a frozen gzip bundle."""
    folder = Path(folder)
    manifest = None
    if (folder / 'sha256.json').exists():
        adapter.require(adapter.sha((folder / 'sha256.json').read_bytes()) == MANIFEST_SHA256, 'changed instrumented manifest')
        manifest = json.loads((folder / 'sha256.json').read_text())
    def raw(name):
        path = folder / name
        data = path.read_bytes() if path.exists() else gzip.decompress((folder / (name + '.gz')).read_bytes())
        if manifest is not None: adapter.require(adapter.sha(data) == manifest[name], 'changed instrumented artifact: ' + name)
        return data
    if manifest is not None:
        for name in manifest: raw(name)
    owner, capture, rgb, ids, historical_dets, historical_rois, historical_masks = baseline.load_retained()
    pre, status = json.loads(raw('preflight.json')), json.loads(raw('process_exit.json'))
    historical_pre = json.loads(gzip.decompress((baseline.RETAINED / 'epd/preflight.json.gz').read_bytes()))
    adapter.require(pre['original_preflight'] == historical_pre, 'changed historical experiment binding')
    adapter.check_preprocessing(pre['preprocessing'])
    adapter.require(json.loads(raw('inference_claim.json')) == {'maximum_inference_calls': 1, 'retry_allowed': False, 'simulator_runs': 0}, 'missing one-shot claim')
    after = json.loads(raw('external_after.json'))
    adapter.require(after['source_sha256'] == pre['external_source_sha256'] and after['git_status'] == pre['external_git_status'], 'changed external checkout')
    loaded = json.loads(raw('loaded_library_sha256.json'))
    # Normalize symlinks lexically using the captured historical actual paths,
    # then compare content hashes; offline hosts need no installed EPD libraries.
    allowed_hashes = set(pre['historical_loaded_library_sha256'].values())
    adapter.require(all(h in allowed_hashes for h in loaded.values()), 'changed loaded runtime dependency hash')
    if manifest is not None:
        inventory = json.loads(raw('retention_inventory.json'))
        for name, digest in {**pre['source_sha256'], **pre['external_source_sha256']}.items():
            adapter.require(adapter.sha(raw(inventory[name])) == digest, 'changed captured source: ' + name)
    adapter.require(status['exit_code'] == 0 and status['inference_attempts'] == 1 and not status['timeout'], 'failed/multiple inference attempts')
    adapter.require(adapter.sha(raw('input.rgb8')) == capture['rgb_sha256'] == pre['input_sha256'], 'changed inference RGB')
    adapter.require(raw('output/epd_input.bgr8') == adapter.prepared_bgr(rgb).tobytes(), 'changed instrumented preprocessing')
    native = json.loads(raw('output/inference.json'))
    adapter.require(len(native['detections']) == 3, 'changed native detection cardinality')
    rows = json.loads(raw('output/session_outputs_after_Run.json'))
    adapter.require([r['name'] for r in rows] == ['boxes', 'labels', 'scores', 'masks'] and
                    [r['onnx_element_type'] for r in rows] == [1, 7, 1, 1], 'unexpected ONNX output contract')
    outputs = []
    for row in rows:
        dtype = '<i8' if row['onnx_element_type'] == 7 else '<f4'
        data = raw('output/' + row['file'])
        adapter.require(len(data) == row['bytes'], 'changed raw tensor size')
        outputs.append(np.frombuffer(data, dtype=dtype).reshape(row['shape']))
    boxes, labels, scores, model_masks = outputs
    adapter.require(boxes.shape == (3, 4) and labels.shape == scores.shape == (3,) and
                    model_masks.shape == (3, 1, 512, 512), 'unexpected actual tensor shapes')
    adapter.require(np.all(np.isfinite(model_masks)) and np.all((model_masks >= 0) & (model_masks <= 1)), 'invalid raw mask probabilities')
    expanded = ids.repeat(2, axis=0).repeat(2, axis=1)
    diagnostics, masks = [], []
    for index, det in enumerate(native['detections']):
        stage = json.loads(raw(f'output/detection_{index}.json'))
        adapter.require(stage['raw_index'] == stage['epd_index'] == index and
                        det['class_index'] == labels[index] and det['confidence'] == float(scores[index]), 'changed detection index/class/confidence')
        adapter.require(stage['floating_box'] == boxes[index].tolist() and stage['roi_xyxy_exclusive'] == det['bbox'], 'changed box/index binding')
        x0, y0, x1, y1 = det['bbox']
        full = np.frombuffer(raw('output/' + stage['full_file']), '<f4').reshape(512, 512)
        roi = np.frombuffer(raw('output/' + stage['roi_file']), '<f4').reshape(y1-y0, x1-x0)
        adapter.require(roi.tobytes() == raw('output/' + det['mask_file']), 'observer/main ROI mismatch')
        model_mask = model_masks[index, 0]
        differences = compare_stages(model_mask, full, roi, det['bbox'])
        embedded = np.zeros((512, 512), np.float32); embedded[y0:y1, x0:x1] = roi
        target = 65523 if index == 1 else 65517
        raw_bg = int(np.count_nonzero((model_mask > .5) & (expanded == 0)))
        added_bg = int(np.count_nonzero((embedded > .5) & ~(model_mask > .5) & (expanded == 0)))
        # A diagnostic float-box crop convention only; not an alternate EPD path.
        yy, xx = np.indices((512, 512))
        fx0, fy0, fx1, fy1 = boxes[index]
        floating_crop = (xx+.5 >= fx0) & (xx+.5 < fx1) & (yy+.5 >= fy0) & (yy+.5 < fy1)
        floating_embedded = np.where(floating_crop, model_mask, np.float32(0))
        mask = adapter.map_mask(roi, det['bbox']); masks.append(mask)
        diagnostics.append({
            'raw_index': index, 'epd_index': stage['epd_index'], 'class_index': int(labels[index]),
            'confidence': float(scores[index]), 'floating_box': boxes[index].tolist(), 'roi_box': det['bbox'],
            'truncated_box_before_clamp': stage['truncated_box_before_clamp'],
            'classification': classify(raw_bg, added_bg), 'stage_differences': differences,
            'raw_foreground_id_counts': baseline.counts(expanded[model_mask > .5]),
            'post_roi_foreground_id_counts': baseline.counts(expanded[embedded > .5]),
            'post_added_background_samples': added_bg,
            'removed_background_samples': int(np.count_nonzero((model_mask > .5) & ~(embedded > .5) & (expanded == 0))),
            'quantization_vs_float_pixel_centre_crop': difference(floating_embedded.astype(np.float32), embedded),
            'float_crop_convention': 'diagnostic only: x+.5,y+.5 inside half-open floating xyxy box',
            'historical_confidence_equal': det['confidence'] == historical_dets[index]['confidence'],
            'historical_bbox_equal': det['bbox'] == historical_dets[index]['bbox'],
            'historical_roi_values_changed': difference(historical_rois[index], roi)['values_changed'],
            'historical_roi_sha256': adapter.sha(historical_rois[index].tobytes()), 'instrumented_roi_sha256': adapter.sha(roi.tobytes()),
            'historical_mapped_pixels_changed': int(np.count_nonzero(mask != historical_masks[index])),
            'raw_background_runs_y_x0_x1_exclusive': baseline.runs((model_mask > .5) & (expanded == 0)),
        })
    association = adapter.associate(owner, capture, rgb, ids, native['detections'], masks)
    return {'baseline': '6640cea2', 'classification': classify(sum(int(np.count_nonzero((m[0] > .5) & (expanded == 0))) for m in model_masks),
                                                           sum(d['post_added_background_samples'] for d in diagnostics)),
            'classification_scope': 'exported ONNX model outputs versus external EPD native-512 processing; inverse union measured separately in baseline regression',
            'manifest_sha256': MANIFEST_SHA256 if manifest is not None else None,
            'tensor_contract': rows, 'accepted': association['accepted'], 'associations': association['detections'], 'detections': diagnostics,
            'ground_truth_limit': 'repeated 2x2 retained 256 ID cells, not independent native 512 ground truth',
            'uncertainty': 'graph-internal mask-head versus paste/resize contribution unobserved; no internal graph tensors requested',
            'inference_attempts': status['inference_attempts'], 'execution_goals': 0}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('folder', type=Path)
    print(json.dumps(analyse(parser.parse_args().folder), indent=2, sort_keys=True))
