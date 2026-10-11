#!/usr/bin/env python3
"""Deterministic offline diagnosis of retained EPD spill. Never invokes inference.

IDs are measurement references only; no mask is trimmed or qualified here.
The 512 reference is the retained 256 raster repeated into exact 2x2 cells,
not a separately observed 512 renderer raster.
"""
import gzip
import json
from pathlib import Path
import tempfile

import cv2
import numpy as np

import stage_a_epd_retained as adapter

ROOT = Path(__file__).resolve().parents[1]
RETAINED = ROOT / 'evidence/stage_a2_geometry/physics_owner/same_fragment/separated_workpieces'
MANIFEST_HASHES = {
    'source_sha256.json': '217d71307406e599f3ec4ba7ee04af004afb3e6c44a685c456ff16b948f2f569',
    'capture/sha256.json': 'd6c89566a759a423c00f4773aec0c1d292c46c95429d45069b1bf01c1a95feda',
    'epd/sha256.json': '16bb70352a9b9864c26722deca21bd978dbb72f45e057dcf393e72d8d5de9aed',
}
CONTRACT_HASHES = {
    'scripts/stage_a_fragment_identity.py': '3a4f4496fdc91775f9cebc49de10d6f818de4efcdd45d0e0a5103f96e4ba4127',
    'scripts/stage_a_physics_contact.py': 'bcfe6bf167ae1b54e0f75ff1286bf7082c526f7770f1afeed2a55fea02fc500c',
}
CONFIDENCES = [0.9756200909614563, 0.8621252775192261, 0.05048725754022598]
MAPPED_COUNTS = [{'0': 123, '65517': 2079}, {'0': 160, '65523': 2166}, {'0': 19, '65517': 10}]
NATIVE_COUNTS = [{'0': 296, '65517': 8313}, {'0': 438, '65523': 8663}, {'0': 58, '65517': 35}]


def verified_artifacts(folder):
    """Pin manifests before trusting any data or source hash they contain."""
    folder = Path(folder)
    for name, digest in MANIFEST_HASHES.items():
        adapter.require(adapter.sha((folder / name).read_bytes()) == digest, 'changed manifest: ' + name)
    artifacts = {}
    for group in ('capture', 'epd'):
        manifest = json.loads((folder / group / 'sha256.json').read_bytes())
        artifacts[group] = {}
        for name, digest in manifest.items():
            raw = gzip.decompress((folder / group / (name + '.gz')).read_bytes())
            adapter.require(adapter.sha(raw) == digest, 'changed retained artifact: ' + group + '/' + name)
            artifacts[group][name] = raw
    source_hashes = json.loads((folder / 'source_sha256.json').read_text())
    pre = json.loads(artifacts['epd']['preflight.json'])
    # Recorded absolute checkout paths are historical. Resolve repository sources
    # in this checkout; external EPD sources remain provenance, not dependencies.
    for name, digest in pre['source_sha256'].items():
        if '/workcell_ws_stage_a2/' in name:
            source_hashes[name.split('/workcell_ws_stage_a2/', 1)[1]] = digest
    source_hashes.update(CONTRACT_HASHES)
    for name, digest in source_hashes.items():
        adapter.require(adapter.sha((ROOT / name).read_bytes()) == digest, 'changed baseline source: ' + name)
    return artifacts


def load_retained(folder=RETAINED):
    artifacts = verified_artifacts(folder)
    cap, epd = artifacts['capture'], artifacts['epd']
    # Existing acquisition validator expects an uncompressed raw manifest. Only
    # decompress into a disposable directory, never into the retained evidence.
    with tempfile.TemporaryDirectory(prefix='epd-spill-replay-') as tmp:
        for name, raw in cap.items():
            (Path(tmp) / name).write_bytes(raw)
        owner, report, rgb, ids, _ = adapter.load_capture(tmp)
    pre, result, native = (json.loads(epd[n]) for n in ('preflight.json', 'result.json', 'inference.json'))
    adapter.check_preprocessing(pre['preprocessing'])
    adapter.check_preprocessing(result['preprocessing'])
    for key in ('session', 'frame', 'acquisition', 'rgb_sha256', 'id_sha256', 'camera_profile'):
        adapter.require(pre[key] == report[key], 'changed EPD capture binding: ' + key)
    adapter.require(epd['input.rgb8'] == rgb.tobytes(), 'changed EPD RGB bytes')
    adapter.require(epd['epd_input.bgr8'] == adapter.prepared_bgr(rgb).tobytes() and
                    adapter.sha(epd['epd_input.bgr8']) == pre['prepared_bgr_sha256'], 'changed BGR preparation')
    adapter.require(native['status'] == 'INFERENCE_COMPLETE' and result['exit_code'] == 0 and
                    json.loads(epd['execution_claim.json']) == {'inference_calls': 1, 'simulator_runs': 0},
                    'inconsistent retained inference provenance')
    dets, rois, masks = native['detections'], [], []
    adapter.require(len(dets) == 3 and [d['confidence'] for d in dets] == CONFIDENCES, 'changed baseline detections')
    for index, det in enumerate(dets):
        x0, y0, x1, y1 = det['bbox']
        raw = epd[det['mask_file']]
        roi = np.frombuffer(raw, dtype='<f4').reshape(y1-y0, x1-x0)
        mask = adapter.map_mask(roi, det['bbox'])
        adapter.require(mask.astype(np.uint8).tobytes() == epd[det['mask_file'] + '.original.bool8'],
                        'changed retained inverse mapping')
        adapter.require(counts(ids[mask]) == MAPPED_COUNTS[index], 'changed baseline mask support')
        rois.append(roi); masks.append(mask)
    replay = adapter.associate(owner, report, rgb, ids, dets, masks)
    adapter.require(replay['accepted'] == result['accepted'] == 0, 'changed baseline acceptance')
    for actual, original in zip(replay['detections'], result['detections']):
        adapter.require(all(original[k] == v for k, v in actual.items()), 'changed baseline association')
    return owner, report, rgb, ids, dets, rois, masks


def counts(values):
    keys, sizes = np.unique(values, return_counts=True)
    return {str(int(k)): int(v) for k, v in zip(keys, sizes)}


def runs(mask):
    """Lossless row runs [y,x_begin,x_end_exclusive] in image coordinates."""
    result = []
    for y in np.flatnonzero(mask.any(axis=1)):
        edges = np.diff(np.pad(mask[y].astype(np.int8), (1, 1)))
        result.extend([int(y), int(a), int(b)] for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))
    return result


def support(mask, ids, target):
    """Exact integer distances and 8-connected component/adjacency diagnostics."""
    silhouette = ids == target
    bg = mask & (ids == 0)
    adjacent = cv2.dilate(silhouette.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool) & ~silhouette
    n, labels, stats, _ = cv2.connectedComponentsWithStats(bg.astype(np.uint8), connectivity=8)
    components = []
    for i in range(1, n):
        components.append({'pixels': int(stats[i, cv2.CC_STAT_AREA]),
                           'touches_silhouette': bool(np.any(adjacent & (labels == i)))})
    # Integer squared Euclidean distance between pixel centres: deterministic,
    # no floating distance-transform approximation, bounded by retained masks.
    true_points = np.argwhere(silhouette).astype(np.int64)
    nearest = []
    for point in np.argwhere(bg):
        nearest.append(int(np.min(np.sum((true_points - point)**2, axis=1))))
    _, _, mask_stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    return {
        'pixels': int(mask.sum()), 'id_counts': counts(ids[mask]),
        'cross_object_pixels': int(np.count_nonzero(mask & (ids != 0) & ~silhouette)),
        'boundary_background_pixels': int(np.count_nonzero(bg & adjacent)),
        'off_boundary_background_pixels': int(np.count_nonzero(bg & ~adjacent)),
        'background_components': components,
        'detached_background_component_pixels': sum(c['pixels'] for c in components if not c['touches_silhouette']),
        'mask_component_sizes': sorted(mask_stats[1:, cv2.CC_STAT_AREA].tolist(), reverse=True),
        'background_distance_squared_histogram': counts(np.asarray(nearest, dtype=np.int64)),
        'background_runs_y_x0_x1_exclusive': runs(bg),
    }


def diagnose(folder=RETAINED):
    owner, capture, rgb, ids, dets, rois, masks = load_retained(folder)
    expanded = ids.repeat(2, axis=0).repeat(2, axis=1)
    association = adapter.associate(owner, capture, rgb, ids, dets, masks)
    rows = []
    for index, (det, roi, mapped) in enumerate(zip(dets, rois, masks)):
        target = 65523 if index == 1 else 65517
        x0, y0, x1, y1 = det['bbox']
        whole = np.zeros((512, 512), bool); whole[y0:y1, x0:x1] = roi > .5
        native = support(whole, expanded, target)
        adapter.require(native['id_counts'] == NATIVE_COUNTS[index], 'changed native baseline support')
        background_cells = (whole & (expanded == 0)).reshape(256, 2, 256, 2).any(axis=(1, 3))
        mapped_bg = mapped & (ids == 0)
        probs = {}
        roi_ids = expanded[y0:y1, x0:x1]
        for name, region in [('object', roi_ids == target), ('background', roi_ids == 0)]:
            values = roi[region]
            selected = values[values > .5]
            probs[name] = {'samples': int(values.size), 'above_half': int(selected.size),
                           'equal_half': int(np.count_nonzero(values == .5)),
                           'min': float(values.min()) if values.size else None,
                           'max': float(values.max()) if values.size else None,
                           'selected_min': float(selected.min()) if selected.size else None,
                           'selected_max': float(selected.max()) if selected.size else None}
        rows.append({**det, 'diagnostic_target_id': target, 'association': association['detections'][index]['association'],
                     'native': native, 'mapped': support(mapped, ids, target), 'roi_probabilities': probs,
                     'bbox_touches_image_edge': x0 == 0 or y0 == 0 or x1 == 512 or y1 == 512,
                     'roi_foreground_edges_top_bottom_left_right': [int((edge > .5).sum()) for edge in (roi[0], roi[-1], roi[:, 0], roi[:, -1])],
                     'inverse_new_background_cells': int(np.count_nonzero(mapped_bg & ~background_cells)),
                     'inverse_background_fill_samples': int(mapped_bg.sum()*4 - np.count_nonzero(whole & (expanded == 0))),
                     'inverse_object_fill_samples': int(np.count_nonzero(mapped & (ids == target))*4 - np.count_nonzero(whole & (expanded == target)))})
    folder = Path(folder)
    pre = json.loads(gzip.decompress((folder / 'epd/preflight.json.gz').read_bytes()))
    return {'baseline': 'ad7a4655', 'mode': 'OFFLINE_REPLAY_ONLY', 'accepted': association['accepted'],
            'source_hashes': {**json.loads((folder / 'source_sha256.json').read_text()), **CONTRACT_HASHES},
            'recorded_epd_source_hashes': pre['source_sha256'],
            'manifest_hashes': MANIFEST_HASHES,
            'artifact_hashes': {g: json.loads((folder / g / 'sha256.json').read_text()) for g in ('capture', 'epd')},
            'reference': '256 same-fragment identity raster; native comparison uses exact repeated 2x2 cells',
            'classification': 'CONFIRMED_SPILL_PRESENT_BEFORE_INVERSE_MAPPING_RELATIVE_TO_RETAINED_ID_CELLS',
            'model_vs_postprocessing': 'UNKNOWN: no raw full-image ONNX outputs or floating boxes retained',
            'true_native_512_silhouette': 'UNKNOWN: only the 256 renderer identity raster was retained',
            'detections': rows}


if __name__ == '__main__':
    print(json.dumps(diagnose(), indent=2, sort_keys=True))
