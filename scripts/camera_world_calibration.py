#!/usr/bin/env python3
"""Camera-to-world calibration of a cell's live RGB-D camera from measured reference points.

The operator places reference targets (ArUco markers, or any point identified by pixel) at physically
measured positions whose world coordinates in the cell are known. ``capture`` measures each target in the
camera's colour optical frame through EPD's own sensor ingress (the same registered depth EPD localizes with),
averaged over several frames, and ``solve`` estimates the rigid transform world <- optical:

* least-squares rigid fit (Kabsch/SVD, proper rotation, no scale),
* outlier rejection by exhaustive minimal-subset consensus (deterministic, so a re-solve is repeatable),
* per-point residuals, RMS, maximum and leave-one-out prediction errors,
* an independent accuracy check: reference points with ``role: check`` never enter the fit; the fitted
  transform must predict each of them within ``max_check_error_m`` before the calibration can qualify (and
  therefore before any calibrated object can be used for simulated robot planning).

A calibration is ``qualified`` only when enough well-spread points agree within the limits below; otherwise it
is ``rejected`` with its reasons and the cell's binding is left unchanged. The record is saved with the cell
(config/camera_calibration/<camera_id>.yaml) with the camera serial, frames, transform, reference-point
evidence, residuals and provenance, and the live camera binding references it by SHA-256. Loading re-checks
the record, so an edited, mismatched or out-of-limit calibration is refused. Nothing here guesses an extrinsic.

Subcommands
  board     write a printable A4 marker sheet and its reference-points file for a stated board pose in the cell
  capture   measure the reference points live (starts EPD's ingress only), then solve and save
  solve     re-solve a saved capture (repeatable, offline)
  status    validate the cell's saved calibration against its live camera binding
"""
import argparse
import datetime
import hashlib
import itertools
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

SCHEMA = 'workcell_camera_calibration/v1'
CAPTURE_SCHEMA = 'workcell_camera_calibration_capture/v1'
POINTS_SCHEMA = 'workcell_camera_reference_points/v1'
WORLD_FRAME = 'world'
QUALIFIED, REJECTED = 'qualified', 'rejected'
# Acceptance limits. A record is re-checked against these on every load; they are never relaxed per cell.
LIMITS = dict(
    min_points=4,                 # inliers needed (3 define a transform; the 4th measures its error)
    inlier_threshold_m=0.015,     # a reference point disagreeing by more is an outlier
    max_outlier_fraction=0.25,
    rms_m=0.010,
    max_residual_m=0.020,
    max_leave_one_out_m=0.025,    # prediction error for a point the fit did not see
    min_spread_m=0.10,            # second principal extent of the world points (not collinear, not clustered)
    max_point_std_m=0.004,        # per-point repeatability across the captured frames
    min_frames_per_point=5,
    min_valid_depth_fraction=0.5,  # of the depth window around a reference pixel
    min_check_points=1,           # independent points held out of the fit
    max_check_error_m=0.015,      # each held-out point must be predicted this well
)
DEPTH_WINDOW_PX = 2               # (2w+1)^2 median window around the reference pixel


class CalibrationError(RuntimeError):
    """The calibration is missing, invalid, mismatched or outside the acceptance limits."""


# --- rigid transform estimation (pure; unit-tested) -------------------------------------------------------
def rigid_fit(camera_points, world_points):
    """Least-squares rotation R and translation t with world = R @ camera + t (Kabsch, det(R) = +1)."""
    a, b = np.asarray(camera_points, float), np.asarray(world_points, float)
    if a.shape != b.shape or a.ndim != 2 or a.shape[1] != 3 or len(a) < 3:
        raise CalibrationError('rigid_fit needs >= 3 corresponding 3D points')
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    d = np.sign(np.linalg.det(vt.T @ u.T)) or 1.0
    rotation = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return rotation, cb - rotation @ ca


def apply(transform, point):
    m = np.asarray(transform, float)
    return (m[:3, :3] @ np.asarray(point, float) + m[:3, 3]).tolist()


def matrix(rotation, translation):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = rotation, translation
    return m


def residuals(transform, camera_points, world_points):
    m = np.asarray(transform, float)
    predicted = (m[:3, :3] @ np.asarray(camera_points, float).T).T + m[:3, 3]
    return np.linalg.norm(predicted - np.asarray(world_points, float), axis=1)


def spread(world_points):
    """Second principal standard extent of the points (0 for collinear points)."""
    p = np.asarray(world_points, float)
    singular = np.linalg.svd(p - p.mean(axis=0), compute_uv=False)
    return float(singular[1] / math.sqrt(len(p))) * 2.0 if len(p) >= 3 else 0.0


def robust_fit(camera_points, world_points, threshold=LIMITS['inlier_threshold_m']):
    """Deterministic consensus: every 3-point subset proposes a fit; the largest agreeing set (then lowest RMS)
    is refitted. Returns (transform, inlier indices)."""
    a, b = np.asarray(camera_points, float), np.asarray(world_points, float)
    best = None
    for subset in itertools.combinations(range(len(a)), 3):
        if spread(b[list(subset)]) < 1e-3:
            continue
        r, t = rigid_fit(a[list(subset)], b[list(subset)])
        error = residuals(matrix(r, t), a, b)
        inliers = tuple(int(i) for i in np.flatnonzero(error <= threshold))
        if len(inliers) < 3:
            continue
        r2, t2 = rigid_fit(a[list(inliers)], b[list(inliers)])
        rms = float(np.sqrt(np.mean(residuals(matrix(r2, t2), a[list(inliers)], b[list(inliers)]) ** 2)))
        key = (-len(inliers), rms, inliers)
        if best is None or key < best[0]:
            best = (key, matrix(r2, t2), inliers)
    if best is None:
        raise CalibrationError('no three reference points define a consistent transform')
    return best[1], list(best[2])


def solve(points, limits=LIMITS):
    """Fit and qualify. ``points``: [{id, camera_xyz, world_xyz, ...}]. Returns the solution dict."""
    reasons = []
    checks = [p for p in points if p.get('role') == 'check']
    points = [p for p in points if p.get('role', 'fit') == 'fit']
    usable = [p for p in points if p.get('camera_xyz') is not None]
    missing = [p['id'] for p in points + checks if p.get('camera_xyz') is None]
    if missing:
        reasons.append(f'reference points not measured: {missing}')
    if len(usable) < limits['min_points']:
        raise CalibrationError(f"{len(usable)} measured reference points; at least {limits['min_points']} are required")
    cam = np.array([p['camera_xyz'] for p in usable], float)
    world = np.array([p['world_xyz'] for p in usable], float)
    transform, inliers = robust_fit(cam, world, limits['inlier_threshold_m'])
    outliers = [i for i in range(len(usable)) if i not in inliers]
    error = residuals(transform, cam, world)
    loo = []
    for i in inliers:
        rest = [j for j in inliers if j != i]
        if len(rest) >= 3:
            r, t = rigid_fit(cam[rest], world[rest])
            loo.append(float(residuals(matrix(r, t), cam[[i]], world[[i]])[0]))
    inlier_error = error[inliers]
    summary = dict(points=len(usable), inliers=len(inliers), outliers=[usable[i]['id'] for i in outliers],
                   rms_m=float(np.sqrt(np.mean(inlier_error ** 2))), max_residual_m=float(inlier_error.max()),
                   max_leave_one_out_m=max(loo) if loo else None, spread_m=spread(world[inliers]))
    if len(inliers) < limits['min_points']:
        reasons.append(f"only {len(inliers)} reference points agree within {limits['inlier_threshold_m'] * 1000:.0f} mm "
                       f"(need {limits['min_points']})")
    if len(outliers) > limits['max_outlier_fraction'] * len(usable):
        reasons.append(f'{len(outliers)} of {len(usable)} reference points are outliers: {summary["outliers"]}')
    if summary['rms_m'] > limits['rms_m']:
        reasons.append(f"RMS residual {summary['rms_m'] * 1000:.1f} mm exceeds {limits['rms_m'] * 1000:.0f} mm")
    if summary['max_residual_m'] > limits['max_residual_m']:
        reasons.append(f"max residual {summary['max_residual_m'] * 1000:.1f} mm exceeds {limits['max_residual_m'] * 1000:.0f} mm")
    if summary['max_leave_one_out_m'] is None or summary['max_leave_one_out_m'] > limits['max_leave_one_out_m']:
        reasons.append('leave-one-out error ' + ('unavailable' if summary['max_leave_one_out_m'] is None else
                       f"{summary['max_leave_one_out_m'] * 1000:.1f} mm exceeds {limits['max_leave_one_out_m'] * 1000:.0f} mm"))
    if summary['spread_m'] < limits['min_spread_m']:
        reasons.append(f"reference points span {summary['spread_m']:.3f} m across their second axis "
                       f"(need {limits['min_spread_m']:.2f} m; spread them out, not in a line)")
    for p in usable:
        if p.get('std_m') is not None and p['std_m'] > limits['max_point_std_m']:
            reasons.append(f"point {p['id']} varies {p['std_m'] * 1000:.1f} mm between frames "
                           f"(limit {limits['max_point_std_m'] * 1000:.0f} mm)")
    measured_checks = [p for p in checks if p.get('camera_xyz') is not None]
    check_rows = [dict(id=p['id'], error_m=float(np.linalg.norm(np.array(apply(transform, p['camera_xyz'])) -
                                                                 np.array(p['world_xyz'], float))))
                  for p in measured_checks]
    summary['independent_check'] = dict(points=check_rows, max_error_m=max((r['error_m'] for r in check_rows), default=None))
    if len(measured_checks) < limits['min_check_points']:
        reasons.append(f"{len(measured_checks)} measured independent check points (role: check); at least "
                       f"{limits['min_check_points']} must verify the fit before planning can use it")
    for row in check_rows:
        if row['error_m'] > limits['max_check_error_m']:
            reasons.append(f"independent check {row['id']} is off by {row['error_m'] * 1000:.1f} mm "
                           f"(limit {limits['max_check_error_m'] * 1000:.0f} mm)")
    per_point = [dict(id=p['id'], residual_m=float(error[i]), inlier=i in inliers) for i, p in enumerate(usable)]
    for row, value in zip([r for r in per_point if r['inlier']], loo):
        row['leave_one_out_m'] = value
    return dict(status=QUALIFIED if not reasons else REJECTED, reasons=reasons, camera_to_world=transform.tolist(),
                residuals=dict(summary, per_point=per_point), limits=dict(limits))


# --- record --------------------------------------------------------------------------------------------
def rotation_to_quaternion(m):
    m = np.asarray(m, float)[:3, :3]
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2.0, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2.0, m[1, 0] - m[0, 1])
    return [x, y, z, w]


def record_path(cell, camera_id):
    return Path(cell) / 'config' / 'camera_calibration' / f'{camera_id}.yaml'


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _git_commit():
    done = subprocess.run(['git', '-C', str(SCRIPTS), 'rev-parse', 'HEAD'], capture_output=True, text=True)
    dirty = subprocess.run(['git', '-C', str(SCRIPTS), 'status', '--porcelain', '--', '.'], capture_output=True, text=True)
    return (done.stdout.strip() + ('-dirty' if dirty.stdout.strip() else '')) if done.returncode == 0 else 'unavailable'


def build_record(solution, capture, binding):
    m = solution['camera_to_world']
    return dict(
        schema=SCHEMA, status=solution['status'], reasons=solution['reasons'],
        camera_id=binding['camera_id'], camera_serial=capture['camera']['serial'],
        source_frame=capture['camera']['optical_frame'], target_frame=WORLD_FRAME,
        camera_to_world=dict(matrix=m, translation_xyz=[row[3] for row in m[:3]],
                             quaternion_xyzw=rotation_to_quaternion(m)),
        residuals=solution['residuals'], limits=solution['limits'],
        reference_points=capture['points'],
        provenance=dict(tool='camera_world_calibration.py', emd_commit=_git_commit(),
                        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                        points_file=capture['points_file'], points_sha256=capture['points_sha256'],
                        measured_by=capture.get('measured_by'), measurement_method=capture.get('measurement_method'),
                        capture=dict(frames=capture['frames'], color_intrinsics=capture['camera']['color_intrinsics'],
                                     depth_source=capture['camera']['depth_source'],
                                     rgb_topic=capture['camera']['rgb_topic'])))


def save(cell, record, binding):
    """Write the record with the cell and point the live camera binding at it (qualified records only)."""
    from authored_yaml import write_preserving
    path = record_path(cell, record['camera_id'])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(record, sort_keys=False))
    if record['status'] != QUALIFIED:
        return path
    environment = Path(cell) / 'environment.yaml'
    doc = yaml.safe_load(environment.read_text()) or {}
    live = doc['perception']['live_camera']
    live['calibration'] = dict(status=QUALIFIED, record=str(path.relative_to(cell)), record_sha256=_sha256(path),
                               camera_serial=record['camera_serial'], camera_to_world=record['camera_to_world']['matrix'],
                               rms_m=record['residuals']['rms_m'],
                               max_leave_one_out_m=record['residuals']['max_leave_one_out_m'],
                               evidence=str(path.relative_to(cell)))
    write_preserving(environment, doc)
    return path


def load_qualified(cell, binding, limits=LIMITS):
    """The binding's qualified calibration record, re-validated; raises CalibrationError otherwise."""
    ref = (binding or {}).get('calibration') or {}
    if ref.get('status') != QUALIFIED or not ref.get('record'):
        raise CalibrationError('CALIBRATION_MISSING: no qualified camera-to-world calibration is saved for this camera')
    path = Path(cell) / ref['record']
    if not path.is_file():
        raise CalibrationError(f'CALIBRATION_MISSING: {ref["record"]} does not exist')
    if _sha256(path) != ref.get('record_sha256'):
        raise CalibrationError('CALIBRATION_INVALID: the calibration record changed after it was accepted')
    record = yaml.safe_load(path.read_text())
    if record.get('schema') != SCHEMA or record.get('status') != QUALIFIED:
        raise CalibrationError('CALIBRATION_INVALID: the record is not a qualified workcell_camera_calibration/v1')
    if record.get('camera_id') != binding.get('camera_id'):
        raise CalibrationError(f"CALIBRATION_MISMATCH: calibrated camera {record.get('camera_id')!r} is not the bound "
                               f"{binding.get('camera_id')!r}")
    if record.get('source_frame') != binding.get('optical_frame') or record.get('target_frame') != WORLD_FRAME:
        raise CalibrationError(f"CALIBRATION_MISMATCH: calibrated {record.get('source_frame')}->{record.get('target_frame')} "
                               f"is not {binding.get('optical_frame')}->{WORLD_FRAME}")
    if not record.get('camera_serial'):
        raise CalibrationError('CALIBRATION_INVALID: the record has no camera serial')
    # Re-solve from the saved evidence: the stored transform and its limits must still hold.
    resolved = solve(record['reference_points'], limits)
    if resolved['status'] != QUALIFIED:
        raise CalibrationError('CALIBRATION_ERROR_EXCESSIVE: ' + '; '.join(resolved['reasons']))
    if not np.allclose(resolved['camera_to_world'], record['camera_to_world']['matrix'], atol=1e-6):
        raise CalibrationError('CALIBRATION_INVALID: the stored transform does not follow from its reference points')
    if not np.allclose(ref.get('camera_to_world'), record['camera_to_world']['matrix'], atol=1e-9):
        raise CalibrationError('CALIBRATION_INVALID: the binding transform differs from its record')
    return record


# --- measurement ---------------------------------------------------------------------------------------
def deproject(depth_m, u, v, k, window=DEPTH_WINDOW_PX, min_valid=LIMITS['min_valid_depth_fraction']):
    """Optical-frame point at pixel (u, v) from a colour-registered depth image (metres); None if unmeasurable."""
    h, w = depth_m.shape
    iu, iv = int(round(u)), int(round(v))
    if not (window <= iu < w - window and window <= iv < h - window):
        return None
    patch = depth_m[iv - window:iv + window + 1, iu - window:iu + window + 1]
    valid = patch[np.isfinite(patch) & (patch > 0)]
    if valid.size < min_valid * patch.size:
        return None
    z = float(np.median(valid))
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    return [(u - cx) / fx * z, (v - cy) / fy * z, z]


def aruco_centres(rgb, dictionary):
    """{marker id: (u, v)} centres of the ArUco markers visible in an RGB image."""
    import cv2
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    aruco = cv2.aruco
    corners, ids, _ = aruco.detectMarkers(gray, aruco.Dictionary_get(getattr(aruco, dictionary)),
                                          parameters=aruco.DetectorParameters_create())
    if ids is None:
        return {}
    return {int(i): tuple(c.reshape(4, 2).mean(axis=0)) for i, c in zip(ids.flatten(), corners)}


def load_points(path):
    doc = yaml.safe_load(Path(path).read_text()) or {}
    if doc.get('schema') != POINTS_SCHEMA or doc.get('world_frame') != WORLD_FRAME:
        raise CalibrationError(f'POINTS_INVALID: expected schema {POINTS_SCHEMA} with world_frame {WORLD_FRAME}')
    points = doc.get('points') or []
    ids = [p.get('id') for p in points]
    if len(set(ids)) != len(ids) or not all(ids):
        raise CalibrationError('POINTS_INVALID: every reference point needs a unique id')
    for p in points:
        xyz = p.get('world_xyz')
        if not (isinstance(xyz, list) and len(xyz) == 3 and all(isinstance(v, (int, float)) and math.isfinite(v) for v in xyz)):
            raise CalibrationError(f"POINTS_INVALID: point {p.get('id')} needs a measured world_xyz [x, y, z] in metres")
        if ('aruco_id' in p) == ('pixel' in p):
            raise CalibrationError(f"POINTS_INVALID: point {p['id']} must name exactly one of aruco_id or pixel")
        if p.get('role', 'fit') not in ('fit', 'check'):
            raise CalibrationError(f"POINTS_INVALID: point {p['id']} role must be fit or check")
    if not any(p.get('role') == 'check' for p in points):
        raise CalibrationError('POINTS_INVALID: mark at least one reference point role: check; it is held out of the '
                               'fit as the independent accuracy check')
    if not doc.get('measured_by') or not doc.get('measurement_method'):
        raise CalibrationError('POINTS_INVALID: record who measured the world coordinates and how (measured_by, '
                               'measurement_method)')
    return doc


def measure(frames, points_doc):
    """Per-point optical-frame position averaged over frames. ``frames``: [(rgb, depth_m, K)]."""
    dictionary = points_doc.get('aruco_dictionary', 'DICT_4X4_50')
    samples = {p['id']: [] for p in points_doc['points']}
    pixels = {p['id']: [] for p in points_doc['points']}
    for rgb, depth_m, k in frames:
        centres = aruco_centres(rgb, dictionary) if any('aruco_id' in p for p in points_doc['points']) else {}
        for p in points_doc['points']:
            uv = centres.get(int(p['aruco_id'])) if 'aruco_id' in p else tuple(p['pixel'])
            if uv is None:
                continue
            xyz = deproject(depth_m, uv[0], uv[1], k)
            if xyz is not None:
                samples[p['id']].append(xyz)
                pixels[p['id']].append(list(uv))
    out = []
    for p in points_doc['points']:
        s = np.array(samples[p['id']], float)
        enough = len(s) >= LIMITS['min_frames_per_point']
        row = dict(id=p['id'], world_xyz=[float(v) for v in p['world_xyz']],
                   target=dict(aruco_id=p['aruco_id']) if 'aruco_id' in p else dict(pixel=p['pixel']),
                   frames_measured=len(s), camera_xyz=s.mean(axis=0).tolist() if enough else None,
                   std_m=float(np.linalg.norm(s.std(axis=0))) if enough else None,
                   pixel_mean=np.array(pixels[p['id']]).mean(axis=0).tolist() if len(s) else None)
        row['role'] = p.get('role', 'fit')
        if p.get('note'):
            row['note'] = p['note']
        out.append(row)
    return out


def _camera_serial(node, rgb_topic, timeout=5.0):
    """The RealSense serial of the camera publishing ``rgb_topic`` (device_info service), else None."""
    import rclpy
    try:
        from realsense2_camera_msgs.srv import DeviceInfo
    except ImportError:
        return None
    namespace = rgb_topic.rsplit('/', 2)[0]          # /camera/camera/color/image_raw -> /camera/camera
    client = node.create_client(DeviceInfo, f'{namespace}/device_info')
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(DeviceInfo.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result().serial_number if future.result() else None


def capture(args):
    """Measure the reference points live through EPD's ingress, then solve and save."""
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import CameraInfo, Image
    import live_perception_session as lps
    record = lps.load_runtime(args.workspace)
    binding = lps.camera_binding(args.cell)
    points_doc = load_points(args.points)
    out = Path(args.output); out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('workcell_camera_calibration')
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    if any(name == 'epd_sensor_ingress' for name, _ in node.get_node_names_and_namespaces()):
        raise CalibrationError('INGRESS_BUSY: an EPD ingress is already running (stop live perception first)')
    serial = _camera_serial(node, binding['rgb_topic'])
    if not serial:
        raise CalibrationError('CAMERA_IDENTITY_UNVERIFIED: the camera serial could not be read from its device_info '
                               'service; a calibration must name the physical camera')
    command, topics = lps.ingress_command(binding)
    proc = lps._spawn(command, lps.shell_environment(record), out / 'ingress.log')
    frames, pending = [], {}
    reliable = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)

    def offer(kind, msg):
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        pending.setdefault(key, {})[kind] = msg
        if len(pending[key]) == 3:
            frames.append(pending.pop(key))
        for stale in [k for k in pending if k < key - 2_000_000_000]:
            pending.pop(stale)

    node.create_subscription(Image, topics['rgb'], lambda m: offer('rgb', m), reliable)
    node.create_subscription(Image, topics['depth'], lambda m: offer('depth', m), reliable)
    node.create_subscription(CameraInfo, topics['info'], lambda m: offer('info', m), reliable)
    deadline = time.time() + args.timeout
    try:
        while len(frames) < args.frames and time.time() < deadline and proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        cleanup = lps._stop({'ingress': proc})
        node.destroy_node(); rclpy.shutdown()
    if len(frames) < args.frames:
        raise CalibrationError(f'CAPTURE_INCOMPLETE: {len(frames)} of {args.frames} synchronized RGB-D frames in '
                               f'{args.timeout:.0f} s (ingress {cleanup})')
    decoded = [_decode(f) for f in frames]
    info = frames[-1]['info']
    if info.header.frame_id != binding['optical_frame'] or any(f['depth'].header.frame_id != binding['optical_frame'] for f in frames):
        raise CalibrationError(f"FRAME_MISMATCH: captured frames are not in the bound {binding['optical_frame']}")
    if any(d != 0 for d in info.d):
        raise CalibrationError('DISTORTION_UNSUPPORTED: the colour CameraInfo has distortion coefficients')
    _save_snapshot(out / 'reference_view.jpg', decoded[-1][0])
    capture_doc = dict(schema=CAPTURE_SCHEMA, cell=str(Path(args.cell).resolve()),
                       points_file=str(Path(args.points).resolve()), points_sha256=_sha256(args.points),
                       measured_by=points_doc['measured_by'], measurement_method=points_doc['measurement_method'],
                       frames=dict(count=len(frames), first_stamp_ns=_stamp(frames[0]['rgb']), last_stamp_ns=_stamp(frames[-1]['rgb'])),
                       camera=dict(camera_id=binding['camera_id'], serial=serial, optical_frame=binding['optical_frame'],
                                   rgb_topic=binding['rgb_topic'], depth_source=topics['depth_source'],
                                   color_intrinsics=dict(width=info.width, height=info.height, k=list(info.k))),
                       points=measure(decoded, points_doc))
    (out / 'capture.yaml').write_text(yaml.safe_dump(capture_doc, sort_keys=False))
    return finish(args.cell, binding, capture_doc, out)


def _stamp(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _decode(frame):
    rgb_msg, depth_msg, info = frame['rgb'], frame['depth'], frame['info']
    rgb = np.frombuffer(rgb_msg.data, np.uint8).reshape(rgb_msg.height, rgb_msg.step)[:, :rgb_msg.width * 3]
    rgb = rgb.reshape(rgb_msg.height, rgb_msg.width, 3)
    if rgb_msg.encoding == 'bgr8':
        rgb = rgb[:, :, ::-1]
    if depth_msg.encoding in ('16UC1', 'mono16'):
        depth = np.frombuffer(depth_msg.data, np.uint16).reshape(depth_msg.height, depth_msg.step // 2)[:, :depth_msg.width] * 0.001
    elif depth_msg.encoding == '32FC1':
        depth = np.frombuffer(depth_msg.data, np.float32).reshape(depth_msg.height, depth_msg.step // 4)[:, :depth_msg.width]
    else:
        raise CalibrationError(f'unsupported depth encoding {depth_msg.encoding}')
    if depth.shape != rgb.shape[:2]:
        raise CalibrationError('depth is not registered to the colour image')
    return rgb, depth.astype(float), list(info.k)


def _save_snapshot(path, rgb):
    import cv2
    cv2.imwrite(str(path), cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR))


def finish(cell, binding, capture_doc, out):
    try:
        solution = solve(capture_doc['points'])
    except CalibrationError as exc:
        solution = dict(status=REJECTED, reasons=[str(exc)], camera_to_world=None, residuals=None, limits=dict(LIMITS))
    report = dict(status=solution['status'].upper(), reasons=solution['reasons'], residuals=solution['residuals'],
                  capture=str(out / 'capture.yaml'))
    if solution['camera_to_world'] is not None:
        record = build_record(solution, capture_doc, binding)
        if solution['status'] == QUALIFIED:
            report['record'] = str(save(cell, record, binding))
        else:
            # A rejected calibration never replaces the cell's accepted one; it is kept with the capture.
            (out / 'rejected_calibration.yaml').write_text(yaml.safe_dump(record, sort_keys=False))
            report['record'] = str(out / 'rejected_calibration.yaml')
        report['camera_to_world'] = record['camera_to_world']
    (out / 'result.json').write_text(json.dumps(report, indent=1))
    print(json.dumps(report))
    return 0 if solution['status'] == QUALIFIED else 3


# A4 sheet: six 50 mm DICT_4X4_50 markers. Centres in the sheet frame (origin at the sheet's top-left corner,
# x along the short edge to the right, y down the long edge), metres. Marker 5 is the held-out check.
BOARD_MARKER_SIDE_M = 0.050
BOARD_CENTRES = {0: (0.050, 0.050), 1: (0.160, 0.050), 2: (0.050, 0.155), 3: (0.160, 0.155),
                 4: (0.050, 0.260), 5: (0.160, 0.260)}
BOARD_CHECK_ID = 5


def board_points(origin_xyz, yaw_rad):
    """World positions of the sheet's marker centres for a sheet lying flat with its top-left corner at
    ``origin_xyz`` and its x edge at ``yaw_rad`` about world z (sheet y runs 90 deg clockwise from x, i.e.
    towards -y of a z-up frame rotated by yaw)."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    rows = []
    for marker, (bx, by) in BOARD_CENTRES.items():
        dx, dy = bx, -by                      # sheet y points "down" the page = -y in the sheet's z-up frame
        rows.append(dict(id=f'board_{marker}', aruco_id=marker,
                         world_xyz=[origin_xyz[0] + c * dx - s * dy, origin_xyz[1] + s * dx + c * dy, origin_xyz[2]],
                         **({'role': 'check'} if marker == BOARD_CHECK_ID else {})))
    return rows


def board(args):
    import cv2
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    aruco, px_per_m = cv2.aruco, 300 / 0.0254
    page = np.full((3508, 2480), 255, np.uint8)
    side = int(round(BOARD_MARKER_SIDE_M * px_per_m))
    for marker, (bx, by) in BOARD_CENTRES.items():
        x, y = int(round(bx * px_per_m)) - side // 2, int(round(by * px_per_m)) - side // 2
        page[y:y + side, x:x + side] = aruco.drawMarker(aruco.Dictionary_get(aruco.DICT_4X4_50), marker, side)
        cv2.putText(page, f'ID {marker}' + (' (CHECK)' if marker == BOARD_CHECK_ID else ''), (x, y + side + 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, 0, 3)
    cv2.putText(page, 'Print at 100% (A4, 300 dpi). Verify: each black square is 50.0 mm. Top-left corner = board origin.',
                (60, 3440), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3)
    cv2.imwrite(str(out / 'calibration_board_A4_300dpi.png'), page)
    doc = dict(schema=POINTS_SCHEMA, world_frame=WORLD_FRAME, aruco_dictionary='DICT_4X4_50',
               measured_by=args.measured_by, measurement_method=(
                   f'printed board (50 mm markers, print scale verified) placed flat with its top-left corner at world '
                   f'{list(args.origin)} and yaw {args.yaw_deg} deg, as authored in the cell; marker {BOARD_CHECK_ID} held out'),
               board=dict(origin_xyz=list(args.origin), yaw_deg=args.yaw_deg, marker_side_m=BOARD_MARKER_SIDE_M),
               points=board_points(args.origin, math.radians(args.yaw_deg)))
    (out / 'reference_points.yaml').write_text(yaml.safe_dump(doc, sort_keys=False))
    print(json.dumps(dict(status='WRITTEN', board=str(out / 'calibration_board_A4_300dpi.png'),
                          points=str(out / 'reference_points.yaml'))))
    return 0


def main(argv=None):
    import live_perception_session as lps
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    c = sub.add_parser('capture')
    c.add_argument('--cell', required=True); c.add_argument('--points', required=True)
    c.add_argument('--output', required=True); c.add_argument('--frames', type=int, default=15)
    c.add_argument('--timeout', type=float, default=30.0)
    c.add_argument('--workspace', type=Path, default=lps.WORKSPACE)
    s = sub.add_parser('solve')
    s.add_argument('--cell', required=True); s.add_argument('--capture', required=True)
    t = sub.add_parser('status'); t.add_argument('--cell', required=True)
    b = sub.add_parser('board')
    b.add_argument('--output', required=True)
    b.add_argument('--origin', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'),
                   help="world position of the sheet's top-left corner as authored in the cell (table top z)")
    b.add_argument('--yaw-deg', type=float, default=0.0)
    b.add_argument('--measured-by', required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == 'board':
            return board(args)
        if args.command == 'capture':
            return capture(args)
        binding = lps.camera_binding(args.cell)
        if args.command == 'solve':
            capture_doc = yaml.safe_load(Path(args.capture).read_text())
            if capture_doc.get('schema') != CAPTURE_SCHEMA or capture_doc['camera']['camera_id'] != binding['camera_id']:
                raise CalibrationError('CAPTURE_INVALID: not a capture of the bound camera')
            return finish(args.cell, binding, capture_doc, Path(args.capture).parent)
        record = load_qualified(args.cell, binding)
        print(json.dumps(dict(status='QUALIFIED', camera_serial=record['camera_serial'],
                              residuals={k: v for k, v in record['residuals'].items() if k != 'per_point'})))
        return 0
    except (CalibrationError, lps.PerceptionUnavailable, OSError, KeyError, ValueError) as exc:
        print(json.dumps(dict(status='BLOCKED', reason=str(exc))))
        return 2


if __name__ == '__main__':
    sys.exit(main())
