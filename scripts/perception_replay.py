#!/usr/bin/env python3
"""Stage-B camera-free perception: record a saved cell's simulated RGB-D camera, replay it through EPD, bind it.

``record``  renders the saved cell's own camera in a render-only Gazebo process built from the same derived
            physics world (static models at their authored rest poses, each object with its authored visual
            mesh). Writes an EPD replay fixture plus ``recording.json``. Provenance is always
            SIMULATOR_RENDERED_SYNTHETIC: never a real-camera recording. The qualified physics world is untouched.
``detect``  replays a recording deterministically through EPD's production node (tracking use case, hybrid
            RGB-D refinement) and its Workcell contract bridge in replay mode, and writes the result as a
            standard ``detected_objects/v1`` world snapshot (source.mode=replayed_snapshot) for Task settings.

Replay-time semantics: every age is measured on the recording's own clock (simulation time), never against
the host wall clock. A replay observation's age is the recording's last frame stamp minus the stamp of the
frame it was computed from. Live snapshots (runtime_mode live) are never accepted as replays.

Calibration: camera-frame detections become world observations only through an explicit camera-to-world
record. A simulator-rendered recording carries ``simulation_exact`` calibration (the pose it was rendered at),
valid only while it equals the cell's current authored camera pose. Anything else is BLOCKED.

Physical binding (``associate``): in the physics simulation each EPD world observation must fall inside exactly
one physically measured object; that object then takes EPD's class and confidence. Unmatched objects stay
``unperceived`` and can never be targets; a target-class detection with no physical object blocks the cycle.
"""
import argparse
import copy
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

RECORDING_SCHEMA = 'workcell_perception_recording/v1'
PROVENANCE = 'SIMULATOR_RENDERED_SYNTHETIC'
PRODUCER = 'epd_replay'
CLOCK_DOMAIN = 'simulation_time'
SIMULATION_EXACT = 'simulation_exact'
UNPERCEIVED = 'unperceived'
# D435/D435i colour stream nominal horizontal field of view (Intel datasheet: 69 deg x 42 deg at 16:9).
D435_COLOR_HFOV_RAD = math.radians(69.0)
ASSOCIATION_MARGIN_M = 0.02          # an EPD world centroid may sit this far outside the physical box
POSE_TOLERANCE = 1e-9
# ROS optical frame (x right, y down, z forward) expressed in the camera link frame (x forward, y left, z up).
OPTICAL_IN_LINK = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))


class PerceptionBlocked(ValueError):
    """A replayed perception input that may not drive selection, with an actionable code."""


def _sha256(data):
    return hashlib.sha256(data if isinstance(data, bytes) else data.encode()).hexdigest()


def _file_sha256(path):
    return _sha256(Path(path).read_bytes())


def _rot(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def _apply(m, v):
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def optical_to_world(camera_link_pose, point_optical):
    """World point of an optical-frame point, given the camera link pose [x, y, z, roll, pitch, yaw] in world."""
    link = _apply(OPTICAL_IN_LINK, point_optical)
    rotated = _apply(_rot(camera_link_pose[3:6]), link)
    return [camera_link_pose[i] + rotated[i] for i in range(3)]


# --- the saved cell ---------------------------------------------------------------------------------------
def load_cell(scene):
    scene = Path(scene)
    document = yaml.safe_load((scene / 'environment.yaml').read_text()) or {}
    return scene, document


def cell_camera(document):
    """The cell's authored camera: id, optical frame and camera link pose in the world frame."""
    camera = document.get('camera') or (document.get('environment') or {}).get('camera') or {}
    pose = camera.get('pose')
    if not camera.get('enabled', True) or not camera.get('camera_id') or not isinstance(pose, list) or len(pose) != 6:
        raise PerceptionBlocked('CAMERA_MISSING: the cell has no enabled authored camera with a 6-DoF pose')
    if camera.get('parent_frame', 'world') != 'world':
        raise PerceptionBlocked('CAMERA_FRAME_UNSUPPORTED: the camera pose must be authored in the world frame')
    stream = camera.get('acceptance_stream') or {}
    return dict(camera_id=camera['camera_id'], frame_id=camera.get('frame_id', 'camera_color_optical_frame'),
                link_pose=[float(v) for v in pose], width=int(stream.get('color_width', 640)),
                height=int(stream.get('color_height', 480)))


def authored_objects(document):
    source = ((document.get('workcell_studio') or {}).get('pick_zone') or {}).get('object_source') or {}
    rows = source.get('manual_observations', [source.get('manual_observation')] if source.get('manual_observation') else [])
    return {row['object_id']: row for row in rows if isinstance(row, dict) and row.get('object_id')}


def calibration_check(calibration, camera):
    """Validate a recording's camera-to-world record against the cell's current camera; return it or raise."""
    if not isinstance(calibration, dict) or not calibration:
        raise PerceptionBlocked('CALIBRATION_MISSING: the recording has no camera-to-world calibration record')
    if calibration.get('status') != SIMULATION_EXACT:
        raise PerceptionBlocked(f"CALIBRATION_INVALID: status {calibration.get('status')!r}; only a simulator-rendered "
                                'recording is calibrated (simulation_exact). A real camera needs a qualified extrinsic '
                                'calibration, which this cell does not have.')
    if calibration.get('camera_id') != camera['camera_id']:
        raise PerceptionBlocked(f"CALIBRATION_INVALID: recorded camera {calibration.get('camera_id')!r} is not the cell "
                                f"camera {camera['camera_id']!r}")
    pose = calibration.get('camera_link_pose_world')
    if (not isinstance(pose, list) or len(pose) != 6 or
            any(abs(float(a) - b) > POSE_TOLERANCE for a, b in zip(pose, camera['link_pose']))):
        raise PerceptionBlocked('CALIBRATION_INVALID: the cell camera moved since the recording '
                                f'(recorded {pose}, authored {camera["link_pose"]}); record the session again')
    if calibration.get('optical_in_link') != [list(row) for row in OPTICAL_IN_LINK]:
        raise PerceptionBlocked('CALIBRATION_INVALID: unknown optical-frame convention')
    return calibration


# --- record -----------------------------------------------------------------------------------------------
def _visual_mesh(row, scene):
    """The authored visual of a physical test object: mesh path (cell-relative), scale and offset from the box centre."""
    visual = row.get('visual')
    if not visual:
        return None
    path = Path(visual['mesh'])
    path = path if path.is_absolute() else Path(scene) / path
    if not path.is_file():
        raise PerceptionBlocked(f"VISUAL_MISSING: {row['object_id']} visual mesh {path} is missing")
    return dict(path=path, scale=[float(v) for v in visual.get('scale', [1, 1, 1])],
                offset=[float(v) for v in visual.get('offset_xyz', [0, 0, 0])] + [float(v) for v in visual.get('offset_rpy', [0, 0, 0])],
                sha256=_file_sha256(path), license=visual.get('license'), source=visual.get('source'))


def render_world(scene, world_text, world_report, camera, hfov, rate_hz):
    """The render-only world: the derived physics world's models made static at authored rest poses,
    authored visual meshes, a Sensors system and the cell camera. Returns (sdf text, visuals)."""
    document = load_cell(scene)[1]
    rows = authored_objects(document)
    root = ET.fromstring(world_text)
    world = root.find('world')
    plugin = ET.SubElement(world, 'plugin', filename='ignition-gazebo-sensors-system',
                           name='ignition::gazebo::systems::Sensors')
    ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    by_name = {o['object_id']: o for o in world_report['objects']}
    visuals = {}
    for model in world.findall('model'):
        static = model.find('static')
        if static is None:
            static = ET.SubElement(model, 'static')
        static.text = 'true'
        obj = by_name.get(model.get('name'))
        if obj is None:
            continue
        model.find('pose').text = ' '.join(repr(float(v)) for v in obj['authored_pose'])
        mesh = _visual_mesh(rows.get(obj['object_id'], {}), scene)
        if mesh is None:
            continue
        link = model.find('link')
        link.remove(link.find('visual'))
        visual = ET.SubElement(link, 'visual', name='visual')
        ET.SubElement(visual, 'pose').text = ' '.join(repr(v) for v in mesh['offset'])
        geometry = ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh')
        ET.SubElement(geometry, 'uri').text = 'file://' + str(mesh['path'])
        ET.SubElement(geometry, 'scale').text = ' '.join(repr(v) for v in mesh['scale'])
        visuals[obj['object_id']] = dict(mesh=str(mesh['path']), mesh_sha256=mesh['sha256'], scale=mesh['scale'],
                                         offset=mesh['offset'], license=mesh['license'], source=mesh['source'])
    cam = ET.SubElement(world, 'model', name='workcell_render_camera')
    ET.SubElement(cam, 'static').text = 'true'
    ET.SubElement(cam, 'pose').text = ' '.join(repr(v) for v in camera['link_pose'])
    sensor = ET.SubElement(ET.SubElement(cam, 'link', name='camera_link'), 'sensor', name='rgbd', type='rgbd_camera')
    ET.SubElement(sensor, 'update_rate').text = repr(float(rate_hz))
    ET.SubElement(sensor, 'topic').text = 'workcell_render/rgbd'
    optics = ET.SubElement(sensor, 'camera')
    ET.SubElement(optics, 'horizontal_fov').text = repr(float(hfov))
    image = ET.SubElement(optics, 'image')
    ET.SubElement(image, 'width').text = str(camera['width'])
    ET.SubElement(image, 'height').text = str(camera['height'])
    clip = ET.SubElement(optics, 'clip')
    ET.SubElement(clip, 'near').text = '0.05'
    ET.SubElement(clip, 'far').text = '6.0'
    return ET.tostring(root, encoding='unicode'), visuals


def _stop(proc, timeout=10):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(wait)
            return
        except subprocess.TimeoutExpired:
            continue


def _capture_frames(count, warmup, timeout):
    """(rgb, depth_mm, camera_info, stamp_ns) frames from the bridged render camera, paired by stamp."""
    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    rclpy.init()
    node = rclpy.create_node('workcell_render_recorder')
    bridge, rgb, depth, info = CvBridge(), {}, {}, {}
    stamp = lambda h: h.stamp.sec * 1_000_000_000 + h.stamp.nanosec
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Image, '/workcell_render/rgbd/image', lambda m: rgb.setdefault(stamp(m.header), m), qos)
    node.create_subscription(Image, '/workcell_render/rgbd/depth_image', lambda m: depth.setdefault(stamp(m.header), m), qos)
    node.create_subscription(CameraInfo, '/workcell_render/rgbd/camera_info', lambda m: info.setdefault(stamp(m.header), m), qos)
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            paired = sorted(set(rgb) & set(depth) & set(info))
            if len(paired) >= count + warmup:
                break
        else:
            raise RuntimeError(f'RENDER_UNAVAILABLE: {len(set(rgb) & set(depth) & set(info))} paired RGB-D frames '
                               f'rendered in {timeout:.0f} s (need {count + warmup}); is headless rendering available?')
        frames = []
        for key in paired[warmup:warmup + count]:
            colour = bridge.imgmsg_to_cv2(rgb[key], 'bgr8')
            metres = bridge.imgmsg_to_cv2(depth[key])
            millimetres = np.where(np.isfinite(metres) & (metres > 0), metres * 1000.0, 0).round().clip(0, 65535).astype(np.uint16)
            frames.append((colour, millimetres, info[key], key))
        return frames
    finally:
        node.destroy_node()
        rclpy.shutdown()


def record(scene, output, frames=6, warmup=3, hfov=D435_COLOR_HFOV_RAD, rate_hz=5.0, timeout=90.0):
    import cv2
    from simulator_world import derive_world
    scene, document = load_cell(scene)
    output = Path(output)
    camera = cell_camera(document)
    world_text, world_report = derive_world(scene)          # SimulatorWorldError (a ValueError) blocks first
    text, visuals = render_world(scene, world_text, world_report, camera, hfov, rate_hz)
    output.mkdir(parents=True, exist_ok=False)
    (output / 'render_world.sdf').write_text(text)
    env = dict(os.environ, IGN_PARTITION=f'workcell_render_{os.getpid()}', ROS_LOCALHOST_ONLY='1',
               ROS_DOMAIN_ID=str(170 + os.getpid() % 50))
    os.environ.update(ROS_DOMAIN_ID=env['ROS_DOMAIN_ID'], ROS_LOCALHOST_ONLY='1')
    log = (output / 'render.log').open('w')
    server = subprocess.Popen(['ign', 'gazebo', '-s', '-r', '--headless-rendering', str(output / 'render_world.sdf')],
                              env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    topics = [f'/workcell_render/rgbd/{t}@sensor_msgs/msg/{m}[ignition.msgs.{m}' for t, m in (
        ('image', 'Image'), ('depth_image', 'Image'), ('camera_info', 'CameraInfo'))]   # one-way: Gazebo -> ROS
    bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', *topics],
                              env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        captured = _capture_frames(frames, warmup, timeout)
    finally:
        _stop(bridge)
        _stop(server)
        log.close()
    observations, rows = [], []
    first_info = captured[0][2]
    for index, (colour, millimetres, info, stamp_ns) in enumerate(captured):
        rgb_name, depth_name = f'rgb_{index:02d}.png', f'depth_{index:02d}.png'
        cv2.imwrite(str(output / rgb_name), colour)
        cv2.imwrite(str(output / depth_name), millimetres)
        observations.append(dict(stamp_ns=int(stamp_ns), rgb=rgb_name, depth=depth_name))
        rows.append(dict(stamp_ns=int(stamp_ns), rgb_sha256=_file_sha256(output / rgb_name),
                         depth_sha256=_file_sha256(output / depth_name),
                         valid_depth_ratio=float((millimetres > 0).mean())))
    k = [float(v) for v in first_info.k]
    fixture = dict(schema_version=1, name=f'{scene.name}-{PROVENANCE.lower()}',
                   camera=dict(width=int(first_info.width), height=int(first_info.height), frame_id=camera['frame_id'],
                               camera_id=camera['camera_id'], k=k),
                   inference_timeout_s=120.0, observations=observations,
                   acceptance=dict(minimum_completed_results=1, minimum_stable_ids=0, minimum_tracks_lost=0,
                                   expected_lost_track_ids=[]))
    (output / 'fixture.json').write_text(json.dumps(fixture, indent=1) + '\n')
    manifest = dict(schema=RECORDING_SCHEMA, provenance=PROVENANCE, clock_domain=CLOCK_DOMAIN,
                    note='Rendered from the simulated cell; not a real-camera recording. The robot is not rendered.',
                    cell=str(scene), environment_sha256=_file_sha256(scene / 'environment.yaml'),
                    physics_world_sha256=world_report['world_sha256'], render_world_sha256=_sha256(text),
                    camera=dict(camera, horizontal_fov_rad=hfov, k=k, distortion=[float(v) for v in first_info.d]),
                    calibration=dict(status=SIMULATION_EXACT, camera_id=camera['camera_id'],
                                     camera_link_pose_world=camera['link_pose'],
                                     optical_in_link=[list(row) for row in OPTICAL_IN_LINK],
                                     source='the pose this session was rendered at (the cell camera)'),
                    objects={o['object_id']: dict(dimensions_m=o['dimensions_m'], authored_pose=o['authored_pose'],
                                                  visual=visuals.get(o['object_id'])) for o in world_report['objects']},
                    fixture='fixture.json', fixture_sha256=_file_sha256(output / 'fixture.json'), frames=rows,
                    recorded_wall_ns=time.time_ns())
    (output / 'recording.json').write_text(json.dumps(manifest, indent=1) + '\n')
    return manifest


# --- detect (EPD replay) ----------------------------------------------------------------------------------
def load_recording(path):
    path = Path(path)
    manifest_path = path / 'recording.json' if path.is_dir() else path
    if not manifest_path.is_file():
        raise PerceptionBlocked(f'RECORDING_MISSING: {manifest_path}')
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('schema') != RECORDING_SCHEMA:
        raise PerceptionBlocked(f"RECORDING_INVALID: schema {manifest.get('schema')!r}")
    if manifest.get('provenance') != PROVENANCE or manifest.get('clock_domain') != CLOCK_DOMAIN:
        raise PerceptionBlocked('RECORDING_INVALID: unknown provenance or clock domain')
    fixture = manifest_path.parent / manifest['fixture']
    if not fixture.is_file() or _file_sha256(fixture) != manifest.get('fixture_sha256'):
        raise PerceptionBlocked('RECORDING_INVALID: the replay fixture is missing or changed after recording')
    for row in manifest['frames']:
        for key, name in (('rgb_sha256', 'rgb'), ('depth_sha256', 'depth')):
            frame = next(o for o in json.loads(fixture.read_text())['observations'] if o['stamp_ns'] == row['stamp_ns'])
            if _file_sha256(manifest_path.parent / frame[name]) != row[key]:
                raise PerceptionBlocked(f"RECORDING_INVALID: frame {row['stamp_ns']} {name} changed after recording")
    return manifest_path, manifest


def _perception_runtime(workspace):
    record_path = Path(workspace) / 'workcell_perception_runtime.json'
    if not record_path.is_file():
        raise PerceptionBlocked(f'PERCEPTION_RUNTIME_UNREGISTERED: {record_path}; register EPD with '
                                'live_perception_session.py register')
    return json.loads(record_path.read_text())


def run_epd_replay(manifest_path, manifest, workspace, evidence, timeout=240.0):
    """Replay the recording through EPD's production node + contract bridge; returns the normalized snapshots."""
    runtime = _perception_runtime(workspace)
    evidence = Path(evidence)
    if evidence.exists():
        # Snapshots are read back from this directory: an earlier run's results must never mix into this one.
        raise PerceptionBlocked(f'EPD_EVIDENCE_EXISTS: {evidence} holds an earlier replay; detect into a new output')
    evidence.mkdir(parents=True)
    fixture = manifest_path.parent / manifest['fixture']
    domain = str(120 + os.getpid() % 40)
    prefix = (f"source {runtime['ros_setup']} && source {runtime['epd_setup']} && export ROS_DOMAIN_ID={domain} "
              f"ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION={runtime['rmw_implementation']} "
              f"EPD_EXECUTION_BACKEND={runtime['epd_execution_backend']} && ")
    share, camera_id = runtime['epd']['share'], manifest['camera']['camera_id']
    topics = ('-p rgb_topic:=/easy_perception_deployment/ingress/color/image_raw '
              '-p depth_topic:=/easy_perception_deployment/ingress/aligned_depth/image_raw '
              '-p camera_info_topic:=/easy_perception_deployment/ingress/color/camera_info')
    commands = dict(
        epd=(f'cd {share} && exec ros2 run easy_perception_deployment easy_perception_deployment --ros-args '
             f'-p use_depth:=true -p camera_id:={camera_id} {topics} -p rgb_input_watchdog_timeout_s:=0.0 '
             '-p usecase_mode_override:=4 -p tracker_type_override:=MEDIANFLOW -p hybrid_refinement_enabled:=true '
             '-r /easy_perception_deployment/image_input:=/easy_perception_deployment/ingress/color/image_raw'),
        bridge=(f"exec python3 {runtime['epd']['bridge']} --scene-id {Path(manifest['cell']).name} --camera-id {camera_id} "
                '--runtime-mode replay --source-mode tracking --require-tracking-ids'),
        replay=(f'exec ros2 run easy_perception_deployment epd_replay.py --ros-args -p fixture:={fixture} '
                f"-p mode:=fast -p summary_output:={evidence / 'epd_replay_summary.json'}"))
    (evidence / 'epd_commands.json').write_text(json.dumps(dict(commands, ros_domain_id=domain), indent=1) + '\n')
    procs, logs = {}, {}
    snapshots = []
    os.environ.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1')
    try:
        for name in ('epd', 'bridge'):
            logs[name] = (evidence / f'{name}.log').open('w')
            procs[name] = subprocess.Popen(['bash', '-c', prefix + commands[name]], stdout=logs[name],
                                           stderr=subprocess.STDOUT, start_new_session=True)
        listener = subprocess.Popen(
            [sys.executable, str(SCRIPT_DIR / 'perception_replay.py'), '_listen', str(evidence / 'epd_snapshots.jsonl')],
            env=dict(os.environ), stdout=subprocess.DEVNULL, stderr=(evidence / 'listener.log').open('w'),
            start_new_session=True)
        procs['listener'] = listener
        logs['replay'] = (evidence / 'replay.log').open('w')
        procs['replay'] = subprocess.Popen(['bash', '-c', prefix + commands['replay']], stdout=logs['replay'],
                                           stderr=subprocess.STDOUT, start_new_session=True)
        try:
            procs['replay'].wait(timeout)
        except subprocess.TimeoutExpired:
            raise RuntimeError('EPD_REPLAY_TIMEOUT: the replay did not reach end of file')
        time.sleep(1.0)                        # the bridge publishes the last result after the final frame
    finally:
        for name in ('replay', 'listener', 'bridge', 'epd'):
            _stop(procs.get(name))
        for handle in logs.values():
            handle.close()
    summary_path = evidence / 'epd_replay_summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.is_file() else {}
    if procs['replay'].returncode != 0 or summary.get('result') != 'PASS':
        raise PerceptionBlocked(f"EPD_REPLAY_FAILED: replay exit {procs['replay'].returncode}, "
                                f"result {summary.get('result')!r}, failures {summary.get('failures')}")
    path = evidence / 'epd_snapshots.jsonl'
    if path.is_file():
        snapshots = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    return snapshots, summary


def _listen(output):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('workcell_perception_replay_listener')
    with open(output, 'x') as stream:              # this run's results only
        def write(msg):
            stream.write(msg.data.replace('\n', ' ') + '\n')
            stream.flush()
        node.create_subscription(String, '/workcell_studio/epd_detection_snapshot_json', write,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, Exception):
            pass


def recording_reference(manifest_path, scene):
    """The recording as the cell references it: cell-relative when it lives inside the cell (portable)."""
    manifest_path, scene = Path(manifest_path).resolve(), Path(scene).resolve()
    return str(manifest_path.relative_to(scene)) if scene in manifest_path.parents else str(manifest_path)


def resolve_recording(reference, scene):
    path = Path(reference or '')
    return path if path.is_absolute() else Path(scene) / path


def replay_snapshot_from_epd(snapshots, manifest, manifest_path, camera, summary=None, scene=None):
    """detected_objects/v1 world snapshot (source.mode=replayed_snapshot) from EPD's replay output."""
    if not snapshots:
        raise PerceptionBlocked('OBSERVATION_UNAVAILABLE: EPD produced no normalized snapshot from the recording')
    frames = [row['stamp_ns'] for row in manifest['frames']]
    latest = max(snapshots, key=lambda s: s.get('timestamp_ns', -1))
    if latest.get('runtime_mode') != 'replay':
        raise PerceptionBlocked(f"LIVE_INPUT_NOT_REPLAYABLE: EPD runtime_mode {latest.get('runtime_mode')!r}")
    if latest.get('timestamp_ns') not in frames:
        raise PerceptionBlocked(f"CLOCK_DOMAIN_MISMATCH: EPD result stamp {latest.get('timestamp_ns')} is not a frame of "
                                'the recording (simulation time)')
    calibration = calibration_check(manifest['calibration'], camera)
    if latest.get('camera_id') != camera['camera_id'] or latest.get('frame_id') != camera['frame_id']:
        raise PerceptionBlocked(f"CAMERA_MISMATCH: EPD result from {latest.get('camera_id')}/{latest.get('frame_id')}")
    age = (max(frames) - latest['timestamp_ns']) / 1e9
    objects = []
    for item in latest.get('objects') or []:
        position = (item.get('pose') or {}).get('position')
        if item.get('pose', {}).get('frame_id') != camera['frame_id'] or not isinstance(position, list) or len(position) != 3:
            continue
        world = optical_to_world(calibration['camera_link_pose_world'], [float(v) for v in position])
        extent = [float(v) for v in item.get('dimensions_xyz') or [0, 0, 0]]
        objects.append(dict(object_id=str(item.get('object_id') or item.get('track_id') or f"epd_{len(objects)}").replace(':', '_'),
                            class_id=item.get('label'), confidence=item.get('confidence'), age_seconds=age,
                            pose=dict(frame_id='world', xyz=world, rpy=[0.0, 0.0, 0.0]),
                            dimensions=[max(v, 1e-3) for v in extent],
                            perception=dict(producer=PRODUCER, camera_frame_position=position,
                                            camera_frame_visible_extent=extent, source_stamp_ns=latest['timestamp_ns'],
                                            replay_age_s=age, quality=(item.get('attributes') or {}).get('quality'),
                                            valid_depth_ratio=(item.get('attributes') or {}).get('valid_depth_ratio'))))
    rejected = [dict(class_id=r.get('label'), confidence=r.get('confidence'), reasons=r.get('failure_reasons'))
                for r in latest.get('rejected_objects') or []]
    return dict(schema_version='detected_objects/v1',
                source=dict(mode='replayed_snapshot', producer=PRODUCER, provenance=PROVENANCE, camera_source='simulated_rgbd',
                            clock_domain=CLOCK_DOMAIN, recording=recording_reference(manifest_path, scene or manifest['cell']),
                            recording_sha256=_file_sha256(manifest_path),
                            physics_world_sha256=manifest['physics_world_sha256'], calibration=calibration,
                            epd=dict(runtime_mode=latest['runtime_mode'], snapshot_timestamp_ns=latest['timestamp_ns'],
                                     snapshots=len(snapshots), inference=(summary or {}).get('performance')),
                            geometry='epd_visible_extent: planning geometry comes only from a physical binding',
                            rejected_detections=rejected),
                objects=objects)


def detect(recording, output, workspace, scene=None):
    manifest_path, manifest = load_recording(recording)
    scene = Path(scene or manifest['cell'])
    camera = cell_camera(load_cell(scene)[1])
    output = Path(output)
    snapshots, summary = run_epd_replay(manifest_path, manifest, workspace, output.parent / f'{output.stem}_epd')
    result = replay_snapshot_from_epd(snapshots, manifest, manifest_path, camera, summary, scene)
    output.write_text(yaml.safe_dump(result, sort_keys=False))
    return result


# --- validation and physical binding ----------------------------------------------------------------------
def validate_replay(snapshot, scene, *, physics_world_sha256=None, max_age_s=None):
    """Raise PerceptionBlocked unless a replayed EPD snapshot may drive selection in this cell."""
    from task_intent_v2 import OBSERVATION_PRODUCER_CEILING_S
    source = snapshot.get('source') if isinstance(snapshot, dict) else None
    if not isinstance(source, dict) or snapshot.get('schema_version') != 'detected_objects/v1':
        raise PerceptionBlocked('REPLAY_INPUT_INVALID: not a detected_objects/v1 snapshot')
    if source.get('mode') != 'replayed_snapshot':
        raise PerceptionBlocked('LIVE_INPUT_NOT_REPLAYABLE: the input is not a replayed session (stale live data is never '
                                'reinterpreted as a replay)')
    if source.get('producer') != PRODUCER or source.get('provenance') != PROVENANCE:
        raise PerceptionBlocked(f"REPLAY_INPUT_INVALID: producer {source.get('producer')!r}, provenance "
                                f"{source.get('provenance')!r}")
    if source.get('clock_domain') != CLOCK_DOMAIN or (source.get('epd') or {}).get('runtime_mode') != 'replay':
        raise PerceptionBlocked('CLOCK_DOMAIN_MISMATCH: replay observations must be stamped in the recording clock')
    manifest_path, manifest = load_recording(resolve_recording(source.get('recording'), scene))
    if _file_sha256(manifest_path) != source.get('recording_sha256'):
        raise PerceptionBlocked('RECORDING_INVALID: the recording changed after detection')
    if source.get('epd', {}).get('snapshot_timestamp_ns') not in [row['stamp_ns'] for row in manifest['frames']]:
        raise PerceptionBlocked('CLOCK_DOMAIN_MISMATCH: the EPD result stamp is not a frame of the recording')
    camera = cell_camera(load_cell(scene)[1])
    calibration_check(source.get('calibration'), camera)
    calibration_check(manifest.get('calibration'), camera)
    if physics_world_sha256 is not None and manifest['physics_world_sha256'] != physics_world_sha256:
        raise PerceptionBlocked('RECORDING_WORLD_MISMATCH: the recording was rendered from a different physics world; '
                                'record the session again')
    ceiling = min(OBSERVATION_PRODUCER_CEILING_S['replay'], max_age_s if max_age_s else math.inf)
    for item in snapshot.get('objects') or []:
        age = (item.get('perception') or {}).get('replay_age_s')
        if not isinstance(age, (int, float)) or age < 0 or item.get('age_seconds') != age:
            raise PerceptionBlocked(f"CLOCK_DOMAIN_MISMATCH: {item.get('object_id')} has no replay-clock age")
        if age > ceiling:
            raise PerceptionBlocked(f"PERCEPTION_STALE: {item.get('object_id')} is {age:.2f} s old in the recording clock "
                                    f'(limit {ceiling:.1f} s)')
    if not snapshot.get('objects'):
        raise PerceptionBlocked('OBSERVATION_UNAVAILABLE: EPD localized no object in the recording')
    return manifest


def _outside_distance(point, pose, size):
    """Distance from a world point to a yawed box (0 inside)."""
    yaw = pose[5]
    dx, dy, dz = (point[i] - pose[i] for i in range(3))
    local = (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy, dz)
    return math.sqrt(sum(max(abs(v) - s / 2, 0.0) ** 2 for v, s in zip(local, size)))


def associate(measured, perceived, target_class=None, margin=ASSOCIATION_MARGIN_M):
    """Bind EPD world observations to measured physical objects.

    measured: {model: (pose [x,y,z,r,p,y], size [x,y,z])}; perceived: detected_objects/v1 replay objects.
    Returns {model: binding} and the unbound detections. A detection binds to the one box that contains its world
    centroid; failing that, to the one box within ``margin`` of it; anything else is unbound (none or ambiguous).
    A model takes a class only from exactly one detection; otherwise it is ``unperceived``.
    """
    hits = {name: [] for name in measured}
    unbound = []
    for item in perceived:
        point = item['pose']['xyz']
        distance = {name: _outside_distance(point, pose, size) for name, (pose, size) in measured.items()}
        inside = [name for name, d in distance.items() if d == 0.0]
        near = [name for name, d in distance.items() if d <= margin]
        chosen = inside if len(inside) == 1 else near if not inside and len(near) == 1 else None
        if chosen:
            hits[chosen[0]].append(item)
        else:
            unbound.append(dict(object_id=item['object_id'], class_id=item['class_id'], world_xyz=point,
                                reason='no physical object at the localized position' if not near else
                                f'ambiguous: within {margin} m of {sorted(near)}'))
    bindings = {}
    for name, items in hits.items():
        pose = measured[name][0]
        if len(items) == 1:
            item = items[0]
            bindings[name] = dict(class_id=item['class_id'], confidence=item.get('confidence'), detection=item['object_id'],
                                  world_xyz=item['pose']['xyz'],
                                  association_distance_m=math.dist(item['pose']['xyz'], pose[:3]),
                                  replay_age_s=item['perception']['replay_age_s'])
        else:
            bindings[name] = dict(class_id=UNPERCEIVED, confidence=None,
                                  reason='no EPD detection' if not items else
                                  f"conflicting detections {[i['object_id'] for i in items]}")
    blocking = [u for u in unbound if target_class and u['class_id'] == target_class]
    if blocking:
        raise PerceptionBlocked(f'PERCEPTION_WORLD_MISMATCH: EPD localized {target_class} where the simulated world has no '
                                f'single object ({blocking}); check calibration or record the session again')
    return bindings, unbound


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)
    r = sub.add_parser('record', help='render the cell camera into an EPD replay recording')
    r.add_argument('--cell', required=True)
    r.add_argument('--output', required=True)
    r.add_argument('--frames', type=int, default=6)
    d = sub.add_parser('detect', help='replay a recording through EPD into a detected_objects/v1 replay snapshot')
    d.add_argument('--recording', required=True)
    d.add_argument('--output', required=True)
    d.add_argument('--workspace', default=str(SCRIPT_DIR.parents[2]))
    d.add_argument('--cell')
    v = sub.add_parser('validate', help='may this replay snapshot drive selection in the cell?')
    v.add_argument('--cell', required=True)
    v.add_argument('snapshot')
    sub.add_parser('_listen').add_argument('output')
    args = parser.parse_args(argv)
    try:
        if args.command == '_listen':
            _listen(args.output)
            return 0
        if args.command == 'record':
            result = record(args.cell, args.output, args.frames)
            print(json.dumps(dict(status='RECORDED', recording=str(Path(args.output) / 'recording.json'),
                                  frames=len(result['frames']), provenance=PROVENANCE)))
        elif args.command == 'detect':
            result = detect(args.recording, args.output, args.workspace, args.cell)
            print(json.dumps(dict(status='DETECTED', output=args.output,
                                  objects=[(o['class_id'], round(o['confidence'] or 0, 3)) for o in result['objects']],
                                  rejected=result['source']['rejected_detections'])))
        else:
            snapshot = yaml.safe_load(Path(args.snapshot).read_text())
            validate_replay(snapshot, args.cell)
            print(json.dumps(dict(status='VALID')))
        return 0
    except (PerceptionBlocked, RuntimeError, OSError, KeyError, ValueError) as exc:
        print(json.dumps(dict(status='BLOCKED', reason=str(exc))))
        return 2


if __name__ == '__main__':
    sys.exit(main())
