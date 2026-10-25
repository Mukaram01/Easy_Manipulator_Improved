#!/usr/bin/env python3
"""Live camera -> EPD perception session for the Workcell Builder (perception only; never robot motion).

Subcommands
  register  record the workspace's perception runtime (EPD install, ROS domain/middleware) once, validated
  env       print the shell environment of that runtime (fails closed when it no longer validates)
  discover  list RGB-D cameras actually publishing in that ROS graph (CameraInfo + sibling image topics)
  bind      save a discovered camera binding into a cell's environment.yaml (perception.live_camera)
  run       explicitly start EPD (ingress, inference, Workcell contract bridge) on the cell's bound camera,
            measure the real streams and EPD results, evaluate every observation against the cell's saved
            TaskIntent and write <session>/status.json + <session>/preview.jpg until stopped (SIGINT/SIGTERM)

Observations stay in the camera optical frame. Robot planning is reported BLOCKED until the cell carries a
qualified camera-to-world calibration; this tool never invents a transform, an object or a timestamp.
"""
import argparse
import hashlib
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

SCRIPTS = Path(__file__).resolve().parent
WORKSPACE = SCRIPTS.parents[2]
RECORD_NAME = 'workcell_perception_runtime.json'
SCHEMA = 'workcell_perception_runtime/v1'
STATUS_SCHEMA = 'workcell_live_perception_status/v1'
sys.path.insert(0, str(SCRIPTS))

RGB_ENCODINGS = {'rgb8': 3, 'bgr8': 3}
DEPTH_ENCODINGS = {'16UC1': 2, 'mono16': 2, '32FC1': 4}
WINDOW_S = 10.0                 # rate measurement window
CAMERA_STALE_S = 2.0            # no frame for this long: camera disconnected or stalled
EPD_STALE_S = 5.0               # no EPD result for this long: EPD not producing


class PerceptionUnavailable(RuntimeError):
    """The perception runtime or the cell's camera binding is missing or no longer valid."""


# ---------------------------------------------------------------------------------------------------------
# perception runtime record (like the qualified physics runtime: registered once, re-validated on every use)
def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _epd_package(setup):
    """EPD package share dir and its configured model, resolved through the registered EPD setup."""
    command = f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash && source {shlex.quote(str(setup))} " \
              "&& ros2 pkg prefix easy_perception_deployment"
    done = subprocess.run(['bash', '-c', command], capture_output=True, text=True, timeout=60)
    if done.returncode != 0 or not done.stdout.strip():
        raise PerceptionUnavailable(f'easy_perception_deployment is not installed under {setup}')
    share = Path(done.stdout.strip().splitlines()[-1]) / 'share/easy_perception_deployment'
    session = json.loads((share / 'config/session_config.json').read_text())
    model = (share / session['path_to_model']).resolve()
    labels = (share / session['path_to_label_list']).resolve()
    bridge = share / 'launch/workcell_contract_bridge.py'
    for path, what in ((model, 'model'), (labels, 'label list'), (bridge, 'Workcell contract bridge')):
        if not path.is_file():
            raise PerceptionUnavailable(f'EPD {what} missing: {path}')
    return dict(share=str(share), model=str(model), model_sha256_prefix=_sha256(model)[:16],
                labels=str(labels), bridge=str(bridge))


def register(args):
    setup = Path(args.epd_setup).resolve()
    if not setup.is_file():
        raise PerceptionUnavailable(f'EPD setup {setup} does not exist')
    record = dict(schema=SCHEMA, workspace=str(Path(args.workspace).resolve()), epd_setup=str(setup),
                  ros_setup=f"/opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash",
                  ros_domain_id=int(args.ros_domain_id), rmw_implementation=args.rmw_implementation,
                  ros_localhost_only=bool(args.ros_localhost_only), epd_execution_backend=args.epd_backend,
                  epd=_epd_package(setup))
    path = Path(args.workspace).resolve() / RECORD_NAME
    path.write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(dict(status='REGISTERED', record=str(path))))
    return 0


def load_runtime(workspace=WORKSPACE):
    path = Path(workspace).resolve() / RECORD_NAME
    if not path.is_file():
        raise PerceptionUnavailable(
            f'no perception runtime is registered for workspace {Path(workspace).resolve()}: run '
            'scripts/live_perception_session.py register --epd-setup <EPD install/setup.bash> --ros-domain-id <id>')
    record = json.loads(path.read_text())
    if record.get('schema') != SCHEMA:
        raise PerceptionUnavailable('unknown perception runtime record schema')
    epd = _epd_package(record['epd_setup'])
    if epd['model'] != record['epd']['model'] or epd['model_sha256_prefix'] != record['epd']['model_sha256_prefix']:
        raise PerceptionUnavailable('the EPD model changed since registration; register the perception runtime again')
    return dict(record, epd=epd)


def shell_environment(record):
    q = shlex.quote
    return '\n'.join([f"source {q(record['ros_setup'])}", f"source {q(record['epd_setup'])}",
                      f"export ROS_DOMAIN_ID={int(record['ros_domain_id'])}",
                      f"export RMW_IMPLEMENTATION={q(record['rmw_implementation'])}",
                      f"export ROS_LOCALHOST_ONLY={1 if record['ros_localhost_only'] else 0}",
                      f"export EPD_EXECUTION_BACKEND={q(record['epd_execution_backend'])}"]) + '\n'


# ---------------------------------------------------------------------------------------------------------
# camera binding (authored with the cell)
def _environment(cell):
    return yaml.safe_load((Path(cell) / 'environment.yaml').read_text()) or {}


def camera_binding(cell):
    binding = ((_environment(cell).get('perception') or {}).get('live_camera'))
    if not isinstance(binding, dict):
        raise PerceptionUnavailable('the cell has no live camera binding: Discover cameras and Save the binding')
    missing = [k for k in ('camera_id', 'rgb_topic', 'aligned_depth_topic', 'camera_info_topic', 'optical_frame')
               if not binding.get(k)]
    if missing:
        raise PerceptionUnavailable(f'the live camera binding lacks {missing}')
    return binding


def calibration_status(cell, binding):
    """Only a recorded, qualified camera-to-world calibration enables robot-frame use (none exists yet)."""
    calibration = binding.get('calibration') or {}
    if calibration.get('status') == 'qualified' and calibration.get('camera_to_world'):
        return dict(status='QUALIFIED', frame=binding['optical_frame'], evidence=calibration.get('evidence'))
    return dict(status='UNCALIBRATED', frame=binding['optical_frame'],
                reason='no qualified camera-to-world transform is recorded for this camera; observations stay in '
                       f"{binding['optical_frame']} and cannot be used for robot planning")


def task_filter(cell):
    """The cell's saved TaskIntent selection policy, used unchanged."""
    path = Path(cell) / 'config/workcell_builder_task_intent.yaml'
    intent = yaml.safe_load(path.read_text()) if path.is_file() else None
    if not isinstance(intent, dict) or intent.get('schema') != 'workcell_builder_task_intent/v2':
        return None
    selection = intent['pick']['selection']
    return dict(source_type=selection.get('source_type'), **selection['object_filter'])


def bind(args):
    """Save a camera binding with the cell (written through the authored-YAML writer)."""
    from authored_yaml import write_preserving
    cell = Path(args.cell)
    doc = _environment(cell)
    previous = ((doc.get('perception') or {}).get('live_camera') or {})
    camera_id = args.camera_id or (doc.get('camera') or {}).get('camera_id')
    if not camera_id:
        raise PerceptionUnavailable('the cell has no authored camera (camera.camera_id) to bind the live camera to')
    binding = dict(camera_id=camera_id, driver=args.driver, rgb_topic=args.rgb_topic,
                   aligned_depth_topic=args.aligned_depth_topic, camera_info_topic=args.camera_info_topic,
                   optical_frame=args.optical_frame, metadata_topic=args.metadata_topic or None,
                   calibration=previous.get('calibration') or dict(status='uncalibrated', camera_to_world=None))
    doc.setdefault('perception', {})['live_camera'] = binding
    write_preserving(cell / 'environment.yaml', doc)
    print(json.dumps(dict(status='BOUND', cell=str(cell), live_camera=binding)))
    return 0


# ---------------------------------------------------------------------------------------------------------
# observation evaluation (pure; unit-tested)
def evaluate_observation(item, *, now_ns, snapshot_ns, filters, live_ceiling_s, calibration, age_s=None, host_clock_reason=None):
    """Task eligibility of one camera-frame observation, with every reason it is not eligible.

    ``age_s``: the conservative frame age (TransportClock) when known, else the stamp age is used.
    ``host_clock_reason``: set when the host clock cannot age frames (HostClock FAULT/VERIFYING).
    """
    reasons = []
    label = item.get('label', '')
    if age_s is None:
        age_s = (now_ns - snapshot_ns) / 1e9 if snapshot_ns else math.inf
    position = (item.get('pose') or {}).get('position') or item.get('centroid')
    confidence = item.get('confidence')
    if position is None:
        reasons.append('no measured 3D position')
    if not item.get('dimensions_xyz'):
        reasons.append('no measured dimensions')
    if host_clock_reason:
        reasons.append(host_clock_reason)
    if age_s > live_ceiling_s:
        reasons.append(f'stale: {age_s:.2f} s old (live limit {live_ceiling_s:.1f} s)')
    if age_s < -0.5:
        reasons.append(f'stamped {-age_s:.2f} s in the future (camera clock domain)')
    excluded = False
    if filters is None:
        reasons.append('the cell has no saved v2 task')
    else:
        authored_age = filters.get('max_age_seconds')
        if authored_age is not None and age_s > authored_age and age_s <= live_ceiling_s:
            reasons.append(f'older than the task limit {authored_age:.1f} s')
        if label in (filters.get('exclude_class_ids') or []):
            excluded = True
            reasons.append(f'class {label!r} is excluded by the task')
        elif filters.get('class_id') and label != filters['class_id']:
            reasons.append(f"class {label!r} is not the task's pick class {filters['class_id']!r}")
        minimum = filters.get('min_confidence')
        if minimum is not None:
            if confidence is None:
                reasons.append('confidence unavailable for the task minimum')
            elif confidence < minimum:
                reasons.append(f'confidence {confidence:.2f} below task minimum {minimum:.2f}')
    eligible = not reasons
    robot = ('BLOCKED: ' + calibration['reason']) if calibration['status'] != 'QUALIFIED' else \
        ('READY' if eligible else 'BLOCKED: not eligible')
    return dict(object_id=item.get('object_id'), label=label, confidence=confidence, position_camera=position,
                dimensions_m=item.get('dimensions_xyz'), roi_xywh=(item.get('attributes') or {}).get('roi_xywh'),
                valid_depth_ratio=(item.get('attributes') or {}).get('valid_depth_ratio'),
                age_s=round(age_s, 3) if math.isfinite(age_s) else None, task_eligible=eligible, excluded=excluded,
                obstacle=('candidate obstacle once a qualified camera-to-world transform exists' if excluded else None),
                reasons=reasons, robot_planning=robot)


# ---------------------------------------------------------------------------------------------------------
class TransportClock:
    """Frame age that published stamps cannot understate, from the camera's own per-frame metadata.

    librealsense metadata carries the device capture clock (hw_timestamp, a 32-bit microsecond counter) and
    the host kernel arrival time (backend_timestamp, ms, host wall clock). arrival is the metadata's receipt
    on CLOCK_MONOTONIC_RAW (hardware clock: no NTP slew or steps), falling back to backend_timestamp. If the
    link stalls and drains, arrival - capture grows while published stamps can be re-anchored to look fresh;
    backlog = (arrival - capture) - its minimum is a stamp-independent lower bound on that extra delay, so a
    frame is at least (now - arrival) + backlog old. A stamp later than its own kernel arrival (same wall
    clock) is physically impossible and marks a clock-domain error.
    """
    WRAP = 2 ** 32
    MAX_FRAMES = 400

    def __init__(self):
        self.floor_s, self.last_hw, self.wraps, self.frames = None, None, 0, {}
        self.backlog_s, self.stamp_after_arrival, self.resets = None, False, 0

    def update(self, stamp_ns, metadata, arrival_s=None):
        hw, arrival_ms = metadata.get('hw_timestamp'), metadata.get('backend_timestamp')
        if not isinstance(hw, (int, float)) or not isinstance(arrival_ms, (int, float)):
            return None
        self.stamp_after_arrival = stamp_ns / 1e9 > arrival_ms / 1000.0 + 0.05
        if self.last_hw is not None and hw < self.last_hw - self.WRAP / 2:
            self.wraps += 1
        elif self.last_hw is not None and hw < self.last_hw:          # device clock restarted: relearn the floor
            self.floor_s, self.wraps, self.resets = None, 0, self.resets + 1
        self.last_hw = hw
        arrival = arrival_ms / 1000.0 if arrival_s is None else arrival_s
        delay = arrival - (hw + self.wraps * self.WRAP) / 1e6
        self.floor_s = delay if self.floor_s is None else min(self.floor_s, delay)
        self.backlog_s = delay - self.floor_s
        self.frames[stamp_ns] = (arrival, self.backlog_s)
        while len(self.frames) > self.MAX_FRAMES:
            self.frames.pop(next(iter(self.frames)))
        return self.backlog_s

    def age(self, stamp_ns, now_s, now_arrival_s=None):
        """(conservative age in s, 'transport' when the transport bound decided it else 'stamp', verified).

        now_s is the wall clock (stamps); now_arrival_s the clock arrivals were recorded on (default now_s)."""
        stamp_age = now_s - stamp_ns / 1e9
        frame = self.frames.get(stamp_ns)
        if frame is None:
            return stamp_age, 'stamp', False
        transport_age = ((now_s if now_arrival_s is None else now_arrival_s) - frame[0]) + frame[1]
        return (transport_age, 'transport', True) if transport_age > stamp_age else (stamp_age, 'stamp', True)


def _raw():
    """CLOCK_MONOTONIC_RAW: the hardware counter, untouched by NTP slewing, kernel tick changes or clock steps."""
    return time.clock_gettime(time.CLOCK_MONOTONIC_RAW)


class HostClock:
    """Whether the host clock can be trusted to age frames: every freshness check compares stamps to it.

    Samples (raw, monotonic, wall) and checks over a sliding window that CLOCK_MONOTONIC runs at the hardware
    rate (NTP may slew at most 500 ppm) and that the wall clock was not stepped. On LENLEG (WSL2) the kernel
    tick was 9010 us instead of 10000: MONOTONIC ran at 0.909x and systemd-timesyncd stepped the wall clock
    +3.2 s every ~35 s, so stamp ages were wrong by seconds in both directions.
    """
    WINDOW_S, MIN_WINDOW_S, MAX_RATE_ERROR, MAX_STEP_S = 90.0, 10.0, 0.002, 0.05

    def __init__(self, clock=None):
        self.clock = clock or (lambda: (_raw(), time.monotonic(), time.time()))
        self.samples, self.steps = [], []

    def sample(self):
        raw, mono, wall = self.clock()
        if self.samples:
            last = self.samples[-1]
            step = (wall - last[2]) - (mono - last[1])           # wall and monotonic share slewing, not steps
            if abs(step) > self.MAX_STEP_S:
                self.steps.append((raw, step))
        self.samples.append((raw, mono, wall))
        while self.samples and raw - self.samples[0][0] > self.WINDOW_S:
            self.samples.pop(0)
        self.steps = [s for s in self.steps if raw - s[0] <= self.WINDOW_S]

    def health(self):
        if not self.samples or self.samples[-1][0] - self.samples[0][0] < self.MIN_WINDOW_S:
            span = self.samples[-1][0] - self.samples[0][0] if self.samples else 0.0
            return dict(state='VERIFYING', reason=f'measuring the host clock ({span:.0f} of {self.MIN_WINDOW_S:.0f} s)')
        first, last = self.samples[0], self.samples[-1]
        rate = (last[1] - first[1]) / (last[0] - first[0])
        report = dict(monotonic_rate_vs_hardware=round(rate, 5), wall_steps_in_window=len(self.steps),
                      wall_step_total_s=round(sum(s[1] for s in self.steps), 3), window_s=round(last[0] - first[0], 1))
        problems = []
        if abs(rate - 1.0) > self.MAX_RATE_ERROR:
            problems.append(f'CLOCK_MONOTONIC runs at {rate:.3f}x the hardware clock (kernel tick or time sync fault)')
        if self.steps:
            problems.append(f'wall clock stepped {len(self.steps)} time(s) by {report["wall_step_total_s"]:+.1f} s '
                            f'in {report["window_s"]:.0f} s')
        if problems:
            return dict(state='FAULT', reason='host clock unreliable, frame ages cannot be verified: ' + '; '.join(problems), **report)
        return dict(state='OK', reason=None, **report)


class Rate:
    def __init__(self):
        self.times = []

    def tick(self, t):
        self.times.append(t)
        cutoff = t - WINDOW_S
        while self.times and self.times[0] < cutoff:
            self.times.pop(0)

    def hz(self, now):
        recent = [t for t in self.times if t >= now - WINDOW_S]
        return round((len(recent) - 1) / (recent[-1] - recent[0]), 3) if len(recent) > 2 and recent[-1] > recent[0] else 0.0


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Session:
    """Measures the bound camera and EPD, evaluates observations, writes status.json and preview.jpg."""

    def __init__(self, node, cell, binding, record, out):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String
        from diagnostic_msgs.msg import DiagnosticArray
        from task_intent_v2 import observation_age_cap
        self.node, self.cell, self.binding, self.record, self.out = node, Path(cell), binding, record, Path(out)
        self.lock = threading.Lock()
        self.filters = task_filter(cell)
        self.calibration = calibration_status(cell, binding)
        self.live_ceiling_s = observation_age_cap('live_epd_realsense', 'live_sensor')
        self.streams = {k: dict(rate=Rate(), last_recv=None, last_stamp_ns=None, info=None, ages=[])
                        for k in ('rgb', 'depth', 'camera_info', 'epd_result', 'epd_image')}
        self.depth_valid_ratio = None
        self.rgb_stamps, self.depth_stamps, self.synced = [], [], Rate()
        self.snapshot, self.snapshot_recv, self.connector, self.inference, self.ingress = None, None, None, None, None
        self.latest_rgb, self.latest_epd_image = None, None
        self.transport, self.host_clock = TransportClock(), HostClock()
        reliable = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(Image, binding['rgb_topic'], lambda m: self.on_image('rgb', m), reliable)
        node.create_subscription(Image, binding['aligned_depth_topic'], lambda m: self.on_image('depth', m), reliable)
        node.create_subscription(CameraInfo, binding['camera_info_topic'], self.on_info, reliable)
        if binding.get('metadata_topic'):
            from realsense2_camera_msgs.msg import Metadata
            node.create_subscription(Metadata, binding['metadata_topic'], self.on_metadata,
                                     QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE))
        node.create_subscription(Image, '/easy_perception_deployment/image_output', lambda m: self.on_image('epd_image', m), reliable)
        node.create_subscription(String, '/workcell_studio/epd_detection_snapshot_json', self.on_snapshot, 10)
        node.create_subscription(String, '/workcell_studio/epd_connector_status', self.on_connector, 10)
        node.create_subscription(DiagnosticArray, '/easy_perception_deployment/inference_diagnostics',
                                 lambda m: self.on_diag('inference', m), 10)
        node.create_subscription(DiagnosticArray, '/easy_perception_deployment/sensor_ingress/health',
                                 lambda m: self.on_diag('ingress', m), 10)

    # --- callbacks -------------------------------------------------------------------------------------
    def _seen(self, kind, header):
        now, raw = time.time(), _raw()
        s = self.streams[kind]
        s['rate'].tick(raw); s['last_recv'] = raw; s['last_stamp_ns'] = _stamp_ns(header.stamp)
        s['ages'] = (s['ages'] + [now - s['last_stamp_ns'] / 1e9])[-50:]
        return now

    def on_image(self, kind, msg):
        with self.lock:
            self._seen(kind, msg.header)
            bpp = (RGB_ENCODINGS if kind != 'depth' else DEPTH_ENCODINGS).get(msg.encoding)
            self.streams[kind]['info'] = dict(encoding=msg.encoding, width=msg.width, height=msg.height, step=msg.step,
                frame_id=msg.header.frame_id, encoding_supported=bpp is not None,
                stride_valid=bpp is not None and msg.step >= msg.width * bpp and len(msg.data) == msg.step * msg.height)
            stamp = _stamp_ns(msg.header.stamp)
            if kind == 'rgb':
                self.latest_rgb = msg
                self.rgb_stamps = (self.rgb_stamps + [stamp])[-60:]
                if stamp in self.depth_stamps: self.synced.tick(_raw())
            elif kind == 'depth':
                self.depth_stamps = (self.depth_stamps + [stamp])[-60:]
                if stamp in self.rgb_stamps: self.synced.tick(_raw())
                if self.streams['depth']['rate'].times and len(self.streams['depth']['rate'].times) % 5 == 1:
                    self.depth_valid_ratio = self._depth_valid(msg)
            elif kind == 'epd_image':
                self.latest_epd_image = msg

    @staticmethod
    def _depth_valid(msg):
        import numpy as np
        if msg.encoding not in DEPTH_ENCODINGS or len(msg.data) != msg.step * msg.height:
            return None
        dtype = np.float32 if msg.encoding == '32FC1' else np.uint16
        depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // np.dtype(dtype).itemsize)[:, :msg.width]
        valid = np.isfinite(depth) & (depth > 0)
        return round(float(valid.mean()), 4)

    def on_info(self, msg):
        with self.lock:
            self._seen('camera_info', msg.header)
            k = [float(v) for v in msg.k]
            self.streams['camera_info']['info'] = dict(frame_id=msg.header.frame_id, width=msg.width, height=msg.height,
                fx=k[0], fy=k[4], cx=k[2], cy=k[5], distortion_model=msg.distortion_model,
                intrinsics_valid=k[0] > 0 and k[4] > 0 and 0 < k[2] < msg.width and 0 < k[5] < msg.height)

    def on_metadata(self, msg):
        with self.lock:
            try:
                self.transport.update(_stamp_ns(msg.header.stamp), json.loads(msg.json_data), arrival_s=_raw())
            except ValueError:
                pass

    def on_snapshot(self, msg):
        with self.lock:
            try:
                snapshot = json.loads(msg.data)
            except ValueError:
                return
            self.snapshot, self.snapshot_recv = snapshot, _raw()
            self.streams['epd_result']['rate'].tick(self.snapshot_recv)
            self.streams['epd_result']['last_recv'] = self.snapshot_recv
            self.streams['epd_result']['last_stamp_ns'] = snapshot.get('timestamp_ns')

    def on_connector(self, msg):
        with self.lock:
            try:
                self.connector = json.loads(msg.data)
            except ValueError:
                pass

    def on_diag(self, which, msg):
        with self.lock:
            for status in msg.status:
                setattr(self, which, dict(name=status.name, message=status.message,
                                          values={v.key: v.value for v in status.values}))

    # --- status ----------------------------------------------------------------------------------------
    def _stream(self, kind, now):
        s = self.streams[kind]
        ages = sorted(s['ages'])
        return dict(rate_hz=s['rate'].hz(now),
                    since_last_s=round(now - s['last_recv'], 3) if s['last_recv'] else None,
                    receipt_minus_stamp_s=round(ages[len(ages) // 2], 3) if ages else None, info=s['info'])

    def status(self, state='RUNNING'):
        with self.lock:
            now = time.time(); now_ns = time.time_ns(); raw = _raw()
            self.host_clock.sample(); host_clock = self.host_clock.health()
            rgb, depth, info = self._stream('rgb', raw), self._stream('depth', raw), self._stream('camera_info', raw)
            camera_reasons = []
            for name, stream in (('RGB', rgb), ('aligned depth', depth), ('CameraInfo', info)):
                if stream['since_last_s'] is None:
                    camera_reasons.append(f'no {name} received')
                elif stream['since_last_s'] > CAMERA_STALE_S:
                    camera_reasons.append(f'{name} stalled {stream["since_last_s"]:.1f} s (camera disconnected or stalled)')
            for name, stream, table in (('RGB', rgb, RGB_ENCODINGS), ('aligned depth', depth, DEPTH_ENCODINGS)):
                i = stream['info']
                if i and not i['encoding_supported']:
                    camera_reasons.append(f'{name} encoding {i["encoding"]} unsupported')
                if i and not i['stride_valid']:
                    camera_reasons.append(f'{name} stride/size invalid')
                if i and i['frame_id'] != self.binding['optical_frame']:
                    camera_reasons.append(f'{name} frame {i["frame_id"]!r} is not the bound {self.binding["optical_frame"]!r}')
            ci = info['info']
            if ci and not ci['intrinsics_valid']:
                camera_reasons.append('CameraInfo intrinsics invalid')
            if rgb['info'] and depth['info'] and (rgb['info']['width'], rgb['info']['height']) != (depth['info']['width'], depth['info']['height']):
                camera_reasons.append('depth is not aligned to the colour image size')
            if rgb['receipt_minus_stamp_s'] is not None and rgb['receipt_minus_stamp_s'] < -0.5:
                camera_reasons.append('frames are stamped ahead of the host clock (clock domain mismatch)')
            transport = self.transport
            rgb_stamp = self.streams['rgb']['last_stamp_ns']
            if self.binding.get('metadata_topic') and transport.stamp_after_arrival:
                camera_reasons.append('frames are stamped later than their own host arrival (camera clock domain error)')
            camera_state = 'BLOCKED' if camera_reasons else 'STREAMING'
            frame_age, age_source, verified = transport.age(rgb_stamp, now, raw) if rgb_stamp else (None, None, False)
            if frame_age is not None and self.streams['rgb']['last_recv']:
                frame_age -= raw - self.streams['rgb']['last_recv']          # age at receipt, like receipt_minus_stamp
            timing = dict(frame_age_at_receipt_s=round(frame_age, 3) if frame_age is not None else None,
                          decided_by=age_source, transport_verified=verified,
                          transport_backlog_s=round(transport.backlog_s, 3) if transport.backlog_s is not None else None,
                          stamp_age_at_receipt_s=rgb['receipt_minus_stamp_s'])
            if not self.binding.get('metadata_topic'):
                timing['warning'] = 'no camera metadata bound: frame age relies on the published stamps alone'
            if camera_state == 'STREAMING' and frame_age is not None and frame_age > self.live_ceiling_s:
                camera_state = 'STALE_FRAMES'
                camera_reasons.append(f'frames arrive {frame_age:.2f} s after capture (live limit {self.live_ceiling_s:.1f} s)')
            epd_reasons = []
            snapshot_ns = (self.snapshot or {}).get('timestamp_ns')
            result_age = transport.age(snapshot_ns, now, raw)[0] if snapshot_ns else None
            since_result = raw - self.snapshot_recv if self.snapshot_recv else None
            if self.snapshot is None:
                epd_state = 'WAITING'; epd_reasons.append('no EPD result yet')
            elif since_result > EPD_STALE_S:
                epd_state = 'STALLED'; epd_reasons.append(f'no EPD result for {since_result:.1f} s')
            else:
                epd_state = 'PRODUCING'
            detections = [evaluate_observation(item, now_ns=now_ns, snapshot_ns=snapshot_ns, filters=self.filters,
                                               live_ceiling_s=self.live_ceiling_s, calibration=self.calibration,
                                               age_s=result_age, host_clock_reason=host_clock['reason'])
                          for item in ((self.snapshot or {}).get('objects') or [])] if epd_state == 'PRODUCING' else []
            rejected = (self.snapshot or {}).get('rejected_objects') or [] if epd_state == 'PRODUCING' else []
            inference = (self.inference or {}).get('values', {})
            perception_ready = camera_state == 'STREAMING' and epd_state == 'PRODUCING' and host_clock['state'] == 'OK' and \
                result_age is not None and result_age <= self.live_ceiling_s
            clock_reasons = [] if host_clock['state'] == 'OK' else [host_clock['reason']]
            readiness = dict(
                camera=camera_state, epd=epd_state,
                perception=('READY' if perception_ready else 'BLOCKED: ' + '; '.join(
                    clock_reasons + camera_reasons + epd_reasons + ([f'EPD results are {result_age:.2f} s old (live limit '
                                                     f'{self.live_ceiling_s:.1f} s)'] if result_age and result_age > self.live_ceiling_s else []))),
                host_clock=host_clock['state'], calibration=self.calibration['status'],
                task=('NO_SAVED_TASK' if self.filters is None else
                      f"{sum(d['task_eligible'] for d in detections)} eligible of {len(detections)}"),
                robot_planning='BLOCKED: ' + (self.calibration.get('reason') or 'not eligible'),
                robot_execution='BLOCKED: real-camera observations never drive robot motion in this milestone')
            status = dict(schema=STATUS_SCHEMA, state=state, updated_wall_ns=now_ns, cell=str(self.cell),
                camera_id=self.binding['camera_id'], binding=self.binding,
                camera=dict(state=camera_state, reasons=camera_reasons, rgb=rgb, aligned_depth=depth, camera_info=info, timing=timing,
                            synchronized_rgbd_hz=self.synced.hz(raw), depth_valid_pixel_ratio=self.depth_valid_ratio),
                epd=dict(state=epd_state, reasons=epd_reasons, results_hz=self.streams['epd_result']['rate'].hz(raw),
                         result_age_s=round(result_age, 3) if result_age is not None else None,
                         process_time_ms=((self.snapshot or {}).get('provenance') or {}).get('process_time_ms'),
                         inference=inference, ingress=(self.ingress or {}).get('values'), connector=self.connector,
                         model=self.record['epd']['model'], backend=self.record['epd_execution_backend']),
                host_clock=host_clock, calibration=self.calibration, task_filter=self.filters, live_ceiling_s=self.live_ceiling_s,
                detections=detections, rejected_detections=rejected, readiness=readiness,
                snapshot_timestamp_ns=snapshot_ns)
            return status

    def preview(self):
        """EPD's own annotated result image (masks, boxes, labels) plus eligibility, else the raw frame."""
        import cv2
        import numpy as np
        with self.lock:
            epd, raw, snapshot_ns = self.latest_epd_image, self.latest_rgb, (self.snapshot or {}).get('timestamp_ns')
        source = epd if epd is not None and snapshot_ns and _stamp_ns(epd.header.stamp) == snapshot_ns else None
        msg = source or raw
        if msg is None:
            return None
        channels = 3
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]
        image = image.reshape(msg.height, msg.width, channels).copy()
        if msg.encoding == 'rgb8':
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        age = (time.time_ns() - _stamp_ns(msg.header.stamp)) / 1e9
        banner = ('EPD RESULT' if source else 'RAW CAMERA (no matching EPD result)') + f'  frame age {age:.1f} s'
        cv2.rectangle(image, (0, 0), (image.shape[1], 22), (0, 0, 0), -1)
        cv2.putText(image, banner, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if source:
            for d in self.status()['detections']:
                if d['roi_xywh']:
                    x, y, w, h = d['roi_xywh']
                    colour = (0, 200, 0) if d['task_eligible'] else (0, 0, 220) if d['excluded'] else (160, 160, 160)
                    cv2.rectangle(image, (x, y), (x + w, y + h), colour, 2)
                    text = f"{d['label']} {d['confidence']:.2f}" if d['confidence'] is not None else d['label']
                    if d['position_camera']:
                        text += f" z={d['position_camera'][2]:.2f}m"
                    cv2.putText(image, text, (x, max(32, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)
        return image


def _write_atomic(path, data):
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_bytes(data)
    os.replace(tmp, path)


def _spawn(command, env_lines, log, cwd=None):
    script = env_lines + (f'cd {shlex.quote(str(cwd))}\n' if cwd else '') + 'exec ' + ' '.join(shlex.quote(c) for c in command)
    return subprocess.Popen(['bash', '-c', script], stdout=open(log, 'w'), stderr=subprocess.STDOUT, start_new_session=True)


def _stop(processes):
    report = {}
    for name, proc in processes.items():
        if proc.poll() is None:
            for sig, wait in ((signal.SIGINT, 6), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    proc.wait(wait); break
                except subprocess.TimeoutExpired:
                    continue
        alive = subprocess.run(['pgrep', '-g', str(proc.pid)], capture_output=True, text=True).stdout.split()
        report[name] = dict(returncode=proc.returncode, remaining_pids=alive)
    return report


def run(args):
    import rclpy
    record = load_runtime(args.workspace)
    binding = camera_binding(args.cell)
    out = Path(args.session_dir); out.mkdir(parents=True, exist_ok=False)
    env_lines = shell_environment(record)
    share = Path(record['epd']['share'])
    # EPD's own isolated ingress re-publishes the bound camera to these EPD-internal topics.
    in_rgb = '/easy_perception_deployment/ingress/color/image_raw'
    in_depth = '/easy_perception_deployment/ingress/aligned_depth/image_raw'
    in_info = '/easy_perception_deployment/ingress/color/camera_info'
    commands = {
        'ingress': (['ros2', 'run', 'easy_perception_deployment', 'epd_sensor_ingress', '--ros-args',
                     '-p', f"rgb_input_topic:={binding['rgb_topic']}", '-p', f"depth_input_topic:={binding['aligned_depth_topic']}",
                     '-p', f"camera_info_input_topic:={binding['camera_info_topic']}", '-p', f'rgb_output_topic:={in_rgb}',
                     '-p', f'depth_output_topic:={in_depth}', '-p', f'camera_info_output_topic:={in_info}',
                     '-p', 'internal_qos_reliability:=reliable'], None),
        'epd': (['ros2', 'run', 'easy_perception_deployment', 'easy_perception_deployment', '--ros-args',
                 '-p', 'usecase_mode_override:=3', '-p', 'use_depth:=true', '-p', f'rgb_topic:={in_rgb}',
                 '-p', f'depth_topic:={in_depth}', '-p', f'camera_info_topic:={in_info}', '-p', 'input_qos_reliability:=reliable',
                 '-p', 'image_output_qos_reliability:=reliable', '-r', f'/easy_perception_deployment/image_input:={in_rgb}'], share),
        'bridge': (['python3', record['epd']['bridge'], '--scene-id', Path(args.cell).name, '--camera-id', binding['camera_id'],
                    '--source-mode', 'localization', '--runtime-mode', 'live'], None),
    }
    (out / 'commands.json').write_text(json.dumps({k: v[0] for k, v in commands.items()}, indent=1))
    stopping = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.set())
    rclpy.init()
    node = rclpy.create_node('workcell_live_perception_session')
    session = Session(node, args.cell, binding, record, out)
    processes = {name: _spawn(cmd, env_lines, out / f'{name}.log', cwd) for name, (cmd, cwd) in commands.items()}
    history = (out / 'history.jsonl').open('a')
    started, last_history = time.time(), 0.0
    try:
        while not stopping.is_set() and (args.duration <= 0 or time.time() - started < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - session.__dict__.setdefault('_last_write', 0) < 0.5:
                continue
            session._last_write = time.time()
            dead = {name: p.returncode for name, p in processes.items() if p.poll() is not None}
            status = session.status('RUNNING' if not dead else 'FAILED')
            status['processes'] = {name: dict(pid=p.pid, running=p.poll() is None, returncode=p.returncode)
                                   for name, p in processes.items()}
            if dead:
                status['readiness']['perception'] = f'BLOCKED: EPD process exited {dead}'
            _write_atomic(out / 'status.json', json.dumps(status, indent=1).encode())
            image = session.preview()
            if image is not None:
                import cv2
                ok, jpg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    _write_atomic(out / 'preview.jpg', jpg.tobytes())
            if time.time() - last_history >= 2.0:
                last_history = time.time(); history.write(json.dumps(status) + '\n'); history.flush()
            if dead:
                break
    finally:
        cleanup = _stop(processes)
        final = session.status('STOPPED')
        final['shutdown'] = dict(processes=cleanup, clean=all(not r['remaining_pids'] for r in cleanup.values()))
        _write_atomic(out / 'status.json', json.dumps(final, indent=1).encode())
        history.write(json.dumps(final) + '\n'); history.close()
        node.destroy_node(); rclpy.shutdown()
    return 0 if final['shutdown']['clean'] else 1


def _sample_frame_id(node, topic, timeout):
    """The frame a camera actually stamps its CameraInfo with (None if none arrives in time)."""
    from sensor_msgs.msg import CameraInfo
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    seen = []
    sub = node.create_subscription(CameraInfo, topic, lambda m: seen.append(m.header.frame_id),
                                   QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    import rclpy
    deadline = time.time() + timeout
    while not seen and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return seen[0] if seen else None


def discover(args):
    """RGB-D cameras publishing in the registered ROS graph: CameraInfo topics with sibling image topics."""
    import rclpy
    rclpy.init(); node = rclpy.create_node('workcell_camera_discovery')
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    topics = dict(node.get_topic_names_and_types())
    images = {t for t, types in topics.items() if 'sensor_msgs/msg/Image' in types}
    cameras = []
    for info_topic in sorted(t for t, types in topics.items() if 'sensor_msgs/msg/CameraInfo' in types):
        base = info_topic.rsplit('/', 1)[0]
        rgb = f'{base}/image_raw'
        if rgb not in images or 'depth' in base.rsplit('/', 1)[-1]:
            continue
        root = base.rsplit('/', 1)[0]
        depth = sorted(t for t in images if t.startswith(root + '/') and 'aligned_depth' in t and t.endswith('/image_raw'))
        publishers = node.get_publishers_info_by_topic(rgb)
        frame = _sample_frame_id(node, info_topic, args.timeout)
        metadata = f'{base}/metadata'
        cameras.append(dict(camera_namespace=root, rgb_topic=rgb, camera_info_topic=info_topic, optical_frame=frame,
                            metadata_topic=metadata if 'realsense2_camera_msgs/msg/Metadata' in topics.get(metadata, []) else None,
                            aligned_depth_topics=depth, publisher_nodes=sorted({f'{p.node_namespace}/{p.node_name}'.replace('//', '/') for p in publishers})))
    node.destroy_node(); rclpy.shutdown()
    print(json.dumps(dict(status='OK', ros_domain_id=os.environ.get('ROS_DOMAIN_ID'), cameras=cameras)))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    reg = sub.add_parser('register')
    reg.add_argument('--epd-setup', required=True); reg.add_argument('--ros-domain-id', type=int, required=True)
    reg.add_argument('--rmw-implementation', default='rmw_fastrtps_cpp')
    reg.add_argument('--ros-localhost-only', type=int, choices=(0, 1), default=1)
    reg.add_argument('--epd-backend', choices=('cpu', 'cuda', 'tensorrt'), default='cpu')
    sub.add_parser('env'); dis = sub.add_parser('discover'); dis.add_argument('--timeout', type=float, default=3.0)
    b = sub.add_parser('bind')
    for key in ('cell', 'rgb-topic', 'aligned-depth-topic', 'camera-info-topic', 'optical-frame'):
        b.add_argument('--' + key, required=True)
    b.add_argument('--camera-id', help="the cell's camera this binds (default: its authored camera.camera_id)")
    b.add_argument('--driver', default='realsense2_camera')
    b.add_argument('--metadata-topic', help='per-frame camera metadata (capture and arrival clocks), when published')
    r = sub.add_parser('run'); r.add_argument('--cell', required=True); r.add_argument('--session-dir', required=True)
    r.add_argument('--duration', type=float, default=0.0, help='seconds; 0 runs until stopped')
    for command in sub.choices.values():
        command.add_argument('--workspace', type=Path, default=WORKSPACE)
    args = parser.parse_args(argv)
    try:
        if args.command == 'register':
            return register(args)
        if args.command == 'bind':
            return bind(args)
        if args.command == 'discover':
            return discover(args)
        if args.command == 'env':
            sys.stdout.write(shell_environment(load_runtime(args.workspace)))
            return 0
        return run(args)
    except (PerceptionUnavailable, OSError, KeyError, ValueError) as exc:
        if args.command == 'env':
            print('echo ' + shlex.quote('PERCEPTION_RUNTIME_UNAVAILABLE: ' + str(exc)) + ' >&2; false')
        else:
            print(json.dumps(dict(status='BLOCKED', reason=str(exc))))
        return 2


if __name__ == '__main__':
    sys.exit(main())
